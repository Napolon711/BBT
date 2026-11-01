// ros2_libxr: STM32 下位机 ↔ ROS 2 转接桥（2027 赛季版）
//
// 设计要点
//  1) 下位机不再做任何裁判协议字段解析。两条裁判链路（常规链路 = 电源管理模块 UART、
//     图传链路 = 图传发送端 UART）上收到的每一个 CRC 通过的帧，整帧原样通过
//     SharedTopic 话题 ref_frame 转发上来，本节点负责组帧/拆帧/字段解析。
//     协议改动（新增 cmd_id、改位域）只需要改这个文件，不需要重新烧录下位机。
//  2) 反方向同理：本节点把要发的帧组好（含 SOF/长度/seq/CRC8/CRC16），通过 ref_tx
//     交给下位机，下位机只负责按 bus 选串口把字节发出去。
//  3) 每个 cmd_id 都发一份结构化消息（rm2027_interfaces），另外发一份给决策层的
//     兼容摘要 /referee/summary，保持既有 Ports 契约不变。
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <fcntl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "rm2027_interfaces/msg/chassis_state.hpp"
#include "rm2027_interfaces/msg/field_event.hpp"
#include "rm2027_interfaces/msg/game_result.hpp"
#include "rm2027_interfaces/msg/game_status.hpp"
#include "rm2027_interfaces/msg/ground_robot_position.hpp"
#include "rm2027_interfaces/msg/power_heat.hpp"
#include "rm2027_interfaces/msg/projectile_allowance.hpp"
#include "rm2027_interfaces/msg/referee_frame.hpp"
#include "rm2027_interfaces/msg/referee_tx.hpp"
#include "rm2027_interfaces/msg/referee_warning.hpp"
#include "rm2027_interfaces/msg/rfid_status.hpp"
#include "rm2027_interfaces/msg/robot_buff.hpp"
#include "rm2027_interfaces/msg/robot_damage.hpp"
#include "rm2027_interfaces/msg/robot_hp.hpp"
#include "rm2027_interfaces/msg/robot_position.hpp"
#include "rm2027_interfaces/msg/robot_status.hpp"
#include "rm2027_interfaces/msg/sentry_command.hpp"
#include "rm2027_interfaces/msg/sentry_info.hpp"


#include <climits>
#include "ros2_libxr/referee_2027.hpp"
#include "ros2_libxr/shared_topic.hpp"

namespace ros2_libxr
{
namespace
{
constexpr std::uint16_t kCmdGameStatus = 0x0001;
constexpr std::uint16_t kCmdGameResult = 0x0002;
constexpr std::uint16_t kCmdRobotHP = 0x0003;
constexpr std::uint16_t kCmdFieldEvent = 0x0101;
constexpr std::uint16_t kCmdWarning = 0x0104;
constexpr std::uint16_t kCmdRobotStatus = 0x0201;
constexpr std::uint16_t kCmdPowerHeat = 0x0202;
constexpr std::uint16_t kCmdRobotPos = 0x0203;
constexpr std::uint16_t kCmdRobotBuff = 0x0204;
constexpr std::uint16_t kCmdRobotDamage = 0x0206;
constexpr std::uint16_t kCmdProjectileAllowance = 0x0208;
constexpr std::uint16_t kCmdRfid = 0x0209;
constexpr std::uint16_t kCmdGroundRobotPos = 0x020B;
constexpr std::uint16_t kCmdSentryInfo = 0x020D;
constexpr std::uint16_t kCmdSentryCmd0301 = 0x0301;

constexpr std::uint8_t kMaxFramesPerEpoch = 16;
constexpr std::size_t kMaxRobotPos = 16;
constexpr std::size_t kMaxBuffs = 32;
constexpr std::size_t kMaxEvents = 16;
constexpr std::size_t kMaxStatic = 32;
constexpr std::int16_t kUnknownI16 = std::numeric_limits<std::int16_t>::min();
constexpr std::int32_t kUnknownI32 = std::numeric_limits<std::int32_t>::min();

// 底盘模式（与下位机 HostChassisMode 一致）
constexpr std::uint8_t kModeRelax = 0;
constexpr std::uint8_t kModeIndependent = 1;
constexpr std::uint8_t kModeRotor = 2;
constexpr std::uint8_t kModeFollow = 3;

constexpr std::uint16_t kSubCmdSentryDecision = 0x0120;
constexpr std::uint16_t kIdRefereeServer = 0x8080;

// 0x0003 机器人血量数据在 data 段里的下标（2027 是"己方/对方"视角，不是红蓝）
enum HpIndex : int
{
  HP_ALLY_HEAVY = 0,
  HP_RESERVED = 1,
  HP_ALLY_STD3 = 2,
  HP_ALLY_STD4 = 3,
  HP_DAMAGE_DIFF = 4,
  HP_ALLY_SENTRY = 5,
  HP_OUTPOST_R = 6,
  HP_ALLY_BASE = 7,
  HP_OUTPOST_B = 8,
  HP_ENEMY_BASE = 9,
  HP_ALLY_CONTROL = 10,
  HP_ENEMY_CONTROL = 11,
  HP_COUNT = 12
};

template <typename T>
T read_le(const std::uint8_t * p)
{
  T v{};
  std::memcpy(&v, p, sizeof(T));
  return v;
}

std::uint32_t steady_ms()
{
  return static_cast<std::uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

int open_serial(const std::string & port, int baud)
{
  const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
  {
    return -1;
  }
  struct termios tio{};
  if (::tcgetattr(fd, &tio) != 0)
  {
    ::close(fd);
    return -1;
  }
  ::cfmakeraw(&tio);
  speed_t sp = B115200;
  switch (baud)
  {
    case 921600: sp = B921600; break;
    case 460800: sp = B460800; break;
    case 230400: sp = B230400; break;
    case 115200: sp = B115200; break;
    default: sp = B115200; break;
  }
  ::cfsetispeed(&tio, sp);
  ::cfsetospeed(&tio, sp);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (::tcsetattr(fd, TCSANOW, &tio) != 0)
  {
    ::close(fd);
    return -1;
  }
  ::tcflush(fd, TCIOFLUSH);
  return fd;
}
}  // namespace

class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode() : rclcpp::Node("ros2_libxr_bridge")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud", 115200);
    robot_id_param_ = declare_parameter<int>("robot_id", 0);
    publish_raw_ = declare_parameter<bool>("publish_raw", true);
    zero_vel_timeout_ms_ = declare_parameter<int>("zero_vel_timeout_ms", 200);
    referee_timeout_ms_ = declare_parameter<int>("referee_timeout_ms", 2000);
    summary_rate_hz_ = declare_parameter<double>("summary_rate_hz", 20.0);
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    move_mode_topic_ = declare_parameter<std::string>("move_mode_topic", "/move_mode");
    spin_topic_ = declare_parameter<std::string>("spin_topic", "/cmd_spin");
    fold_cmd_topic_ = declare_parameter<std::string>("fold_cmd_topic", "/gimbal_fold_cmd");
    sentry_cmd_rate_hz_ = declare_parameter<double>("sentry_cmd_rate_hz", 5.0);
    sentry_cmd_hold_s_ = declare_parameter<double>("sentry_cmd_hold_s", 3.0);

    if (zero_vel_timeout_ms_ <= 0 || referee_timeout_ms_ <= 0 ||
        !std::isfinite(summary_rate_hz_) || summary_rate_hz_ <= 0.0 ||
        !std::isfinite(sentry_cmd_rate_hz_) || sentry_cmd_rate_hz_ <= 0.0 || sentry_cmd_rate_hz_ > 10.0 ||
        !std::isfinite(sentry_cmd_hold_s_) || sentry_cmd_hold_s_ <= 0.0 ||
        (robot_id_param_ != 0 && robot_id_param_ != 7 && robot_id_param_ != 107))
      throw std::invalid_argument("invalid bridge timeout/rate/robot_id parameters");
    if (robot_id_param_ > 0)
    {
      robot_id_ = robot_id_param_;
    }

    init_topics();
    init_publishers();
    init_subscribers();

    summary_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, summary_rate_hz_)),
        [this]() { publish_summary(); });
    watchdog_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {
      publish_zero_velocity_if_timed_out();
      maybe_send_sentry_command();
    });

    serial_thread_ = std::thread([this]() { serial_loop(); });
  }

  ~BridgeNode() override
  {
    running_.store(false);
    if (serial_thread_.joinable())
    {
      serial_thread_.join();
    }
    std::lock_guard<std::mutex> lock(write_mtx_);
    if (fd_ >= 0)
    {
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  // ------------------------------------------------------------------ 话题表
  void init_topics()
  {
    rx_names_ = {"ref_frame", "chassis_state", "ahrs_quaternion", "yawmotor_angle"};
    tx_names_ = {"chassis_data", "chassis_mode", "ref_tx", "fold_cmd"};
    for (const auto & n : rx_names_)
    {
      rx_crc_[TopicNameCrc32(n)] = n;
    }
    for (const auto & n : tx_names_)
    {
      tx_crc_[n] = TopicNameCrc32(n);
    }
  }

  void init_publishers()
  {
    raw_pub_ = create_publisher<rm2027_interfaces::msg::RefereeFrame>(
        "referee/raw", rclcpp::QoS(20));
    game_status_pub_ = create_publisher<rm2027_interfaces::msg::GameStatus>(
        "referee/game_status", rclcpp::QoS(1));
    game_result_pub_ = create_publisher<rm2027_interfaces::msg::GameResult>(
        "referee/game_result", rclcpp::QoS(1));
    robot_hp_pub_ = create_publisher<rm2027_interfaces::msg::RobotHP>(
        "referee/robot_hp", rclcpp::QoS(1));
    field_event_pub_ = create_publisher<rm2027_interfaces::msg::FieldEvent>(
        "referee/field_event", rclcpp::QoS(1));
    warning_pub_ = create_publisher<rm2027_interfaces::msg::RefereeWarning>(
        "referee/warning", rclcpp::QoS(5));
    robot_status_pub_ = create_publisher<rm2027_interfaces::msg::RobotStatus>(
        "referee/robot_status", rclcpp::QoS(1));
    power_heat_pub_ = create_publisher<rm2027_interfaces::msg::PowerHeat>(
        "referee/power_heat", rclcpp::QoS(1));
    robot_pos_pub_ = create_publisher<rm2027_interfaces::msg::RobotPosition>(
        "referee/robot_pos", rclcpp::QoS(1));
    buff_pub_ = create_publisher<rm2027_interfaces::msg::RobotBuff>(
        "referee/buff", rclcpp::QoS(1));
    damage_pub_ = create_publisher<rm2027_interfaces::msg::RobotDamage>(
        "referee/damage", rclcpp::QoS(10));
    allowance_pub_ = create_publisher<rm2027_interfaces::msg::ProjectileAllowance>(
        "referee/projectile_allowance", rclcpp::QoS(1));
    rfid_pub_ =
        create_publisher<rm2027_interfaces::msg::RfidStatus>("referee/rfid", rclcpp::QoS(1));
    ground_pos_pub_ = create_publisher<rm2027_interfaces::msg::GroundRobotPosition>(
        "referee/ground_robot_position", rclcpp::QoS(1));
    sentry_info_pub_ = create_publisher<rm2027_interfaces::msg::SentryInfo>(
        "referee/sentry_info", rclcpp::QoS(1));

    summary_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/referee/summary", rclcpp::QoS(1));
    operator_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/referee/operator_cmd", rclcpp::QoS(10));
    yaw_pub_ = create_publisher<std_msgs::msg::Float32>("/yawmotor_angle", rclcpp::QoS(50));
    fold_state_pub_ =
        create_publisher<std_msgs::msg::Int32>("/gimbal_fold_state", rclcpp::QoS(1));
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "serial/gimbal_joint_state", rclcpp::QoS(20));
    chassis_state_pub_ = create_publisher<rm2027_interfaces::msg::ChassisState>(
        "chassis/state", rclcpp::QoS(5));
  }

  void init_subscribers()
  {
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          // 车体系：ROS 的 x 向前、y 向左；下位机 struct move_vec 用 vx/vy/wz。
          if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
              !std::isfinite(msg->angular.z)) return;
          MoveVecWire mv{};
          mv.vx = static_cast<float>(msg->linear.x);
          mv.vy = static_cast<float>(msg->linear.y);
          mv.wz = static_cast<float>(effective_wz(msg->angular.z));
          if (!std::isfinite(mv.vx) || !std::isfinite(mv.vy) || !std::isfinite(mv.wz)) return;
          last_move_ = mv;
          send_topic("chassis_data", &mv, sizeof(mv));
          last_cmd_vel_ms_.store(steady_ms());
          zero_vel_sent_ = false;
        });

    // 底盘模式：0 RELAX / 1 INDEPENDENT / 2 ROTOR / 3 FOLLOW
    mode_sub_ = create_subscription<std_msgs::msg::Int32>(
        move_mode_topic_, 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
          const int m = msg->data;
          if (m < 0 || m > 3)
          {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "底盘模式 %d 超出 0-3，已忽略", m);
            return;
          }
          chassis_mode_ = static_cast<std::uint8_t>(m);
          const std::uint8_t v = chassis_mode_;
          send_topic("chassis_mode", &v, sizeof(v));
          // 切到 ROTOR 时立刻把当前小陀螺角速度下发一次，避免底盘用旧值起转
          MoveVecWire mv{};
          const bool fresh = steady_ms() - last_cmd_vel_ms_.load() < static_cast<std::uint32_t>(zero_vel_timeout_ms_);
          mv.vx = fresh ? last_move_.vx : 0.0F;
          mv.vy = fresh ? last_move_.vy : 0.0F;
          mv.wz = static_cast<float>(effective_wz(0.0));
          send_topic("chassis_data", &mv, sizeof(mv));
        });

    // 小陀螺目标角速度：只在 ROTOR 模式下生效，见 effective_wz()
    spin_sub_ = create_subscription<std_msgs::msg::Float32>(
        spin_topic_, 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
          if (!std::isfinite(msg->data)) return;
          spin_wz_ = static_cast<double>(msg->data);
          last_spin_ms_ = steady_ms();
          if (chassis_mode_ == kModeRotor)
          {
            MoveVecWire mv{};
            const bool fresh = steady_ms() - last_cmd_vel_ms_.load() < static_cast<std::uint32_t>(zero_vel_timeout_ms_);
            mv.vx = fresh ? last_move_.vx : 0.0F;
            mv.vy = fresh ? last_move_.vy : 0.0F;
            mv.wz = static_cast<float>(effective_wz(0.0));
            send_topic("chassis_data", &mv, sizeof(mv));
          }
        });

    fold_cmd_sub_ = create_subscription<std_msgs::msg::UInt8>(
        fold_cmd_topic_, 10, [this](const std_msgs::msg::UInt8::SharedPtr msg) {
          std::uint8_t v = msg->data;
          send_topic("fold_cmd", &v, sizeof(v));
        });

    sentry_cmd_sub_ = create_subscription<rm2027_interfaces::msg::SentryCommand>(
        "referee/sentry_command", 10,
        [this](const rm2027_interfaces::msg::SentryCommand::SharedPtr msg) {
          std::lock_guard<std::mutex> state_lock(state_mtx_);
          pending_sentry_cmd_ = *msg;
          have_pending_sentry_cmd_ = true;
          sentry_cmd_rx_ms_ = steady_ms();
        });
  }

  // ------------------------------------------------------------------ 串口线程
  void serial_loop()
  {
    SharedTopicParser parser(8192);
    parser.set_handler([this](const SharedTopicParser::Frame & f) { on_shared_frame(f); });

    std::vector<std::uint8_t> buffer(4096);
    while (running_.load())
    {
      if (fd_ < 0)
      {
        const int fd = open_serial(port_, baud_);
        if (fd < 0)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                               "等待下位机串口 %s（%d）…", port_.c_str(), baud_);
          std::this_thread::sleep_for(std::chrono::milliseconds(300));
          continue;
        }
        {
          std::lock_guard<std::mutex> lock(write_mtx_);
          fd_ = fd;
        }
        RCLCPP_INFO(get_logger(), "下位机串口已连接：%s @ %d", port_.c_str(), baud_);
      }

      struct pollfd pfd{};
      pfd.fd = fd_;
      pfd.events = POLLIN;
      const int pr = ::poll(&pfd, 1, 50);
      if (pr < 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      if (pr == 0)
      {
        continue;
      }
      if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
      {
        RCLCPP_WARN(get_logger(), "下位机串口断开，重连中");
        std::lock_guard<std::mutex> lock(write_mtx_);
        ::close(fd_);
        fd_ = -1;
        continue;
      }
      const ssize_t n = ::read(fd_, buffer.data(), buffer.size());
      if (n > 0)
      {
        parser.push(buffer.data(), static_cast<std::size_t>(n));
        if (parser.frames_bad() > bad_frames_reported_ + 100)
        {
          bad_frames_reported_ = parser.frames_bad();
          RCLCPP_WARN(get_logger(), "SharedTopic 坏帧累计 %lu（丢字节 %lu）",
                      static_cast<unsigned long>(parser.frames_bad()),
                      static_cast<unsigned long>(parser.bytes_dropped()));
        }
      }
      else if (n == 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  }

  void send_topic(const std::string & name, const void * payload, std::size_t len)
  {
    const auto it = tx_crc_.find(name);
    if (it == tx_crc_.end())
    {
      return;
    }
    std::uint8_t packet[512];
    const std::size_t total = EncodeSharedTopic(it->second, payload, len, now_us(), packet,
                                                sizeof(packet));
    if (total == 0)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(write_mtx_);
    if (fd_ < 0)
    {
      return;
    }
    std::size_t offset = 0;
    while (offset < total)
    {
      const ssize_t w = ::write(fd_, packet + offset, total - offset);
      if (w <= 0)
      {
        break;
      }
      offset += static_cast<std::size_t>(w);
    }
  }

  std::uint64_t now_us() const
  {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
  }

  // 做什么：算出真正要下发的底盘角速度。
  // 参数：cmd_vel_wz — /cmd_vel 给的角速度，rad/s。
  // 返回：ROTOR 模式下返回小陀螺目标角速度（/cmd_spin），否则返回 cmd_vel_wz。
  // 副作用：无。
  // 说明：下位机的 chassis_data.wz 只有一个字段，所以两种语义按模式复用：
  //       ROTOR 模式 = 小陀螺目标角速度；其他模式 = 底盘转向角速度。
  double effective_wz(double cmd_vel_wz) const
  {
    const bool spin_fresh = last_spin_ms_ != 0 &&
        steady_ms() - last_spin_ms_ < static_cast<std::uint32_t>(zero_vel_timeout_ms_);
    return chassis_mode_ == kModeRotor ? (spin_fresh ? spin_wz_ : 0.0) : cmd_vel_wz;
  }

  // ------------------------------------------------------------------ 收包分发
  void on_shared_frame(const SharedTopicParser::Frame & frame)
  {
    std::lock_guard<std::mutex> state_lock(state_mtx_);
    const auto it = rx_crc_.find(frame.topic_crc32);
    if (it == rx_crc_.end())
    {
      return;
    }
    const std::string & name = it->second;
    if (name == "ref_frame")
    {
      // 下位机按 sizeof(RefFrameWire)=136 发；这里容忍尾部被截断（调试/伪终端场景），
      // 只要求头部 8 字节在，且 raw 里的 len 不超过实际收到的字节数。
      if (frame.payload_len < 8)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "ref_frame 长度 %zu 小于最小头部 8", frame.payload_len);
        return;
      }
      RefFrameWire wire{};
      const std::size_t avail = std::min<std::size_t>(
          frame.payload_len, sizeof(RefFrameWire));
      std::memcpy(&wire, frame.payload, avail);
      if (wire.len > avail - 8)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "ref_frame 声明 len=%u 但只收到 %zu 字节",
                             static_cast<unsigned>(wire.len), avail - 8);
        return;
      }
      on_ref_frame(wire);
    }
    else if (name == "chassis_state")
    {
      if (frame.payload_len < sizeof(ChassisStateWire))
      {
        return;
      }
      ChassisStateWire wire{};
      std::memcpy(&wire, frame.payload, sizeof(wire));
      on_chassis_state(wire);
    }
    else if (name == "yawmotor_angle")
    {
      if (frame.payload_len < sizeof(float))
      {
        return;
      }
      float angle = 0.0F;
      std::memcpy(&angle, frame.payload, sizeof(angle));
      std_msgs::msg::Float32 msg;
      msg.data = angle;
      yaw_pub_->publish(msg);
      yawmotor_angle_ = angle;
      have_yaw_ = true;
      publish_joint_state();
    }
    else if (name == "ahrs_quaternion")
    {
      if (frame.payload_len < sizeof(QuatWire))
      {
        return;
      }
      QuatWire q{};
      std::memcpy(&q, frame.payload, sizeof(q));
      quat_ = q;
      have_quat_ = true;
      publish_joint_state();
    }
  }

  void publish_joint_state()
  {
    if (!have_quat_ || !have_yaw_)
    {
      return;
    }
    const double roll = 0.0;
    (void)roll;
    // 与旧桥保持一致：ZYX 顺序，pitch = asin(2(wy - zx))
    double sinp = 2.0 * (static_cast<double>(quat_.w) * quat_.y -
                         static_cast<double>(quat_.z) * quat_.x);
    sinp = std::max(-1.0, std::min(1.0, sinp));
    const double pitch = std::asin(sinp);
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = {"gimbal_pitch_joint", "gimbal_yaw_joint"};
    js.position = {pitch, static_cast<double>(yawmotor_angle_)};
    joint_pub_->publish(js);
  }

  void on_chassis_state(const ChassisStateWire & wire)
  {
    rm2027_interfaces::msg::ChassisState msg;
    msg.header.stamp = now();
    msg.vx = wire.vx;
    msg.vy = wire.vy;
    msg.wz = wire.wz;
    msg.fold_state = wire.fold_state;
    msg.posture = wire.posture;
    msg.buffer_energy_j = wire.buffer_energy_j;
    msg.power_w = wire.power_w;
    msg.tick_ms = wire.tick_ms;
    chassis_state_pub_->publish(msg);

    std_msgs::msg::Int32 fold;
    fold.data = wire.fold_state;
    fold_state_pub_->publish(fold);

    if (wire.posture > 0)
    {
      posture_from_chassis_ = wire.posture;
      have_posture_from_chassis_ = true;
    }
  }

  // ------------------------------------------------------------------ 裁判帧
  void on_ref_frame(const RefFrameWire & wire)
  {
    const std::uint16_t len = std::min<std::uint16_t>(wire.len, kRefRawCap);
    RefFrameView view{};
    const bool ok = ParseRefFrame(wire.raw, len, view);

    if (publish_raw_)
    {
      rm2027_interfaces::msg::RefereeFrame msg;
      msg.header.stamp = now();
      msg.bus = wire.bus;
      msg.seq = wire.seq;
      msg.tick_ms = wire.tick_ms;
      msg.len = len;
      msg.cmd_id = ok ? view.cmd_id : 0;
      msg.raw.assign(wire.raw, wire.raw + len);
      raw_pub_->publish(msg);
    }
    if (!ok)
    {
      ++bad_ref_frames_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "裁判帧校验失败（bus=%u len=%u，累计 %lu）", wire.bus, len,
                           static_cast<unsigned long>(bad_ref_frames_));
      return;
    }
    handle_command(wire.bus, view, wire.tick_ms);
  }

  void handle_command(std::uint8_t bus, const RefFrameView & view, std::uint32_t tick_ms)
  {
    const std::uint8_t * p = view.payload;
    const std::uint16_t n = view.payload_len;
    const rclcpp::Time stamp = now();

    auto need = [&](std::size_t want) {
      if (n < want)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "cmd 0x%04X 长度 %u 小于协议 %zu", view.cmd_id, n, want);
        return false;
      }
      command_rx_ms_[view.cmd_id] = steady_ms();
      return true;
    };

    switch (view.cmd_id)
    {
      case kCmdGameStatus: {
        if (!need(11))
        {
          return;
        }
        rm2027_interfaces::msg::GameStatus msg;
        msg.header.stamp = stamp;
        msg.game_type = p[0] & 0x0FU;
        msg.game_progress = (p[0] >> 4) & 0x0FU;
        msg.stage_remain_time = read_le<std::uint16_t>(p + 1);
        msg.sync_time_stamp = read_le<std::uint64_t>(p + 3);
        game_status_pub_->publish(msg);
        if (st_.game_progress == 4 && msg.game_progress != 4)
        {
          last_allowance_total_ = 0;
          last_remote_allowance_ = 0;
          last_remote_hp_ = 0;
          have_pending_sentry_cmd_ = false;
        }
        st_.game_progress = msg.game_progress;
        st_.game_type = msg.game_type;
        st_.stage_remain_time = msg.stage_remain_time;
        st_.has_game_status = true;
        break;
      }
      case kCmdGameResult: {
        if (!need(1))
        {
          return;
        }
        rm2027_interfaces::msg::GameResult msg;
        msg.header.stamp = stamp;
        msg.winner = p[0];
        game_result_pub_->publish(msg);
        break;
      }
      case kCmdRobotHP: {
        if (!need(24))
        {
          return;
        }
        rm2027_interfaces::msg::RobotHP msg;
        msg.header.stamp = stamp;
        msg.ally_1_robot_hp = read_le<std::uint16_t>(p + 0);
        msg.reserved = read_le<std::uint16_t>(p + 2);
        msg.ally_3_robot_hp = read_le<std::uint16_t>(p + 4);
        msg.ally_4_robot_hp = read_le<std::uint16_t>(p + 6);
        msg.damage_difference = read_le<std::int16_t>(p + 8);
        msg.ally_7_robot_hp = read_le<std::uint16_t>(p + 10);
        msg.outpost_r_hp = read_le<std::uint16_t>(p + 12);
        msg.ally_base_hp = read_le<std::uint16_t>(p + 14);
        msg.outpost_b_hp = read_le<std::uint16_t>(p + 16);
        msg.enemy_base_hp = read_le<std::uint16_t>(p + 18);
        msg.ally_control_degree = read_le<std::uint16_t>(p + 20);
        msg.enemy_control_degree = read_le<std::uint16_t>(p + 22);
        robot_hp_pub_->publish(msg);
        st_.hp[HP_ALLY_HEAVY] = msg.ally_1_robot_hp;
        st_.hp[HP_RESERVED] = msg.reserved;
        st_.hp[HP_ALLY_STD3] = msg.ally_3_robot_hp;
        st_.hp[HP_ALLY_STD4] = msg.ally_4_robot_hp;
        st_.hp[HP_ALLY_SENTRY] = msg.ally_7_robot_hp;
        st_.hp[HP_OUTPOST_R] = msg.outpost_r_hp;
        st_.hp[HP_OUTPOST_B] = msg.outpost_b_hp;
        st_.hp[HP_ALLY_BASE] = msg.ally_base_hp;
        st_.hp[HP_ENEMY_BASE] = msg.enemy_base_hp;
        st_.hp[HP_ALLY_CONTROL] = msg.ally_control_degree;
        st_.hp[HP_ENEMY_CONTROL] = msg.enemy_control_degree;
        st_.damage_difference = msg.damage_difference;
        st_.has_robot_hp = true;
        break;
      }
      case kCmdFieldEvent: {
        if (!need(8))
        {
          return;
        }
        const std::uint32_t e1 = read_le<std::uint32_t>(p);
        const std::uint32_t e2 = read_le<std::uint32_t>(p + 4);
        rm2027_interfaces::msg::FieldEvent msg;
        msg.header.stamp = stamp;
        msg.event_data = e1;
        msg.event_data_2 = e2;
        msg.supply_zone_active = static_cast<std::uint8_t>(e1 & 0x1U);
        msg.small_energy_state = static_cast<std::uint8_t>((e1 >> 1) & 0x3U);
        msg.big_energy_state = static_cast<std::uint8_t>((e1 >> 3) & 0x3U);
        msg.fortress_r_state = static_cast<std::uint8_t>((e1 >> 5) & 0x7U);
        msg.fortress_b_state = static_cast<std::uint8_t>((e1 >> 8) & 0x7U);
        msg.base_gain_r_state = static_cast<std::uint8_t>((e1 >> 11) & 0x7U);
        msg.base_gain_b_state = static_cast<std::uint8_t>((e1 >> 14) & 0x7U);
        msg.own_highland_gain = ((e1 >> 17) & 0x1U) != 0U;
        msg.central_highland_r_state = static_cast<std::uint8_t>((e1 >> 18) & 0x3U);
        msg.central_highland_b_state = static_cast<std::uint8_t>((e1 >> 20) & 0x3U);
        msg.outpost_gain_r_state = static_cast<std::uint8_t>((e1 >> 22) & 0x7U);
        msg.outpost_gain_b_state = static_cast<std::uint8_t>((e1 >> 25) & 0x7U);
        msg.outpost_r_control = static_cast<std::uint8_t>((e1 >> 28) & 0x3U);
        msg.outpost_b_control = static_cast<std::uint8_t>((e1 >> 30) & 0x3U);
        msg.base_r_control = static_cast<std::uint8_t>(e2 & 0x3U);
        msg.base_b_control = static_cast<std::uint8_t>((e2 >> 2) & 0x3U);
        msg.enemy_dart_last_hit_time_s = static_cast<std::uint16_t>((e2 >> 4) & 0x1FFU);
        msg.enemy_dart_last_hit_target = static_cast<std::uint8_t>((e2 >> 13) & 0x7U);
        msg.rmul_r_highland_activate = static_cast<std::uint8_t>((e2 >> 16) & 0x3U);
        msg.rmul_r_highland_control = static_cast<std::uint8_t>((e2 >> 18) & 0x3U);
        msg.rmul_b_highland_activate = static_cast<std::uint8_t>((e2 >> 20) & 0x3U);
        msg.rmul_b_highland_control = static_cast<std::uint8_t>((e2 >> 22) & 0x3U);
        msg.rmul_center_activate = static_cast<std::uint8_t>((e2 >> 24) & 0x3U);
        msg.rmul_center_control = static_cast<std::uint8_t>((e2 >> 26) & 0x3U);
        field_event_pub_->publish(msg);
        st_.event_data = e1;
        st_.event_data_2 = e2;
        st_.has_field_event = true;
        break;
      }
      case kCmdWarning: {
        if (!need(3))
        {
          return;
        }
        rm2027_interfaces::msg::RefereeWarning msg;
        msg.header.stamp = stamp;
        msg.level = p[0];
        msg.offending_robot_id = p[1];
        msg.count = p[2];
        warning_pub_->publish(msg);
        break;
      }
      case kCmdRobotStatus: {
        if (!need(17))
        {
          return;
        }
        rm2027_interfaces::msg::RobotStatus msg;
        msg.header.stamp = stamp;
        msg.robot_id = p[0];
        msg.robot_level = p[1];
        msg.current_hp = read_le<std::uint16_t>(p + 2);
        msg.maximum_hp = read_le<std::uint16_t>(p + 4);
        msg.shooter_barrel_cooling_value = read_le<std::uint16_t>(p + 6);
        msg.shooter_barrel_heat_limit = read_le<std::uint16_t>(p + 8);
        msg.chassis_power_limit = read_le<std::uint16_t>(p + 10);
        msg.shooter_initial_speed = read_le<float>(p + 12);
        msg.power_gimbal_output = (p[16] & 0x1U) != 0U;
        msg.power_chassis_output = (p[16] & 0x2U) != 0U;
        msg.power_launcher_output = (p[16] & 0x4U) != 0U;
        robot_status_pub_->publish(msg);
        st_.robot_status = msg;
        st_.has_robot_status = true;
        if (robot_id_param_ == 0)
        {
          robot_id_ = (msg.robot_id == 7 || msg.robot_id == 107) ? msg.robot_id : 0;
        }
        break;
      }
      case kCmdPowerHeat: {
        if (!need(14))
        {
          return;
        }
        rm2027_interfaces::msg::PowerHeat msg;
        msg.header.stamp = stamp;
        msg.buffer_energy = read_le<std::uint16_t>(p + 8);
        msg.shooter_17mm_heat = read_le<std::uint16_t>(p + 10);
        msg.shooter_42mm_heat = read_le<std::uint16_t>(p + 12);
        power_heat_pub_->publish(msg);
        st_.power_heat = msg;
        st_.has_power_heat = true;
        break;
      }
      case kCmdRobotPos: {
        if (!need(12))
        {
          return;
        }
        rm2027_interfaces::msg::RobotPosition msg;
        msg.header.stamp = stamp;
        msg.x = read_le<float>(p + 0);
        msg.y = read_le<float>(p + 4);
        msg.angle = read_le<float>(p + 8);
        robot_pos_pub_->publish(msg);
        st_.robot_pos = msg;
        st_.has_robot_pos = true;
        break;
      }
      case kCmdRobotBuff: {
        if (!need(7))
        {
          return;
        }
        rm2027_interfaces::msg::RobotBuff msg;
        msg.header.stamp = stamp;
        msg.recovery_buff = p[0];
        msg.cooling_buff = read_le<std::uint16_t>(p + 1);
        msg.defence_buff = p[3];
        msg.vulnerability_buff = p[4];
        msg.attack_buff = read_le<std::uint16_t>(p + 5);
        buff_pub_->publish(msg);
        break;
      }
      case kCmdRobotDamage: {
        if (!need(7))
        {
          return;
        }
        rm2027_interfaces::msg::RobotDamage msg;
        msg.header.stamp = stamp;
        msg.armor_id = static_cast<std::uint8_t>(p[0] & 0x0FU);
        msg.hp_deduction_reason = static_cast<std::uint8_t>((p[0] >> 4) & 0x0FU);
        msg.hit_sequence = read_le<std::uint16_t>(p + 1);
        msg.hit_position_x = read_le<std::int16_t>(p + 3);
        msg.hit_position_y = read_le<std::int16_t>(p + 5);
        damage_pub_->publish(msg);
        break;
      }
      case kCmdProjectileAllowance: {
        if (!need(8))
        {
          return;
        }
        rm2027_interfaces::msg::ProjectileAllowance msg;
        msg.header.stamp = stamp;
        msg.projectile_allowance_17mm = read_le<std::uint16_t>(p + 0);
        msg.projectile_allowance_42mm = read_le<std::uint16_t>(p + 2);
        msg.remaining_gold_coin = read_le<std::uint16_t>(p + 4);
        msg.projectile_allowance_fortress = read_le<std::uint16_t>(p + 6);
        allowance_pub_->publish(msg);
        st_.allowance = msg;
        st_.has_allowance = true;
        break;
      }
      case kCmdRfid: {
        if (!need(6))
        {
          return;
        }
        const std::uint32_t w1 = read_le<std::uint32_t>(p);
        const std::uint16_t w2 = read_le<std::uint16_t>(p + 4);
        rm2027_interfaces::msg::RfidStatus msg;
        msg.header.stamp = stamp;
        msg.rfid_status = w1;
        msg.rfid_status_2 = w2;
        auto bit = [](std::uint32_t v, int i) { return ((v >> i) & 0x1U) != 0U; };
        msg.supply_zone_own = bit(w1, 0);
        msg.assembly_own = bit(w1, 1);
        msg.base_gain_r = bit(w1, 2);
        msg.base_gain_b = bit(w1, 3);
        msg.fortress_r = bit(w1, 4);
        msg.fortress_b = bit(w1, 5);
        msg.central_highland_r = bit(w1, 6);
        msg.central_highland_b = bit(w1, 7);
        msg.outpost_r = bit(w1, 8);
        msg.outpost_b = bit(w1, 9);
        msg.highland_own = bit(w1, 10);
        msg.rmul_r_highland = bit(w1, 11);
        msg.rmul_b_highland = bit(w1, 12);
        msg.rmul_center = bit(w1, 13);
        msg.terrain_cross_own_center_down = bit(w1, 14);
        msg.terrain_cross_own_center_up = bit(w1, 15);
        msg.terrain_cross_enemy_center_down = bit(w1, 16);
        msg.terrain_cross_enemy_center_up = bit(w1, 17);
        msg.terrain_cross_own_highway_down = bit(w1, 18);
        msg.terrain_cross_own_highway_up = bit(w1, 19);
        msg.terrain_cross_enemy_highway_down = bit(w1, 20);
        msg.terrain_cross_enemy_highway_up = bit(w1, 21);
        msg.tunnel_own_highway_down = bit(w1, 22);
        msg.tunnel_own_highway_mid = bit(w1, 23);
        msg.tunnel_own_highway_up = bit(w1, 24);
        msg.tunnel_near_own_trapezoid_low = bit(w1, 25);
        msg.tunnel_near_own_trapezoid_mid = bit(w1, 26);
        msg.tunnel_near_own_trapezoid_high = bit(w1, 27);
        msg.tunnel_far_own_trapezoid_far = bit(w1, 28);
        msg.tunnel_far_own_trapezoid_mid = bit(w1, 29);
        msg.tunnel_far_own_trapezoid_near = bit(w1, 30);
        msg.tunnel_enemy_highway_down = bit(w2, 0);
        msg.tunnel_enemy_highway_mid = bit(w2, 1);
        msg.tunnel_enemy_highway_up = bit(w2, 2);
        msg.tunnel_near_enemy_trapezoid_low = bit(w2, 3);
        msg.tunnel_near_enemy_trapezoid_mid = bit(w2, 4);
        msg.tunnel_near_enemy_trapezoid_high = bit(w2, 5);
        msg.tunnel_far_enemy_trapezoid_far = bit(w2, 6);
        msg.tunnel_far_enemy_trapezoid_mid = bit(w2, 7);
        msg.tunnel_far_enemy_trapezoid_near = bit(w2, 8);
        rfid_pub_->publish(msg);
        st_.rfid = msg;
        st_.has_rfid = true;
        break;
      }
      case kCmdGroundRobotPos: {
        if (!need(40))
        {
          return;
        }
        rm2027_interfaces::msg::GroundRobotPosition msg;
        msg.header.stamp = stamp;
        msg.heavy_engineer.x = read_le<float>(p + 0);
        msg.heavy_engineer.y = read_le<float>(p + 4);
        msg.standard_3.x = read_le<float>(p + 16);
        msg.standard_3.y = read_le<float>(p + 20);
        msg.standard_4.x = read_le<float>(p + 24);
        msg.standard_4.y = read_le<float>(p + 28);
        msg.valid = true;
        ground_pos_pub_->publish(msg);
        st_.ground_pos = msg;
        st_.has_ground_pos = true;
        break;
      }
      case kCmdSentryInfo: {
        if (!need(14))
        {
          return;
        }
        const std::uint32_t w1 = read_le<std::uint32_t>(p);
        const std::uint16_t w2 = read_le<std::uint16_t>(p + 4);
        const std::uint64_t w3 = read_le<std::uint64_t>(p + 6);
        rm2027_interfaces::msg::SentryInfo msg;
        msg.header.stamp = stamp;
        msg.sentry_info = w1;
        msg.sentry_info_2 = w2;
        msg.sentry_info_3 = w3;
        msg.exchanged_projectile_allowance = static_cast<std::uint16_t>(w1 & 0x7FFU);
        msg.remote_allowance_exchange_times = static_cast<std::uint8_t>((w1 >> 11) & 0x0FU);
        msg.remote_hp_exchange_times = static_cast<std::uint8_t>((w1 >> 15) & 0x0FU);
        msg.can_free_respawn = ((w1 >> 19) & 0x1U) != 0U;
        msg.can_buy_respawn = ((w1 >> 20) & 0x1U) != 0U;
        msg.respawn_cost_coin = static_cast<std::uint16_t>((w1 >> 21) & 0x3FFU);
        msg.out_of_combat = (w2 & 0x1U) != 0U;
        msg.team_allowance_exchangeable = static_cast<std::uint16_t>((w2 >> 1) & 0x7FFU);
        msg.posture = static_cast<std::uint8_t>((w2 >> 12) & 0x3U);
        msg.energy_mechanism_activatable = ((w2 >> 14) & 0x1U) != 0U;
        msg.posture_strengthened = ((w2 >> 15) & 0x1U) != 0U;
        msg.attack_pose_remain_s = static_cast<std::uint8_t>(w3 & 0xFFU);
        msg.defence_pose_remain_s = static_cast<std::uint8_t>((w3 >> 8) & 0xFFU);
        msg.move_pose_remain_s = static_cast<std::uint8_t>((w3 >> 16) & 0xFFU);
        msg.strengthened_attack_remain_s = static_cast<std::uint8_t>((w3 >> 32) & 0xFFU);
        msg.strengthened_defence_remain_s = static_cast<std::uint8_t>((w3 >> 40) & 0xFFU);
        msg.strengthened_move_remain_s = static_cast<std::uint8_t>((w3 >> 48) & 0xFFU);
        sentry_info_pub_->publish(msg);
        st_.sentry_info = msg;
        st_.has_sentry_info = true;
        if (msg.posture > 0)
        {
          posture_from_referee_ = msg.posture + (msg.posture_strengthened ? 3 : 0);
          have_posture_from_referee_ = true;
        }
        break;
      }
      case 0x0303: {
        if (!need(12)) return;
        const float x = read_le<float>(p);
        const float y = read_le<float>(p + 4);
        if (!std::isfinite(x) || !std::isfinite(y)) return;
        const std::vector<std::uint8_t> payload(p, p + 12);
        if (payload == last_operator_payload_) return; // Server repeats each click.
        last_operator_payload_ = payload;
        if (++operator_seq_ == 0) ++operator_seq_;
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {p[9] == 0 ? 4.0 : 5.0, p[9] == 0 ? -1.0 : double(p[9]),
                    double(operator_seq_), double(x), double(y)};
        operator_pub_->publish(msg);
        break;
      }
      default:
        break;
    }
  }

  // 红方 ID 为 1..7，蓝方为 101..107
  bool is_red() const { return robot_id_ > 0 && (robot_id_ / 100) == 0; }

  // ------------------------------------------------------------------ 摘要
  void publish_summary()
  {
    std::lock_guard<std::mutex> state_lock(state_mtx_);
    const auto now_ms = steady_ms();
    const auto age = [&](std::uint16_t cmd) -> std::uint32_t {
      const auto it = command_rx_ms_.find(cmd);
      return it == command_rx_ms_.end() ? UINT32_MAX : now_ms - it->second;
    };
    const auto source_age = std::max({age(kCmdGameStatus), age(kCmdRobotStatus),
                                      age(kCmdProjectileAllowance)});
    // Do not turn a live timer into a heartbeat for a disconnected referee.
    if (source_age > static_cast<std::uint32_t>(referee_timeout_ms_)) return;
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(14, -1.0);
    msg.data[7] = 0.0;
    const bool red = is_red();
    const bool color_known = robot_id_ == 7 || robot_id_ == 107;

    if (st_.has_game_status)
    {
      msg.data[0] = static_cast<double>(st_.game_progress);
      msg.data[8] = static_cast<double>(st_.stage_remain_time);
    }
    if (st_.has_robot_status)
    {
      msg.data[1] = static_cast<double>(st_.robot_status.current_hp);
      msg.data[2] = static_cast<double>(st_.robot_status.maximum_hp);
    }
    if (st_.has_allowance)
    {
      msg.data[3] = static_cast<double>(st_.allowance.projectile_allowance_17mm);
      msg.data[4] = static_cast<double>(st_.allowance.remaining_gold_coin);
    }
    if (color_known && st_.has_robot_hp && age(kCmdRobotHP) <= static_cast<std::uint32_t>(referee_timeout_ms_))
    {
      const int own_outpost = red ? HP_OUTPOST_R : HP_OUTPOST_B;
      msg.data[5] = static_cast<double>(st_.hp[own_outpost]);
      msg.data[13] = st_.hp[red ? HP_OUTPOST_B : HP_OUTPOST_R];
      msg.data[10] = static_cast<double>(st_.hp[HP_ALLY_BASE]);
      // [11] 己方累计控制度（0x0003 偏移 20，量纲以协议为准，协议没写单位，别当百分比用）
      msg.data[11] = static_cast<double>(st_.hp[HP_ALLY_CONTROL]);
    }
    if (color_known && st_.has_field_event && age(kCmdFieldEvent) <= static_cast<std::uint32_t>(referee_timeout_ms_))
    {
      const std::uint8_t ctrl = red ? ((st_.event_data >> 28) & 0x3U)
                                    : ((st_.event_data >> 30) & 0x3U);
      // [6] 己方前哨阵地控制状态原始码：0 完全控制 / 1 临时控制 / 2 未控制
      msg.data[6] = static_cast<double>(ctrl);
      msg.data[7] = 1.0;
    }
    msg.data[9] = static_cast<double>(now_ms - source_age);
    msg.data[12] = source_age;
    summary_pub_->publish(msg);
  }

  // ------------------------------------------------------------------ 上行
  void publish_zero_velocity_if_timed_out()
  {
    const std::uint32_t now = steady_ms();
    const std::uint32_t last = last_cmd_vel_ms_.load();
    if ((last == 0 && last_spin_ms_ == 0) ||
        (last != 0 && now - last < static_cast<std::uint32_t>(zero_vel_timeout_ms_)) ||
        (last == 0 && now - last_spin_ms_ < static_cast<std::uint32_t>(zero_vel_timeout_ms_)))
    {
      return;
    }
    if (!zero_vel_sent_)
    {
      RCLCPP_WARN(get_logger(), "/cmd_vel 超过 %d ms 没有更新，下发零速", zero_vel_timeout_ms_);
      zero_vel_sent_ = true;
    }
    // 断流时所有自由度归零；电控必须实现独立的通信看门狗。
    spin_wz_ = 0.0;
    MoveVecWire mv{};
    send_topic("chassis_data", &mv, sizeof(mv));
  }

  void maybe_send_sentry_command()
  {
    std::lock_guard<std::mutex> state_lock(state_mtx_);
    if (!have_pending_sentry_cmd_)
    {
      return;
    }
    const std::uint32_t now = steady_ms();
    const double age_s = (now - sentry_cmd_rx_ms_) / 1000.0;
    if (age_s > sentry_cmd_hold_s_)
    {
      have_pending_sentry_cmd_ = false;
      return;
    }
    const std::uint32_t period_ms =
        static_cast<std::uint32_t>(1000.0 / std::max(1.0, sentry_cmd_rate_hz_));
    if (last_sentry_tx_ms_ != 0 && now - last_sentry_tx_ms_ < period_ms)
    {
      return;
    }
    last_sentry_tx_ms_ = now;
    if (!send_sentry_command(pending_sentry_cmd_))
    {
      return;
    }
  }

  bool send_sentry_command(const rm2027_interfaces::msg::SentryCommand & cmd)
  {
    if ((robot_id_ != 7 && robot_id_ != 107) ||
        command_rx_ms_.count(kCmdSentryInfo) == 0 ||
        steady_ms() - command_rx_ms_.at(kCmdSentryInfo) > static_cast<std::uint32_t>(referee_timeout_ms_)) return false;
    if (cmd.exchange_allowance_total > 2047 || cmd.remote_allowance_times > 15 || cmd.remote_hp_times > 15) return false;
    // Synchronize after bridge restart; a request counter may only advance by one.
    last_allowance_total_ = std::max(last_allowance_total_, st_.sentry_info.exchanged_projectile_allowance);
    last_remote_allowance_ = std::max(last_remote_allowance_, st_.sentry_info.remote_allowance_exchange_times);
    last_remote_hp_ = std::max(last_remote_hp_, st_.sentry_info.remote_hp_exchange_times);
    if (cmd.remote_allowance_times > last_remote_allowance_ + 1 || cmd.remote_hp_times > last_remote_hp_ + 1) return false;
    if (command_rx_ms_.count(kCmdGameStatus) == 0 ||
        steady_ms() - command_rx_ms_.at(kCmdGameStatus) > static_cast<std::uint32_t>(referee_timeout_ms_) ||
        st_.game_progress != 4) return false;
    // 姿态：0 表示"不改"，用裁判回读的姿态兜底；都没有就拒绝发送，绝不瞎猜。
    std::uint8_t posture = cmd.posture;
    if (posture == 0)
    {
      if (have_posture_from_referee_)
      {
        posture = posture_from_referee_;
      }
      else if (have_posture_from_chassis_)
      {
        posture = posture_from_chassis_;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "姿态未知（既没收到 0x020D 回读，也没有过底盘状态），"
                             "暂不发送 0x0120");
        return false;
      }
    }
    if (posture > 6)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "姿态 %u 超出 1~6，丢弃",
                           posture);
      return false;
    }

    // 协议要求这些累计值单调不减，这里做钳位，避免因乱序/重发被服务器判非法。
    last_allowance_total_ = std::max(last_allowance_total_, cmd.exchange_allowance_total);
    last_remote_allowance_ = std::max(last_remote_allowance_, cmd.remote_allowance_times);
    last_remote_hp_ = std::max(last_remote_hp_, cmd.remote_hp_times);

    std::uint32_t bits = 0;
    bits |= (cmd.confirm_respawn ? 1U : 0U) << 0;
    bits |= (cmd.confirm_buy_respawn ? 1U : 0U) << 1;
    bits |= (static_cast<std::uint32_t>(last_allowance_total_) & 0x7FFU) << 2;
    bits |= (static_cast<std::uint32_t>(last_remote_allowance_) & 0x0FU) << 13;
    bits |= (static_cast<std::uint32_t>(last_remote_hp_) & 0x0FU) << 17;
    bits |= (static_cast<std::uint32_t>(posture) & 0x07U) << 21;
    bits |= (cmd.activate_energy_mechanism ? 1U : 0U) << 24;

    // 0x0301 内容数据段 = data_cmd_id(2) + sender_id(2) + receiver_id(2) + 子内容
    std::uint8_t data[10];
    data[0] = static_cast<std::uint8_t>(kSubCmdSentryDecision & 0xFFU);
    data[1] = static_cast<std::uint8_t>((kSubCmdSentryDecision >> 8) & 0xFFU);
    data[2] = static_cast<std::uint8_t>(robot_id_ & 0xFFU);
    data[3] = static_cast<std::uint8_t>((robot_id_ >> 8) & 0xFFU);
    data[4] = static_cast<std::uint8_t>(kIdRefereeServer & 0xFFU);
    data[5] = static_cast<std::uint8_t>((kIdRefereeServer >> 8) & 0xFFU);
    std::memcpy(&data[6], &bits, sizeof(bits));

    // ref_tx 送的是内容数据段，组帧（SOF/长度/seq/CRC8/CRC16）由下位机完成。
    RefTxWire wire{};
    wire.cmd_id = kCmdSentryCmd0301;
    wire.len = static_cast<std::uint16_t>(sizeof(data));
    wire.bus = 0;
    std::memset(wire.rsv, 0, sizeof(wire.rsv));
    std::memcpy(wire.data, data, sizeof(data));
    send_topic("ref_tx", &wire, sizeof(wire));
    return true;
  }

  // ------------------------------------------------------------------ 状态
  struct RefereeState
  {
    bool has_game_status{false};
    std::uint8_t game_progress{0};
    std::uint8_t game_type{0};
    std::uint16_t stage_remain_time{0};
    bool has_robot_hp{false};
    std::uint16_t hp[HP_COUNT]{};
    std::int16_t damage_difference{0};
    bool has_field_event{false};
    std::uint32_t event_data{0};
    std::uint32_t event_data_2{0};
    bool has_robot_status{false};
    rm2027_interfaces::msg::RobotStatus robot_status;
    bool has_power_heat{false};
    rm2027_interfaces::msg::PowerHeat power_heat;
    bool has_robot_pos{false};
    rm2027_interfaces::msg::RobotPosition robot_pos;
    bool has_allowance{false};
    rm2027_interfaces::msg::ProjectileAllowance allowance;
    bool has_rfid{false};
    rm2027_interfaces::msg::RfidStatus rfid;
    bool has_ground_pos{false};
    rm2027_interfaces::msg::GroundRobotPosition ground_pos;
    bool has_sentry_info{false};
    rm2027_interfaces::msg::SentryInfo sentry_info;
  };

  // 参数
  std::string port_;
  int baud_{115200};
  int robot_id_param_{0};
  int robot_id_{0};
  bool publish_raw_{true};
  int zero_vel_timeout_ms_{200};
  int referee_timeout_ms_{2000};
  double summary_rate_hz_{20.0};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string move_mode_topic_{"/move_mode"};
  std::string spin_topic_{"/cmd_spin"};
  std::string fold_cmd_topic_{"/gimbal_fold_cmd"};
  std::uint8_t chassis_mode_{kModeIndependent};
  MoveVecWire last_move_{};
  double spin_wz_{0.0};
  std::uint32_t last_spin_ms_{0};
  double sentry_cmd_rate_hz_{5.0};
  double sentry_cmd_hold_s_{3.0};
  // 串口
  std::atomic<bool> running_{true};
  std::atomic<int> fd_{-1};
  std::mutex state_mtx_;
  std::map<std::uint16_t, std::uint32_t> command_rx_ms_;
  std::mutex write_mtx_;
  std::thread serial_thread_;
  std::uint64_t bad_frames_reported_{0};
  std::uint64_t bad_ref_frames_{0};
  std::uint8_t tx_seq_{0};

  // 话题
  std::vector<std::string> rx_names_;
  std::vector<std::string> tx_names_;
  std::map<std::uint32_t, std::string> rx_crc_;
  std::map<std::string, std::uint32_t> tx_crc_;

  // 发布者
  rclcpp::Publisher<rm2027_interfaces::msg::RefereeFrame>::SharedPtr raw_pub_;
  rclcpp::Publisher<rm2027_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<rm2027_interfaces::msg::GameResult>::SharedPtr game_result_pub_;
  rclcpp::Publisher<rm2027_interfaces::msg::RobotHP>::SharedPtr robot_hp_pub_;
  rclcpp::Publisher<rm2027_interfaces::msg::FieldEvent>::SharedPtr field_event_pub_;
  rclcpp::Publisher<rm2027_interfaces::msg::RefereeWarning>::SharedPtr warning_pub_;
  rclcpp::Publisher<rm2027_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<rm2027_interfaces::msg::PowerHeat>::SharedPtr power_heat_pub_;
  rclcpp::Publisher<rm2027_interfaces::msg::RobotPosition>::SharedPtr robot_pos_pub_;
  rclcpp::Publisher<rm2027_interfaces::msg::RobotBuff>::SharedPtr buff_pub_;
  rclcpp::Publisher<rm2027_interfaces::msg::RobotDamage>::SharedPtr damage_pub_;
  rclcpp::Publisher<rm2027_interfaces::msg::ProjectileAllowance>::SharedPtr allowance_pub_;
  rclcpp::Publisher<rm2027_interfaces::msg::RfidStatus>::SharedPtr rfid_pub_;
  rclcpp::Publisher<rm2027_interfaces::msg::GroundRobotPosition>::SharedPtr ground_pos_pub_;
  rclcpp::Publisher<rm2027_interfaces::msg::SentryInfo>::SharedPtr sentry_info_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr summary_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr operator_pub_;
  std::vector<std::uint8_t> last_operator_payload_;
  std::uint32_t operator_seq_{0};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fold_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<rm2027_interfaces::msg::ChassisState>::SharedPtr chassis_state_pub_;

  // 订阅者
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr spin_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr fold_cmd_sub_;
  rclcpp::Subscription<rm2027_interfaces::msg::SentryCommand>::SharedPtr sentry_cmd_sub_;

  rclcpp::TimerBase::SharedPtr summary_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // 运行时状态
  RefereeState st_;
  std::atomic<std::uint32_t> last_cmd_vel_ms_{0};
  bool zero_vel_sent_{false};
  QuatWire quat_{};
  bool have_quat_{false};
  float yawmotor_angle_{0.0F};
  bool have_yaw_{false};
  std::uint8_t posture_from_referee_{0};
  bool have_posture_from_referee_{false};
  std::uint8_t posture_from_chassis_{0};
  bool have_posture_from_chassis_{false};
  rm2027_interfaces::msg::SentryCommand pending_sentry_cmd_;
  bool have_pending_sentry_cmd_{false};
  std::uint32_t sentry_cmd_rx_ms_{0};
  std::uint32_t last_sentry_tx_ms_{0};
  std::uint16_t last_allowance_total_{0};
  std::uint8_t last_remote_allowance_{0};
  std::uint8_t last_remote_hp_{0};
};

}  // namespace ros2_libxr

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_libxr::BridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
