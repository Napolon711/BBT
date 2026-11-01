// BBT ROS 适配层实现：只做"抽象动作 ↔ 话题消息"的搬运，不做任何决策。
#include "bbt_decide/ros_ports.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace bbt_decide
{
namespace
{
// 从四元数解算偏航角（避免引 tf2 依赖；odom 的姿态是标准 ZYX 旋转）。
double yaw_from_quat(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
}  // namespace

RosPorts::RosPorts(rclcpp::Node* node, const RosPortsConfig& cfg) : node_(node), cfg_(cfg)
{
  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(cfg_.nav_goal_topic, 10);
  cancel_pub_ = node_->create_publisher<std_msgs::msg::Empty>(cfg_.nav_cancel_topic, 10);
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cfg_.cmd_vel_topic, 10);
  mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(cfg_.chassis_mode_topic, 10);
  spin_pub_ = node_->create_publisher<std_msgs::msg::Float32>(cfg_.spin_topic, 10);
  fold_pub_ = node_->create_publisher<std_msgs::msg::UInt8>(cfg_.fold_cmd_topic, 10);
  buy_ammo_pub_ = node_->create_publisher<std_msgs::msg::Int32>(cfg_.buy_ammo_topic, 10);
  buy_hp_pub_ = node_->create_publisher<std_msgs::msg::Int32>(cfg_.buy_hp_topic, 10);
  buy_revive_pub_ = node_->create_publisher<std_msgs::msg::Int32>(cfg_.buy_revive_topic, 10);
  attack_pub_ =
      node_->create_publisher<std_msgs::msg::Int32MultiArray>(cfg_.attack_cmd_topic, 10);

  // ---- 订阅：全部状态类话题，回调在锁内写缓存 ----
  nav_status_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      cfg_.nav_status_topic, 10, [this](const std_msgs::msg::Int32::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mtx_);
        note_frame_locked("nav_status");
        const int v = m->data;
        nav_state_ = (v >= 0 && v <= 3) ? static_cast<GoalState>(v) : kGoalIdle;
      });

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      cfg_.odom_topic, 10, [this](const nav_msgs::msg::Odometry::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mtx_);
        const auto& pose = m->pose.pose;
        if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
            !std::isfinite(pose.orientation.x) || !std::isfinite(pose.orientation.y) ||
            !std::isfinite(pose.orientation.z) || !std::isfinite(pose.orientation.w)) return;
        note_frame_locked("odom");
        pose_.x = m->pose.pose.position.x;
        pose_.y = m->pose.pose.position.y;
        const auto& q = m->pose.pose.orientation;
        pose_.yaw = yaw_from_quat(q.x, q.y, q.z, q.w);
        // 新鲜度用"本机收到时刻"（GEN-03 口径，与 referee 相同）：
        // sense() 用它判 odom 是否断流，断流后位姿按无效处理
        pose_.stamp_ms = static_cast<uint64_t>(now_s() * 1000.0);
        pose_.valid = true;
      });

  fold_state_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      cfg_.fold_state_topic, 10, [this](const std_msgs::msg::Int32::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (m->data < kFoldUnknown || m->data > kFoldUnfolded) return;
        note_frame_locked("fold_state");
        fold_state_ = m->data;  // 0–4，编码同 FoldState
      });

  referee_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      cfg_.referee_topic, 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mtx_);
        const auto& v = m->data;
        // Arena 9/12, 27BT-UC 14, legacy BBT 19. Reject truncated layouts.
        bool numeric_valid = true;
        for (size_t i = 0; i < v.size(); ++i)
        {
          const bool timestamp = v.size() != 19 && i == 9;
          const double upper = timestamp ? double(UINT32_MAX) : double(std::numeric_limits<int>::max());
          numeric_valid &= std::isfinite(v[i]) && v[i] >= -1.0 && v[i] <= upper;
        }
        if ((v.size() != 9 && v.size() != 12 && v.size() != 14 && v.size() != 19) ||
            !numeric_valid)
        {
          auto it = find_link_locked("referee");
          if (it != links_.end()) ++it->dropped;
          return;
        }
        note_frame_locked("referee");
        ref_ = RefereeSnapshot{};
        ref_.game_progress = static_cast<int>(v[0]);
        ref_.hp = static_cast<int>(v[1]);
        ref_.hp_max = static_cast<int>(v[2]);
        ref_.shots = static_cast<int>(v[3]);
        ref_.coins = static_cast<int>(v[4]);
        ref_source_age_ms_ = 0.0;
        if (v.size() == 19)
        {
          ref_.our_outpost_hp = static_cast<int>(v[5]);
          ref_.our_outpost_ctrl = static_cast<int>(v[6]);
          ref_.enemy_outpost_hp = static_cast<int>(v[7]);
          ref_.enemy_outpost_ctrl = static_cast<int>(v[8]);
          ref_.stage_remain_s = v[9];
          ref_.our_base_hp = static_cast<int>(v[10]);
          ref_.enemy_base_hp = static_cast<int>(v[11]);
          ref_.ally_ctrl_deg = static_cast<int>(v[12]);
          ref_.enemy_ctrl_deg = static_cast<int>(v[13]);
          ref_.robot_id = static_cast<int>(v[14]);
          ref_.revive_available = static_cast<int>(v[15]);
          ref_.revive_cost = static_cast<int>(v[16]);
          ref_.remote_ammo_pool = static_cast<int>(v[17]);
          ref_.out_of_combat = static_cast<int>(v[18]);
        }
        else
        {
          ref_.stage_remain_s = v[8];
          if (v.size() >= 12)
          {
            ref_.our_outpost_hp = static_cast<int>(v[5]);
            ref_.our_outpost_ctrl = v[7] == 1.0 ? static_cast<int>(v[6]) : kCtrlUnknown;
            ref_.our_base_hp = static_cast<int>(v[10]);
            ref_.ally_ctrl_deg = static_cast<int>(v[11]);
          }
          if (v.size() == 14)
          {
            ref_source_age_ms_ = std::max(0.0, v[12]);
            ref_.enemy_outpost_hp = static_cast<int>(v[13]);
          }
        }
        if (sim_source_seen_) ref_.game_progress = 0;
      });

  // 操作手指令两路订阅（按配置二选一，空话题名 = 不建）：
  //   Int32 版 [type, param, seq]（Arena 仿真链路）
  //   Float64 版 [type, param, seq, x, y]（实车桥：裁判 0x0303 小地图点击坐标）
  // 两路独立去重，再按本地接收顺序生成统一序号。
  op_cmd_sub_ = cfg_.op_cmd_topic.empty()
                    ? nullptr
                    : node_->create_subscription<std_msgs::msg::Int32MultiArray>(
                          cfg_.op_cmd_topic, 10,
                          [this](const std_msgs::msg::Int32MultiArray::SharedPtr m) {
                            std::lock_guard<std::mutex> lock(mtx_);
                            note_frame_locked("op_cmd");
                            // 契约：[0]type [1]param [2]seq。seq 每条 +1（0 保留），
                            // type 1-5 之外丢弃。
                            if (m->data.size() < 3)
                            {
                              auto it = find_link_locked("op_cmd");
                              if (it != links_.end()) ++it->dropped;
                              return;
                            }
                            const uint32_t seq = static_cast<uint32_t>(m->data[2]);
                            const int type = m->data[0];
                            if (seq == 0 || type < 1 || type > 5)
                            {
                              auto it = find_link_locked("op_cmd");
                              if (it != links_.end()) ++it->dropped;
                              return;
                            }
                            if (op_cmd_int_.seq == seq) return;
                            op_cmd_int_ = OpCmd{seq, type, m->data[1], 0.0, 0.0};
                            op_latest_ = op_cmd_int_;
                            if (++op_local_seq_ == 0) ++op_local_seq_;
                            op_latest_.seq = op_local_seq_;
                          });
  op_cmd_xy_sub_ = cfg_.op_cmd_xy_topic.empty()
                       ? nullptr
                       : node_->create_subscription<std_msgs::msg::Float64MultiArray>(
                             cfg_.op_cmd_xy_topic, 10,
                             [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
                               std::lock_guard<std::mutex> lock(mtx_);
                               note_frame_locked("op_cmd_xy");
                               // 契约：[0]type [1]param [2]seq [3]x [4]y（场地系坐标）。
                               if (m->data.size() < 5)
                               {
                                 auto it = find_link_locked("op_cmd_xy");
                                 if (it != links_.end()) ++it->dropped;
                                 return;
                               }
                               if (!std::all_of(m->data.begin(), m->data.end(), [](double x) {
                                 return std::isfinite(x);
                               }) || m->data[2] < 1 || m->data[2] > UINT32_MAX ||
                                   m->data[0] < 1 || m->data[0] > 5 ||
                                   m->data[1] < -1 || m->data[1] > std::numeric_limits<int>::max() ||
                                   std::trunc(m->data[0]) != m->data[0] ||
                                   std::trunc(m->data[1]) != m->data[1] ||
                                   std::trunc(m->data[2]) != m->data[2])
                               {
                                 auto it = find_link_locked("op_cmd_xy");
                                 if (it != links_.end()) ++it->dropped;
                                 return;
                               }
                               const uint32_t seq = static_cast<uint32_t>(m->data[2]);
                               const int type = static_cast<int>(m->data[0]);
                               if (op_cmd_xy_.seq == seq) return;
                               op_cmd_xy_ = OpCmd{seq, type, static_cast<int>(m->data[1]),
                                                  m->data[3], m->data[4]};
                               op_latest_ = op_cmd_xy_;
                               if (++op_local_seq_ == 0) ++op_local_seq_;
                               op_latest_.seq = op_local_seq_;
                             });

  // 发布链路注册（统计用）
  register_link("nav_goal", cfg_.nav_goal_topic, "geometry_msgs/msg/PoseStamped", true);
  register_link("nav_cancel", cfg_.nav_cancel_topic, "std_msgs/msg/Empty", true);
  register_link("cmd_vel", cfg_.cmd_vel_topic, "geometry_msgs/msg/Twist", true);
  register_link("chassis_mode", cfg_.chassis_mode_topic, "std_msgs/msg/Int32", true);
  register_link("spin", cfg_.spin_topic, "std_msgs/msg/Float32", true);
  register_link("fold_cmd", cfg_.fold_cmd_topic, "std_msgs/msg/UInt8", true);
  register_link("buy_ammo", cfg_.buy_ammo_topic, "std_msgs/msg/Int32", true);
  register_link("buy_hp", cfg_.buy_hp_topic, "std_msgs/msg/Int32", true);
  register_link("buy_revive", cfg_.buy_revive_topic, "std_msgs/msg/Int32", true);
  register_link("attack_cmd", cfg_.attack_cmd_topic, "std_msgs/msg/Int32MultiArray", true);
  // 订阅链路
  register_link("nav_status", cfg_.nav_status_topic, "std_msgs/msg/Int32", false);
  register_link("odom", cfg_.odom_topic, "nav_msgs/msg/Odometry", false);
  register_link("fold_state", cfg_.fold_state_topic, "std_msgs/msg/Int32", false);
  register_link("referee", cfg_.referee_topic, "std_msgs/msg/Float64MultiArray", false);
  if (!cfg_.op_cmd_topic.empty())
    register_link("op_cmd", cfg_.op_cmd_topic, "std_msgs/msg/Int32MultiArray", false);
  if (!cfg_.op_cmd_xy_topic.empty())
    register_link("op_cmd_xy", cfg_.op_cmd_xy_topic, "std_msgs/msg/Float64MultiArray", false);

  if (cfg_.referee_2027)
  {
    purchase_pub_ = node_->create_publisher<rm2027_interfaces::msg::SentryCommand>(cfg_.purchase_topic, 10);
    register_link("purchase", cfg_.purchase_topic, "rm2027_interfaces/msg/SentryCommand", true);
    register_link("robot_status", cfg_.robot_status_topic, "rm2027_interfaces/msg/RobotStatus", false);
    robot_status_sub_ = node_->create_subscription<rm2027_interfaces::msg::RobotStatus>(
        cfg_.robot_status_topic, 10, [this](rm2027_interfaces::msg::RobotStatus::SharedPtr m) {
          std::lock_guard<std::mutex> lock(mtx_);
          robot_status_ = *m;
          note_frame_locked("robot_status");
        });
    register_link("robot_hp", cfg_.robot_hp_topic, "rm2027_interfaces/msg/RobotHP", false);
    robot_hp_sub_ = node_->create_subscription<rm2027_interfaces::msg::RobotHP>(
        cfg_.robot_hp_topic, 10, [this](rm2027_interfaces::msg::RobotHP::SharedPtr m) {
          std::lock_guard<std::mutex> lock(mtx_);
          robot_hp_ = *m;
          note_frame_locked("robot_hp");
        });
    register_link("field_event", cfg_.field_event_topic, "rm2027_interfaces/msg/FieldEvent", false);
    field_event_sub_ = node_->create_subscription<rm2027_interfaces::msg::FieldEvent>(
        cfg_.field_event_topic, 10, [this](rm2027_interfaces::msg::FieldEvent::SharedPtr m) {
          std::lock_guard<std::mutex> lock(mtx_);
          field_event_ = *m;
          note_frame_locked("field_event");
        });
    register_link("sentry_info", cfg_.sentry_info_topic, "rm2027_interfaces/msg/SentryInfo", false);
    sentry_info_sub_ = node_->create_subscription<rm2027_interfaces::msg::SentryInfo>(
        cfg_.sentry_info_topic, 10, [this](rm2027_interfaces::msg::SentryInfo::SharedPtr m) {
          std::lock_guard<std::mutex> lock(mtx_);
          sentry_info_ = *m;
          note_frame_locked("sentry_info");
        });
    purchase_timer_ = node_->create_wall_timer(std::chrono::milliseconds(200), [this]() { service_purchase(); });
  }

  RCLCPP_INFO(node_->get_logger(),
              "RosPorts 就绪: nav=[%s %s %s %s] fold=[%s %s] ref=[%s] op=[%s]",
              cfg_.nav_goal_topic.c_str(), cfg_.nav_cancel_topic.c_str(),
              cfg_.nav_status_topic.c_str(), cfg_.odom_topic.c_str(),
              cfg_.fold_cmd_topic.c_str(), cfg_.fold_state_topic.c_str(),
              cfg_.referee_topic.c_str(), cfg_.op_cmd_topic.c_str());
}

bool RosPorts::nav_set_goal(double x, double y)
{
  if (!std::isfinite(x) || !std::isfinite(y)) return false;
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "map";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.orientation.w = 1.0;  // 朝向不给，由导航自己决定
  goal_pub_->publish(msg);
  note_tx("nav_goal");
  std::lock_guard<std::mutex> lock(mtx_);
  nav_state_ = kGoalRunning;  // 乐观置位，之后以 /nav_status 为准
  return true;
}

void RosPorts::nav_cancel()
{
  cancel_pub_->publish(std_msgs::msg::Empty());
  note_tx("nav_cancel");
  std::lock_guard<std::mutex> lock(mtx_);
  nav_state_ = kGoalIdle;
}

GoalState RosPorts::nav_state()
{
  std::lock_guard<std::mutex> lock(mtx_);
  return nav_state_;
}

BasePose RosPorts::base_pose()
{
  std::lock_guard<std::mutex> lock(mtx_);
  return pose_;
}

void RosPorts::base_stop()
{
  geometry_msgs::msg::Twist msg;  // 全零 = 停住
  vel_pub_->publish(msg);
  note_tx("cmd_vel");
}

bool RosPorts::base_set_mode(int mode)
{
  if (mode < 0 || mode > 3) return false;
  std_msgs::msg::Int32 msg;
  msg.data = mode;
  mode_pub_->publish(msg);
  chassis_mode_ = mode;
  note_tx("chassis_mode");
  return true;
}

void RosPorts::base_spin(double w)
{
  if (!std::isfinite(w)) return;
  // Reference bridge consumes /cmd_spin only in ROTOR mode.
  if (cfg_.referee_2027)
  {
    const int desired = std::abs(w) > 1e-6 ? 2 : cfg_.mode_independent;
    if (desired != chassis_mode_) base_set_mode(desired);
  }
  std_msgs::msg::Float32 msg;
  msg.data = static_cast<float>(w);
  spin_pub_->publish(msg);
  note_tx("spin");
}

bool RosPorts::fold_set(bool fold)
{
  std_msgs::msg::UInt8 msg;
  msg.data = fold ? 1 : 2;  // 1 折叠 / 2 展开
  {
    std::lock_guard<std::mutex> lock(mtx_);
    fold_state_ = kFoldUnknown; // A new command requires subsequent feedback.
  }
  fold_pub_->publish(msg);
  note_tx("fold_cmd");
  return true;
}

int RosPorts::fold_state()
{
  std::lock_guard<std::mutex> lock(mtx_);
  return input_age_ms_locked("fold_state") <= cfg_.fold_stale_ms ? fold_state_ : kFoldUnknown;
}

RefereeSnapshot RosPorts::referee()
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto r = ref_;
  if (cfg_.referee_2027)
  {
    const auto fresh = [this](const char* key) { return input_age_ms_locked(key) <= cfg_.referee_stale_ms; };
    r.robot_id = fresh("robot_status") ? robot_status_.robot_id : 0;
    const bool color_known = r.robot_id == 7 || r.robot_id == 107;
    const bool red = r.robot_id == 7;
    if (!color_known)
    {
      r.our_outpost_hp = r.enemy_outpost_hp = -1;
      r.our_outpost_ctrl = r.enemy_outpost_ctrl = kCtrlUnknown;
    }
    if (color_known && fresh("robot_hp"))
    {
      r.our_outpost_hp = red ? robot_hp_.outpost_r_hp : robot_hp_.outpost_b_hp;
      r.enemy_outpost_hp = red ? robot_hp_.outpost_b_hp : robot_hp_.outpost_r_hp;
      r.our_base_hp = robot_hp_.ally_base_hp;
      r.enemy_base_hp = robot_hp_.enemy_base_hp;
      r.ally_ctrl_deg = robot_hp_.ally_control_degree;
      r.enemy_ctrl_deg = robot_hp_.enemy_control_degree;
    }
    if (color_known && fresh("field_event"))
    {
      r.our_outpost_ctrl = red ? field_event_.outpost_r_control : field_event_.outpost_b_control;
      r.enemy_outpost_ctrl = red ? field_event_.outpost_b_control : field_event_.outpost_r_control;
    }
    if (fresh("sentry_info"))
    {
      r.revive_available = sentry_info_.can_free_respawn ? 1 : (sentry_info_.can_buy_respawn ? 2 : 0);
      r.revive_cost = sentry_info_.respawn_cost_coin;
      r.remote_ammo_pool = sentry_info_.team_allowance_exchangeable;
      r.out_of_combat = sentry_info_.out_of_combat ? 1 : 0;
    }
  }
  return r;
}

OpCmd RosPorts::op_cmd()
{
  std::lock_guard<std::mutex> lock(mtx_);
  // 各通道独立去重，以本机收到新指令的顺序合并，避免低序号通道永久饿死。
  return op_latest_;
}

void RosPorts::buy_ammo(int rounds)
{
  if (cfg_.referee_2027 && !request_purchase(1, rounds)) return;
  std_msgs::msg::Int32 msg;
  msg.data = rounds;
  buy_ammo_pub_->publish(msg);
  note_tx("buy_ammo");
}

void RosPorts::buy_hp(int amount)
{
  if (cfg_.referee_2027 && !request_purchase(2, amount)) return;
  std_msgs::msg::Int32 msg;
  msg.data = amount;
  buy_hp_pub_->publish(msg);
  note_tx("buy_hp");
}

void RosPorts::buy_revive()
{
  if (cfg_.referee_2027 && !request_purchase(3, 1)) return;
  std_msgs::msg::Int32 msg;
  msg.data = 1;
  buy_revive_pub_->publish(msg);
  note_tx("buy_revive");
}

void RosPorts::attack_cmd(int target_type, int target_id, int fire, int free_fire)
{
  std_msgs::msg::Int32MultiArray msg;
  msg.data = {target_type, target_id, fire, free_fire};
  attack_pub_->publish(msg);
  note_tx("attack_cmd");
}

double RosPorts::now_s()
{
  // steady_clock：不受系统时间跳变 / 仿真时间影响
  const auto t = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(t).count();
}

double RosPorts::input_age_ms_locked(const std::string& name)
{
  const auto it = find_link_locked(name);
  if (it == links_.end()) return std::numeric_limits<double>::infinity();
  const double last = link_last_s_[it - links_.begin()];
  return last < 0.0 ? std::numeric_limits<double>::infinity() : (now_s() - last) * 1000.0;
}

bool RosPorts::request_purchase(int kind, int amount)
{
  std::unique_lock<std::mutex> lock(mtx_);
  const double now = now_s();
  if (purchase_kind_ != 0 || now < purchase_ready_s_ || sim_source_seen_ ||
      input_age_ms_locked("referee") + ref_source_age_ms_ > cfg_.referee_stale_ms ||
      input_age_ms_locked("sentry_info") > cfg_.referee_stale_ms ||
      input_age_ms_locked("robot_status") > cfg_.referee_stale_ms ||
      (robot_status_.robot_id != 7 && robot_status_.robot_id != 107) ||
      ref_.game_progress != 4) return false;
  if (kind == 1 || kind == 2)
  {
    if (ref_.hp <= 0 || !sentry_info_.out_of_combat ||
        (kind == 1 ? amount != 100 : amount != 1)) return false;
    if (kind == 2 && ref_.stage_remain_s < 0.0) return false;
    const int cost = kind == 1 ? 150 :
        50 + static_cast<int>(std::ceil(std::clamp(420.0 - ref_.stage_remain_s, 0.0, 420.0) / 3.0));
    const int count = kind == 1 ? sentry_info_.remote_allowance_exchange_times : sentry_info_.remote_hp_exchange_times;
    if (count >= 15 || ref_.coins < cost ||
        (kind == 1 && sentry_info_.team_allowance_exchangeable < 100)) return false;
  }
  else if (kind == 3)
  {
    if (ref_.hp != 0 || (!sentry_info_.can_free_respawn &&
        (!sentry_info_.can_buy_respawn || ref_.coins < sentry_info_.respawn_cost_coin))) return false;
  }
  else return false;

  purchase_command_ = rm2027_interfaces::msg::SentryCommand{};
  purchase_command_.exchange_allowance_total = sentry_info_.exchanged_projectile_allowance;
  purchase_command_.remote_allowance_times = sentry_info_.remote_allowance_exchange_times + (kind == 1 ? 1 : 0);
  purchase_command_.remote_hp_times = sentry_info_.remote_hp_exchange_times + (kind == 2 ? 1 : 0);
  purchase_command_.confirm_respawn = kind == 3;
  purchase_command_.confirm_buy_respawn = kind == 3 && !sentry_info_.can_free_respawn;
  purchase_kind_ = kind;
  purchase_started_s_ = now;
  purchase_ready_s_ = now + 6.0;
  const auto command = purchase_command_;
  lock.unlock();
  purchase_pub_->publish(command);
  note_tx("purchase");
  return true;
}

void RosPorts::service_purchase()
{
  std::unique_lock<std::mutex> lock(mtx_);
  if (purchase_kind_ == 0) return;
  const double now = now_s();
  const bool fresh = input_age_ms_locked("referee") + ref_source_age_ms_ <= cfg_.referee_stale_ms;
  if (sim_source_seen_ || (fresh && ref_.game_progress != 4) ||
      (fresh && purchase_kind_ == 3 && ref_.hp > 0))
  {
    // Explicitly withdraw resurrection flags from the bridge's hold window.
    purchase_command_.confirm_respawn = false;
    purchase_command_.confirm_buy_respawn = false;
    const auto command = purchase_command_;
    purchase_kind_ = 0;
    purchase_ready_s_ = now;
    lock.unlock();
    purchase_pub_->publish(command);
    note_tx("purchase");
    return;
  }
  if (input_age_ms_locked("sentry_info") <= cfg_.referee_stale_ms &&
      ((purchase_kind_ == 1 && sentry_info_.remote_allowance_exchange_times >= purchase_command_.remote_allowance_times) ||
       (purchase_kind_ == 2 && sentry_info_.remote_hp_exchange_times >= purchase_command_.remote_hp_times)))
  {
    purchase_kind_ = 0;
    purchase_ready_s_ = now + 6.0; // Counter acknowledgment precedes actual delivery.
    return;
  }
  if (fresh && !sim_source_seen_ && now - purchase_started_s_ <= 12.0 &&
      input_age_ms_locked("sentry_info") <= cfg_.referee_stale_ms &&
      ((purchase_kind_ == 3 && ref_.hp == 0) || (purchase_kind_ != 3 && ref_.hp > 0)))
  {
    const auto command = purchase_command_;
    lock.unlock();
    purchase_pub_->publish(command); // Retransmit the same counter, never skip it.
    note_tx("purchase");
  }
}

void RosPorts::register_link(const std::string& name, const std::string& topic,
                             const std::string& msg_type, bool outbound)
{
  LinkStat s;
  s.name = name;
  s.topic = topic;
  s.msg_type = msg_type;
  s.outbound = outbound;
  links_.push_back(s);
  link_last_s_.push_back(-1.0);
  link_win_.push_back(0);
  if (links_.size() == 1) link_win_start_s_ = now_s();
}

void RosPorts::note_frame_locked(const std::string& name)
{
  auto it = find_link_locked(name);
  if (it == links_.end()) return;
  const double now = now_s();
  ++it->count;
  ++link_win_[it - links_.begin()];
  link_last_s_[it - links_.begin()] = now;
}

void RosPorts::note_tx(const std::string& name, bool dropped)
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = find_link_locked(name);
  if (it == links_.end()) return;
  const double now = now_s();
  if (dropped)
    ++it->dropped;
  else
  {
    ++it->count;
    ++link_win_[it - links_.begin()];
  }
  link_last_s_[it - links_.begin()] = now;
}

std::vector<LinkStat>::iterator RosPorts::find_link_locked(const std::string& name)
{
  for (auto it = links_.begin(); it != links_.end(); ++it)
  {
    if (it->name == name) return it;
  }
  return links_.end();
}

std::vector<LinkStat> RosPorts::iface_report()
{
  std::lock_guard<std::mutex> lock(mtx_);
  const double now = now_s();
  const double win = std::max(1e-6, now - link_win_start_s_);
  for (size_t i = 0; i < links_.size(); ++i)
  {
    links_[i].rate_hz = static_cast<double>(link_win_[i]) / win;
    links_[i].last_age_s =
        link_last_s_[i] < 0.0 ? -1.0 : std::max(0.0, now - link_last_s_[i]);
    link_win_[i] = 0;
  }
  link_win_start_s_ = now;
  return links_;
}

void RosPorts::enable_data_source_guard()
{
  if (guard_enabled_) return;
  guard_enabled_ = true;
  source_sub_ = node_->create_subscription<std_msgs::msg::String>(
      cfg_.data_source_topic, 10, [this](const std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mtx_);
        note_frame_locked("data_source");
        if (m->data == "sim")
        {
          if (!sim_source_seen_)
          {
            RCLCPP_ERROR(node_->get_logger(),
                         "实车模式听到仿真数据源声明（%s = \"sim\"）：比赛状态作废，决策待机。"
                         "上实车前请把节点参数 data_source 改回 real 或停掉仿真桥。",
                         cfg_.data_source_topic.c_str());
          }
          sim_source_seen_ = true;
          ref_.game_progress = 0;  // 持续压住：只要 sim 还在说话，比赛状态恒为"未开始"
        }
      });
  register_link("data_source", cfg_.data_source_topic, "std_msgs/msg/String", false);
}

bool RosPorts::sim_source_seen()
{
  std::lock_guard<std::mutex> lock(mtx_);
  return sim_source_seen_;
}

}  // namespace bbt_decide
