// BBT ROS 适配层实现：只做"抽象动作 ↔ 话题消息"的搬运，不做任何决策。
#include "bbt_decide/ros_ports.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

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

  // ---- 订阅：全部状态类话题，回调在锁内写缓存 ----
  nav_status_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      cfg_.nav_status_topic, 10, [this](const std_msgs::msg::Int32::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mtx_);
        note_rx_locked("nav_status");
        const int v = m->data;
        nav_state_ = (v >= 0 && v <= 3) ? static_cast<GoalState>(v) : kGoalIdle;
      });

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      cfg_.odom_topic, 10, [this](const nav_msgs::msg::Odometry::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mtx_);
        note_rx_locked("odom");
        pose_.x = m->pose.pose.position.x;
        pose_.y = m->pose.pose.position.y;
        const auto& q = m->pose.pose.orientation;
        pose_.yaw = yaw_from_quat(q.x, q.y, q.z, q.w);
        // 新鲜度用"本机收到时刻"（GEN-03 口径，与 referee 相同）：
        // sense() 用它判 odom 是否断流，断流后位姿按无效处理
        pose_.stamp_ms = static_cast<uint32_t>(now_s() * 1000.0);
        pose_.valid = true;
      });

  fold_state_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      cfg_.fold_state_topic, 10, [this](const std_msgs::msg::Int32::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mtx_);
        note_rx_locked("fold_state");
        fold_state_ = m->data;  // 0–4，编码同 FoldState
      });

  referee_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      cfg_.referee_topic, 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mtx_);
        note_rx_locked("referee");
        // 契约（见文件头）：至少 9 个元素，字段顺序与主工作区/Arena 桥一致
        if (m->data.size() < 9)
        {
          auto it = find_link_locked("referee");
          if (it != links_.end()) ++it->dropped;
          return;
        }
        ref_.game_progress = static_cast<int>(m->data[0]);
        ref_.hp = static_cast<int>(m->data[1]);
        ref_.hp_max = static_cast<int>(m->data[2]);
        ref_.shots = static_cast<int>(m->data[3]);
        ref_.coins = static_cast<int>(m->data[4]);
        ref_.stage_remain_s = m->data[8];
      });

  op_cmd_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
      cfg_.op_cmd_topic, 10, [this](const std_msgs::msg::Int32MultiArray::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mtx_);
        note_rx_locked("op_cmd");
        // 契约：[0]type [1]param [2]seq。seq 每条 +1（0 保留），type 1-4 之外丢弃。
        if (m->data.size() < 3)
        {
          auto it = find_link_locked("op_cmd");
          if (it != links_.end()) ++it->dropped;
          return;
        }
        const uint32_t seq = static_cast<uint32_t>(m->data[2]);
        const int type = m->data[0];
        if (seq == 0 || type < 1 || type > 4)
        {
          auto it = find_link_locked("op_cmd");
          if (it != links_.end()) ++it->dropped;
          return;
        }
        op_cmd_ = OpCmd{seq, type, m->data[1]};
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
  // 订阅链路
  register_link("nav_status", cfg_.nav_status_topic, "std_msgs/msg/Int32", false);
  register_link("odom", cfg_.odom_topic, "nav_msgs/msg/Odometry", false);
  register_link("fold_state", cfg_.fold_state_topic, "std_msgs/msg/Int32", false);
  register_link("referee", cfg_.referee_topic, "std_msgs/msg/Float64MultiArray", false);
  register_link("op_cmd", cfg_.op_cmd_topic, "std_msgs/msg/Int32MultiArray", false);

  RCLCPP_INFO(node_->get_logger(),
              "RosPorts 就绪: nav=[%s %s %s %s] fold=[%s %s] ref=[%s] op=[%s]",
              cfg_.nav_goal_topic.c_str(), cfg_.nav_cancel_topic.c_str(),
              cfg_.nav_status_topic.c_str(), cfg_.odom_topic.c_str(),
              cfg_.fold_cmd_topic.c_str(), cfg_.fold_state_topic.c_str(),
              cfg_.referee_topic.c_str(), cfg_.op_cmd_topic.c_str());
}

bool RosPorts::nav_set_goal(double x, double y)
{
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
  std_msgs::msg::Int32 msg;
  msg.data = mode;
  mode_pub_->publish(msg);
  note_tx("chassis_mode");
  return true;
}

void RosPorts::base_spin(double w)
{
  std_msgs::msg::Float32 msg;
  msg.data = static_cast<float>(w);
  spin_pub_->publish(msg);
  note_tx("spin");
}

bool RosPorts::fold_set(bool fold)
{
  std_msgs::msg::UInt8 msg;
  msg.data = fold ? 1 : 2;  // 1 折叠 / 2 展开
  fold_pub_->publish(msg);
  note_tx("fold_cmd");
  return true;
}

int RosPorts::fold_state()
{
  std::lock_guard<std::mutex> lock(mtx_);
  return fold_state_;
}

RefereeSnapshot RosPorts::referee()
{
  std::lock_guard<std::mutex> lock(mtx_);
  return ref_;
}

OpCmd RosPorts::op_cmd()
{
  std::lock_guard<std::mutex> lock(mtx_);
  return op_cmd_;
}

void RosPorts::buy_ammo(int rounds)
{
  std_msgs::msg::Int32 msg;
  msg.data = rounds;
  buy_ammo_pub_->publish(msg);
  note_tx("buy_ammo");
}

void RosPorts::buy_hp(int amount)
{
  std_msgs::msg::Int32 msg;
  msg.data = amount;
  buy_hp_pub_->publish(msg);
  note_tx("buy_hp");
}

void RosPorts::buy_revive()
{
  std_msgs::msg::Int32 msg;
  msg.data = 1;
  buy_revive_pub_->publish(msg);
  note_tx("buy_revive");
}

double RosPorts::now_s()
{
  // steady_clock：不受系统时间跳变 / 仿真时间影响
  const auto t = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(t).count();
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

void RosPorts::note_rx_locked(const std::string& name)
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
    ++it->count;
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
        note_rx_locked("data_source");
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
