// BBT ROS 适配层：真实话题是唯一输入/输出通道。
// 实车：27BT-UC 的 14 元素 /referee/summary + RobotStatus/RobotHP/FieldEvent/SentryInfo。
// 兼容 Arena 的 9/12 元素摘要及旧 BBT 的完整 19 元素摘要；其他长度丢弃。
// 购买执行口为 /referee/sentry_command；/sentry/buy_* 只作意图观测，不可二次执行。
// /referee/operator_cmd：Float64 [type,param,seq,x,y]；仿真 ID 指点另用 /sentry/operator_point。
// 折叠状态需持续刷新；新指令清除旧完成值，反馈超过 fold_stale_ms 按未知处理。
// 底盘速度、模式、折叠、攻击意图及导航话题详见 README.md 和 docs/电控接口文档.md。
#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <rm2027_interfaces/msg/field_event.hpp>
#include <rm2027_interfaces/msg/robot_hp.hpp>
#include <rm2027_interfaces/msg/robot_status.hpp>
#include <rm2027_interfaces/msg/sentry_info.hpp>
#include <rm2027_interfaces/msg/sentry_command.hpp>

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "bbt_decide/ports.hpp"

namespace bbt_decide
{

// 适配层配置：话题名 + 底盘模式枚举。构造 RosPorts 时值拷贝一份。
struct RosPortsConfig
{
  bool referee_2027 = false;
  double fold_stale_ms = 1000.0;
  double referee_stale_ms = 2000.0;
  std::string robot_status_topic = "/referee/robot_status";
  std::string robot_hp_topic = "/referee/robot_hp";
  std::string field_event_topic = "/referee/field_event";
  std::string sentry_info_topic = "/referee/sentry_info";
  std::string purchase_topic = "/referee/sentry_command";
  std::string nav_goal_topic = "/goal_pose";
  std::string nav_status_topic = "/nav_status";
  std::string nav_cancel_topic = "/nav_cancel";
  std::string odom_topic = "/odom";
  std::string cmd_vel_topic = "/cmd_vel";
  std::string chassis_mode_topic = "/move_mode";
  std::string spin_topic = "/cmd_spin";
  std::string fold_cmd_topic = "/gimbal_fold_cmd";
  std::string fold_state_topic = "/gimbal_fold_state";
  std::string referee_topic = "/referee/summary";
  std::string op_cmd_topic = "/sentry/operator_point";    // Int32 ID 指点；可空
  std::string op_cmd_xy_topic = "/referee/operator_cmd";  // Float64 坐标指点；可空
  std::string buy_ammo_topic = "/sentry/buy_ammo";
  std::string buy_hp_topic = "/sentry/buy_hp";
  std::string buy_revive_topic = "/sentry/buy_revive";
  std::string attack_cmd_topic = "/sentry/attack_cmd";
  std::string data_source_topic = "/sentry/data_source";
  int mode_independent = 1;  // 底盘模式枚举里的 INDEPENDENT（比赛开始沿切一次）
};

// 单条接口链路的运行统计：帧数/频率/年龄/丢弃数。
// 由 RosPorts 维护，节点每秒发到 iface_status 话题——"数据确实从真实接口进来了"就看它。
struct LinkStat
{
  std::string name;       // Ports 侧语义名（如 "referee"）
  std::string topic;      // 实际话题名
  std::string msg_type;   // 消息类型
  bool outbound = false;  // true = 决策层发布，false = 订阅
  uint64_t count = 0;     // 累计帧数
  double last_age_s = -1.0;  // 距最近一帧秒数；-1 = 一帧都没收到过
  double rate_hz = 0.0;      // 最近统计窗口的实测频率
  uint64_t dropped = 0;      // 到达但因内容非法被丢弃的帧数
};

// Ports 的 ROS 2 实现。线程模型：订阅回调在 ROS 执行器线程加锁写缓存，
// 决策线程通过 getter 加锁读快照——所有 getter 不阻塞、不等话题。
// 下发类接口"发出去即返回"，真实是否生效看对应状态话题（nav_status / fold_state）。
class RosPorts : public Ports
{
public:
  // 构造：按配置建好全部 publisher / subscription。
  // 参数：node — 非拥有的节点指针（只用来建 pub/sub、取时钟和 logger，由调用方保活）；
  //       cfg — 话题名等，值拷贝进成员。
  // 副作用：按配置创建话题及购买重发定时器。
  RosPorts(rclcpp::Node* node, const RosPortsConfig& cfg);

  // ---- 哨兵导航 ----
  // 发布目标点（PoseStamped，orientation.w=1.0），并乐观把 nav_state_ 置 running，
  // 之后以 /nav_status 回调为准。非法数值拒绝；发出不代表导航已接受，无 Goal UUID 回执。
  bool nav_set_goal(double x, double y) override;
  // 发布 /nav_cancel 并立刻把 nav_state_ 置 idle（不等导航侧确认）。
  void nav_cancel() override;
  // 读缓存的导航状态（来自 /nav_status）。
  GoalState nav_state() override;
  // 读最近一帧 /odom 的位姿快照；valid = 收到过至少一帧。
  BasePose base_pose() override;

  // ---- 电控 ----
  // 发 Twist(0,0,0) 停车（决策层不下发巡航速度，速度闭环归导航）。
  void base_stop() override;
  // 发 /move_mode。返回恒 true（占位契约无 ack）。
  bool base_set_mode(int mode) override;
  // 发 /cmd_spin（Float32，rad/s）。
  void base_spin(double w) override;
  // 发 /gimbal_fold_cmd（UInt8：true → 1 折叠 / false → 2 展开）。返回恒 true。
  bool fold_set(bool fold) override;
  // 读新鲜的折叠反馈；未知/过期返回 kFoldUnknown。
  int fold_state() override;

  // ---- 裁判系统 ----
  // 读最近一帧 /referee/summary 解析出的快照。
  RefereeSnapshot referee() override;
  // 读最近一条 /referee/operator_cmd 操作手指令；seq == 0 = 没有。
  OpCmd op_cmd() override;

  // ---- 自动购买下单（决策层自己决定何时买，见 planner 的 run_auto_buy）----
  // 实车下发 SentryCommand 并发布买弹意图；到账从
  // /referee/summary 里看（血量/弹量涨了就是到账）。
  void buy_ammo(int rounds) override;
  // 远程回血一次（amount=1），恢复量由裁判决定。
  void buy_hp(int amount) override;
  // 发 /sentry/buy_revive（Int32 1）。
  void buy_revive() override;
  // 发 /sentry/attack_cmd（Int32MultiArray(4) 自瞄意图提示；电控可忽略）。
  void attack_cmd(int target_type, int target_id, int fire, int free_fire) override;

  // ---- 时间 ----
  // steady_clock 单调秒（不走 ROS 时钟，避免被仿真时间影响）。
  double now_s() override;

  // ---- 接口统计 ----
  // 取全部链路的数据流统计快照（顺带结算本窗口频率），固定 1 Hz 调一次。
  std::vector<LinkStat> iface_report();

  // ---- 数据源硬保护 ----
  // 实车模式开启：订阅 data_source_topic，一旦听到发送方自称 "sim"，
  // 持续把裁判快照的比赛状态压成 0（未开始）——门控失败，决策树落到待机停车。
  void enable_data_source_guard();
  // 是否听到过自称 "sim" 的发送方。
  bool sim_source_seen();
  void service_purchase();

private:
  // 注册一条链路（记进 links_）。
  void register_link(const std::string& name, const std::string& topic,
                     const std::string& msg_type, bool outbound);
  // 记一次收/发帧。调用方必须已持有 mtx_。
  void note_frame_locked(const std::string& name);
  // 记一次"发出一帧 / 丢弃一帧"。内部自己加锁。
  void note_tx(const std::string& name, bool dropped = false);
  // 按语义名找链路（须持锁），找不到返回 end()。
  std::vector<LinkStat>::iterator find_link_locked(const std::string& name);
  double input_age_ms_locked(const std::string& name);
  bool request_purchase(int kind, int amount);

  rclcpp::Node* node_;  // 非拥有节点指针
  RosPortsConfig cfg_;

  // 发布器（构造时建好）
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr spin_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr fold_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr buy_ammo_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr buy_hp_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr buy_revive_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr attack_pub_;
  rclcpp::SubscriptionBase::SharedPtr nav_status_sub_;
  rclcpp::SubscriptionBase::SharedPtr odom_sub_;
  rclcpp::SubscriptionBase::SharedPtr fold_state_sub_;
  rclcpp::SubscriptionBase::SharedPtr referee_sub_;
  rclcpp::SubscriptionBase::SharedPtr op_cmd_sub_;     // 操作手指令 Int32 版（可空 = 不建）
  rclcpp::SubscriptionBase::SharedPtr op_cmd_xy_sub_;  // 操作手指令 Float64 坐标版（可空）
  rclcpp::SubscriptionBase::SharedPtr source_sub_;  // data_source（守卫开启后才有）

  // 状态缓存（读写都要 mtx_）
  rclcpp::SubscriptionBase::SharedPtr robot_status_sub_, robot_hp_sub_, field_event_sub_, sentry_info_sub_;
  rclcpp::Publisher<rm2027_interfaces::msg::SentryCommand>::SharedPtr purchase_pub_;
  rclcpp::TimerBase::SharedPtr purchase_timer_;
  rm2027_interfaces::msg::RobotStatus robot_status_;
  rm2027_interfaces::msg::RobotHP robot_hp_;
  rm2027_interfaces::msg::FieldEvent field_event_;
  rm2027_interfaces::msg::SentryInfo sentry_info_;
  double ref_source_age_ms_ = 0.0;
  rm2027_interfaces::msg::SentryCommand purchase_command_;
  int purchase_kind_ = 0; // 0 无 / 1 买弹 / 2 回血 / 3 复活；未确认请求不跳号
  double purchase_started_s_ = 0.0;
  double purchase_ready_s_ = 0.0;
  int chassis_mode_ = -1;
  mutable std::mutex mtx_;
  GoalState nav_state_ = kGoalIdle;  // 导航状态缓存
  BasePose pose_{};                  // 位姿缓存
  int fold_state_ = kFoldUnknown;    // 折叠状态缓存
  RefereeSnapshot ref_{};            // 裁判快照缓存；sim 守卫触发时 game_progress 压 0
  OpCmd op_latest_{};
  uint32_t op_local_seq_ = 0;
  OpCmd op_cmd_int_{};               // Int32 版操作手指令缓存
  OpCmd op_cmd_xy_{};                // Float64 坐标版操作手指令缓存
  bool guard_enabled_ = false;       // 实车守卫是否开启
  bool sim_source_seen_ = false;     // 是否听到过 "sim" 声明

  // 接口统计
  std::vector<LinkStat> links_;
  std::vector<double> link_last_s_;   // 每条链路最近一帧的单调秒
  std::vector<uint64_t> link_win_;    // 本窗口帧数
  double link_win_start_s_ = 0.0;     // 窗口起点
};

}  // namespace bbt_decide
