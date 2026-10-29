// BBT ROS 适配层：把抽象 Ports 接到具体话题。
//
// 这是本包**唯一**的数据进出口（约定沿用主工作区 GEN-03/GEN-05）：
//   1) 所有输入都从真实 ROS 话题进来，没有任何进程内注水通道；
//      仿真（Arena）数据由 tools/arena_ros_bridge.py 按下面同一套话题名发进来，
//      所以"仿真里跑"和"实车上跑"是同一套消息契约、同一条数据通路。
//   2) 未知就是未知：没收到过的量返回 -1 / valid=false，绝不用 0 顶替。
//
// 话题契约（与主工作区临时契约一致，Arena 桥零改动可对接）：
//   /goal_pose         PoseStamped          [发] 导航目标点，map 系，单位 m（朝向由导航定）
//   /nav_cancel        Empty                [发] 取消当前导航目标
//   /nav_status        Int32                [收] 0 idle / 1 running / 2 success / 3 failure
//   /odom              Odometry             [收] 位姿（x/y m，yaw rad，四元数解算）
//   /cmd_vel           Twist                [发] 底盘速度；本包只发 0 停车，不发巡航速度
//   /move_mode         Int32                [发] 底盘模式 0 RELAX/1 INDEPENDENT/2 ROTOR/3 FOLLOW
//   /cmd_spin          Float32              [发] 小陀螺角速度 rad/s（非过洞段 = spin_w，
//                                              过洞段恒 0）
//   /gimbal_fold_cmd   UInt8                [发] 0 保持 / 1 折叠 / 2 展开
//   /gimbal_fold_state Int32                [收] 0 未知/1 折叠中/2 已折叠/3 展开中/4 已展开
//   /referee/summary   Float64MultiArray(≥9)[收] [0]game_progress [1]hp [2]hp_max [3]shots
//                                              [4]coins ... [8]stage_remain_s
//   /referee/operator_cmd Int32MultiArray(3)[收] 操作手指令（裁判系统通道）：
//                                              [0]type [1]param [2]seq
//                                              type 4 指点(param=点表 id)——决策层唯一
//                                              消费的类型；1 买弹/2 买血/3 买复活是
//                                              操作手和裁判客户端的事，决策层忽略
//                                              （seq 每条 +1，0 保留，决策按它去重）
//   /sentry/buy_ammo   Int32                [发] 自动买弹下单（决策层自己决定何时买）
//   /sentry/buy_hp     Int32                [发] 自动买血下单
//   /sentry/buy_revive Int32                [发] 自动买复活下单（hp==0 即阵亡触发）
//   /sentry/data_source String              [收] 数据源声明 "sim"/"real"（实车模式硬保护用）
//   /sentry/iface_status String(JSON)       [发] 1 Hz 接口数据流统计（确认数据在真实流动）
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
  std::string op_cmd_topic = "/referee/operator_cmd";
  std::string buy_ammo_topic = "/sentry/buy_ammo";
  std::string buy_hp_topic = "/sentry/buy_hp";
  std::string buy_revive_topic = "/sentry/buy_revive";
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
  // 副作用：建 6 个 publisher、5 个 subscription；打一条启动日志。
  RosPorts(rclcpp::Node* node, const RosPortsConfig& cfg);

  // ---- 哨兵导航 ----
  // 发布目标点（PoseStamped，orientation.w=1.0），并乐观把 nav_state_ 置 running，
  // 之后以 /nav_status 回调为准。返回恒 true（发出 ≠ 导航已接受，占位契约无 ack）。
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
  // 读缓存的折叠状态（来自 /gimbal_fold_state）；没收到过为 0（kFoldUnknown）。
  int fold_state() override;

  // ---- 裁判系统 ----
  // 读最近一帧 /referee/summary 解析出的快照。
  RefereeSnapshot referee() override;
  // 读最近一条 /referee/operator_cmd 操作手指令；seq == 0 = 没有。
  OpCmd op_cmd() override;

  // ---- 自动购买下单（决策层自己决定何时买，见 planner 的 run_auto_buy）----
  // 发 /sentry/buy_ammo（Int32 发数）。执行在裁判客户端/电控侧，到不到账从
  // /referee/summary 里看（血量/弹量涨了就是到账）。
  void buy_ammo(int rounds) override;
  // 发 /sentry/buy_hp（Int32 量）。
  void buy_hp(int amount) override;
  // 发 /sentry/buy_revive（Int32 1）。
  void buy_revive() override;

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

private:
  // 注册一条链路（记进 links_）。
  void register_link(const std::string& name, const std::string& topic,
                     const std::string& msg_type, bool outbound);
  // 记一次"收到一帧"。调用方必须已持有 mtx_。
  void note_rx_locked(const std::string& name);
  // 记一次"发出一帧 / 丢弃一帧"。内部自己加锁。
  void note_tx(const std::string& name, bool dropped = false);
  // 按语义名找链路（须持锁），找不到返回 end()。
  std::vector<LinkStat>::iterator find_link_locked(const std::string& name);

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
  rclcpp::SubscriptionBase::SharedPtr nav_status_sub_;
  rclcpp::SubscriptionBase::SharedPtr odom_sub_;
  rclcpp::SubscriptionBase::SharedPtr fold_state_sub_;
  rclcpp::SubscriptionBase::SharedPtr referee_sub_;
  rclcpp::SubscriptionBase::SharedPtr op_cmd_sub_;     // 操作手指令（裁判系统通道）
  rclcpp::SubscriptionBase::SharedPtr source_sub_;  // data_source（守卫开启后才有）

  // 状态缓存（读写都要 mtx_）
  mutable std::mutex mtx_;
  GoalState nav_state_ = kGoalIdle;  // 导航状态缓存
  BasePose pose_{};                  // 位姿缓存
  int fold_state_ = kFoldUnknown;    // 折叠状态缓存
  RefereeSnapshot ref_{};            // 裁判快照缓存；sim 守卫触发时 game_progress 压 0
  OpCmd op_cmd_{};                   // 最近一条操作手指令缓存（seq 0 = 没有）
  bool guard_enabled_ = false;       // 实车守卫是否开启
  bool sim_source_seen_ = false;     // 是否听到过 "sim" 声明

  // 接口统计
  std::vector<LinkStat> links_;
  std::vector<double> link_last_s_;   // 每条链路最近一帧的单调秒
  std::vector<uint64_t> link_win_;    // 本窗口帧数
  double link_win_start_s_ = 0.0;     // 窗口起点
};

}  // namespace bbt_decide
