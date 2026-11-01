// BBT 抽象接口（Port）：决策树只依赖这些，不依赖任何具体话题名。
// 具体实现在适配层（RosPorts 接真实 ROS 话题；StubPorts/DebugPorts 供单元测试）。
//
// 接口范围按使用者的要求裁剪，只留三类 + 操作手指点口 + 自动买输出口：
//   哨兵导航：nav_set_goal / nav_cancel / nav_state / base_pose
//   电控：    base_stop（停车）/ base_set_mode / base_spin / fold_set / fold_state
//   裁判系统：referee（比赛状态/血量/弹量/金币——自动买的判定与到账确认都看它）
//   操作手（经裁判系统通道）：op_cmd 收指令（只消费"指点"，买类忽略）
//   购买：    buy_ammo / buy_hp / buy_revive —— 决策层自动买的下单口
// 外加一个 now_s 单调时钟。RL / 雷达 / 自瞄接口一律没有（自瞄归电控）。
#pragma once

#include <cstdint>

#include "bbt_decide/types.hpp"

namespace bbt_decide
{

// 导航目标状态：一次导航任务走到了哪一步。
enum GoalState : int
{
  kGoalIdle = 0,     // 空闲：没有目标，或目标刚被取消
  kGoalRunning = 1,  // 运行中：目标已下发，还没到
  kGoalSuccess = 2,  // 成功：已到达（是否算"到"由导航侧容差判定）
  kGoalFailure = 3   // 失败：不可达 / 超时 / 卡死，需要决策层换点或重试
};

// 底盘位姿：定位侧给出的当前坐标。
struct BasePose
{
  double x = 0.0;         // X 坐标，单位 m；0 是默认值，不代表有效
  double y = 0.0;         // Y 坐标，单位 m；配合 valid 才有意义
  double yaw = 0.0;       // 车头朝向，单位 rad，[-pi, pi]；本版决策不用，透传
  uint64_t stamp_ms = 0;  // 64 位单调毫秒，不在系统运行 49.7 天后回绕
  bool valid = false;     // 位姿是否可信；false 时别拿 x/y 算距离/判侧
};

// 折叠状态（电控上报）。"折叠" = 把云台压下去，过洞前必须做，否则撞洞顶。
enum FoldState : int
{
  kFoldUnknown = 0,    // 未知：没收到过状态，或这台车没折叠能力
  kFoldFolding = 1,    // 折叠中：动作进行中，别中途下发新动作
  kFoldFolded = 2,     // 已折叠：可以过洞
  kFoldUnfolding = 3,  // 展开中：动作进行中
  kFoldUnfolded = 4    // 已展开：常规导航状态（默认目标状态）
};

// 端口集合：决策树看到的全部外部能力。全部纯虚，适配层各写一份实现。
class Ports
{
public:
  virtual ~Ports() = default;

  // ---- 哨兵导航 ----
  // 下发导航目标点，底盘自己走过去（路径规划、对正、绕障都在导航侧）。
  // 参数：x/y — 目标点坐标，单位 m。
  // 返回：true = 目标已被接受；false = 被拒或没接线。
  // 副作用：覆盖导航侧现有目标；接受后 nav_state() 变为 kGoalRunning。
  virtual bool nav_set_goal(double x, double y) = 0;
  // 取消当前导航目标。
  // 副作用：nav_state() 回到 kGoalIdle，底盘自己停住。
  virtual void nav_cancel() = 0;
  // 查导航目标当前状态。
  // 返回：GoalState。
  virtual GoalState nav_state() = 0;
  // 读当前底盘位姿（来源：定位/odom）。
  // 返回：BasePose；先看 valid 再用坐标。
  virtual BasePose base_pose() = 0;

  // ---- 电控 ----
  // 停车。决策层不直接发巡航速度（路径与速度归导航），只保留这条"停住"指令：
  // 取消 Goal 之后没有它，换点/中止过洞时车会滑。
  virtual void base_stop() = 0;
  // 切换底盘运动模式。参数：mode — 0 RELAX / 1 INDEPENDENT / 2 ROTOR / 3 FOLLOW。
  // 返回：true = 已接受。
  virtual bool base_set_mode(int mode) = 0;
  // 小陀螺自转。参数：w — 角速度 rad/s，0 = 停转；过洞期间恒发 0。
  virtual void base_spin(double w) = 0;
  // 请求折叠/展开云台。参数：fold — true 折叠（过洞前），false 展开（过洞后）。
  // 返回：true = 指令已接受；false = 这台车没有折叠能力。
  // 副作用：要等 fold_state() 变成 kFoldFolded / kFoldUnfolded 才算完成。
  virtual bool fold_set(bool fold) = 0;
  // 读当前折叠状态。返回：FoldState。
  virtual int fold_state() = 0;

  // ---- 自瞄意图提示（自瞄主权在电控，这里只是"建议"，电控可忽略）----
  // target_type：0 无指定 / 1 机器人 / 2 前哨建筑 / 3 基地；target_id：type=2 时
  // 1 = 对面前哨（约定同主工作区）。fire：1 允许开火；free_fire：1 让自瞄自由选目标。
  // 常规导航段每 tick 重发；比赛外/待机不发（门控同其他输出口）。
  virtual void attack_cmd(int target_type, int target_id, int fire, int free_fire)
  {
    (void)target_type;
    (void)target_id;
    (void)fire;
    (void)free_fire;
  }

  // ---- 裁判系统 ----
  // 读裁判系统快照。返回：RefereeSnapshot（比赛状态/血量/弹量等）。
  virtual RefereeSnapshot referee() = 0;

  // ---- 操作手（经裁判系统通道）----
  // 读最近一条操作手指令。返回：OpCmd；seq == 0 = 没有。
  // 调用方按 seq 去重——同一条指令只处理一次（决策层只消费"指点"，买类忽略）。
  virtual OpCmd op_cmd() { return OpCmd{}; }

  // ---- 购买（决策层自动买的输出口；操作手的买指令不经这里）----
  // 自动买弹下单：血/弹低了、金币够，决策层自己调（见 run_auto_buy）。
  // 参数：rounds — 买弹发数。默认实现丢弃。
  virtual void buy_ammo(int rounds) { (void)rounds; }
  // 自动买血下单。参数：amount — 买血量。默认实现丢弃。
  virtual void buy_hp(int amount) { (void)amount; }
  // 自动买复活下单（hp==0 即阵亡）。默认实现丢弃。
  virtual void buy_revive() {}

  // ---- 时间 ----
  // 单调时钟，单位 s。所有计时（驻留/超时/冷却）都以它为基准。
  virtual double now_s() = 0;
};

// 安全默认实现：不产生任何动作，供未接线阶段与单元测试使用。
class StubPorts : public Ports
{
public:
  bool nav_set_goal(double, double) override { return false; }
  void nav_cancel() override {}
  GoalState nav_state() override { return kGoalIdle; }
  BasePose base_pose() override { return BasePose{}; }
  void base_stop() override {}
  bool base_set_mode(int) override { return true; }
  void base_spin(double) override {}
  bool fold_set(bool) override { return false; }
  int fold_state() override { return kFoldUnknown; }
  RefereeSnapshot referee() override { return RefereeSnapshot{}; }
  double now_s() override { return 0.0; }
};

// 调试/测试用：可外部注入状态、可断言"下发了什么"，便于无硬件跑通整棵决策树。
class DebugPorts : public StubPorts
{
public:
  RefereeSnapshot ref;         // referee() 原样返回；测试里手动填
  BasePose pose;               // base_pose() 返回它
  double time_s = 0.0;         // 模拟时钟，单位 s；测试里手动推进
  GoalState goal = kGoalIdle;  // 模拟导航状态
  double goal_set_s = 0.0;     // 最近一次下发目标的时刻
  double auto_arrive_s = 0.0;  // >0 = 下发后这么多秒自动报到达；0 = 永远 running
  int fold = kFoldUnfolded;    // 折叠状态；默认已展开
  bool fold_capable = true;    // 是否具备折叠能力（false 时 fold_set 返回 false）

  double goal_x = 0.0;     // 最近一次导航目标
  double goal_y = 0.0;
  int goal_count = 0;      // nav_set_goal 被调用的累计次数
  int cancel_count = 0;    // nav_cancel 被调用的累计次数
  int stop_count = 0;      // base_stop 被调用的累计次数
  int spin_count = 0;      // base_spin 被调用的累计次数（任意角速度）
  int spin_zero_count = 0; // 其中 base_spin(0) 的次数（过洞段/比赛结束收尾）
  double spin_last_w = -999.0;  // 最近一次 base_spin 的角速度
  int mode_count = 0;      // base_set_mode 被调用的累计次数
  int mode_last = -1;      // 最近一次底盘模式
  int fold_cmd_count = 0;  // fold_set(true) 被调用的累计次数
  int unfold_cmd_count = 0;// fold_set(false) 被调用的累计次数

  OpCmd op;                     // op_cmd() 原样返回；测试里手动填（seq 递增模拟新指令）
  int buy_ammo_calls = 0;       // 自动买弹下单次数
  int buy_ammo_param = 0;       // 最近一次买弹发数
  int buy_hp_calls = 0;         // 自动买血下单次数
  int buy_hp_param = 0;         // 最近一次买血量
  int buy_revive_calls = 0;     // 自动买复活下单次数
  int attack_calls = 0;         // attack_cmd 被调用的累计次数
  int attack_last_type = -1;    // 最近一次 attack_cmd 的 target_type
  int attack_last_id = -1;      // 最近一次 attack_cmd 的 target_id
  int attack_last_fire = -1;    // 最近一次 attack_cmd 的 fire

  // 记录目标点并立刻进入 running。
  bool nav_set_goal(double x, double y) override
  {
    goal_x = x;
    goal_y = y;
    ++goal_count;
    goal = kGoalRunning;
    goal_set_s = time_s;
    return true;
  }
  void nav_cancel() override
  {
    goal = kGoalIdle;
    ++cancel_count;
  }
  GoalState nav_state() override
  {
    if (auto_arrive_s > 0.0 && goal == kGoalRunning && time_s - goal_set_s >= auto_arrive_s)
    {
      goal = kGoalSuccess;
    }
    return goal;
  }
  BasePose base_pose() override { return pose; }
  void base_stop() override { ++stop_count; }
  void base_spin(double w) override
  {
    ++spin_count;
    spin_last_w = w;
    if (w == 0.0) ++spin_zero_count;
  }
  bool base_set_mode(int m) override
  {
    ++mode_count;
    mode_last = m;
    return true;
  }
  bool fold_set(bool f) override
  {
    if (!fold_capable) return false;
    ++(f ? fold_cmd_count : unfold_cmd_count);
    fold = f ? kFoldFolded : kFoldUnfolded;  // 直接跳到终态，不模拟中间过程
    return true;
  }
  int fold_state() override { return fold; }
  RefereeSnapshot referee() override { return ref; }
  OpCmd op_cmd() override { return op; }
  void buy_ammo(int rounds) override
  {
    ++buy_ammo_calls;
    buy_ammo_param = rounds;
  }
  void buy_hp(int amount) override
  {
    ++buy_hp_calls;
    buy_hp_param = amount;
  }
  void buy_revive() override { ++buy_revive_calls; }
  void attack_cmd(int target_type, int target_id, int fire, int free_fire) override
  {
    (void)free_fire;
    ++attack_calls;
    attack_last_type = target_type;
    attack_last_id = target_id;
    attack_last_fire = fire;
  }
  double now_s() override { return time_s; }
};

}  // namespace bbt_decide
