// BBT 保底决策树 + 执行器：纯规则，无 RL。
//
// 决策树（每 tick 重算；比赛门控在执行层 IsMatchRunning，这里只管选点）：
//
//   ┌ 0. 操作手指点（最高优先级，人比规则大）：去操作手给的点，到达即清除，
//   │    之后自动恢复下面的正常决策。期间的换点不走防抖（人下了指令立即执行）
//   ├ 1. 血量 < hp_home_ratio 或 弹量 < ammo_home_min（锁存；血量 ≥ resume 比例
//   │    且弹量 ≥ ammo_resume_min 两个条件同时满足才解除）→ 回家
//   ├ 2. 我方前哨：被打掉（hp=0）或前哨阵地不被我方完全控制（ctrl≠0）→ 去 retake
//   │    点（站在我方前哨阵地的增益点上把控制权抢回来；完全控制后前哨站会重建）
//   ├ 3. 对面前哨：前哨阵地不被我方完全控制（ctrl≠0）→ 去 outpost 攻击位
//   │    （站在对面前哨阵地的增益点上转控制 + 打前哨站；打掉 + 完全控制即完成）
//   └ 4. 其余情况（两个前哨阵地都被我方完全控制 / 数据未知）──→ 巡逻
//        （环形轮换：到达（导航 success / 进入 arrive_radius）或驻留满
//          patrol_dwell_s 换下一个）
//
// 前哨争夺（2/3 段）判据只有裁判数据（0x0003 前哨血量 + 0x0101 前哨阵地控制状态），
// 不需要雷达；顺序固定"先保家再打人"（我方前哨优先于对面前哨）。数据未知（-1）的
// 一侧不参与判定——不知道就当没事，落到巡逻，绝不因为缺数据乱跑。
// outpost.enable: false 时 2/3 段整体不存在（没有前哨的地图用）。
//
// 旁路（不动目标点，每 tick 独立跑）：自动购买——弹/血低了、金币够就自己下单
// （run_auto_buy，条件见其注释）；hp==0 自动买复活。买没买到看裁判数据涨没涨。
// 打前哨的自瞄意图提示（attack_cmd）在执行器 nav_cruise 里按目标点 zone 发。
//
// 没有阵亡分支：死了下位机锁死、发什么都动不了（hp==0 只用来触发自动买复活）；
// 低血/低弹回家的锁存在死亡期间自然保持（hp=0 也低于阈值），复活后残血→从原地
// 回家、满血→解锁巡逻。
//
// 执行器（把"目标点"翻译成对导航/电控的动作）：
//   ┌ 比赛未开始/已结束 → 静默：不发任何消息。结束的那一拍做一次收尾
//   │   （取消导航 + 停自旋 + 终止过洞流程），之后闭嘴等比赛开始；
//   │   开始的那一拍切一次底盘模式。操作手的买类指令不受此门控（随时转发）
//   ├ 过洞流程进行中 ──→ 五段状态机独占：入口对正点 → 折叠 → 出口对正点 → 展开 → 交回
//   ├ 目标点区域 ≠ 车当前区域 ──→ 按 region 路由过洞（必须折叠后穿，绝不裸穿）
//   └ 其余 ──→ 常规导航：换点先取消旧 Goal 再发新 Goal（操作手重指同一点也算
//       "目标更新"，取消重发）；导航失败按节奏重试。所有非过洞段恒发受击旋转
//       spin_w（含巡逻段），过洞段恒 0。
//
// 过洞总开关 enable_tunnel（与 region 耦合）：
//   false = 不过洞 → region 表 / 点的 region / 洞表全部不需要，任何目标直接发导航；
//   true  = 按 region 路由。region 这套概念只为过洞而存在，开关关掉它整体消失。
//
// 过洞路由（按 region，与主工作区同款语义；不依赖任何几何猜测）：
//   区域是用户在 yaml 里自定义的名字表（走廊可以就 left/right）。每条洞的两个
//   对正点各自声明它服务的区域（align_region）。判定链条：
//     车当前区域（纯记账：开赛沿 start_region + 到达出口写出口）vs 目标点 region
//       → 不同则 route_tunnels() 在"区域图"上 BFS 出要依次过的洞
//       → 入口 = 绑定"车当前区域"的对正点，出口 = 绑定"目标区域"的对正点
//       → 过完一条写记账区域 = 出口区域，重新评估，自动串联下一条。
//   因此换地图 = 换 regions 表 + 重标点和对正点的区域归属，逻辑一行不动；
//   走廊里定义 left/right 两个区域、一个洞绑两侧，照样完整跑巡逻 + 折叠过洞。
#pragma once

#include "bbt_decide/ports.hpp"
#include "bbt_decide/types.hpp"

#include <vector>

namespace bbt_decide
{

// ---------------- 几何（纯函数） ----------------
// 两点直线距离，单位 m。
double dist(const Vec2& a, const Vec2& b);
// 点 p 在线段 a→b 上的投影参数：a 处 0、b 处 1、a 外侧 < 0、b 外侧 > 1。
double project_param(const Vec2& p, const Vec2& a, const Vec2& b);
// 点 p 到线段 a→b 的最短距离（投影截断到 [0,1]，含端点），单位 m。
double dist_to_segment(const Vec2& p, const Vec2& a, const Vec2& b);

// ---------------- 点 / 洞查询 ----------------
// 按 id 在点表里找点；找不到返回 nullptr。
const PointDef* find_point(const Config& cfg, int id);
// 找第一个指定 zone 的点（家只有一个时即"回家点"）；找不到返回 nullptr。
const PointDef* find_point_by_zone(const Config& cfg, int zone);
// 找第一个指定 zone 且当前区域可达、不贴闸线的点（outpost/retake 选点用）；
// 找不到返回 nullptr（调用方落到下一段决策）。
const PointDef* pick_zone_point(const Config& cfg, int zone, int cur_region);
// 点是否离任一洞的闸线（两对正点连线）太近（< cfg.gate_clear_m）：太近会挡住过洞路径。
bool too_close_to_gate(const Config& cfg, const Vec2& p);
// 找连接区域 a 与区域 b 的洞下标（两端 align_region 恰为 {a,b}，不分前后）；
// 找不到返回 -1；align_region 未填(-1)的洞整条跳过。
int find_tunnel(const Config& cfg, int a, int b);
// 区域路由：从 from 区域到 to 区域需要依次过的洞下标序列（区域图 BFS，边 = 洞）。
// 同区域返回空；不可达也返回空（调用方用 regions_reachable 区分）。
std::vector<int> route_tunnels(const Config& cfg, int from, int to);
// 两个区域之间是否可达（同区域也算可达）。
bool regions_reachable(const Config& cfg, int from, int to);
// 这个点当前能不能去：region 已填、从 cur_region 出发可达（同区域或存在洞路由）、
// 且不贴闸线。跨区域但无路 = 选了也到不了，不如不选。
bool point_usable(const Config& cfg, int cur_region, const PointDef& p);

// ---------------- 感知 ----------------
// 一帧感知快照：从 Ports 抓裁判 / 位姿 / 导航状态。
struct Sense
{
  bool match_running = false;       // 裁判 game_progress == 4（比赛进行中）——唯一门控
  double hp_ratio = 0.0;            // hp / hp_max（回家锁存用；阵亡不单独判：
                                    // 死了下位机锁死发什么都动不了，血量低锁存
                                    // 自然保持，复活后（残血→继续回家 / 满血→巡逻）
                                    // 直接从原地接着走）
  int hp = 0;                       // 原始血量（0 = 阵亡 → 自动买复活的触发条件）
  int shots = -1;                   // 剩余弹量；-1 = 未知（未知时不参与弹量锁存/自动买）
  int coins = -1;                   // 金币；-1 = 未知（未知时不自动买——宁可不花不错花）
  Vec2 pos;                         // 当前位置，单位 m
  bool pos_valid = false;           // 位姿是否可信（新鲜且有效）
  GoalState nav_state = kGoalIdle;  // 导航目标状态
  // ---- 前哨（2027 前哨阵地争夺；R/B 已由桥按我方颜色映射，这里只有"我方/对方"）----
  // known = 血量及控制状态均已知：未知的一侧不参与争夺判定（不知道就当没事，别乱跑）。
  bool our_outpost_known = false;   // 我方前哨数据是否有效
  int our_outpost_hp = -1;          // 我方前哨站血量；0 = 被打掉（阵地易主会以 500 血重建）
  int our_outpost_ctrl = kCtrlUnknown;  // 我方前哨阵地控制状态（kCtrl*）
  bool enemy_outpost_known = false; // 对面前哨数据是否有效
  int enemy_outpost_hp = -1;        // 对面前哨站血量
  int enemy_outpost_ctrl = kCtrlUnknown;  // 对面前哨阵地控制状态
  // ---- 兑换/复活门控（自动买用；-1 = 未知不拦）----
  int revive_available = -1;        // 0 不可复活 / 1 可免费 / 2 可金币兑换立即复活
  int revive_cost = -1;             // 兑换立即复活所需金币
  int remote_ammo_pool = -1;        // 队伍 17mm 剩余可兑换数；0 = 远程兑换池空了，别下单
  int out_of_combat = -1;           // 1 脱战 / 0 交战中（远程兑换血量要求脱战）
};

// 从 Ports 抓一帧感知（只读，不改任何状态）。
Sense sense(Ports& ports, const Config& cfg, double now_s);

// ---- 前哨争夺判定（2027 规则 5.6.2 前哨阵地；判据只有裁判数据，不需要雷达）----
// 我方前哨是否需要处理：被打掉（hp=0），或前哨阵地不被我方完全控制（ctrl≠0）。
// 数据未知（known=false）恒 false——不知道就当没事，别乱跑。
bool our_outpost_needs(const Sense& s);
// 对面前哨是否需要处理：前哨阵地不被我方完全控制（ctrl≠0）即去争（打掉它只是
// 争夺手段之一；完全控制才是"占领"）。数据未知恒 false。
bool enemy_outpost_needs(const Sense& s);

// ---------------- 决策树 ----------------
// 决策跨帧记忆：锁存 / 防抖 / 巡逻进度 / 区域记账 / 操作手指点。
struct PlanState
{
  bool home_latched = false;  // 回家锁存：低阈值（血量或弹量）触发，
                              // 血量+弹量都回到高阈值才解除（双条件防抖）
  int cur_point = -1;         // 当前生效的目标点 id；-1 = 还没有
  double cur_since_s = 0.0;   // 该点生效时刻（驻留计时基准），单调秒
  bool cur_arrived = false;   // 是否已到过当前点（导航 success 或进入 arrive_radius）
  int pending_point = -1;     // 换点防抖：候选点 id
  int pending_count = 0;      // 候选点连续一致帧数
  int current_region = 0;     // 区域记账（开局 = start_region；过洞完成后写出口区域；
                              // 无外部区域覆盖）
  double op_since_s = 0.0;
  int op_goal = -1;           // 操作手指的点表 id；-1 = 无。到达后清除，恢复正常决策
  bool op_fresh = false;      // 指点刚到（还没下发过导航）：即使与当前目标同一点也要
                              // 取消重发——这就是"导航目标的更新"（操作手再按一次 =
                              // 强制重发，卡了也能拉起来）；由 nav_cruise 消费清零
  uint32_t op_seq_seen = 0;   // 已处理过的操作手指令序号（按 seq 去重，一条只办一次）
  double buy_ammo_next_s = 0.0;   // 自动买弹下一次允许下单的时刻（冷却记账）
  double buy_hp_next_s = 0.0;     // 自动买血
  double buy_revive_next_s = 0.0; // 自动买复活
};

// 保底决策树本体：按"待机 → 阵亡 → 电量 → 回血 → 巡逻"的顺序出目标点。
// 参数：s — 感知快照；cfg — 配置；plan — 决策记忆（会被写）；now_s — 单调秒；
//       nav_goal_for — 执行器当前导航目标对应的点 id（-2 = 无 / 正在过洞）。
//       只有它等于当前点时，导航报的 success 才算"到达了当前点"——过洞期间导航
//       对对正点报的 success 不能当成巡逻点到点，否则会跳点。
// 返回：Decision；target_point == -1 表示待机/停车。
// 副作用：写 plan 的锁存与防抖记账；不碰 Ports（不下发任何指令）。
Decision decide_tree(const Sense& s, const Config& cfg, PlanState& plan, double now_s,
                     int nav_goal_for = -2);

// ---------------- 过洞五段状态机 ----------------
// 一条洞的穿越流程：入口对正点 → 折叠 → 出口对正点 → 展开 → 交回常规导航。
// 入口/出口由 align_region 决定（车在哪个区域就从绑定该区域的对正点进），
// 不做几何猜测；过洞期间独占执行权，完成时把记账区域写成出口区域。
struct TunnelRunner
{
  enum Phase : int
  {
    kOff = 0,      // 未在过洞
    kNavEntry,     // 已发"去入口对正点"的导航目标
    kWaitEntry,    // 等导航到达入口
    kFold,         // 已发折叠指令
    kWaitFold,     // 等折叠到位
    kNavExit,      // 已发"去出口对正点"的导航目标（穿洞）
    kWaitExit,     // 等导航到达出口
    kUnfold,       // 已发展开指令
    kWaitUnfold    // 等展开到位
  };

  Phase phase = kOff;          // 当前阶段
  int tunnel = -1;             // 正在过的洞下标
  int after_point = -1;        // 过完洞要继续去的点（流程期间锁死，不被换点打断）
  Vec2 entry;                  // 入口对正点（绑定车当前区域），单位 m
  Vec2 exit;                   // 出口对正点（绑定目标区域），单位 m
  int exit_region = -1;        // 出口对正点绑定的区域；完成时写进 plan.current_region
  double phase_since_s = 0.0;  // 进入当前阶段的时刻，单调秒
  double block_until_s = 0.0;  // 失败冷却截止时刻；期间不再启动任何过洞
  bool crossing_started = false;  // 已向出口运动：失败后必须保留穿洞恢复事务
  bool exit_reached = false;      // 已确认到出口，区域记账不再依赖展开反馈

  // 是否正在过洞（独占执行权）。
  bool active() const { return phase != kOff; }

  // 启动一次穿越。参数：ports — 读折叠状态/停车；cfg；tid — 洞下标；
  //   from_region — 车当前区域（= plan.current_region）；from — 车当前位置指针
  //   （nullptr = 位姿无效，跳过洞口保护判定）；target — 过洞后要去的点；now — 单调秒。
  // 返回：true = 已接受，调用方应紧接着调 step() 发第一个导航目标；
  //       false = 拒绝（from_region 与洞两端区域都对不上 / 车未折叠却已在洞口 /
  //       折叠状态未知 / 洞定义退化），已自带冷却与停车。
  // 副作用：拒绝路径会调用 base_stop / nav_cancel；写本结构状态。
  bool start(Ports& ports, const Config& cfg, int tid, int from_region, const Vec2* from,
             const PointDef& target, double now);

  // 状态机推进，每 tick 调一次（仅 active() 时有意义）。
  // 副作用：nav_set_goal / fold_set / base_spin / base_stop / nav_cancel；
  //   到达出口时写 plan.current_region；展开完成后把 phase 置回 kOff
  //   （洞内/展开失败保持事务，冷却后重试）。
  void step(Ports& ports, const Config& cfg, PlanState& plan, double now);

  // 中止（比赛结束 / 阵亡 / 上层要求待机）：停车、取消导航、进冷却。
  // 不写区域记账（中止 ≠ 已到出口）；折叠状态故意不动——中止不等于已经离开洞口。
  void abort(Ports& ports, const Config& cfg, double now);
};

// 执行器跨帧状态（常规导航的下发记账 + 过洞状态机 + 比赛沿检测）。
struct ExecState
{
  int applied_goal = -2;      // 已下发导航目标的点 id；-2 = 无
  double retry_until_s = 0.0; // 导航失败后的重试解禁时刻
  bool stop_sent = false; // 当前停止/冷却窗口已收尾，避免每 tick 重复取消
  double goal_since_s = 0.0; // 当前普通导航目标下发时间
  TunnelRunner tunnel;        // 过洞状态机
  bool match_on = false;      // 比赛当前是否进行（沿检测，两路执行器共用）
  bool standby_cleaned = true;  // 进待机后是否已完成一次性收尾（true = 已收尾/静默中）
};

// 操作手指令处理（step 与 BT 的 IsMatchRunning 节点共用）：
// 按 seq 去重后分发——**只消费"指点"（type=4）**：写 plan.op_goal 并置 op_fresh。
// 买类（type 1–3）不是决策层的事，记下 seq 即忽略（操作手经裁判客户端/电控直接办，
// 到不到账从裁判数据看）。点表里不存在的 id 也静默忽略。
// 副作用：写 plan.op_goal / op_fresh / op_seq_seen。
void handle_operator(Ports& ports, const Config& cfg, PlanState& plan);

// 自动购买（step 与 BT 的 IsMatchRunning 节点共用）：决策层**自己**下单。
//   条件 = 比赛进行中（比赛外一条消息不发）+ 裁判数据触发 + 金币够 + 冷却已过：
//     买弹：弹量 < ammo_buy_min_shots 且金币 ≥ ammo_buy_min_coins
//     买血：活着且血量比例 < hp_buy_ratio 且金币 ≥ hp_buy_min_coins
//           （比回家锁存阈值高：先远程买一口，买不到再走回家那条路）
//     买复活：hp == 0（=阵亡，不需要单独的阵亡判定）且金币 ≥ revive_buy_min_coins
//   远程兑换有延迟：每类下单后冷却 buy_cooldown_s 再允许重试；买没买到看裁判
//   数据——血量/弹量涨了触发条件自然消失，没涨冷却过了会再下单。
//   各类可用阈值写负数单独关掉；enable_auto_buy = false 全关。
// 副作用：ports.buy_ammo / buy_hp / buy_revive 下单；写 plan.buy_*_next_s 冷却。
void run_auto_buy(Ports& ports, const Config& cfg, const Sense& s, PlanState& plan,
                  double now);

// ---------------- 区域记账 ----------------
// 区域 = 纯记账（不比最近点、不订阅区域话题，零区域接口）：
//   开赛沿（比赛开始的那一拍）把 plan.current_region 重置为 start_region——
//   新开赛车被摆回开局位；之后**唯一**的更新点是到达出口写出口对正点的区域。
//   记账为什么自洽：车只按决策发的导航目标移动，跨区移动必走"过洞流程"，
//   过洞完成必写出口区域 → 不存在车绕开决策换区的路径。
//   （代价：被外力推着穿过洞/被搬走，记账不会自纠——靠洞口"未折叠拒绝裸穿"
//   保护和下一次开赛沿重置兜底。）

// 过洞执行段（run_executor 的前半，BT 的 TunnelCross 节点共用）：
// 目标在别的区域 → 启动/推进过洞流程；跨区但被拒/冷却/不可达 → 停车等待。
// enable_tunnel = false 时恒返回 false（整套过洞/区域逻辑不存在）。
// 返回：true = 过洞相关事务占用了本 tick（调用方不要再发常规导航）；
//       false = 本 tick 与过洞无关（同区域目标或空目标，交给 nav_cruise）。
bool tunnel_handled(Ports& ports, const Config& cfg, const Sense& s, const Decision& d,
                    PlanState& plan, ExecState& ex, double now);

// 常规导航执行段（run_executor 的后半，BT 的 NavCruise 节点共用）：
// 换点先取消旧 Goal 再发新 Goal；导航失败按 nav_retry_s 节奏重试；
// 目标为空（待机/阵亡）→ 停车 + 取消。force_resend（操作手重指点）即使目标没变
// 也取消重发一次，发出后清零。所有路径恒发受击旋转 spin_w（本段 = 非过洞段）。
void nav_cruise(Ports& ports, const Config& cfg, const Sense& s, int target_point,
                ExecState& ex, double now, bool& force_resend);

// 执行器：把决策的目标点翻译成导航/电控动作。
// 比赛门控（沿触发，不再每拍发消息）：结束的那一拍收尾一次（取消导航 + 停自旋 +
// 终止过洞流程，折叠保持），之后**完全静默**——比赛外一条消息都不发；开始的
// 那一拍切一次底盘模式。收尾/静默状态记在 ex.match_on / ex.standby_cleaned。
// 参数：ports — 下发出口；cfg；s — 感知；d — 决策树输出；
//       plan — 决策记忆（过洞完成时写区域记账）；ex — 执行器状态（会被写）；
//       now — 单调秒。
// 副作用：见 TunnelRunner 与常规导航逻辑；过洞进行中独占执行权。
bool update_match_gate(Ports& ports, const Config& cfg, const Sense& s,
                       PlanState& plan, ExecState& ex);

void run_executor(Ports& ports, const Config& cfg, const Sense& s, const Decision& d,
                  PlanState& plan, ExecState& ex, double now);

// 一步总入口（操作手指令 → 感知 → 区域同步 → 决策树 → 执行器），节点主循环每 tick 调它。
// 与行为树共用门控函数；操作手只消费指点，区域采用开局/出口记账。
// 返回：本 tick 实际生效的目标点 id（-1 = 待机；过洞中 = 过完要去的点）。
int step(Ports& ports, const Config& cfg, PlanState& plan, ExecState& ex, double now);

}  // namespace bbt_decide
