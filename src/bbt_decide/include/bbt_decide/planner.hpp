// Geometry, sensing and blackboard facts for the BBT production behavior tree.
// Target priority, purchase branches and crossing workflow are composed in main.xml.
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
// 等长最少洞路线可用 origin 到首洞入口的距离打破平局；无有效位置时按洞表顺序。
std::vector<int> route_tunnels(const Config& cfg, int from, int to,
                               const Vec2* origin = nullptr);
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

// Blackboard facts shared by the production behavior tree. No business phase enum is used.
struct PlanState
{
  bool home_latched = false;
  int cur_point = -1;
  PointDef cur_target;
  bool cur_target_valid = false;
  bool cur_operator_target = false;
  uint64_t cur_generation = 0;
  double cur_since_s = 0.0;
  bool cur_arrived = false;
  int pending_point = -1;
  int pending_count = 0;
  int current_region = 0;
  bool op_active = false;
  int op_goal = -1;
  PointDef op_target;
  uint64_t op_generation = 0;
  uint32_t op_seq_seen = 0;
  double op_since_s = 0.0;
  double op_arrived_s = -1.0;
  double op_clear_since_s = -1.0;
  double buy_ammo_next_s = 0.0;
  double buy_hp_next_s = 0.0;
  double buy_revive_next_s = 0.0;
};

// An owned physical crossing transaction persists after the traversal request is submitted.
// The timestamps and booleans are navigation/position evidence, not workflow stages.
struct TunnelTransaction
{
  int tunnel = -1;
  int after_point = -1;
  uint64_t generation = 0;
  Vec2 entry;
  Vec2 center;
  Vec2 exit;
  bool has_center = false;
  int exit_region = -1;
  double entry_arrived_s = -1.0;
  bool crossing_started = false;
  bool center_reached = false;
  bool exit_reached = false;
  double block_until_s = 0.0;
  double fold_requested_s = -1.0;
  double unfold_requested_s = -1.0;
  bool active() const { return tunnel >= 0; }
};

// The purpose records the external navigation request owner, not a business workflow phase.
enum class NavigationPurpose { none, ordinary, tunnel_entry, tunnel_crossing };
struct NavigationRequest
{
  NavigationPurpose purpose = NavigationPurpose::none;
  int target_point = -2;
  uint64_t generation = 0;
  double started_s = 0.0;
  bool active() const { return purpose != NavigationPurpose::none; }
};

struct ExecState
{
  int applied_goal = -2;
  uint64_t applied_generation = 0;
  NavigationRequest navigation;
  double retry_until_s = 0.0;
  int retry_target = -2;
  uint64_t retry_generation = 0;
  bool stop_sent = false;
  double goal_since_s = 0.0;
  TunnelTransaction tunnel;
  bool match_on = false;
  bool standby_cleaned = true;
};

// Pure selection helpers: routing priorities and lifecycle branches live in the XML tree.
int pick_patrol(const Sense& s, const Config& cfg, PlanState& plan, double now_s,
                int nav_goal_for);
void update_home_latch(const Sense& s, const Config& cfg, PlanState& plan);
void handle_operator(Ports& ports, const Config& cfg, PlanState& plan);
void remember_tunnel_progress(const Sense& s, TunnelTransaction& transaction);

// Compatibility entry point. Executes the same production XML and registered BT nodes.
int step(Ports& ports, const Config& cfg, PlanState& plan, ExecState& ex, double now);

}  // namespace bbt_decide
