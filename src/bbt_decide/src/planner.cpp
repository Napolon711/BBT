// BBT 保底决策树与执行器实现。纯逻辑：不依赖 ROS，所有跨帧状态由调用方持有。
// 过洞路由按 region（用户自定义区域表 + 对正点绑定区域），与主工作区同款语义；
// 几何用于洞口保护、等长路线择近与中心通过进度；区域始终由显式绑定记账。
#include "bbt_decide/planner.hpp"
#include "bbt_decide/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace bbt_decide
{

// ---------------- 几何 ----------------
double dist(const Vec2& a, const Vec2& b) { return std::hypot(a.x - b.x, a.y - b.y); }

double project_param(const Vec2& p, const Vec2& a, const Vec2& b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  if (len2 < 1e-12) return 0.0;
  return ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
}

double dist_to_segment(const Vec2& p, const Vec2& a, const Vec2& b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  if (len2 < 1e-12) return dist(p, a);
  double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
  t = std::clamp(t, 0.0, 1.0);
  const Vec2 proj{a.x + t * dx, a.y + t * dy};
  return dist(p, proj);
}

// ---------------- 点 / 洞查询 ----------------
const PointDef* find_point(const Config& cfg, int id)
{
  for (const auto& p : cfg.points)
  {
    if (p.id == id) return &p;
  }
  return nullptr;
}

const PointDef* find_point_by_zone(const Config& cfg, int zone)
{
  for (const auto& p : cfg.points)
  {
    if (p.zone == zone) return &p;
  }
  return nullptr;
}

const PointDef* pick_zone_point(const Config& cfg, int zone, int cur_region)
{
  for (const auto& p : cfg.points)
  {
    if (p.zone != zone) continue;
    if (!point_usable(cfg, cur_region, p)) continue;
    return &p;
  }
  return nullptr;
}

bool our_outpost_needs(const Sense& s)
{
  if (!s.our_outpost_known) return false;  // 未知就当没事：缺数据绝不触发跑位
  // 打掉（hp=0）或阵地失守（非我方完全控制）都要去 retake；"hp>0 且完全控制"才算安好
  return !(s.our_outpost_hp > 0 && s.our_outpost_ctrl == kCtrlFull);
}

bool enemy_outpost_needs(const Sense& s)
{
  if (!s.enemy_outpost_known) return false;
  // 完全控制（kCtrlFull）才算占领；临时控制/未控制都继续争。前哨站打没打掉不单独
  // 判——打掉只是把对方"完全控制"打成"临时控制"的手段，站上增益点转化才是占领。
  return s.enemy_outpost_ctrl != kCtrlFull;
}

bool too_close_to_gate(const Config& cfg, const Vec2& p)
{
  for (const auto& t : cfg.tunnels)
  {
    if (t.align_region[0] < 0 || t.align_region[1] < 0 ||
        t.align_region[0] == t.align_region[1] || dist(t.align[0], t.align[1]) < 1e-6) continue;
    if (dist_to_segment(p, t.align[0], t.align[1]) < cfg.gate_clear_m) return true;
  }
  return false;
}

int find_tunnel(const Config& cfg, int a, int b)
{
  for (size_t i = 0; i < cfg.tunnels.size(); ++i)
  {
    const auto& t = cfg.tunnels[i];
    if (t.align_region[0] < 0 || t.align_region[1] < 0) continue;  // 未绑区域的洞跳过
    const bool ab = (t.align_region[0] == a && t.align_region[1] == b);
    const bool ba = (t.align_region[0] == b && t.align_region[1] == a);
    if (ab || ba) return static_cast<int>(i);
  }
  return -1;
}

std::vector<int> route_tunnels(const Config& cfg, int from, int to, const Vec2* origin)
{
  std::vector<int> route;
  if (from == to) return route;
  const int n = static_cast<int>(cfg.regions.size());
  if (from < 0 || to < 0 || from >= n || to >= n) return route;

  // 从终点反向 BFS 取得最少穿洞数。并行洞或等长路线均保留为候选，
  // 执行时仅用车到首洞入口的距离择近，不会为近入口选更长的跨区路线。
  std::vector<int> hops(n, -1);
  std::vector<int> queue{to};
  hops[to] = 0;
  for (size_t head = 0; head < queue.size(); ++head)
  {
    const int cur = queue[head];
    for (const auto& t : cfg.tunnels)
    {
      if (t.align_region[0] < 0 || t.align_region[1] < 0) continue;
      for (int k = 0; k < 2; ++k)
      {
        if (t.align_region[k] != cur) continue;
        const int nxt = t.align_region[1 - k];
        if (nxt < 0 || nxt >= n || hops[nxt] >= 0) continue;
        hops[nxt] = hops[cur] + 1;
        queue.push_back(nxt);
      }
    }
  }
  if (hops[from] < 0) return route;
  const bool origin_valid = origin && std::isfinite(origin->x) && std::isfinite(origin->y);
  int cur = from;
  while (cur != to)
  {
    int chosen = -1, next_region = -1;
    double nearest = 0.0;
    for (size_t i = 0; i < cfg.tunnels.size(); ++i)
    {
      const auto& t = cfg.tunnels[i];
      if (t.align_region[0] < 0 || t.align_region[1] < 0) continue;
      for (int k = 0; k < 2; ++k)
      {
        if (t.align_region[k] != cur) continue;
        const int nxt = t.align_region[1 - k];
        if (nxt < 0 || nxt >= n || hops[nxt] != hops[cur] - 1) continue;
        const double approach = route.empty() && origin_valid ? dist(*origin, t.align[k]) : 0.0;
        if (chosen < 0 || approach < nearest)
        {
          chosen = static_cast<int>(i);
          next_region = nxt;
          nearest = approach;
        }
      }
    }
    if (chosen < 0) return {}; // 防御性保护：区域图在完整配置中应始终可回溯。
    route.push_back(chosen);
    cur = next_region;
  }
  return route;
}

bool regions_reachable(const Config& cfg, int from, int to)
{
  if (from == to) return true;
  return !route_tunnels(cfg, from, to).empty();
}

bool point_usable(const Config& cfg, int cur_region, const PointDef& p)
{
  if (!cfg.enable_tunnel) return true;  // 不过洞：区域/闸线整套不存在，谁都能去
  if (p.region < 0) return false;  // 没标区域的点不能选
  if (!regions_reachable(cfg, cur_region, p.region)) return false;  // 跨区域却无洞路
  if (too_close_to_gate(cfg, p.pos)) return false;  // 贴闸线会挡住过洞路径
  return true;
}

// ---------------- 感知 ----------------
Sense sense(Ports& ports, const Config& cfg, double now_s)
{
  Sense s;
  const RefereeSnapshot r = ports.referee();
  s.match_running = (r.game_progress == 4);  // 门控只看比赛状态：开始/结束
  s.hp_ratio = static_cast<double>(r.hp) / static_cast<double>(std::max(1, r.hp_max));
  s.hp = r.hp;
  s.shots = r.shots;  // -1 = 未知：不参与弹量锁存/自动买
  s.coins = r.coins;  // -1 = 未知：不自动买（宁可不花不错花）
  // 前哨：R/B 已由桥按我方颜色映射成我方/对方；血量 -1 = 没收到过（不参与判定）
  s.our_outpost_known = r.our_outpost_hp >= 0 &&
                        r.our_outpost_ctrl >= kCtrlFull && r.our_outpost_ctrl <= kCtrlNone;
  s.our_outpost_hp = r.our_outpost_hp;
  s.our_outpost_ctrl = r.our_outpost_ctrl;
  s.enemy_outpost_known = r.enemy_outpost_hp >= 0 &&
                          r.enemy_outpost_ctrl >= kCtrlFull && r.enemy_outpost_ctrl <= kCtrlNone;
  s.enemy_outpost_hp = r.enemy_outpost_hp;
  s.enemy_outpost_ctrl = r.enemy_outpost_ctrl;
  s.revive_available = r.revive_available;
  s.revive_cost = r.revive_cost;
  s.remote_ammo_pool = r.remote_ammo_pool;
  s.out_of_combat = r.out_of_combat;
  const BasePose p = ports.base_pose();
  s.pos = Vec2{p.x, p.y};
  // 位姿新鲜度：odom 断流（超过 odom_stale_ms 没更新）后不能继续用最后一帧位置
  // 判区域/到达/洞口——过期数据当无效处理，决策自然退到安全路径（区域保记账、
  // 到达只认导航 success、洞口保护不判）。
  const double pose_age_ms = now_s * 1000.0 - static_cast<double>(p.stamp_ms);
  s.pos_valid = p.valid && std::isfinite(p.x) && std::isfinite(p.y) && pose_age_ms >= 0.0 && pose_age_ms <= cfg.odom_stale_ms;
  s.nav_state = ports.nav_state();
  return s;
}


// ---------------- 决策树 ----------------
namespace
{
// 巡逻候选按显式环顺序；未配置环时沿用patrol分类的点表顺序。
std::vector<const PointDef*> patrol_candidates(const Config& cfg, int cur_region)
{
  std::vector<const PointDef*> cand;
  if (!cfg.patrol_point_ids.empty())
  {
    // An explicit patrol ring uses its listed order; zones and the full point table remain intact.
    for (const int id : cfg.patrol_point_ids)
    {
      const auto* point = find_point(cfg, id);
      if (point && point->zone == kZonePatrol && point_usable(cfg, cur_region, *point))
        cand.push_back(point);
    }
    return cand;
  }
  for (const auto& point : cfg.points)
  {
    if (point.zone == kZonePatrol && point_usable(cfg, cur_region, point))
      cand.push_back(&point);
  }
  return cand;
}

// 是否算"已到达目标点"：导航对**这个点的目标**报 success，或位姿有效且已站在点半径内。
// nav_goal_for != p.id 时（目标还没发过 / 正在过洞），导航的 success 属于别的目标，不算。
bool arrived_at(const Sense& s, const Config& cfg, const PointDef& p, int nav_goal_for)
{
  if (nav_goal_for == p.id && s.nav_state == kGoalSuccess) return true;
  return s.pos_valid && dist(s.pos, p.pos) <= cfg.arrive_radius_m;
}

// 巡逻选点：到达或驻留满 patrol_dwell_s 换下一个；否则保持当前点。
}  // namespace

int pick_patrol(const Sense& s, const Config& cfg, PlanState& plan, double now_s,
                int nav_goal_for)
{
  const auto cand = patrol_candidates(cfg, plan.current_region);
  if (cand.empty())
  {
    // 一个可用的巡逻点都没有：退回家站着（家也不可用 → -1 停车）
    const PointDef* home = find_point_by_zone(cfg, kZoneHome);
    if (home != nullptr && point_usable(cfg, plan.current_region, *home)) return home->id;
    return -1;
  }

  // 当前点不在候选里（首次 / 刚从家回来 / 区域变了 / 点表改过）→ 取最近的
  // （位姿无效取第一个）
  size_t cur_idx = cand.size();
  for (size_t i = 0; i < cand.size(); ++i)
  {
    if (cand[i]->id == plan.cur_point)
    {
      cur_idx = i;
      break;
    }
  }
  if (cur_idx == cand.size())
  {
    if (!s.pos_valid) return cand.front()->id;
    size_t near_i = 0;
    double near_d = 1e9;
    for (size_t i = 0; i < cand.size(); ++i)
    {
      const double d = dist(s.pos, cand[i]->pos);
      if (d < near_d)
      {
        near_d = d;
        near_i = i;
      }
    }
    return cand[near_i]->id;
  }

  const PointDef* cur = cand[cur_idx];
  if (arrived_at(s, cfg, *cur, nav_goal_for)) plan.cur_arrived = true;  // 锁存"到过了"
  const bool dwelled = (now_s - plan.cur_since_s) >= cfg.patrol_dwell_s;
  if (plan.cur_arrived || dwelled) return cand[(cur_idx + 1) % cand.size()]->id;
  return cur->id;
}

// ---------------- 操作手指令 ----------------
namespace
{
void apply_operator_command(Ports& ports, const Config& cfg, PlanState& plan,
                            const OpCmd& command)
{
  if (command.type != 4) return; // Human purchases are executed directly by the referee.

  PointDef target;
  if (command.param >= 0)
  {
    const auto* configured = find_point(cfg, command.param);
    if (!configured || !point_usable(cfg, plan.current_region, *configured)) return;
    target = *configured;
  }
  else
  {
    target.id = -3;
    Vec2 field{command.x, command.y};
    if (cfg.own_team == "blue" && cfg.geometry_reference != "none")
      field = {28.0 - field.x, 15.0 - field.y};
    const FrameTransform& map_transform =
        cfg.own_team.empty() ? cfg.field_to_map : cfg.active_field_to_map;
    target.pos = apply_frame_transform(field, map_transform);
    target.region = region_for_position(cfg, target.pos);
    target.zone = kZonePatrol;
    if (!std::isfinite(target.pos.x) || !std::isfinite(target.pos.y) ||
        ((cfg.enable_tunnel || !cfg.region_geometry.empty()) && target.region < 0) ||
        point_inside_tunnel(cfg, target.pos) ||
        (cfg.enable_tunnel && !regions_reachable(cfg, plan.current_region, target.region))) return;
  }
  if (plan.op_active && target.pos.x == plan.op_target.pos.x &&
      target.pos.y == plan.op_target.pos.y) return;
  plan.op_active = true;
  plan.op_goal = target.id;
  plan.op_target = target;
  ++plan.op_generation;
  plan.op_since_s = ports.now_s();
  plan.op_arrived_s = -1.0;
  plan.op_clear_since_s = -1.0;
}
} // namespace

void handle_operator(Ports& ports, const Config& cfg, PlanState& plan)
{
  // Queue adapters preserve reception order. Snapshot adapters stop at the repeated sequence.
  for (;;)
  {
    const auto command = ports.op_cmd();
    if (command.seq == 0 || command.seq == plan.op_seq_seen) return;
    plan.op_seq_seen = command.seq;
    apply_operator_command(ports, cfg, plan, command);
  }
}

void update_home_latch(const Sense& s, const Config& cfg, PlanState& plan)
{
  const bool ammo_watch = cfg.ammo_home_min >= 0 && s.shots >= 0;
  const bool resource_low = s.hp_ratio < cfg.hp_home_ratio ||
                            (ammo_watch && s.shots < cfg.ammo_home_min);
  const bool resources_recovered = s.hp_ratio >= cfg.hp_resume_ratio &&
                                  (!ammo_watch || s.shots >= cfg.ammo_resume_min);
  if (resource_low) plan.home_latched = true;
  else if (resources_recovered) plan.home_latched = false;
}

void remember_tunnel_progress(const Sense& s, TunnelTransaction& transaction)
{
  if (!s.pos_valid || !transaction.crossing_started || transaction.center_reached ||
      !transaction.has_center) return;
  const double ahead = (transaction.center.x - s.pos.x) *
                           (transaction.exit.x - transaction.entry.x) +
                       (transaction.center.y - s.pos.y) *
                           (transaction.exit.y - transaction.entry.y);
  if (ahead <= 0.0 || dist(s.pos, transaction.center) <= 0.08)
    transaction.center_reached = true;
}

}  // namespace bbt_decide
