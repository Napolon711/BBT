// BBT 保底决策树与执行器实现。纯逻辑：不依赖 ROS，所有跨帧状态由调用方持有。
// 过洞路由按 region（用户自定义区域表 + 对正点绑定区域），与主工作区同款语义；
// 唯一的几何用途是"车是否已在洞口/洞内"的保护判定。
#include "bbt_decide/planner.hpp"

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

bool too_close_to_gate(const Config& cfg, const Vec2& p)
{
  for (const auto& t : cfg.tunnels)
  {
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

std::vector<int> route_tunnels(const Config& cfg, int from, int to)
{
  std::vector<int> route;
  if (from == to) return route;
  const int n = static_cast<int>(cfg.regions.size());
  if (from < 0 || to < 0 || from >= n || to >= n) return route;

  // 区域图 BFS：节点 = 区域，边 = 连接两区域的洞
  std::vector<int> prev_region(n, -1);   // 到达该区域的前一个区域
  std::vector<int> prev_tunnel(n, -1);   // 用的哪条洞
  std::vector<bool> visited(n, false);
  std::vector<int> queue{from};
  visited[from] = true;
  while (!queue.empty())
  {
    const int cur = queue.front();
    queue.erase(queue.begin());
    for (size_t i = 0; i < cfg.tunnels.size(); ++i)
    {
      const auto& t = cfg.tunnels[i];
      if (t.align_region[0] < 0 || t.align_region[1] < 0) continue;
      for (int k = 0; k < 2; ++k)
      {
        if (t.align_region[k] != cur) continue;
        const int nxt = t.align_region[1 - k];
        if (nxt < 0 || nxt >= n || visited[nxt]) continue;
        visited[nxt] = true;
        prev_region[nxt] = cur;
        prev_tunnel[nxt] = static_cast<int>(i);
        if (nxt == to)
        {
          // 回溯出按通过顺序排列的洞序列
          std::vector<int> rev;
          int node = to;
          while (node != from && node >= 0)
          {
            rev.push_back(prev_tunnel[node]);
            node = prev_region[node];
          }
          route.assign(rev.rbegin(), rev.rend());
          return route;
        }
        queue.push_back(nxt);
      }
    }
  }
  return route;  // 不可达，空
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
  const BasePose p = ports.base_pose();
  s.pos = Vec2{p.x, p.y};
  // 位姿新鲜度：odom 断流（超过 odom_stale_ms 没更新）后不能继续用最后一帧位置
  // 判区域/到达/洞口——过期数据当无效处理，决策自然退到安全路径（区域保记账、
  // 到达只认导航 success、洞口保护不判）。
  const double pose_age_ms = now_s * 1000.0 - static_cast<double>(p.stamp_ms);
  s.pos_valid = p.valid && pose_age_ms >= 0.0 && pose_age_ms <= cfg.odom_stale_ms;
  s.nav_state = ports.nav_state();
  return s;
}


// ---------------- 决策树 ----------------
namespace
{
// 巡逻点候选：所有 patrol 分类、当前区域可达、不贴闸线的点（顺序 = 点表顺序 = 轮换顺序）。
std::vector<const PointDef*> patrol_candidates(const Config& cfg, int cur_region)
{
  std::vector<const PointDef*> cand;
  for (const auto& p : cfg.points)
  {
    if (p.zone != kZonePatrol) continue;
    if (!point_usable(cfg, cur_region, p)) continue;
    cand.push_back(&p);
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
}  // namespace

// ---------------- 操作手指令 ----------------
void handle_operator(Ports& ports, const Config& cfg, PlanState& plan)
{
  const OpCmd c = ports.op_cmd();
  if (c.seq == 0 || c.seq == plan.op_seq_seen) return;  // 无指令 / 处理过了
  plan.op_seq_seen = c.seq;
  if (c.type == 4)
  {
    // 指点：点表里存在才接。即使与当前目标同一点也置 op_fresh——操作手再按一次
    // 就是"强制重发导航目标"（卡死拉起来），这就是导航目标更新的处理。
    if (find_point(cfg, c.param) != nullptr)
    {
      plan.op_goal = c.param;
      plan.op_fresh = true;
    }
  }
  // 其余类型（买弹/买血/买复活等）：不是决策层的事，记下 seq 即忽略——
  // 操作手经裁判客户端/电控直接办，到不到账从裁判数据里看。
}

// ---------------- 自动购买 ----------------
void run_auto_buy(Ports& ports, const Config& cfg, const Sense& s, PlanState& plan,
                  double now)
{
  // 比赛外不下单（比赛外一条消息不发；备赛阶段的采购是操作手手动的事）
  if (!cfg.enable_auto_buy || !s.match_running) return;

  // 买弹：弹量跌破阈值且金币够
  if (cfg.ammo_buy_min_shots >= 0 && s.shots >= 0 && s.shots < cfg.ammo_buy_min_shots &&
      s.coins >= cfg.ammo_buy_min_coins && now >= plan.buy_ammo_next_s)
  {
    ports.buy_ammo(cfg.ammo_buy_rounds);
    plan.buy_ammo_next_s = now + cfg.buy_cooldown_s;
  }
  // 买血：活着且血量比例跌破阈值且金币够。阈值故意比回家锁存高——先远程补一口，
  // 补不上（冷却过了还没涨）再靠 hp_home_ratio 锁存走回家那条路。
  if (cfg.hp_buy_ratio > 0.0 && s.hp > 0 && s.hp_ratio < cfg.hp_buy_ratio &&
      s.coins >= cfg.hp_buy_min_coins && now >= plan.buy_hp_next_s)
  {
    ports.buy_hp(cfg.hp_buy_amount);
    plan.buy_hp_next_s = now + cfg.buy_cooldown_s;
  }
  // 买复活：hp == 0 即阵亡（死没死不需要单独知道），金币够就下单。
  // 死亡期间回家锁存本来就保持着，复活（残血/满血）后自然从原地接着走。
  if (cfg.revive_buy_min_coins >= 0 && s.hp == 0 &&
      s.coins >= cfg.revive_buy_min_coins && now >= plan.buy_revive_next_s)
  {
    ports.buy_revive();
    plan.buy_revive_next_s = now + cfg.buy_cooldown_s;
  }
}

// ---------------- 决策树 ----------------
Decision decide_tree(const Sense& s, const Config& cfg, PlanState& plan, double now_s,
                     int nav_goal_for)
{
  Decision d;
  d.stamp_ms = static_cast<uint32_t>(now_s * 1000.0);

  // 回家锁存（血量 OR 弹量触发；两个都恢复才解除）：
  //   低血或低弹都该回家（回血/补给都在家），双条件解除防止"补了血没补弹"
  //   就出门又立刻被弹量阈值拉回来。弹量未知(-1)或阈值关掉(<0)不参与判定。
  //   死亡期间 hp=0 → 锁存自然保持；复活残血 → 从原地接着回家。
  const bool hp_low = s.hp_ratio < cfg.hp_home_ratio;
  const bool hp_ok = s.hp_ratio >= cfg.hp_resume_ratio;
  const bool ammo_watch = cfg.ammo_home_min >= 0 && s.shots >= 0;
  const bool ammo_low = ammo_watch && s.shots < cfg.ammo_home_min;
  const bool ammo_ok = !ammo_watch || s.shots >= cfg.ammo_resume_min;
  if (!plan.home_latched && (hp_low || ammo_low)) plan.home_latched = true;
  else if (plan.home_latched && hp_ok && ammo_ok) plan.home_latched = false;

  // 0) 操作手指点：最高优先级（人比规则大），不走换点防抖——立即生效。
  //    到达即清除（op_fresh 期间不信导航 success：那是上一个目标的到达回执），
  //    之后自动恢复 1)/2) 的正常决策；锁存状态在上面照常更新，指完点该回家回家。
  if (plan.op_goal >= 0)
  {
    const PointDef* op = find_point(cfg, plan.op_goal);
    if (op == nullptr)
    {
      plan.op_goal = -1;  // 点表改过、点没了：丢弃指点
    }
    else if (arrived_at(s, cfg, *op, plan.op_fresh ? -2 : nav_goal_for))
    {
      plan.op_goal = -1;   // 到了：消费掉这条指点
      plan.op_fresh = false;
    }
    else
    {
      plan.cur_point = op->id;  // 直接生效（覆盖防抖记账，恢复巡逻时会重选最近点）
      plan.cur_since_s = now_s;
      plan.cur_arrived = false;
      plan.pending_point = -1;
      plan.pending_count = 0;
      d.target_point = plan.cur_point;
      return d;
    }
  }

  const PointDef* home = find_point_by_zone(cfg, kZoneHome);
  const int home_id = (home != nullptr && point_usable(cfg, plan.current_region, *home))
                          ? home->id
                          : -1;

  int want = -1;
  // 1) 回家锁存
  if (plan.home_latched) want = home_id;
  // 2) 巡逻
  if (want < 0) want = pick_patrol(s, cfg, plan, now_s, nav_goal_for);

  // 换点防抖：新目标需连续 switch_confirm_frames 帧一致，且当前点驻留满
  // switch_dwell_s（或已到达）才切换；第一个点（cur_point < 0）立即生效。
  if (want == plan.cur_point)
  {
    plan.pending_point = -1;
    plan.pending_count = 0;
  }
  else
  {
    if (want != plan.pending_point)
    {
      plan.pending_point = want;
      plan.pending_count = 1;
    }
    else if (plan.pending_count < cfg.switch_confirm_frames)
    {
      ++plan.pending_count;
    }
    const bool dwell_ok = plan.cur_point < 0 || plan.cur_arrived ||
                          (now_s - plan.cur_since_s) >= cfg.switch_dwell_s;
    if (plan.cur_point < 0 ||
        (plan.pending_count >= cfg.switch_confirm_frames && dwell_ok))
    {
      plan.cur_point = want;
      plan.cur_since_s = now_s;
      plan.cur_arrived = false;
      plan.pending_point = -1;
      plan.pending_count = 0;
    }
  }
  d.target_point = plan.cur_point;
  return d;
}

// ---------------- 过洞五段状态机 ----------------
bool TunnelRunner::start(Ports& ports, const Config& cfg, int tid, int from_region,
                         const Vec2* from, const PointDef& target, double now)
{
  const TunnelDef& t = cfg.tunnels[tid];
  if (dist(t.align[0], t.align[1]) < 1e-6 ||
      t.align_region[0] < 0 || t.align_region[1] < 0)
  {
    // 对正点重合 / 没绑区域：洞定义不完整，冷却避开（配置问题，日志由节点层报）
    block_until_s = now + cfg.tunnel_retry_cooldown_s;
    return false;
  }
  const int fstate = ports.fold_state();
  if (fstate == kFoldUnknown)
  {
    // 折叠状态未知：无法确认"已折叠就绪"，拒绝穿洞（宁可不去，不可裸穿撞云台）
    ports.base_stop();
    ports.nav_cancel();
    block_until_s = now + cfg.tunnel_retry_cooldown_s;
    return false;
  }

  // 入口 = 绑定"车当前区域"的对正点；出口 = 另一个。对不上就拒绝，不做几何猜测——
  // 猜错入口会在洞口另一侧，导航过去必然穿过洞体（还没折叠），云台就撞了。
  int entry_idx = -1;
  if (t.align_region[0] == from_region) entry_idx = 0;
  else if (t.align_region[1] == from_region) entry_idx = 1;
  if (entry_idx < 0)
  {
    ports.base_stop();
    ports.nav_cancel();
    block_until_s = now + cfg.tunnel_retry_cooldown_s;
    return false;
  }
  const int exit_idx = 1 - entry_idx;
  entry = t.align[entry_idx];
  exit = t.align[exit_idx];
  exit_region = t.align_region[exit_idx];
  tunnel = tid;
  after_point = target.id;

  // 车已贴近洞轴中段（洞口/洞内，仅位姿有效时判定）：未折叠绝不能沿洞轴再动，
  // 已折叠则只许前进到出口（不许倒车）。
  bool in_mouth = false;
  if (from != nullptr)
  {
    const double tf = project_param(*from, t.align[0], t.align[1]);
    in_mouth = tf >= 0.0 && tf <= 1.0 &&
               dist_to_segment(*from, t.align[0], t.align[1]) < cfg.in_tunnel_margin_m;
  }
  if (in_mouth && fstate != kFoldFolded)
  {
    ports.base_stop();
    ports.nav_cancel();
    block_until_s = now + cfg.tunnel_retry_cooldown_s;
    return false;
  }
  phase = (in_mouth && fstate == kFoldFolded) ? kNavExit : kNavEntry;
  phase_since_s = now;
  return true;  // 第一个动作（发入口/出口导航目标）由调用方紧接着调 step() 执行
}

void TunnelRunner::step(Ports& ports, const Config& cfg, PlanState& plan, double now)
{
  if (phase == kOff) return;
  ports.base_spin(0.0);  // 过洞期间禁止自旋（云台/底盘姿态必须稳定）

  switch (phase)
  {
    case kNavEntry:
      ports.nav_set_goal(entry.x, entry.y);
      phase = kWaitEntry;
      phase_since_s = now;
      break;
    case kWaitEntry:
    {
      const GoalState g = ports.nav_state();
      if (g == kGoalSuccess)
      {
        phase = kFold;  // 到入口 → 折叠
        phase_since_s = now;
      }
      else if (g == kGoalFailure || now - phase_since_s > cfg.nav_timeout_s)
      {
        abort(ports, cfg, now);
      }
      break;
    }
    case kFold:
      ports.fold_set(true);  // 通知电控折叠云台
      phase = kWaitFold;
      phase_since_s = now;
      break;
    case kWaitFold:
      if (ports.fold_state() == kFoldFolded)
      {
        phase = kNavExit;  // 折叠到位 → 穿洞
        phase_since_s = now;
      }
      else if (now - phase_since_s > cfg.fold_timeout_s)
      {
        abort(ports, cfg, now);
      }
      break;
    case kNavExit:
      ports.nav_set_goal(exit.x, exit.y);
      phase = kWaitExit;
      phase_since_s = now;
      break;
    case kWaitExit:
    {
      const GoalState g = ports.nav_state();
      if (g == kGoalSuccess)
      {
        ports.base_stop();  // 到出口先停住再展开
        phase = kUnfold;
        phase_since_s = now;
      }
      else if (g == kGoalFailure || now - phase_since_s > cfg.nav_timeout_s)
      {
        abort(ports, cfg, now);
      }
      break;
    }
    case kUnfold:
      ports.fold_set(false);  // 出洞后展开
      phase = kWaitUnfold;
      phase_since_s = now;
      break;
    case kWaitUnfold:
      if (ports.fold_state() == kFoldUnfolded)
      {
        // 展开完成：区域记账改成出口对正点绑定的区域（纯记账的唯一更新点），
        // 交回常规导航。
        if (exit_region >= 0) plan.current_region = exit_region;
        phase = kOff;
        tunnel = -1;
        after_point = -1;
      }
      else if (now - phase_since_s > cfg.fold_timeout_s)
      {
        abort(ports, cfg, now);
      }
      break;
    case kOff:
    default:
      break;
  }
}

void TunnelRunner::abort(Ports& ports, const Config& cfg, double now)
{
  ports.base_stop();
  ports.nav_cancel();
  phase = kOff;
  tunnel = -1;
  after_point = -1;
  block_until_s = now + cfg.tunnel_retry_cooldown_s;
  // 不写区域记账（中止 ≠ 已到出口）；折叠状态故意不动：中止 ≠ 已离开洞口
}

// ---------------- 执行器 ----------------
namespace
{
// 停车 + 取消导航 + 清下发记账。
void stop_and_cancel(Ports& ports, ExecState& ex)
{
  ports.base_stop();
  ports.nav_cancel();
  ex.applied_goal = -2;
}
}  // namespace

bool tunnel_handled(Ports& ports, const Config& cfg, const Sense& s, const Decision& d,
                    PlanState& plan, ExecState& ex, double now)
{
  if (!cfg.enable_tunnel) return false;  // 过洞总开关关闭：整套过洞/区域逻辑不存在
  // 过洞流程进行中：状态机独占执行权。比赛停/阵亡/目标被清 → 中止（折叠保持）。
  if (ex.tunnel.active())
  {
    if (!s.match_running || d.target_point < 0)
    {
      ex.tunnel.abort(ports, cfg, now);
    }
    else
    {
      ex.tunnel.step(ports, cfg, plan, now);
    }
    return true;
  }
  if (d.target_point < 0) return false;  // 空目标：交给 nav_cruise 停车
  const PointDef* p = find_point(cfg, d.target_point);
  if (p == nullptr) return false;

  // 目标点在别的区域 → 必须过洞（绝不裸穿：没折叠就发跨区域目标等于撞云台）。
  // 按区域路由取第一条要过的洞；过完一条回到这里重新评估，自动串联。
  if (p->region >= 0 && p->region != plan.current_region)
  {
    const auto route = route_tunnels(cfg, plan.current_region, p->region);
    if (route.empty())
    {
      stop_and_cancel(ports, ex);  // 跨区域却无洞路：决策不该选出这种点，停车保底
      return true;
    }
    const int tid = route.front();
    if (now < ex.tunnel.block_until_s)
    {
      stop_and_cancel(ports, ex);  // 失败冷却期：停车等，不绕、不硬闯
      return true;
    }
    if (ex.tunnel.start(ports, cfg, tid, plan.current_region,
                        s.pos_valid ? &s.pos : nullptr, *p, now))
    {
      ex.tunnel.step(ports, cfg, plan, now);  // 立刻执行第一个动作（发入口导航目标）
      return true;
    }
    stop_and_cancel(ports, ex);  // start 拒绝（区域对不上/洞口未折叠，自带冷却）
    return true;
  }
  return false;  // 同区域目标：与过洞无关
}

void nav_cruise(Ports& ports, const Config& cfg, const Sense& s, int target_point,
                ExecState& ex, double now, bool& force_resend)
{
  // 本段 = 非过洞段：受击旋转恒开（含巡逻段；过洞段在 TunnelRunner 里恒发 0）。
  ports.base_spin(cfg.spin_w);

  // 待机分支：目标为空 / 点不存在
  if (target_point < 0)
  {
    stop_and_cancel(ports, ex);
    return;
  }
  const PointDef* p = find_point(cfg, target_point);
  if (p == nullptr)
  {
    stop_and_cancel(ports, ex);
    return;
  }

  // 常规导航：换点（或操作手重指点 force_resend——目标没变也取消重发一次，
  // 这是导航目标更新的处理）先取消旧 Goal；失败按 nav_retry_s 节奏重试
  // （驻留超时后决策自然会换点）。
  if (ex.applied_goal != p->id || force_resend)
  {
    if (now < ex.retry_until_s)
    {
      ports.base_stop();
      return;
    }
    if (ex.applied_goal >= 0 || force_resend) ports.nav_cancel();
    if (ports.nav_set_goal(p->pos.x, p->pos.y))
    {
      ex.applied_goal = p->id;
      force_resend = false;  // 已按新指令发出，消费掉
    }
    else
    {
      ex.applied_goal = -2;
      ex.retry_until_s = now + cfg.nav_retry_s;
    }
    return;
  }
  if (s.nav_state == kGoalFailure)
  {
    ports.nav_cancel();
    ex.applied_goal = -2;
    ex.retry_until_s = now + cfg.nav_retry_s;
    ports.base_stop();
  }
}

void run_executor(Ports& ports, const Config& cfg, const Sense& s, const Decision& d,
                  PlanState& plan, ExecState& ex, double now)
{
  // 比赛门控（沿触发，与 BT 的 IsMatchRunning→Standby 同一语义）：
  //   结束的那一拍收尾一次——终止过洞流程（只清状态，不动折叠：中止 ≠ 已离开
  //   洞口）、取消导航、停自旋；**不**发停车指令（没必要为比赛外停车专门留一条
  //   给电控的口——取消导航后底盘自己停）。之后完全静默：比赛外一条消息不发。
  //   开始的那一拍切一次底盘模式（模式也在比赛外不发）。
  if (!s.match_running)
  {
    if (!ex.standby_cleaned)
    {
      ex.tunnel.phase = TunnelRunner::kOff;
      ex.tunnel.tunnel = -1;
      ex.tunnel.after_point = -1;
      ports.nav_cancel();
      ports.base_spin(0.0);
      ex.applied_goal = -2;
      ex.standby_cleaned = true;
    }
    ex.match_on = false;
    return;
  }
  if (!ex.match_on)
  {
    ex.match_on = true;
    ex.standby_cleaned = false;
    ports.base_set_mode(cfg.mode_independent);
    if (cfg.enable_tunnel)
    {
      plan.current_region = cfg.start_region;  // 开赛沿重置记账（新开赛车摆回开局位）
    }
  }
  if (tunnel_handled(ports, cfg, s, d, plan, ex, now)) return;  // 过洞事务独占本 tick
  nav_cruise(ports, cfg, s, d.target_point, ex, now, plan.op_fresh);
}

int step(Ports& ports, const Config& cfg, PlanState& plan, ExecState& ex, double now)
{
  const Sense s = sense(ports, cfg, now);

  // 操作手指令（只消费"指点"，买类忽略）+ 自动购买（决策层自己下单，不动目标点）；
  // 运动部分由 run_executor 的门控把关（比赛外不产生任何运动消息，自动买同理）。
  handle_operator(ports, cfg, plan);
  run_auto_buy(ports, cfg, s, plan, now);

  // 区域不再每 tick 解析：纯记账（开赛沿重置 + 过洞完成写出口，见文件头），
  // plan.current_region 在这两处之外不变。

  // 过洞中执行器发的是对正点目标：告诉决策树"当前导航目标不属于任何巡逻点"，
  // 防止对正点的 success 被误当成巡逻到点而跳点。
  const int nav_goal_for = ex.tunnel.active() ? -2 : ex.applied_goal;
  const Decision d = decide_tree(s, cfg, plan, now, nav_goal_for);
  run_executor(ports, cfg, s, d, plan, ex, now);
  return ex.tunnel.active() ? ex.tunnel.after_point : d.target_point;
}

}  // namespace bbt_decide
