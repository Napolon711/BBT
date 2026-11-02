// Production behavior-tree leaves. Priority and workflow composition live in main.xml.
#include "bbt_decide/bt.hpp"
#include "bbt_decide/geometry.hpp"

#include <functional>
#include <utility>

namespace bbt_decide
{
namespace
{
using Status = BT::NodeStatus;
using Purpose = NavigationPurpose;

class CallbackAction : public BT::StatefulActionNode
{
public:
  CallbackAction(const std::string& name, const BT::NodeConfig& config,
                 std::function<Status()> callback)
      : BT::StatefulActionNode(name, config), callback_(std::move(callback)) {}
  static BT::PortsList providedPorts() { return {}; }
private:
  Status onStart() override { return callback_(); }
  Status onRunning() override { return callback_(); }
  void onHalted() override {} // Blackboard physical evidence outlives a reactive halt.
  std::function<Status()> callback_;
};

double now(const Context& c) { return c.ports->now_s(); }
Status check(bool condition) { return condition ? Status::SUCCESS : Status::FAILURE; }

void forget_navigation(Context& c)
{
  c.ex.navigation = {};
  c.ex.applied_goal = -2;
  c.ex.applied_generation = 0;
}

void explicit_stop(Context& c)
{
  if (!c.ex.stop_sent)
  {
    c.ports->nav_cancel();
    c.ports->base_spin(0.0);
    c.ports->base_stop();
    c.ports->attack_cmd(0, 0, 0, 0);
    c.ex.stop_sent = true;
  }
  forget_navigation(c);
}

void cancel_owned_navigation(Context& c)
{
  if (c.ex.navigation.active()) c.ports->nav_cancel();
  forget_navigation(c);
}

void tactical_outputs(Context& c)
{
  c.ports->base_spin(c.cfg.spin_w);
  if (c.decision.target.zone == kZoneOutpost) c.ports->attack_cmd(2, 1, 1, 0);
  else c.ports->attack_cmd(0, 0, 1, 1);
  c.ex.stop_sent = false;
}

bool owns_ordinary_request(const Context& c)
{
  return c.ex.navigation.purpose == Purpose::ordinary &&
         c.ex.navigation.target_point == c.decision.target_point &&
         c.ex.navigation.generation == c.decision.generation;
}

void set_request(Context& c, Purpose purpose, int target, uint64_t generation)
{
  c.ex.navigation = {purpose, target, generation, now(c)};
  c.ex.applied_goal = purpose == Purpose::ordinary ? target : -2;
  c.ex.applied_generation = generation;
  c.ex.goal_since_s = now(c);
  c.ex.stop_sent = false;
}

void ordinary_failure(Context& c)
{
  c.ex.retry_target = c.decision.target_point;
  c.ex.retry_generation = c.decision.generation;
  c.ex.retry_until_s = now(c) + c.cfg.nav_retry_s;
  c.ex.stop_sent = false;
  explicit_stop(c);
}

void tunnel_failure(Context& c)
{
  remember_tunnel_progress(c.sense, c.ex.tunnel);
  const double blocked_until = now(c) + c.cfg.tunnel_retry_cooldown_s;
  if (!c.ex.tunnel.crossing_started) c.ex.tunnel = {};
  c.ex.tunnel.block_until_s = blocked_until;
  c.ex.tunnel.fold_requested_s = -1.0;
  c.ex.tunnel.unfold_requested_s = -1.0;
  c.ex.stop_sent = false;
  explicit_stop(c);
}

void select_point(Context& c, const PointDef* target, bool is_operator = false)
{
  c.decision = {};
  c.decision.stamp_ms = static_cast<uint64_t>(now(c) * 1000.0);
  if (!target) return;
  c.decision.target = *target;
  c.decision.target_point = target->id;
  c.decision.target_valid = true;
  c.decision.operator_target = is_operator;
  c.decision.generation = is_operator ? c.plan.op_generation : 0;
}

void consume_operator(Context& c)
{
  c.plan.op_active = false;
  c.plan.op_goal = -1;
  c.plan.op_arrived_s = -1.0;
  c.plan.op_clear_since_s = -1.0;
}

Status read_inputs(Context& c)
{
  c.sense = sense(*c.ports, c.cfg, now(c));
  handle_operator(*c.ports, c.cfg, c.plan);
  return Status::SUCCESS;
}

Status begin_match(Context& c)
{
  if (!c.ex.match_on)
  {
    c.ex.match_on = true;
    c.ex.standby_cleaned = false;
    c.ex.stop_sent = false;
    c.ports->base_set_mode(c.cfg.mode_independent);
    if (c.cfg.enable_tunnel) c.plan.current_region = c.cfg.start_region;
  }
  return Status::SUCCESS;
}

Status standby(Context& c)
{
  if (!c.ex.standby_cleaned)
  {
    c.ex.stop_sent = false;
    explicit_stop(c);
    c.ex.tunnel = {};
    c.ex.retry_until_s = 0.0;
    c.plan.cur_point = -1;
    c.plan.cur_target_valid = false;
    c.plan.pending_point = -1;
    c.plan.pending_count = 0;
    consume_operator(c);
    c.ex.standby_cleaned = true;
  }
  // Consume buffered pre-match commands, but never resurrect a consumed task.
  consume_operator(c);
  c.decision = {};
  c.ex.match_on = false;
  return Status::RUNNING;
}

Status observe_operator(Context& c)
{
  if (!c.plan.op_active) return Status::SUCCESS;
  const bool own_final = c.ex.navigation.purpose == Purpose::ordinary &&
      c.ex.navigation.target_point == c.plan.op_target.id &&
      c.ex.navigation.generation == c.plan.op_generation;
  if (c.plan.op_arrived_s < 0.0 && own_final && c.sense.nav_state == kGoalSuccess)
    c.plan.op_arrived_s = now(c);
  if (c.plan.op_arrived_s >= 0.0 && c.sense.out_of_combat == 1)
  {
    if (c.plan.op_clear_since_s < 0.0) c.plan.op_clear_since_s = now(c);
  }
  else c.plan.op_clear_since_s = -1.0;
  return Status::SUCCESS;
}

Status commit_target(Context& c)
{
  auto& p = c.plan;
  const auto& candidate = c.decision;
  const bool same = p.cur_target_valid == candidate.target_valid &&
      (!candidate.target_valid ||
       (p.cur_point == candidate.target_point && p.cur_generation == candidate.generation &&
        p.cur_operator_target == candidate.operator_target));
  bool accept = same || candidate.operator_target || !p.cur_target_valid;
  if (!same && !accept)
  {
    if (p.pending_point != candidate.target_point)
    {
      p.pending_point = candidate.target_point;
      p.pending_count = 1;
    }
    else if (p.pending_count < c.cfg.switch_confirm_frames) ++p.pending_count;
    accept = p.pending_count >= c.cfg.switch_confirm_frames &&
        (p.cur_arrived || now(c) - p.cur_since_s >= c.cfg.switch_dwell_s);
  }
  if (accept && !same)
  {
    p.cur_point = candidate.target_point;
    p.cur_target = candidate.target;
    p.cur_target_valid = candidate.target_valid;
    p.cur_operator_target = candidate.operator_target;
    p.cur_generation = candidate.generation;
    p.cur_since_s = now(c);
    p.cur_arrived = false;
  }
  if (accept)
  {
    p.pending_point = -1;
    p.pending_count = 0;
  }
  c.decision.target_point = p.cur_point;
  c.decision.target = p.cur_target;
  c.decision.target_valid = p.cur_target_valid;
  c.decision.operator_target = p.cur_operator_target;
  c.decision.generation = p.cur_generation;
  return Status::SUCCESS;
}

bool needs_tunnel(const Context& c)
{
  return c.cfg.enable_tunnel && (c.ex.tunnel.active() ||
      (c.decision.target_valid && c.decision.target.region != c.plan.current_region));
}

Status prepare_tunnel(Context& c)
{
  auto& t = c.ex.tunnel;
  // Before crossing is committed, a new target may choose a different first tunnel.
  if (t.active() && !t.crossing_started &&
      (t.after_point != c.decision.target_point || t.generation != c.decision.generation))
  {
    cancel_owned_navigation(c);
    const double cooldown = t.block_until_s;
    t = {};
    t.block_until_s = cooldown;
  }
  if (t.active())
  {
    remember_tunnel_progress(c.sense, t);
    return Status::SUCCESS;
  }
  if (now(c) < t.block_until_s || !c.sense.pos_valid || !c.decision.target_valid)
    return Status::FAILURE;
  const auto route = route_tunnels(c.cfg, c.plan.current_region, c.decision.target.region,
                                   &c.sense.pos);
  if (route.empty()) return Status::FAILURE;
  const auto& definition = c.cfg.tunnels[route.front()];
  const int side = definition.align_region[0] == c.plan.current_region ? 0 :
                   definition.align_region[1] == c.plan.current_region ? 1 : -1;
  const int geometric_region = region_for_position(c.cfg, c.sense.pos);
  if (side < 0 || dist(definition.align[0], definition.align[1]) < 1e-6 ||
      c.ports->fold_state() == kFoldUnknown ||
      (geometric_region >= 0 && geometric_region != c.plan.current_region))
  {
    tunnel_failure(c);
    return Status::FAILURE;
  }
  const double along = project_param(c.sense.pos, definition.align[0], definition.align[1]);
  const bool in_mouth = along >= 0.0 && along <= 1.0 &&
      dist_to_segment(c.sense.pos, definition.align[0], definition.align[1]) <
          c.cfg.in_tunnel_margin_m;
  if (in_mouth && c.ports->fold_state() != kFoldFolded)
  {
    tunnel_failure(c);
    return Status::FAILURE;
  }
  cancel_owned_navigation(c);
  t.tunnel = route.front();
  t.after_point = c.decision.target_point;
  t.generation = c.decision.generation;
  t.entry = definition.align[side];
  t.exit = definition.align[1 - side];
  t.center = definition.center;
  t.has_center = definition.has_center;
  t.exit_region = definition.align_region[1 - side];
  t.crossing_started = in_mouth;
  t.center_reached = !t.has_center;
  if (in_mouth) t.entry_arrived_s = now(c);
  remember_tunnel_progress(c.sense, t);
  return Status::SUCCESS;
}

Status navigate_entry(Context& c)
{
  auto& t = c.ex.tunnel;
  if (t.entry_arrived_s >= 0.0) return Status::SUCCESS;
  if (c.ex.navigation.purpose != Purpose::tunnel_entry)
  {
    cancel_owned_navigation(c);
    if (!c.ports->nav_set_goal(t.entry.x, t.entry.y))
    {
      tunnel_failure(c);
      return Status::RUNNING;
    }
    set_request(c, Purpose::tunnel_entry, -2, t.generation);
    return Status::RUNNING;
  }
  const auto state = c.ports->nav_state();
  if (state == kGoalSuccess)
  {
    t.entry_arrived_s = now(c);
    return Status::SUCCESS;
  }
  if (state == kGoalFailure || now(c) - c.ex.navigation.started_s >= c.cfg.nav_timeout_s)
    tunnel_failure(c);
  return Status::RUNNING;
}

Status request_fold(Context& c, bool fold)
{
  auto& t = c.ex.tunnel;
  double& requested_s = fold ? t.fold_requested_s : t.unfold_requested_s;
  if (requested_s < 0.0)
  {
    if (!c.ports->fold_set(fold))
    {
      tunnel_failure(c);
      return Status::RUNNING;
    }
    requested_s = now(c);
  }
  else if (now(c) - requested_s >= c.cfg.fold_timeout_s) tunnel_failure(c);
  return Status::RUNNING;
}

Status traverse_tunnel(Context& c)
{
  auto& t = c.ex.tunnel;
  if (c.ex.navigation.purpose != Purpose::tunnel_crossing)
  {
    cancel_owned_navigation(c);
    // A one-pose recovery is still an authorized folded traversal, not ordinary navigation.
    const std::vector<Vec2> poses = t.has_center && !t.center_reached
        ? std::vector<Vec2>{t.center, t.exit} : std::vector<Vec2>{t.exit};
    if (!c.ports->nav_through_poses(poses))
    {
      tunnel_failure(c);
      return Status::RUNNING;
    }
    // A synchronous rejection did not submit a task. Actual occupancy, if known,
    // was already recorded by PrepareTunnel; accepted submissions commit ownership.
    t.crossing_started = true;
    set_request(c, Purpose::tunnel_crossing, -2, t.generation);
    return Status::RUNNING;
  }
  const auto state = c.ports->nav_state();
  if (state == kGoalSuccess)
  {
    t.exit_reached = true;
    c.plan.current_region = t.exit_region;
    explicit_stop(c);
  }
  else if (state == kGoalFailure || now(c) - c.ex.navigation.started_s >= c.cfg.nav_timeout_s)
    tunnel_failure(c);
  return Status::RUNNING;
}

Status submit_ordinary(Context& c)
{
  cancel_owned_navigation(c);
  if (!c.ports->nav_set_goal(c.decision.target.pos.x, c.decision.target.pos.y))
  {
    ordinary_failure(c);
    return Status::RUNNING;
  }
  set_request(c, Purpose::ordinary, c.decision.target_point, c.decision.generation);
  tactical_outputs(c);
  return Status::RUNNING;
}
} // namespace

void register_nodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<Context> ctx)
{
  const auto action = [&](const std::string& name, std::function<Status(Context&)> callback) {
    factory.registerBuilder<CallbackAction>(name,
        [ctx, callback](const std::string& node_name, const BT::NodeConfig& config) {
          return std::make_unique<CallbackAction>(node_name, config,
              [ctx, callback] { return callback(*ctx); });
        });
  };
  const auto condition = [&](const std::string& name, std::function<bool(const Context&)> callback) {
    factory.registerSimpleCondition(name, [ctx, callback](BT::TreeNode&) {
      return check(callback(*ctx));
    });
  };
  action("ReadInputs", read_inputs);
  condition("MatchRunning", [](const Context& c) { return c.sense.match_running; });
  action("BeginMatch", begin_match);
  action("Standby", standby);
  action("UpdateHomeLatch", [](Context& c) {
    update_home_latch(c.sense, c.cfg, c.plan); return Status::SUCCESS;
  });
  action("ObserveOperator", observe_operator);
  condition("AutoBuyEnabled", [](const Context& c) { return c.cfg.enable_auto_buy; });
  condition("AmmoPurchaseDue", [](const Context& c) {
    return c.cfg.ammo_buy_min_shots >= 0 && c.sense.shots >= 0 &&
      c.sense.shots < c.cfg.ammo_buy_min_shots && c.sense.coins >= c.cfg.ammo_buy_min_coins &&
      c.sense.remote_ammo_pool != 0 && now(c) >= c.plan.buy_ammo_next_s;
  });
  action("BuyAmmo", [](Context& c) {
    c.ports->buy_ammo(c.cfg.ammo_buy_rounds);
    c.plan.buy_ammo_next_s = now(c) + c.cfg.buy_cooldown_s; return Status::SUCCESS;
  });
  condition("HpPurchaseDue", [](const Context& c) {
    return c.cfg.hp_buy_ratio > 0.0 && c.sense.hp > 0 &&
      c.sense.hp_ratio < c.cfg.hp_buy_ratio && c.sense.coins >= c.cfg.hp_buy_min_coins &&
      c.sense.out_of_combat != 0 && now(c) >= c.plan.buy_hp_next_s;
  });
  action("BuyHp", [](Context& c) {
    c.ports->buy_hp(c.cfg.hp_buy_amount);
    c.plan.buy_hp_next_s = now(c) + c.cfg.buy_cooldown_s; return Status::SUCCESS;
  });
  condition("RevivePurchaseDue", [](const Context& c) {
    return c.cfg.enable_auto_revive && c.sense.hp == 0 &&
      now(c) >= c.plan.buy_revive_next_s;
  });
  condition("FreeReviveAllowed", [](const Context& c) { return c.sense.revive_available == 1; });
  condition("PaidReviveAllowed", [](const Context& c) {
    return c.sense.revive_available == 2 && c.sense.revive_cost >= 0 &&
      c.sense.coins >= c.sense.revive_cost;
  });
  action("BuyRevive", [](Context& c) {
    c.ports->buy_revive(); c.plan.buy_revive_next_s = now(c) + c.cfg.buy_cooldown_s;
    return Status::SUCCESS;
  });
  condition("HasOperatorTarget", [](const Context& c) { return c.plan.op_active; });
  condition("OperatorHoldElapsed", [](const Context& c) {
    return c.plan.op_arrived_s >= 0.0 && now(c) - c.plan.op_arrived_s >= c.cfg.op_min_hold_s;
  });
  condition("OperatorClearElapsed", [](const Context& c) {
    return c.plan.op_clear_since_s >= 0.0 &&
      now(c) - c.plan.op_clear_since_s >= c.cfg.op_combat_clear_s;
  });
  condition("HomeLatched", [](const Context& c) { return c.plan.home_latched; });
  action("ConsumeOperator", [](Context& c) { consume_operator(c); return Status::SUCCESS; });
  action("SelectOperator", [](Context& c) { select_point(c, &c.plan.op_target, true); return Status::SUCCESS; });
  action("SelectHome", [](Context& c) {
    const auto* home = find_point_by_zone(c.cfg, kZoneHome);
    select_point(c, home && point_usable(c.cfg, c.plan.current_region, *home) ? home : nullptr);
    return Status::SUCCESS;
  });
  condition("OwnOutpostNeeds", [](const Context& c) { return c.cfg.enable_outpost && our_outpost_needs(c.sense); });
  condition("EnemyOutpostNeeds", [](const Context& c) { return c.cfg.enable_outpost && enemy_outpost_needs(c.sense); });
  action("SelectRetake", [](Context& c) {
    const auto* point = pick_zone_point(c.cfg, kZoneRetake, c.plan.current_region);
    if (!point) return Status::FAILURE;
    select_point(c, point); return Status::SUCCESS;
  });
  action("SelectOutpost", [](Context& c) {
    const auto* point = pick_zone_point(c.cfg, kZoneOutpost, c.plan.current_region);
    if (!point) return Status::FAILURE;
    select_point(c, point); return Status::SUCCESS;
  });
  action("SelectPatrol", [](Context& c) {
    const int owned = c.ex.navigation.purpose == Purpose::ordinary ? c.ex.applied_goal : -2;
    select_point(c, find_point(c.cfg, pick_patrol(c.sense, c.cfg, c.plan, now(c), owned)));
    return Status::SUCCESS;
  });
  action("CommitTarget", commit_target);
  condition("NeedsTunnel", needs_tunnel);
  action("PrepareTunnel", prepare_tunnel);
  action("TunnelQuiet", [](Context& c) {
    c.ports->base_spin(0.0); c.ports->attack_cmd(0, 0, 0, 0); return Status::SUCCESS;
  });
  condition("TunnelCooldown", [](const Context& c) { return now(c) < c.ex.tunnel.block_until_s; });
  condition("TunnelExitReached", [](const Context& c) { return c.ex.tunnel.exit_reached; });
  condition("CrossingCommitted", [](const Context& c) { return c.ex.tunnel.crossing_started; });
  condition("CrossingNavigationActive", [](const Context& c) {
    return c.ex.navigation.purpose == Purpose::tunnel_crossing;
  });
  action("FailTunnel", [](Context& c) { tunnel_failure(c); return Status::RUNNING; });
  condition("Folded", [](const Context& c) { return c.ports->fold_state() == kFoldFolded; });
  condition("Unfolded", [](const Context& c) { return c.ports->fold_state() == kFoldUnfolded; });
  action("NavigateEntry", navigate_entry);
  action("ExplicitStop", [](Context& c) { explicit_stop(c); return Status::SUCCESS; });
  action("WaitStopped", [](Context& c) { explicit_stop(c); return Status::RUNNING; });
  action("RequestFold", [](Context& c) { return request_fold(c, true); });
  action("RequestUnfold", [](Context& c) { return request_fold(c, false); });
  action("TraverseTunnel", traverse_tunnel);
  action("CompleteTunnel", [](Context& c) {
    c.ex.tunnel = {}; forget_navigation(c); return Status::RUNNING;
  });
  condition("HasTarget", [](const Context& c) { return c.decision.target_valid; });
  condition("OwnsOrdinaryRequest", owns_ordinary_request);
  condition("OrdinaryFailed", [](const Context& c) {
    const auto state = c.ports->nav_state();
    return state == kGoalFailure || (state != kGoalSuccess &&
      now(c) - c.ex.navigation.started_s >= c.cfg.nav_timeout_s);
  });
  action("FailOrdinary", [](Context& c) { ordinary_failure(c); return Status::RUNNING; });
  condition("OrdinaryRetryBlocked", [](const Context& c) {
    return c.ex.retry_target == c.decision.target_point &&
      c.ex.retry_generation == c.decision.generation && now(c) < c.ex.retry_until_s;
  });
  action("TacticalKeep", [](Context& c) { tactical_outputs(c); return Status::RUNNING; });
  action("SubmitOrdinary", submit_ordinary);
}

int step(Ports& ports, const Config& cfg, PlanState& plan, ExecState& ex, double)
{
  auto ctx = std::make_shared<Context>();
  ctx->cfg = cfg;
  ctx->ports = std::shared_ptr<Ports>(&ports, [](Ports*) {});
  ctx->plan = plan;
  ctx->ex = ex;
  BT::BehaviorTreeFactory factory;
  register_nodes(factory, ctx);
  auto tree = factory.createTreeFromFile(BBT_TREE_PATH);
  tree.tickOnce();
  plan = ctx->plan;
  ex = ctx->ex;
  return ex.tunnel.active() ? ex.tunnel.after_point : ctx->decision.target_point;
}
} // namespace bbt_decide
