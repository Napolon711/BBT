// Production leaves use only their declared typed XML ports. Workflow lives in main.xml.
#include "bbt_decide/bt.hpp"
#include "bbt_decide/geometry.hpp"

#include <functional>
#include <tuple>
#include <type_traits>
#include <utility>

namespace bbt_decide
{
namespace
{
using Status = BT::NodeStatus;
using Purpose = NavigationPurpose;
using Settings = std::shared_ptr<const Config>;
using IO = std::shared_ptr<Ports>;

class PortAction : public BT::StatefulActionNode
{
public:
  PortAction(const std::string& name, const BT::NodeConfig& config,
             std::function<Status(BT::TreeNode&)> callback)
      : BT::StatefulActionNode(name, config), callback_(std::move(callback)) {}
private:
  Status onStart() override { return callback_(*this); }
  Status onRunning() override { return callback_(*this); }
  void onHalted() override {} // Durable request/physical facts belong to the blackboard.
  std::function<Status(BT::TreeNode&)> callback_;
};

// Bind a typed callback to declared native ports. Outputs are published even when
// an action returns RUNNING, so a reactive halt cannot discard physical evidence.
template <typename T, BT::PortDirection Direction>
struct PortBinding
{
  using Value = T;
  std::string name;

  auto declaration() const
  {
    if constexpr (Direction == BT::PortDirection::INPUT) return BT::InputPort<T>(name);
    else if constexpr (Direction == BT::PortDirection::OUTPUT) return BT::OutputPort<T>(name);
    else return BT::BidirectionalPort<T>(name);
  }
  T read(BT::TreeNode& node) const
  {
    if constexpr (Direction == BT::PortDirection::OUTPUT) return T{};
    else
    {
      auto result = node.getInput<T>(name);
      if (!result) throw BT::RuntimeError(node.name(), ": input '", name, "': ", result.error());
      if constexpr (std::is_same_v<T, Settings> || std::is_same_v<T, IO>)
        if (!result.value()) throw BT::RuntimeError(node.name(), ": null input '", name, "'");
      return result.value();
    }
  }
  decltype(auto) argument(T& value) const
  {
    if constexpr (Direction == BT::PortDirection::INPUT) return std::as_const(value);
    else return (value);
  }
  void publish(BT::TreeNode& node, const T& value) const
  {
    if constexpr (Direction != BT::PortDirection::INPUT)
    {
      auto result = node.setOutput(name, value);
      if (!result) throw BT::RuntimeError(node.name(), ": output '", name, "': ", result.error());
    }
  }
};

template <typename T> auto input(const char* name)
{ return PortBinding<T, BT::PortDirection::INPUT>{name}; }
template <typename T> auto output(const char* name)
{ return PortBinding<T, BT::PortDirection::OUTPUT>{name}; }
template <typename T> auto inout(const char* name)
{ return PortBinding<T, BT::PortDirection::INOUT>{name}; }

template <typename Callback, typename Bindings, size_t... I>
Status invoke_ports(BT::TreeNode& node, const Callback& callback, const Bindings& bindings,
                    std::index_sequence<I...>)
{
  auto values = std::make_tuple(std::get<I>(bindings).read(node)...);
  const auto result = callback(std::get<I>(bindings).argument(std::get<I>(values))...);
  (std::get<I>(bindings).publish(node, std::get<I>(values)), ...);
  if constexpr (std::is_same_v<std::decay_t<decltype(result)>, bool>)
    return result ? Status::SUCCESS : Status::FAILURE;
  else return result;
}

template <typename Callback, typename... Bindings>
void register_leaf(BT::BehaviorTreeFactory& factory, const std::string& id,
                   BT::NodeType type, Callback callback, Bindings... bindings)
{
  BT::PortsList ports{bindings.declaration()...};
  auto invoke = [callback, bindings = std::make_tuple(bindings...)](BT::TreeNode& node) {
    return invoke_ports(node, callback, bindings, std::index_sequence_for<Bindings...>{});
  };
  if (type == BT::NodeType::CONDITION) factory.registerSimpleCondition(id, invoke, ports);
  else factory.registerBuilder({type, id, ports, {}},
      [invoke](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<PortAction>(name, config, invoke);
      });
}

void forget_navigation(ExecState& ex)
{
  ex.navigation = {};
  ex.applied_goal = -2;
  ex.applied_generation = 0;
}

void explicit_stop(Ports& io, ExecState& ex)
{
  if (!ex.stop_sent)
  {
    io.nav_cancel();
    io.base_spin(0.0);
    io.base_stop();
    io.attack_cmd(0, 0, 0, 0);
    ex.stop_sent = true;
  }
  forget_navigation(ex);
}

void cancel_owned_navigation(Ports& io, ExecState& ex)
{
  if (ex.navigation.active()) io.nav_cancel();
  forget_navigation(ex);
}

void tactical_outputs(const Config& cfg, Ports& io, ExecState& ex, const Decision& decision)
{
  io.base_spin(cfg.spin_w);
  if (decision.target.zone == kZoneOutpost) io.attack_cmd(2, 1, 1, 0);
  else io.attack_cmd(0, 0, 1, 1);
  ex.stop_sent = false;
}

bool owns_ordinary_request(const ExecState& ex, const Decision& decision)
{
  return ex.navigation.purpose == Purpose::ordinary &&
         ex.navigation.target_point == decision.target_point &&
         ex.navigation.generation == decision.generation;
}

void set_request(Ports& io, ExecState& ex, Purpose purpose, int target, uint64_t generation)
{
  ex.navigation = {purpose, target, generation, io.now_s()};
  ex.applied_goal = purpose == Purpose::ordinary ? target : -2;
  ex.applied_generation = generation;
  ex.goal_since_s = io.now_s();
  ex.stop_sent = false;
}

void ordinary_failure(const Config& cfg, Ports& io, ExecState& ex, const Decision& decision)
{
  ex.retry_target = decision.target_point;
  ex.retry_generation = decision.generation;
  ex.retry_until_s = io.now_s() + cfg.nav_retry_s;
  ex.stop_sent = false;
  explicit_stop(io, ex);
}

void tunnel_failure(const Config& cfg, Ports& io, const Sense& sensed, ExecState& ex)
{
  remember_tunnel_progress(sensed, ex.tunnel);
  const double blocked_until = io.now_s() + cfg.tunnel_retry_cooldown_s;
  if (!ex.tunnel.crossing_started) ex.tunnel = {};
  ex.tunnel.block_until_s = blocked_until;
  ex.tunnel.fold_requested_s = -1.0;
  ex.tunnel.unfold_requested_s = -1.0;
  ex.stop_sent = false;
  explicit_stop(io, ex);
}

void select_point(Ports& io, Decision& decision, const PointDef* target, uint64_t generation = 0, bool is_operator = false)
{
  decision = {};
  decision.stamp_ms = static_cast<uint64_t>(io.now_s() * 1000.0);
  if (!target) return;
  decision.target = *target;
  decision.target_point = target->id;
  decision.target_valid = true;
  decision.operator_target = is_operator;
  decision.generation = generation;
}

void consume_operator(PlanState& plan)
{
  plan.op_active = false;
  plan.op_goal = -1;
  plan.op_arrived_s = -1.0;
  plan.op_clear_since_s = -1.0;
}

Status read_inputs(const Config& cfg, Ports& io, PlanState& plan, Sense& sensed)
{
  sensed = sense(io, cfg, io.now_s());
  handle_operator(io, cfg, plan);
  return Status::SUCCESS;
}

Status begin_match(const Config& cfg, Ports& io, PlanState& plan, ExecState& ex)
{
  if (!ex.match_on)
  {
    ex.match_on = true;
    ex.standby_cleaned = false;
    ex.stop_sent = false;
    io.base_set_mode(cfg.mode_independent);
    if (cfg.enable_tunnel) plan.current_region = cfg.start_region;
  }
  return Status::SUCCESS;
}

Status standby(Ports& io, PlanState& plan, ExecState& ex, Decision& decision)
{
  if (!ex.standby_cleaned)
  {
    ex.stop_sent = false;
    explicit_stop(io, ex);
    ex.tunnel = {};
    ex.retry_until_s = 0.0;
    plan.cur_point = -1;
    plan.cur_target_valid = false;
    plan.pending_point = -1;
    plan.pending_count = 0;
    consume_operator(plan);
    ex.standby_cleaned = true;
  }
  // Consume buffered pre-match commands, but never resurrect a consumed task.
  consume_operator(plan);
  decision = {};
  ex.match_on = false;
  return Status::RUNNING;
}

Status observe_operator(Ports& io, PlanState& plan, const ExecState& ex, const Sense& sensed)
{
  if (!plan.op_active) return Status::SUCCESS;
  const bool own_final = ex.navigation.purpose == Purpose::ordinary &&
      ex.navigation.target_point == plan.op_target.id &&
      ex.navigation.generation == plan.op_generation;
  if (plan.op_arrived_s < 0.0 && own_final && sensed.nav_state == kGoalSuccess)
    plan.op_arrived_s = io.now_s();
  if (plan.op_arrived_s >= 0.0 && sensed.out_of_combat == 1)
  {
    if (plan.op_clear_since_s < 0.0) plan.op_clear_since_s = io.now_s();
  }
  else plan.op_clear_since_s = -1.0;
  return Status::SUCCESS;
}

Status commit_target(const Config& cfg, Ports& io, PlanState& plan, Decision& decision)
{
  auto& p = plan;
  const auto& candidate = decision;
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
    else if (p.pending_count < cfg.switch_confirm_frames) ++p.pending_count;
    accept = p.pending_count >= cfg.switch_confirm_frames &&
        (p.cur_arrived || io.now_s() - p.cur_since_s >= cfg.switch_dwell_s);
  }
  if (accept && !same)
  {
    p.cur_point = candidate.target_point;
    p.cur_target = candidate.target;
    p.cur_target_valid = candidate.target_valid;
    p.cur_operator_target = candidate.operator_target;
    p.cur_generation = candidate.generation;
    p.cur_since_s = io.now_s();
    p.cur_arrived = false;
  }
  if (accept)
  {
    p.pending_point = -1;
    p.pending_count = 0;
  }
  decision.target_point = p.cur_point;
  decision.target = p.cur_target;
  decision.target_valid = p.cur_target_valid;
  decision.operator_target = p.cur_operator_target;
  decision.generation = p.cur_generation;
  return Status::SUCCESS;
}

bool needs_tunnel(const Config& cfg, const PlanState& plan, const ExecState& ex, const Decision& decision)
{
  return cfg.enable_tunnel && (ex.tunnel.active() ||
      (decision.target_valid && decision.target.region != plan.current_region));
}

Status prepare_tunnel(const Config& cfg, Ports& io, const Sense& sensed, const PlanState& plan, ExecState& ex, const Decision& decision)
{
  auto& t = ex.tunnel;
  // Before crossing is committed, a new target may choose a different first tunnel.
  if (t.active() && !t.crossing_started &&
      (t.after_point != decision.target_point || t.generation != decision.generation))
  {
    cancel_owned_navigation(io, ex);
    const double cooldown = t.block_until_s;
    t = {};
    t.block_until_s = cooldown;
  }
  if (t.active())
  {
    remember_tunnel_progress(sensed, t);
    return Status::SUCCESS;
  }
  if (io.now_s() < t.block_until_s || !sensed.pos_valid || !decision.target_valid)
    return Status::FAILURE;
  const auto route = route_tunnels(cfg, plan.current_region, decision.target.region,
                                   &sensed.pos);
  if (route.empty()) return Status::FAILURE;
  const auto& definition = cfg.tunnels[route.front()];
  const int side = definition.align_region[0] == plan.current_region ? 0 :
                   definition.align_region[1] == plan.current_region ? 1 : -1;
  const int geometric_region = region_for_position(cfg, sensed.pos);
  if (side < 0 || dist(definition.align[0], definition.align[1]) < 1e-6 ||
      io.fold_state() == kFoldUnknown ||
      (geometric_region >= 0 && geometric_region != plan.current_region))
  {
    tunnel_failure(cfg, io, sensed, ex);
    return Status::FAILURE;
  }
  const double along = project_param(sensed.pos, definition.align[0], definition.align[1]);
  const bool in_mouth = along >= 0.0 && along <= 1.0 &&
      dist_to_segment(sensed.pos, definition.align[0], definition.align[1]) <
          cfg.in_tunnel_margin_m;
  if (in_mouth && io.fold_state() != kFoldFolded)
  {
    tunnel_failure(cfg, io, sensed, ex);
    return Status::FAILURE;
  }
  cancel_owned_navigation(io, ex);
  t.tunnel = route.front();
  t.after_point = decision.target_point;
  t.generation = decision.generation;
  t.entry = definition.align[side];
  t.exit = definition.align[1 - side];
  t.center = definition.center;
  t.has_center = definition.has_center;
  t.exit_region = definition.align_region[1 - side];
  t.crossing_started = in_mouth;
  t.center_reached = !t.has_center;
  if (in_mouth) t.entry_arrived_s = io.now_s();
  remember_tunnel_progress(sensed, t);
  return Status::SUCCESS;
}

Status navigate_entry(const Config& cfg, Ports& io, const Sense& sensed, ExecState& ex)
{
  auto& t = ex.tunnel;
  if (t.entry_arrived_s >= 0.0) return Status::SUCCESS;
  if (ex.navigation.purpose != Purpose::tunnel_entry)
  {
    cancel_owned_navigation(io, ex);
    if (!io.nav_set_goal(t.entry.x, t.entry.y))
    {
      tunnel_failure(cfg, io, sensed, ex);
      return Status::RUNNING;
    }
    set_request(io, ex, Purpose::tunnel_entry, -2, t.generation);
    return Status::RUNNING;
  }
  const auto state = io.nav_state();
  if (state == kGoalSuccess)
  {
    t.entry_arrived_s = io.now_s();
    return Status::SUCCESS;
  }
  if (state == kGoalFailure || io.now_s() - ex.navigation.started_s >= cfg.nav_timeout_s)
    tunnel_failure(cfg, io, sensed, ex);
  return Status::RUNNING;
}

Status request_fold(const Config& cfg, Ports& io, const Sense& sensed, ExecState& ex, bool fold)
{
  auto& t = ex.tunnel;
  double& requested_s = fold ? t.fold_requested_s : t.unfold_requested_s;
  if (requested_s < 0.0)
  {
    if (!io.fold_set(fold))
    {
      tunnel_failure(cfg, io, sensed, ex);
      return Status::RUNNING;
    }
    requested_s = io.now_s();
  }
  else if (io.now_s() - requested_s >= cfg.fold_timeout_s) tunnel_failure(cfg, io, sensed, ex);
  return Status::RUNNING;
}

Status traverse_tunnel(const Config& cfg, Ports& io, const Sense& sensed, PlanState& plan, ExecState& ex)
{
  auto& t = ex.tunnel;
  if (ex.navigation.purpose != Purpose::tunnel_crossing)
  {
    cancel_owned_navigation(io, ex);
    // A one-pose recovery is still an authorized folded traversal, not ordinary navigation.
    const std::vector<Vec2> poses = t.has_center && !t.center_reached
        ? std::vector<Vec2>{t.center, t.exit} : std::vector<Vec2>{t.exit};
    if (!io.nav_through_poses(poses))
    {
      tunnel_failure(cfg, io, sensed, ex);
      return Status::RUNNING;
    }
    // A synchronous rejection did not submit a task. Actual occupancy, if known,
    // was already recorded by PrepareTunnel; accepted submissions commit ownership.
    t.crossing_started = true;
    set_request(io, ex, Purpose::tunnel_crossing, -2, t.generation);
    return Status::RUNNING;
  }
  const auto state = io.nav_state();
  if (state == kGoalSuccess)
  {
    t.exit_reached = true;
    plan.current_region = t.exit_region;
    explicit_stop(io, ex);
  }
  else if (state == kGoalFailure || io.now_s() - ex.navigation.started_s >= cfg.nav_timeout_s)
    tunnel_failure(cfg, io, sensed, ex);
  return Status::RUNNING;
}

Status submit_ordinary(const Config& cfg, Ports& io, ExecState& ex, const Decision& decision)
{
  cancel_owned_navigation(io, ex);
  if (!io.nav_set_goal(decision.target.pos.x, decision.target.pos.y))
  {
    ordinary_failure(cfg, io, ex, decision);
    return Status::RUNNING;
  }
  set_request(io, ex, Purpose::ordinary, decision.target_point, decision.generation);
  tactical_outputs(cfg, io, ex, decision);
  return Status::RUNNING;
}
} // namespace

BT::Blackboard::Ptr make_blackboard(std::shared_ptr<const Config> config,
                                    std::shared_ptr<Ports> io,
                                    PlanState plan, ExecState execution)
{
  if (!config || !io) throw BT::RuntimeError("BBT requires config and io");
  auto board = BT::Blackboard::create();
  board->set("config", std::move(config));
  board->set("io", std::move(io));
  board->set("sense", Sense{});
  board->set("plan", std::move(plan));
  board->set("execution", std::move(execution));
  board->set("target", Decision{});
  return board;
}

void register_nodes(BT::BehaviorTreeFactory& factory)
{
  const auto config = input<Settings>("config");
  const auto io = input<IO>("io");
  const auto sense_in = input<Sense>("sense");
  const auto plan_in = input<PlanState>("plan");
  const auto plan = inout<PlanState>("plan");
  const auto execution_in = input<ExecState>("execution");
  const auto execution = inout<ExecState>("execution");
  const auto target_in = input<Decision>("target");
  const auto target = inout<Decision>("target");
  const auto target_out = output<Decision>("target");
  const auto action = [&](const auto& id, auto callback, auto... ports) {
    register_leaf(factory, id, BT::NodeType::ACTION, callback, ports...);
  };
  const auto condition = [&](const auto& id, auto callback, auto... ports) {
    register_leaf(factory, id, BT::NodeType::CONDITION, callback, ports...);
  };

  action("ReadInputs", [](const Settings& c, const IO& p, PlanState& m, Sense& s) {
    return read_inputs(*c, *p, m, s);
  }, config, io, plan, output<Sense>("sense"));
  condition("MatchRunning", [](const Sense& s) { return s.match_running; }, sense_in);
  action("BeginMatch", [](const Settings& c, const IO& p, PlanState& m, ExecState& e) {
    return begin_match(*c, *p, m, e);
  }, config, io, plan, execution);
  action("Standby", [](const IO& p, PlanState& m, ExecState& e, Decision& d) {
    return standby(*p, m, e, d);
  }, io, plan, execution, target_out);
  action("UpdateHomeLatch", [](const Settings& c, const Sense& s, PlanState& m) {
    update_home_latch(s, *c, m); return Status::SUCCESS;
  }, config, sense_in, plan);
  action("ObserveOperator", [](const IO& p, PlanState& m, const ExecState& e, const Sense& s) {
    return observe_operator(*p, m, e, s);
  }, io, plan, execution_in, sense_in);

  condition("AutoBuyEnabled", [](const Settings& c) { return c->enable_auto_buy; }, config);
  condition("AmmoPurchaseDue", [](const Settings& c, const IO& p, const Sense& s, const PlanState& m) {
    return c->ammo_buy_min_shots >= 0 && s.shots >= 0 && s.shots < c->ammo_buy_min_shots &&
      s.coins >= c->ammo_buy_min_coins && s.remote_ammo_pool != 0 && p->now_s() >= m.buy_ammo_next_s;
  }, config, io, sense_in, plan_in);
  action("BuyAmmo", [](const Settings& c, const IO& p, PlanState& m) {
    p->buy_ammo(c->ammo_buy_rounds); m.buy_ammo_next_s = p->now_s() + c->buy_cooldown_s;
    return Status::SUCCESS;
  }, config, io, plan);
  condition("HpPurchaseDue", [](const Settings& c, const IO& p, const Sense& s, const PlanState& m) {
    return c->hp_buy_ratio > 0.0 && s.hp > 0 && s.hp_ratio < c->hp_buy_ratio &&
      s.coins >= c->hp_buy_min_coins && s.out_of_combat != 0 && p->now_s() >= m.buy_hp_next_s;
  }, config, io, sense_in, plan_in);
  action("BuyHp", [](const Settings& c, const IO& p, PlanState& m) {
    p->buy_hp(c->hp_buy_amount); m.buy_hp_next_s = p->now_s() + c->buy_cooldown_s;
    return Status::SUCCESS;
  }, config, io, plan);
  condition("RevivePurchaseDue", [](const Settings& c, const IO& p, const Sense& s, const PlanState& m) {
    return c->enable_auto_revive && s.hp == 0 && p->now_s() >= m.buy_revive_next_s;
  }, config, io, sense_in, plan_in);
  condition("FreeReviveAllowed", [](const Sense& s) { return s.revive_available == 1; }, sense_in);
  condition("PaidReviveAllowed", [](const Sense& s) {
    return s.revive_available == 2 && s.revive_cost >= 0 && s.coins >= s.revive_cost;
  }, sense_in);
  action("BuyRevive", [](const Settings& c, const IO& p, PlanState& m) {
    p->buy_revive(); m.buy_revive_next_s = p->now_s() + c->buy_cooldown_s; return Status::SUCCESS;
  }, config, io, plan);

  condition("HasOperatorTarget", [](const PlanState& m) { return m.op_active; }, plan_in);
  condition("OperatorHoldElapsed", [](const Settings& c, const IO& p, const PlanState& m) {
    return m.op_arrived_s >= 0.0 && p->now_s() - m.op_arrived_s >= c->op_min_hold_s;
  }, config, io, plan_in);
  condition("OperatorClearElapsed", [](const Settings& c, const IO& p, const PlanState& m) {
    return m.op_clear_since_s >= 0.0 && p->now_s() - m.op_clear_since_s >= c->op_combat_clear_s;
  }, config, io, plan_in);
  condition("HomeLatched", [](const PlanState& m) { return m.home_latched; }, plan_in);
  action("ConsumeOperator", [](PlanState& m) { consume_operator(m); return Status::SUCCESS; }, plan);
  action("SelectOperator", [](const IO& p, const PlanState& m, Decision& d) {
    select_point(*p, d, &m.op_target, m.op_generation, true); return Status::SUCCESS;
  }, io, plan_in, target_out);
  action("SelectHome", [](const Settings& c, const IO& p, const PlanState& m, Decision& d) {
    const auto* home = find_point_by_zone(*c, kZoneHome);
    select_point(*p, d, home && point_usable(*c, m.current_region, *home) ? home : nullptr);
    return Status::SUCCESS;
  }, config, io, plan_in, target_out);
  condition("OwnOutpostNeeds", [](const Settings& c, const Sense& s) {
    return c->enable_outpost && our_outpost_needs(s);
  }, config, sense_in);
  condition("EnemyOutpostNeeds", [](const Settings& c, const Sense& s) {
    return c->enable_outpost && enemy_outpost_needs(s);
  }, config, sense_in);
  action("SelectRetake", [](const Settings& c, const IO& p, const PlanState& m, Decision& d) {
    const auto* point = pick_zone_point(*c, kZoneRetake, m.current_region);
    if (!point) return Status::FAILURE;
    select_point(*p, d, point); return Status::SUCCESS;
  }, config, io, plan_in, target_out);
  action("SelectOutpost", [](const Settings& c, const IO& p, const PlanState& m, Decision& d) {
    const auto* point = pick_zone_point(*c, kZoneOutpost, m.current_region);
    if (!point) return Status::FAILURE;
    select_point(*p, d, point); return Status::SUCCESS;
  }, config, io, plan_in, target_out);
  action("SelectPatrol", [](const Settings& c, const IO& p, const Sense& s,
                            PlanState& m, const ExecState& e, Decision& d) {
    const int owned = e.navigation.purpose == Purpose::ordinary ? e.applied_goal : -2;
    select_point(*p, d, find_point(*c, pick_patrol(s, *c, m, p->now_s(), owned)));
    return Status::SUCCESS;
  }, config, io, sense_in, plan, execution_in, target_out);
  action("CommitTarget", [](const Settings& c, const IO& p, PlanState& m, Decision& d) {
    return commit_target(*c, *p, m, d);
  }, config, io, plan, target);

  condition("NeedsTunnel", [](const Settings& c, const PlanState& m, const ExecState& e, const Decision& d) {
    return needs_tunnel(*c, m, e, d);
  }, config, plan_in, execution_in, target_in);
  action("PrepareTunnel", [](const Settings& c, const IO& p, const Sense& s,
                             const PlanState& m, ExecState& e, const Decision& d) {
    return prepare_tunnel(*c, *p, s, m, e, d);
  }, config, io, sense_in, plan_in, execution, target_in);
  action("TunnelQuiet", [](const IO& p) {
    p->base_spin(0.0); p->attack_cmd(0, 0, 0, 0); return Status::SUCCESS;
  }, io);
  condition("TunnelCooldown", [](const IO& p, const ExecState& e) {
    return p->now_s() < e.tunnel.block_until_s;
  }, io, execution_in);
  condition("TunnelExitReached", [](const ExecState& e) { return e.tunnel.exit_reached; }, execution_in);
  condition("CrossingCommitted", [](const ExecState& e) { return e.tunnel.crossing_started; }, execution_in);
  condition("CrossingNavigationActive", [](const ExecState& e) {
    return e.navigation.purpose == Purpose::tunnel_crossing;
  }, execution_in);
  action("FailTunnel", [](const Settings& c, const IO& p, const Sense& s, ExecState& e) {
    tunnel_failure(*c, *p, s, e); return Status::RUNNING;
  }, config, io, sense_in, execution);
  condition("Folded", [](const IO& p) { return p->fold_state() == kFoldFolded; }, io);
  condition("Unfolded", [](const IO& p) { return p->fold_state() == kFoldUnfolded; }, io);
  action("NavigateEntry", [](const Settings& c, const IO& p, const Sense& s, ExecState& e) {
    return navigate_entry(*c, *p, s, e);
  }, config, io, sense_in, execution);
  action("ExplicitStop", [](const IO& p, ExecState& e) {
    explicit_stop(*p, e); return Status::SUCCESS;
  }, io, execution);
  action("WaitStopped", [](const IO& p, ExecState& e) {
    explicit_stop(*p, e); return Status::RUNNING;
  }, io, execution);
  action("RequestFold", [](const Settings& c, const IO& p, const Sense& s, ExecState& e) {
    return request_fold(*c, *p, s, e, true);
  }, config, io, sense_in, execution);
  action("RequestUnfold", [](const Settings& c, const IO& p, const Sense& s, ExecState& e) {
    return request_fold(*c, *p, s, e, false);
  }, config, io, sense_in, execution);
  action("TraverseTunnel", [](const Settings& c, const IO& p, const Sense& s, PlanState& m, ExecState& e) {
    return traverse_tunnel(*c, *p, s, m, e);
  }, config, io, sense_in, plan, execution);
  action("CompleteTunnel", [](ExecState& e) {
    e.tunnel = {}; forget_navigation(e); return Status::RUNNING;
  }, execution);

  condition("HasTarget", [](const Decision& d) { return d.target_valid; }, target_in);
  condition("OwnsOrdinaryRequest", owns_ordinary_request, execution_in, target_in);
  condition("OrdinaryFailed", [](const Settings& c, const IO& p, const ExecState& e) {
    const auto state = p->nav_state();
    return state == kGoalFailure || (state != kGoalSuccess &&
      p->now_s() - e.navigation.started_s >= c->nav_timeout_s);
  }, config, io, execution_in);
  action("FailOrdinary", [](const Settings& c, const IO& p, ExecState& e, const Decision& d) {
    ordinary_failure(*c, *p, e, d); return Status::RUNNING;
  }, config, io, execution, target_in);
  condition("OrdinaryRetryBlocked", [](const IO& p, const ExecState& e, const Decision& d) {
    return e.retry_target == d.target_point && e.retry_generation == d.generation &&
      p->now_s() < e.retry_until_s;
  }, io, execution_in, target_in);
  action("TacticalKeep", [](const Settings& c, const IO& p, ExecState& e, const Decision& d) {
    tactical_outputs(*c, *p, e, d); return Status::RUNNING;
  }, config, io, execution, target_in);
  action("SubmitOrdinary", [](const Settings& c, const IO& p, ExecState& e, const Decision& d) {
    return submit_ordinary(*c, *p, e, d);
  }, config, io, execution, target_in);
}

int step(Ports& ports, const Config& cfg, PlanState& plan, ExecState& execution, double)
{
  auto config = std::make_shared<const Config>(cfg);
  auto io = std::shared_ptr<Ports>(&ports, [](Ports*) {});
  auto board = make_blackboard(config, io, plan, execution);
  BT::BehaviorTreeFactory factory;
  register_nodes(factory);
  auto tree = factory.createTreeFromFile(BBT_TREE_PATH, board);
  tree.tickOnce();
  plan = board->get<PlanState>("plan");
  execution = board->get<ExecState>("execution");
  return execution.tunnel.active() ? execution.tunnel.after_point : board->get<Decision>("target").target_point;
}
} // namespace bbt_decide
