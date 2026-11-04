#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <unistd.h>
#include <stdexcept>
#include "bbt_decide/planner.hpp"
using namespace bbt_decide;
#define CHECK(x) do { if (!(x)) throw std::runtime_error("line " + std::to_string(__LINE__) + ": " #x); } while (0)

class TempYaml
{
public:
  explicit TempYaml(const std::string& content)
  {
    char name[] = "/tmp/bbt_config_test_XXXXXX";
    const int fd = mkstemp(name);
    CHECK(fd >= 0);
    path = name;
    close(fd);
    std::ofstream out(path);
    out << content;
    CHECK(out.good());
  }
  ~TempYaml() { std::remove(path.c_str()); }
  TempYaml(const TempYaml&) = delete;
  TempYaml& operator=(const TempYaml&) = delete;
  std::string path;
};

static Config valid_config(const std::string& yaml)
{
  TempYaml file(yaml);
  Config cfg;
  std::string error;
  std::vector<std::string> warnings;
  if (!load_config(file.path, cfg, error, warnings))
    throw std::runtime_error("valid configuration rejected: " + error + "\n" + yaml);
  return cfg;
}

static void invalid_config(const std::string& yaml, bool expect_location = false)
{
  TempYaml file(yaml);
  Config cfg;
  std::string error;
  std::vector<std::string> warnings;
  if (load_config(file.path, cfg, error, warnings))
    throw std::runtime_error("invalid configuration accepted:\n" + yaml);
  CHECK(!error.empty());
  if (expect_location)
  {
    const auto position = error.find(file.path + ":");
    CHECK(position != std::string::npos);
    const char line = error.at(position + file.path.size() + 1);
    CHECK(line >= '0' && line <= '9');
  }
}

static const std::string minimal_yaml =
    "enable_tunnel: false\npoints: [{id: 1, x: 1, y: 2, zone: home}]\n";

static std::string tunnel_yaml(const std::array<std::string, 3>& coordinates)
{
  return "regions: [left, right]\n"
         "points: [{id: 1, x: 1, y: 2, zone: home, region: left}]\n"
         "tunnels:\n"
         "  - name: corridor\n"
         "    align:\n"
         "      - {p: " + coordinates[0] + ", region: left}\n"
         "      - {p: " + coordinates[1] + ", region: right}\n"
         "    center: " + coordinates[2] + "\n";
}

static void operator_topic_defaults()
{
  auto cfg = valid_config(minimal_yaml);
  for (const auto* topics : {&cfg.topics_sim, &cfg.topics_real})
  {
    CHECK(topics->op_cmd == "/sentry/operator_point");
    CHECK(topics->op_cmd_xy == "/referee/operator_cmd");
  }
  cfg = valid_config(minimal_yaml +
      "topics:\n"
      "  sim: {op_cmd: /custom/operator_point, op_cmd_xy: /custom/operator_xy}\n"
      "  real: {op_cmd: '', op_cmd_xy: /real/operator_xy}\n");
  CHECK(cfg.topics_sim.op_cmd == "/custom/operator_point");
  CHECK(cfg.topics_sim.op_cmd_xy == "/custom/operator_xy");
  CHECK(cfg.topics_real.op_cmd.empty());
  CHECK(cfg.topics_real.op_cmd_xy == "/real/operator_xy");
  std::string error;
  CHECK(cfg.topics_real.all_named(error));

  for (const auto* name : {"decision.yaml", "corridor_demo.yaml", "plain_demo.yaml"})
  {
    std::vector<std::string> warnings;
    CHECK(load_config(std::string(SRC_DIR) + "/config/" + name, cfg, error, warnings));
    CHECK(cfg.topics_sim.op_cmd == "/sentry/operator_point");
    CHECK(cfg.topics_sim.op_cmd_xy == "/referee/operator_cmd");
    CHECK(cfg.topics_real.op_cmd.empty() || cfg.topics_real.op_cmd == "/sentry/operator_point");
    CHECK(cfg.topics_real.op_cmd_xy == "/referee/operator_cmd");
  }
}

static void configuration_validation()
{
  for (const auto& bad : {
           std::string("enable_tunnel: false\npoints: [{id: 1, x: abc, y: 0}]\n"),
           std::string("enable_tunnel: false\npoints: [{x: 1, y: 0}]\n"),
           minimal_yaml + "auto_buy: {ammo_rounds: 50}\n",
           minimal_yaml + "safety: {hp_home_ratio: 0.8, hp_resume_ratio: 0.1}\n"})
    invalid_config(bad);

  struct Field { const char* section; const char* name; };
  const Field numeric_fields[] = {
      {"field_to_map", "dx"}, {"field_to_map", "dy"}, {"field_to_map", "dyaw_deg"},
      {"blue_field_to_map", "dx"}, {"blue_field_to_map", "dy"},
      {"blue_field_to_map", "dyaw_deg"},
      {"safety", "hp_home_ratio"}, {"safety", "hp_resume_ratio"}, {"safety", "spin_w"},
      {"safety", "odom_stale_ms"}, {"safety", "patrol_dwell_s"}, {"safety", "switch_dwell_s"},
      {"safety", "gate_clear_m"}, {"safety", "op_min_hold_s"}, {"safety", "op_combat_clear_s"},
      {"safety", "op_goal_timeout_s"}, {"safety", "nav_timeout_s"},
      {"safety", "nav_retry_s"}, {"safety", "fold_timeout_s"}, {"safety", "fold_stale_ms"},
      {"safety", "tunnel_retry_cooldown_s"}, {"safety", "in_tunnel_margin_m"},
      {"safety", "arrive_radius_m"}, {"outpost", "snap_m"},
      {"auto_buy", "hp_ratio"}, {"auto_buy", "cooldown_s"}};
  for (const auto& field : numeric_fields)
    for (const auto* value : {".nan", ".inf", "-.inf"})
      invalid_config(minimal_yaml + field.section + ": {" + field.name + ": " + value + "}\n", true);

  for (const auto* value : {".nan", ".inf", "-.inf"})
  {
    for (const auto* axis : {"x", "y"})
      invalid_config(std::string("enable_tunnel: false\npoints: [{id: 1, ") +
          (std::string(axis) == "x" ? std::string("x: ") + value + ", y: 2" :
                                    std::string("x: 1, y: ") + value) + "}]\n", true);
    for (size_t point = 0; point < 3; ++point)
      for (int axis = 0; axis < 2; ++axis)
      {
        std::array<std::string, 3> coordinates = {"[9, 5]", "[11, 5]", "[10, 5]"};
        coordinates[point] = axis == 0 ? std::string("[") + value + ", 5]" :
                                         std::string("[10, ") + value + "]";
        invalid_config(tunnel_yaml(coordinates), true);
      }
  }

  const Field nonnegative_fields[] = {
      {"safety", "gate_clear_m"}, {"safety", "in_tunnel_margin_m"},
      {"safety", "arrive_radius_m"}, {"outpost", "snap_m"},
      {"safety", "op_min_hold_s"}, {"safety", "op_combat_clear_s"},
      {"safety", "patrol_dwell_s"}, {"safety", "switch_dwell_s"},
      {"safety", "nav_retry_s"}, {"auto_buy", "cooldown_s"}};
  for (const auto& field : nonnegative_fields)
    invalid_config(minimal_yaml + field.section + ": {" + field.name + ": -0.1}\n");
  for (const auto* timeout : {"nav_timeout_s", "fold_timeout_s", "fold_stale_ms",
                            "odom_stale_ms", "tunnel_retry_cooldown_s"})
    for (const auto* value : {"0", "-0.1"})
      invalid_config(minimal_yaml + "safety: {" + timeout + ": " + value + "}\n");
  invalid_config(minimal_yaml + "auto_buy: {hp_ratio: 1.1}\n");

  const auto zero = valid_config(minimal_yaml +
      "field_to_map: {enabled: true, dx: -1, dy: -2, dyaw_deg: -90}\n"
      "blue_field_to_map: {enabled: true, dx: 3, dy: 4, dyaw_deg: 15}\n"
      "safety: {hp_home_ratio: 0, hp_resume_ratio: 0, gate_clear_m: 0, in_tunnel_margin_m: 0, "
      "arrive_radius_m: 0, patrol_dwell_s: 0, switch_dwell_s: 0, nav_retry_s: 0, "
      "spin_w: -1.5, op_min_hold_s: 0, op_combat_clear_s: 0}\n"
      "outpost: {snap_m: 0}\n"
      "auto_buy: {hp_ratio: -1, cooldown_s: 0}\n");
  CHECK(zero.gate_clear_m == 0 && zero.in_tunnel_margin_m == 0 && zero.arrive_radius_m == 0);
  CHECK(zero.op_snap_m == 0 && zero.patrol_dwell_s == 0 && zero.switch_dwell_s == 0);
  CHECK(zero.nav_retry_s == 0 && zero.buy_cooldown_s == 0);
  CHECK(zero.hp_buy_ratio == -1 && zero.spin_w == -1.5);
  CHECK(zero.op_min_hold_s == 0 && zero.op_combat_clear_s == 0);
  CHECK(zero.has_field_to_map_blue && zero.field_to_map_blue.enabled);
  CHECK(zero.field_to_map_blue.dx == 3 && zero.field_to_map_blue.dy == 4);
  CHECK(!valid_config(minimal_yaml + "auto_buy: {revive_enable: false}\n").enable_auto_revive);
  CHECK(!valid_config(minimal_yaml + "auto_buy: {revive_min_coins: -1}\n").enable_auto_revive);
  CHECK(valid_config(minimal_yaml + "auto_buy: {revive_min_coins: -1, revive_enable: true}\n").enable_auto_revive);
  CHECK(valid_config(minimal_yaml + "auto_buy: {hp_ratio: 0}\n").hp_buy_ratio == 0);
  CHECK(valid_config(minimal_yaml + "auto_buy: {hp_ratio: 1}\n").hp_buy_ratio == 1);

  // Finite inputs can still overflow when translated into the navigation map.
  invalid_config("enable_tunnel: false\nfield_to_map: {enabled: true, dx: 1.7e308}\n"
      "points: [{id: 1, x: 1.7e308, y: 2}]\n");
  invalid_config(tunnel_yaml({"[1.7e308, 5]", "[11, 5]", "[10, 5]"}) +
      "field_to_map: {enabled: true, dx: 1.7e308}\n");
  invalid_config(tunnel_yaml({"[9, 5]", "[11, 5]", "[1.7e308, 5]"}) +
      "field_to_map: {enabled: true, dx: 1.7e308}\n");
  invalid_config(tunnel_yaml({"[9, 5]", "[11, 5]", "[10]"}));
}



static void configuration_structure_and_disabled_features()
{
  invalid_config("regions: [left, left]\npoints: [{id: 1, x: 0, y: 0, region: left}]\n");
  invalid_config("regions: [left, right]\nstart_region: missing\n"
      "points: [{id: 1, x: 0, y: 0, region: left}]\n");
  invalid_config("regions: [left, right]\npoints: [{id: 1, x: 0, y: 0, region: missing}]\n");
  invalid_config("points: [{id: 1, x: 0, y: 0, region: left}]\n");
  invalid_config("enable_tunnel: false\npoints: [{id: 1, x: 0, y: 0, zone: invalid}]\n");
  invalid_config("enable_tunnel: false\npoints: [{id: 1, x: 0, y: 0}, {id: 1, x: 2, y: 2}]\n");
  invalid_config("enable_tunnel: false\npoints: []\n");
  invalid_config("regions: [left, right]\npoints: [{id: 1, x: 0, y: 0, region: left}]\n"
      "tunnels: [{name: bad, align: [{p: [0, 0], region: left}, {p: [1, 1], region: missing}]}]\n");

  TempYaml islands("regions: [left, right, lonely]\nstart_region: left\n"
      "points: [{id: 1, x: 1, y: 5, zone: home, region: left}]\n"
      "tunnels: [{name: simple, align: [{p: [9, 5], region: left}, {p: [11, 5], region: right}]}]\n");
  Config cfg;
  std::string error;
  std::vector<std::string> warnings;
  CHECK(load_config(islands.path, cfg, error, warnings));
  CHECK(!cfg.tunnels[0].has_center);
  bool warned = false;
  for (const auto& warning : warnings) if (warning.find("lonely") != std::string::npos) warned = true;
  CHECK(warned);

  TempYaml disabled("enable_tunnel: false\n"
      "safety: {spin_w: 2, ammo_home_min: 40, ammo_resume_min: 120}\n"
      "points: [{id: 1, x: 0, y: 0, zone: home, region: irrelevant}]\n"
      "tunnels: [{name: ignored, align: invalid}]\n"
      "auto_buy: {enable: false, ammo_min_shots: 25, ammo_rounds: 150, hp_ratio: 0.35, cooldown_s: 15}\n");
  CHECK(load_config(disabled.path, cfg, error, warnings));
  CHECK(cfg.regions.empty() && cfg.points[0].region == -1 && cfg.tunnels.empty());
  CHECK(cfg.spin_w == 2 && cfg.ammo_home_min == 40 && cfg.ammo_resume_min == 120);
  CHECK(!cfg.enable_auto_buy && cfg.ammo_buy_min_shots == 25 && cfg.ammo_buy_rounds == 150);
  CHECK(cfg.hp_buy_ratio == .35 && cfg.buy_cooldown_s == 15);
  CHECK(cfg.ammo_buy_min_coins == 200 && cfg.hp_buy_amount == 1);
  warned = false;
  for (const auto& warning : warnings) if (warning.find("洞表已忽略") != std::string::npos) warned = true;
  CHECK(warned);
}

// These regressions use the same production-tree entry point as the ROS node.
// Fold acceptance is deliberately separate from physical fold feedback.
struct FeedbackPorts : DebugPorts
{
  bool confirm_fold = false;
  bool fold_set(bool request) override
  {
    if (!fold_capable) return false;
    if (confirm_fold) return DebugPorts::fold_set(request);
    ++(request ? fold_cmd_count : unfold_cmd_count);
    return true;
  }
};
static Config corridor()
{
  Config c;
  c.regions = {"left", "right"};
  c.enable_auto_buy = false; c.enable_outpost = false;
  c.switch_dwell_s = 0; c.switch_confirm_frames = 1; c.patrol_dwell_s = 1000;
  c.points = {{1, {1, 5}, kZoneHome, 0}, {10, {5, 5}, kZonePatrol, 0},
              {11, {15, 5}, kZonePatrol, 1}};
  TunnelDef t;
  t.align[0] = {9, 5}; t.align[1] = {11, 5};
  t.align_region[0] = 0; t.align_region[1] = 1;
  t.has_center = true; t.center = {10, 5};
  c.tunnels = {t};
  return c;
}
struct Bench
{
  Config c;
  FeedbackPorts p;
  PlanState plan;
  ExecState ex;
  explicit Bench(Config config = corridor()) : c(std::move(config))
  {
    p.ref.game_progress = 4; p.ref.hp = p.ref.hp_max = 400; p.ref.shots = 200;
    p.pose = {5, 5, 0, 0, true};
    p.op = {1, 4, 11};
  }
  void tick(double dt = .05, bool refresh = true)
  {
    p.time_s += dt;
    if (refresh) p.pose.stamp_ms = static_cast<uint64_t>(p.time_s * 1000);
    step(p, c, plan, ex, p.time_s);
  }
  template<class Ready> bool until(Ready ready, int limit = 20)
  {
    for (int i = 0; i < limit && !ready(); ++i) tick();
    return ready();
  }
  bool target(double x, double y = 5) const
  { return p.goal_count && std::abs(p.goal_x-x) < 1e-8 && std::abs(p.goal_y-y) < 1e-8; }
  void success(double x)
  { p.pose.x = x; p.goal = kGoalSuccess; }
  void enter()
  {
    tick(); CHECK(target(9));
    success(9);
    CHECK(until([&] { return p.fold_cmd_count == 1; }));
    CHECK(p.through_count == 0);
    p.fold = kFoldFolded;
    CHECK(until([&] { return p.through_count == 1; }));
    CHECK(target(11) && p.goal == kGoalRunning && ex.tunnel.active());
  }
};

static void match_before_first_pose()
{
  auto c = corridor();
  auto parallel = c.tunnels[0];
  parallel.align[0] = {9, 9}; parallel.align[1] = {11, 9}; parallel.center = {10, 9};
  c.tunnels.push_back(parallel);
  Bench b(c); b.p.pose.valid = false;
  for (int i = 0; i < 200; ++i) b.tick();
  CHECK(!b.ex.tunnel.active() && b.p.goal_count == 0 && b.p.fold_cmd_count == 0);
  CHECK(b.p.cancel_count == 1 && b.p.stop_count == 1);
  CHECK(b.ex.tunnel.block_until_s == 0);
  b.p.pose = {8, 9, 0, 10000, true}; b.tick();
  CHECK(b.ex.tunnel.active() && b.ex.tunnel.tunnel == 1 && b.target(9, 9));
  // Losing position cannot hand a committed transaction to ordinary navigation.
  b.p.pose.valid = false; b.p.goal = kGoalSuccess;
  CHECK(b.until([&] { return b.p.fold_cmd_count == 1; }));
  b.p.fold = kFoldFolded;
  CHECK(b.until([&] { return b.p.through_count == 1; }));
  CHECK(b.target(11, 9) && b.p.through_poses.front().x == 10);
}


static Config geometry_corridor()
{
  auto c = corridor();
  // Shared x=10 boundary is deliberately not assigned to either region.
  c.region_geometry = {
      {"left", {{0, 0}, {10, 0}, {10, 10}, {0, 10}}},
      {"right", {{10, 0}, {20, 0}, {20, 10}, {10, 10}}}};
  c.tunnels[0].corners = {{9.25, 4.5}, {10.75, 4.5}, {10.75, 5.5}, {9.25, 5.5}};
  return c;
}

static void ambiguous_region_keeps_bookkeeping_and_fold_requirement()
{
  Bench b(geometry_corridor());
  b.p.pose = {10, 8, 0, 0, true}; // Shared boundary, safely outside the tunnel mouth.
  b.p.op = {1, 4, -1, 15.25, 7}; // A genuine coordinate, absent from the point table.
  b.tick();
  CHECK(b.plan.current_region == 0 && b.target(9));
  CHECK(b.p.through_count == 0 && b.p.fold_cmd_count == 0);
  CHECK(!b.target(15.25, 7)); // Ambiguous position cannot silently become the far region.
  b.success(9);
  CHECK(b.until([&] { return b.p.fold_cmd_count == 1; }));
  for (int i = 0; i < 5; ++i) b.tick();
  CHECK(b.plan.current_region == 0 && b.p.through_count == 0);
  CHECK(b.p.stop_count > 0 && b.p.spin_last_w == 0 && b.p.attack_last_fire == 0);
  b.p.fold = kFoldFolded;
  CHECK(b.until([&] { return b.p.through_count == 1; }));
  CHECK(b.target(11) && b.plan.current_region == 0);

  // Ambiguity at the actual tunnel body never permits an unfolded traversal.
  Bench mouth(geometry_corridor());
  mouth.p.pose = {10, 5, 0, 0, true};
  mouth.p.op = {1, 4, -1, 15.25, 7}; mouth.tick();
  CHECK(mouth.plan.current_region == 0 && mouth.p.goal_count == 0);
  CHECK(mouth.p.through_count == 0 && mouth.p.stop_count == 1);
  CHECK(mouth.p.spin_last_w == 0 && mouth.p.attack_last_fire == 0);
}

static void contradictory_region_waits_without_guessing_new_region()
{
  Bench b(geometry_corridor());
  b.p.pose = {15, 8, 0, 0, true}; // Unambiguously right, but the trusted ledger is left.
  b.p.op = {1, 4, -1, 15.25, 7};
  b.tick();
  CHECK(b.plan.current_region == 0 && b.p.goal_count == 0 && b.p.through_count == 0);
  CHECK(b.p.fold_cmd_count == 0 && b.p.stop_count == 1 && b.p.cancel_count == 1);
  CHECK(b.p.spin_last_w == 0 && b.p.attack_last_fire == 0);
  // A corrected pose is not allowed to bypass the five-second rejection cooldown.
  b.p.pose.x = 5;
  b.tick(4.9);
  CHECK(b.plan.current_region == 0 && b.p.goal_count == 0 && b.p.through_count == 0);
  CHECK(b.p.cancel_count == 1 && b.p.stop_count == 1);
  b.tick(.11);
  CHECK(b.plan.current_region == 0 && b.target(9));
  CHECK(b.p.through_count == 0 && b.p.fold_cmd_count == 0);
}

static void center_feedback_and_recovery()
{
  Bench b; b.enter();
  b.p.fold = kFoldUnknown; b.p.pose.x = 9.5;
  const int cancelled = b.p.cancel_count;
  b.tick();
  CHECK(b.ex.tunnel.active() && b.ex.tunnel.crossing_started);
  CHECK(!b.ex.tunnel.center_reached && b.p.cancel_count == cancelled + 1);
  b.p.op = {2, 4, 1}; // New intent must not reverse a committed crossing.
  b.tick(); CHECK(b.p.goal_count == 2);
  b.tick(5.1);
  CHECK(b.p.fold_cmd_count == 2 && b.p.goal_count == 2);
  b.p.fold = kFoldFolded;
  CHECK(b.until([&] { return b.p.through_count == 2; }));
  CHECK(b.p.through_poses.size() == 2 && b.p.through_poses.front().x == 10);
  b.p.pose.x = 10.2; b.tick();
  CHECK(b.ex.tunnel.center_reached && b.plan.current_region == 0);
  CHECK(b.p.goal == kGoalRunning && b.p.unfold_cmd_count == 0);
  b.p.fold = kFoldUnknown; b.p.pose.x = 9.5; b.tick();
  CHECK(b.ex.tunnel.center_reached); // Physical progress cannot be undone by later poses.
  b.tick(5.1); b.p.fold = kFoldFolded;
  CHECK(b.until([&] { return b.p.through_count == 3; }));
  CHECK(b.p.through_poses.size() == 1 && b.p.through_poses.front().x == 11);
  CHECK(b.ex.tunnel.after_point == 11 && b.target(11));
  b.success(11);
  CHECK(b.until([&] { return b.p.unfold_cmd_count == 1; }));
  CHECK(b.ex.tunnel.active() && b.plan.current_region == 1);
  b.p.fold = kFoldUnfolded;
  CHECK(b.until([&] { return !b.ex.tunnel.active(); }));
  // The latest intent is still home, but a reverse tunnel is required first.
  b.tick(); CHECK(b.target(11) && !b.target(1));
  // The robot is still at the mouth, unfolded: first move safely clear on the
  // served side, then let the unchanged reverse intent retry its entry route.
  b.p.pose.x = 15; b.tick(5.1);
  CHECK(b.until([&] { return b.ex.tunnel.active(); }));
  CHECK(b.target(11) && b.plan.current_region == 1);
}

static void center_resume_direction()
{
  for (int side : {0, 1})
  {
    auto c = corridor(); c.start_region = side;
    for (double start : {10.0, side == 0 ? 10.2 : 9.8})
    {
      Bench b(c); b.p.pose.x = start; b.p.fold = kFoldFolded;
      b.p.op = {1, 4, side == 0 ? 11 : 10};
      b.tick();
      CHECK(b.target(side == 0 ? 11 : 9));
      CHECK(b.ex.tunnel.crossing_started && b.ex.tunnel.center_reached);
      CHECK(b.p.through_count == 1 && b.p.through_poses.size() == 1);
    }
    Bench b(c); b.p.pose.x = side == 0 ? 9.5 : 10.5; b.p.fold = kFoldFolded;
    b.p.op = {1, 4, side == 0 ? 11 : 10}; b.tick();
    CHECK(b.p.through_poses.size() == 2);
    b.p.pose.x = side == 0 ? 10.2 : 9.8; b.p.goal = kGoalFailure; b.tick();
    CHECK(b.ex.tunnel.center_reached && b.ex.tunnel.active());
    b.tick(5.1); b.p.fold = kFoldFolded;
    CHECK(b.until([&] { return b.p.through_count == 2; }));
    CHECK(b.p.through_poses.size() == 1 && b.target(side == 0 ? 11 : 9));
    CHECK(!b.ex.tunnel.exit_reached && b.plan.current_region == side);
  }

  Bench stale; stale.enter();
  stale.p.pose.x = 10.2; stale.p.pose.stamp_ms = 0; stale.p.fold = kFoldUnknown;
  stale.tick(2, false);
  CHECK(!stale.ex.tunnel.center_reached);
  stale.tick(5.1, false); stale.p.fold = kFoldFolded; stale.p.pose.valid = false;
  CHECK(stale.until([&] { return stale.p.through_count == 2; }));
  CHECK(stale.p.through_poses.size() == 2); // Expired pose cannot skip center.
  stale.p.pose.valid = false; stale.p.goal = kGoalSuccess; stale.p.fold = kFoldUnknown;
  stale.tick();
  CHECK(stale.ex.tunnel.active() && !stale.ex.tunnel.exit_reached);
  CHECK(stale.p.unfold_cmd_count == 0); // Missing fold feedback beats a stale success.
}

static void center_and_exit_share_timeout()
{
  auto c = corridor(); c.nav_timeout_s = 1;
  Bench b(c); b.enter();
  b.p.pose.x = 10.2; b.tick(.7);
  CHECK(b.ex.tunnel.center_reached && b.p.goal == kGoalRunning);
  b.tick(.4);
  CHECK(b.ex.tunnel.active() && b.ex.tunnel.crossing_started && !b.ex.tunnel.exit_reached);
  CHECK(b.p.unfold_cmd_count == 0 && b.plan.current_region == 0 && b.p.spin_last_w == 0);
}

static void unfolding_timeout()
{
  Bench b; b.enter(); b.success(11);
  CHECK(b.until([&] { return b.p.unfold_cmd_count == 1; }));
  CHECK(b.plan.current_region == 1);
  b.tick(3.1);
  CHECK(b.ex.tunnel.active() && b.ex.tunnel.exit_reached);
  const int goals = b.p.goal_count;
  b.p.op = {2, 4, 1}; b.p.fold = kFoldUnfolded; b.tick();
  CHECK(b.ex.tunnel.active() && b.p.goal_count == goals); // Cooldown still owns the exit.
  b.tick(5.1);
  CHECK(b.until([&] { return !b.ex.tunnel.active(); }));
  CHECK(b.p.goal_count == goals && b.plan.current_region == 1);
}

static void navigation_timeout_and_empty_target_are_idempotent()
{
  auto c = corridor(); c.enable_tunnel = false;
  Bench b(c); b.p.op = {}; b.p.ref.hp = 50;
  b.tick(); CHECK(b.target(1));
  b.tick(26);
  CHECK(b.p.cancel_count == 1 && b.p.stop_count == 1 && b.p.spin_last_w == 0);
  CHECK(b.ex.applied_goal == -2);
  b.tick(1); CHECK(b.p.goal_count == 1);
  b.tick(2); CHECK(b.p.goal_count == 2 && b.target(1));
  c.points.clear();
  Bench empty(c); empty.p.op = {};
  for (int i = 0; i < 200; ++i) empty.tick();
  CHECK(empty.p.cancel_count == 1 && empty.p.stop_count == 1 && empty.p.goal_count == 0);
  CHECK(empty.p.spin_last_w == 0);
}
static void uptime_and_config()
{
  Config c=corridor(); DebugPorts p; const double now=5000000;
  p.pose={1,2,0,static_cast<uint64_t>(now*1000),true};
  CHECK(sense(p,c,now).pos_valid);
  CHECK(!sense(p,c,now+2).pos_valid);
  std::string error; std::vector<std::string> warnings;
  CHECK(load_config(SRC_DIR "/config/decision.yaml",c,error,warnings));
  CHECK(c.require_map_alignment);
  CHECK(c.field_to_map.enabled);
  CHECK(c.field_to_map.dx==0 && c.field_to_map.dy==0 && c.field_to_map.dyaw_deg==0);
  CHECK(std::abs(find_point(c,100)->pos.x-2.61)<1e-8);
  CHECK(std::abs(find_point(c,100)->pos.y-1.52)<1e-8);
  CHECK(find_point(c,100)->region==c.start_region);
  CHECK(c.regions[find_point(c,30)->region]=="mid"); // Preserve canonical UC point regions.

}

static void configured_team_geometry()
{
  Config original;
  std::string error; std::vector<std::string> warnings;
  CHECK(load_config(SRC_DIR "/config/decision.yaml", original, error, warnings));
  CHECK(original.geometry_reference == "red");
  CHECK(original.points.size() == 39 && original.tunnels.size() == 6);
  Config red = original, blue = original;
  configure_own_team(red, "red");
  configure_own_team(blue, "blue");
  CHECK(red.own_team == "red" && blue.own_team == "blue");
  CHECK(std::abs(find_point(blue,100)->pos.x - 25.39) < 1e-8);
  CHECK(std::abs(find_point(blue,100)->pos.y - 13.48) < 1e-8);
  CHECK(red.start_region == blue.start_region && red.regions == blue.regions);
  for (size_t i = 0; i < original.points.size(); ++i)
  {
    const auto& source = original.points[i];
    CHECK(red.points[i].id == source.id && blue.points[i].id == source.id);
    CHECK(red.points[i].region == source.region && blue.points[i].region == source.region);
    CHECK(red.points[i].zone == source.zone && blue.points[i].zone == source.zone);
    CHECK(dist(red.points[i].pos, source.pos) < 1e-8);
    CHECK(dist(blue.points[i].pos, {28 - source.pos.x, 15 - source.pos.y}) < 1e-8);
  }
  for (size_t i = 0; i < original.tunnels.size(); ++i)
  {
    const auto& source = original.tunnels[i];
    CHECK(source.has_center && red.tunnels[i].has_center && blue.tunnels[i].has_center);
    CHECK(red.tunnels[i].name == source.name && blue.tunnels[i].name == source.name);
    CHECK(dist(red.tunnels[i].center, source.center) < 1e-8);
    CHECK(dist(blue.tunnels[i].center, {28-source.center.x, 15-source.center.y}) < 1e-8);
    for (int endpoint = 0; endpoint < 2; ++endpoint)
    {
      CHECK(blue.tunnels[i].align_region[endpoint] == source.align_region[endpoint]);
      CHECK(dist(blue.tunnels[i].align[endpoint],
                 {28-source.align[endpoint].x, 15-source.align[endpoint].y}) < 1e-8);
    }
  }
  const Config once = blue;
  configure_own_team(blue, "blue");
  for (size_t i = 0; i < blue.points.size(); ++i)
    CHECK(dist(blue.points[i].pos, once.points[i].pos) < 1e-8);
  for (size_t i = 0; i < blue.tunnels.size(); ++i)
    CHECK(dist(blue.tunnels[i].center, once.tunnels[i].center) < 1e-8);
  bool rejected = false;
  try { configure_own_team(blue, "red"); }
  catch (const std::invalid_argument&) { rejected = true; }
  CHECK(rejected && blue.own_team == "blue");

  // Rotate in the field frame before applying a real nonidentity map alignment.
  Config aligned = valid_config(tunnel_yaml({"[9, 5]", "[11, 5]", "[10, 5]"}) +
      "geometry_reference: red\n"
      "field_to_map: {enabled: true, dx: 1, dy: 2, dyaw_deg: 90}\n");
  configure_own_team(aligned, "blue");
  CHECK(dist(aligned.points[0].pos, {-12, 29}) < 1e-8); // (1,2)->(27,13)->(-12,29).
  CHECK(dist(aligned.tunnels[0].center, {-9, 20}) < 1e-8);
  CHECK(dist(aligned.tunnels[0].align[0], {-9, 21}) < 1e-8);
  CHECK(dist(aligned.tunnels[0].align[1], {-9, 19}) < 1e-8);

  Config custom = corridor();
  configure_own_team(custom, "blue");
  CHECK(custom.geometry_reference == "none" && custom.own_team == "blue");
  CHECK(custom.points[0].pos.x == 1 && custom.points[0].pos.y == 5);
  CHECK(custom.tunnels[0].align[0].x == 9 && custom.tunnels[0].align[1].x == 11);
  Config blue_reference = valid_config(minimal_yaml + "geometry_reference: blue\n");
  configure_own_team(blue_reference, "blue");
  CHECK(blue_reference.points[0].pos.x == 1 && blue_reference.points[0].pos.y == 2);
  invalid_config(minimal_yaml + "geometry_reference: yellow\n");
  rejected = false;
  try { configure_own_team(custom, "yellow"); }
  catch (const std::invalid_argument&) { rejected = true; }
  CHECK(rejected);
}

static void operator_coordinates_use_team_calibration()
{
  const auto cfg_text = minimal_yaml +
      "geometry_reference: red\n"
      "field_to_map: {enabled: true, dx: 3, dy: -4, dyaw_deg: 90}\n"
      "blue_field_to_map: {enabled: true, dx: -7, dy: 2, dyaw_deg: -30}\n";
  for (const auto* team : {"red", "blue"})
  {
    Config cfg = valid_config(cfg_text);
    configure_own_team(cfg, team);
    cfg.enable_tunnel = false;
    cfg.region_geometry.clear();
    DebugPorts ports;
    ports.op = {1, 4, -1, 7.69, 3.92};
    PlanState plan;
    handle_operator(ports, cfg, plan);
    const Vec2 field = std::string(team) == "blue"
                           ? Vec2{20.31, 11.08} : Vec2{7.69, 3.92};
    const Vec2 expected = apply_frame_transform(field, cfg.active_field_to_map);
    CHECK(plan.op_active);
    CHECK(std::abs(plan.op_target.pos.x - expected.x) < 1e-8);
    CHECK(std::abs(plan.op_target.pos.y - expected.y) < 1e-8);
  }

  // Older YAML keeps using field_to_map for blue until a separate calibration exists.
  Config legacy = valid_config(minimal_yaml +
      "field_to_map: {enabled: true, dx: 3, dy: -4, dyaw_deg: 90}\n");
  configure_own_team(legacy, "blue");
  CHECK(!legacy.has_field_to_map_blue);
  CHECK(legacy.active_field_to_map.dx == 3 && legacy.active_field_to_map.dyaw_deg == 90);

  // Custom corridors opt out of the 28x15 field convention entirely.
  auto custom = corridor();
  custom.region_geometry = {
      {"left", {{0, 0}, {9, 0}, {9, 10}, {0, 10}}},
      {"right", {{11, 0}, {20, 0}, {20, 10}, {11, 10}}}};
  configure_own_team(custom, "blue");
  DebugPorts ports;
  ports.op = {1, 4, -1, 7.69, 3.92};
  PlanState plan;
  handle_operator(ports, custom, plan);
  CHECK(plan.op_active);
  CHECK(dist(plan.op_target.pos, {7.69, 3.92}) < 1e-8);
}

int main()
{
  try
  {
    configured_team_geometry();
    operator_coordinates_use_team_calibration();
    operator_topic_defaults();
    configuration_validation();
    configuration_structure_and_disabled_features();
    uptime_and_config();
    match_before_first_pose();
    ambiguous_region_keeps_bookkeeping_and_fold_requirement();
    contradictory_region_waits_without_guessing_new_region();
    center_feedback_and_recovery();
    center_resume_direction();
    center_and_exit_share_timeout();
    unfolding_timeout();
    navigation_timeout_and_empty_target_are_idempotent();
    std::puts("PASS: production tree recovery, timeout, goal ownership and configuration regressions");
    return 0;
  }
  catch (const std::exception& e) { std::fprintf(stderr, "%s\n", e.what()); return 1; }
}
