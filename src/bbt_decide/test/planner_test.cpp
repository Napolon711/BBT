// Public behavior tests: drive the production tree through Ports, never a
// workflow phase or a private executor function.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <stdexcept>
#include <string>

#include "bbt_decide/planner.hpp"

using namespace bbt_decide;

#define CHECK(x) do { if (!(x)) throw std::runtime_error("line " + std::to_string(__LINE__) + ": " #x); } while (0)
static bool near(double a, double b) { return std::abs(a - b) < 1e-8; }

struct TestPorts : DebugPorts
{
  bool confirm_fold = true;
  bool accept_goal = true;
  bool fold_set(bool requested) override
  {
    if (!fold_capable) return false;
    if (confirm_fold) return DebugPorts::fold_set(requested);
    ++(requested ? fold_cmd_count : unfold_cmd_count);
    return true;
  }
  bool nav_set_goal(double x, double y) override
  {
    return accept_goal && DebugPorts::nav_set_goal(x, y);
  }
};

struct Bench
{
  Config cfg;
  TestPorts p;
  PlanState plan;
  ExecState ex;
  explicit Bench(Config config) : cfg(std::move(config))
  {
    p.ref.game_progress = 4;
    p.ref.hp = p.ref.hp_max = 400;
    p.ref.shots = 200;
    p.pose = {2, 5, 0, 0, true};
    plan.current_region = cfg.start_region;
  }
  int tick(double dt = .05, bool refresh_pose = true)
  {
    p.time_s += dt;
    if (refresh_pose) p.pose.stamp_ms = static_cast<uint64_t>(p.time_s * 1000);
    return step(p, cfg, plan, ex, p.time_s);
  }
  bool until(const std::function<bool()>& ready, int count = 20)
  {
    for (int i = 0; i < count && !ready(); ++i) tick();
    return ready();
  }
  bool target(double x, double y) const
  { return p.goal_count && near(p.goal_x, x) && near(p.goal_y, y); }
  void arrive(double x, double y)
  {
    p.pose.x = x; p.pose.y = y;
    p.goal = kGoalSuccess;
  }
};

static Config corridor()
{
  Config c;
  c.regions = {"left", "right"};
  c.region_geometry = {
      {"left", {{0, 0}, {9.5, 0}, {9.5, 10}, {0, 10}}},
      {"right", {{10.5, 0}, {20, 0}, {20, 10}, {10.5, 10}}}};
  c.enable_auto_buy = false;
  c.enable_outpost = false;
  c.switch_confirm_frames = 1;
  c.switch_dwell_s = 0;
  c.patrol_dwell_s = 1000;
  c.points = {{1, {1, 5}, kZoneHome, 0}, {10, {5, 5}, kZonePatrol, 0},
              {11, {15, 5}, kZonePatrol, 1}, {12, {19, 5}, kZonePatrol, 1}};
  TunnelDef t;
  t.name = "corridor";
  t.align[0] = {9, 5}; t.align_region[0] = 0;
  t.align[1] = {11, 5}; t.align_region[1] = 1;
  t.has_center = true; t.center = {10, 5};
  t.corners = {{9.25, 4.5}, {10.75, 4.5}, {10.75, 5.5}, {9.25, 5.5}};
  c.tunnels = {t};
  return c;
}

static Config local()
{
  Config c = corridor();
  c.enable_tunnel = false;
  c.regions = {"local"};
  c.region_geometry = {{"local", {{0, 0}, {20, 0}, {20, 10}, {0, 10}}}};
  c.tunnels.clear();
  for (auto& point : c.points) point.region = 0;
  return c;
}

static void test_geometry_and_routing()
{
  const Vec2 a{0, 0}, b{4, 0};
  CHECK(near(dist(a, b), 4));
  CHECK(near(project_param({2, 0}, a, b), .5));
  CHECK(project_param({-1, 0}, a, b) < 0);
  CHECK(project_param({5, 0}, a, b) > 1);
  CHECK(near(dist_to_segment({2, 3}, a, b), 3));
  CHECK(near(dist_to_segment({-1, 3}, a, b), std::sqrt(10)));
  Config c = corridor();
  auto parallel = c.tunnels[0];
  parallel.align[0] = {9, 9}; parallel.align[1] = {11, 9};
  parallel.center = {10, 9};
  c.tunnels.push_back(parallel);
  c.regions.push_back("detour");
  auto detour = parallel;
  detour.align_region[1] = 2; c.tunnels.push_back(detour);
  detour.align_region[0] = 2; detour.align_region[1] = 1; c.tunnels.push_back(detour);
  CHECK(route_tunnels(c, 0, 1) == std::vector<int>{0});
  const Vec2 near_first{8, 5}, near_parallel{8, 9}, reverse{12, 9};
  CHECK(route_tunnels(c, 0, 1, &near_first) == std::vector<int>{0});
  CHECK(route_tunnels(c, 0, 1, &near_parallel) == std::vector<int>{1});
  CHECK(route_tunnels(c, 1, 0, &reverse) == std::vector<int>{1});
  CHECK(find_tunnel(c, 0, 1) == 0 && find_tunnel(c, 0, 2) == 2);
  CHECK(regions_reachable(c, 0, 2));
  c.regions.push_back("island");
  CHECK(!regions_reachable(c, 0, 3));
  CHECK(!point_usable(c, 0, {99, {30, 5}, kZonePatrol, 3}));
  CHECK(!point_usable(c, 0, {99, {10, 5}, kZonePatrol, 0}));
  CHECK(point_usable(c, 0, c.points[2]));
}

static void test_continuous_center_crossing()
{
  for (int side : {0, 1})
  {
    auto c = corridor(); c.start_region = side;
    Bench b(c);
    b.p.confirm_fold = false;
    b.p.pose.x = side == 0 ? 5 : 15;
    b.p.op = {1, 4, side == 0 ? 11 : 10};
    const double entry = side == 0 ? 9 : 11;
    const double exit = side == 0 ? 11 : 9;
    b.tick();
    CHECK(b.target(entry, 5) && b.p.goal_count == 1);
    b.arrive(entry, 5);
    CHECK(b.until([&] { return b.p.fold_cmd_count == 1; }));
    const int entry_goals = b.p.goal_count;
    for (int i = 0; i < 4; ++i) b.tick();
    CHECK(b.p.goal_count == entry_goals && b.p.through_count == 0);
    CHECK(b.p.spin_last_w == 0 && b.p.attack_last_fire == 0 && b.p.stop_count > 0);
    b.p.fold = kFoldFolded;
    CHECK(b.until([&] { return b.p.through_count == 1; }));
    CHECK(b.target(exit, 5) && b.p.through_poses.size() == 2);
    CHECK(near(b.p.through_poses.front().x, 10));
    const int crossing_goals = b.p.goal_count;
    const int stopped = b.p.stop_count;
    const int cancelled = b.p.cancel_count;
    b.p.pose.x = 10; b.tick(); b.tick();
    CHECK(b.p.goal == kGoalRunning && b.p.goal_count == crossing_goals);
    CHECK(b.p.stop_count == stopped && b.p.cancel_count == cancelled);
    CHECK(b.p.unfold_cmd_count == 0 && b.plan.current_region == side);
    b.arrive(exit, 5);
    CHECK(b.until([&] { return b.p.unfold_cmd_count == 1; }));
    CHECK(b.plan.current_region == 1 - side && b.ex.tunnel.active());
    b.p.fold = kFoldUnfolded;
    CHECK(b.until([&] { return !b.ex.tunnel.active(); }));
    CHECK(b.until([&] { return b.target(side == 0 ? 15 : 5, 5); }));
    CHECK(b.plan.op_arrived_s < 0); // Exit success is not final-target success.
    CHECK(b.p.spin_last_w == c.spin_w && b.p.attack_last_fire == 1);
  }
}

static void test_chained_regions_and_region_reset()
{
  Config c = corridor();
  c.region_geometry.clear(); // Legacy ID channel is still supported without polygons.
  c.regions = {"north", "middle", "south"};
  c.points = {{1, {1, 5}, kZoneHome, 0}, {31, {13, 5}, kZonePatrol, 2}};
  auto first = c.tunnels[0];
  first.align[0] = {5, 5}; first.align[1] = {7, 5}; first.center = {6, 5};
  first.corners.clear();
  auto second = first;
  second.align[0] = {9, 5}; second.align[1] = {11, 5}; second.center = {10, 5};
  second.align_region[0] = 1; second.align_region[1] = 2;
  c.tunnels = {first, second};
  CHECK(route_tunnels(c, 0, 2) == (std::vector<int>{0, 1}));
  Bench b(c); b.plan.current_region = 2;
  b.tick();
  CHECK(b.plan.current_region == 0 && b.target(5, 5));
  for (int i = 0; i < 2; ++i)
  {
    const double entry = i == 0 ? 5 : 9;
    const double exit = i == 0 ? 7 : 11;
    CHECK(b.until([&] { return b.target(entry, 5); }));
    b.arrive(entry, 5);
    CHECK(b.until([&] { return b.target(exit, 5); }));
    b.arrive(exit, 5);
    CHECK(b.until([&] { return b.plan.current_region == i + 1; }));
    CHECK(b.until([&] { return !b.ex.tunnel.active() || b.ex.tunnel.tunnel == i + 1; }));
  }
  CHECK(b.until([&] { return b.target(13, 5); }));
  b.p.ref.game_progress = 5; b.tick();
  b.p.ref.game_progress = 4; b.p.pose.x = 2; b.tick();
  CHECK(b.plan.current_region == 0);
}

static void test_patrol_dwell_and_stale_pose()
{
  auto c = local(); c.patrol_dwell_s = 15;
  Bench b(c); b.p.pose.x = 5.6;
  b.tick(); CHECK(b.plan.cur_point == 10);
  b.p.pose.x = 5.2;
  b.tick(1.5, false); // Old odometry cannot satisfy the distance fallback.
  CHECK(b.plan.cur_point == 10);
  b.tick(); CHECK(b.plan.cur_point == 11);
  Bench timed(c); timed.tick();
  timed.tick(15.1);
  CHECK(timed.plan.cur_point == 11); // Preserve rotation even without arrival.
}

static void test_explicit_patrol_list_preserves_zone_and_order()
{
  auto c = corridor();
  c.enable_outpost = true;
  c.patrol_dwell_s = 15;
  c.points = {{1, {1, 5}, kZoneHome, 0},
              {101, {8, 6}, kZonePatrol, 0},
              {99, {15, 6}, kZonePatrol, 1},
              {28, {5, 6}, kZonePatrol, 0},
              {70, {7, 6}, kZonePatrol, 0},
              {40, {6, 6}, kZonePatrol, 0},
              {109, {7, 8}, kZoneOutpost, 0},
              {114, {8, 8}, kZoneOutpost, 0}};
  c.patrol_point_ids = {28, 40, 70, 101};
  Bench b(c); b.p.pose.x = 4;
  b.tick(); CHECK(b.target(5, 6));
  for (const Vec2 target : {Vec2{6, 6}, Vec2{7, 6}, Vec2{8, 6}, Vec2{5, 6}})
  {
    b.tick(15.1);
    CHECK(b.target(target.x, target.y));
    CHECK(b.p.through_count == 0); // Other-region patrol points remain configured, unused.
  }
  b.p.ref.enemy_outpost_hp = 500;
  b.p.ref.enemy_outpost_ctrl = kCtrlNone;
  b.tick(); CHECK(b.target(7, 8) && b.p.attack_last_type == 2);
  b.cfg.points.erase(std::remove_if(b.cfg.points.begin(), b.cfg.points.end(),
      [](const PointDef& point) { return point.id == 109; }), b.cfg.points.end());
  b.tick(); CHECK(b.target(8, 8) && b.p.attack_last_type == 2);
  CHECK(find_point(b.cfg, 114)->zone == kZoneOutpost);
  CHECK(find_point(b.cfg, 99)->zone == kZonePatrol); // No zone rewriting is needed.

  // Missing/empty lists preserve the legacy point-table patrol order.
  auto fallback = c;
  fallback.patrol_point_ids.clear();
  Bench legacy(fallback); legacy.p.pose.x = 4;
  legacy.tick(); CHECK(legacy.target(5, 6));
  legacy.tick(15.1); CHECK(legacy.target(7, 6));
}

static void test_resource_latches_and_priority()
{
  auto c = local(); c.enable_outpost = true;
  c.points.push_back({20, {8, 5}, kZoneRetake, 0});
  c.points.push_back({21, {12, 5}, kZoneOutpost, 0});
  Bench b(c);
  b.p.ref.our_outpost_hp = 500; b.p.ref.our_outpost_ctrl = kCtrlFull;
  b.p.ref.enemy_outpost_hp = 500; b.p.ref.enemy_outpost_ctrl = kCtrlNone;
  b.tick(); CHECK(b.plan.cur_point == 21);
  CHECK(b.p.attack_last_type == 2 && b.p.attack_last_id == 1);
  b.p.ref.our_outpost_hp = 0; b.tick();
  CHECK(b.plan.cur_point == 20 && b.p.attack_last_type == 0);
  b.p.ref.hp = 80; b.p.ref.shots = 30; b.tick();
  CHECK(b.plan.home_latched && b.plan.cur_point == 1);
  b.p.ref.hp = 280; b.tick(); CHECK(b.plan.home_latched);
  b.p.ref.shots = 200; b.tick(); CHECK(!b.plan.home_latched && b.plan.cur_point == 20);
  b.p.ref.our_outpost_hp = 500; b.p.ref.our_outpost_ctrl = kCtrlTemp; b.tick();
  CHECK(b.plan.cur_point == 20);
  b.p.ref.our_outpost_ctrl = kCtrlFull; b.p.ref.enemy_outpost_ctrl = kCtrlFull; b.tick();
  CHECK(b.plan.cur_point == 10);
  b.p.ref.our_outpost_ctrl = 3; b.p.ref.enemy_outpost_ctrl = 3; b.tick();
  CHECK(b.plan.cur_point == 10); // Unknown control values are never a takeover trigger.
  b.p.ref.shots = -1; b.p.ref.hp = 80; b.tick();
  b.p.ref.hp = 300; b.tick(); CHECK(!b.plan.home_latched);
  b.cfg.ammo_home_min = -1; b.p.ref.shots = 0; b.tick(); CHECK(!b.plan.home_latched);
}

static void test_match_and_death_contract()
{
  Bench b(local()); b.p.ref.game_progress = 0;
  b.tick(); b.tick();
  CHECK(b.p.goal_count == 0 && b.p.mode_count == 0 && b.p.stop_count == 0);
  b.p.ref.game_progress = 4; b.tick();
  CHECK(b.p.goal_count == 1 && b.p.mode_count == 1 && b.p.spin_last_w == b.cfg.spin_w);
  b.tick(); CHECK(b.p.goal_count == 1); // Cached slow referee data remains usable.
  b.p.ref.hp = 0; b.tick();
  CHECK(b.plan.home_latched && b.target(1, 5));
  CHECK(b.p.spin_last_w == b.cfg.spin_w); // Do not add a death-only stop branch.
  const int cancelled = b.p.cancel_count, stopped = b.p.stop_count;
  b.p.ref.game_progress = 5; b.tick();
  CHECK(b.p.cancel_count == cancelled + 1 && b.p.stop_count == stopped + 1);
  CHECK(b.p.spin_last_w == 0 && b.p.attack_last_fire == 0);
  const int spins = b.p.spin_count;
  b.tick(); b.tick();
  CHECK(b.p.cancel_count == cancelled + 1 && b.p.spin_count == spins);
  b.p.ref.game_progress = 4; b.tick(); CHECK(b.p.mode_count == 2);
}

static void test_buying_is_independent_and_uses_referee_revive_quote()
{
  auto c = local(); c.enable_auto_buy = true;
  Bench b(c); b.p.ref.coins = 1000; b.p.ref.shots = 10;
  b.p.op = {1, 4, 11}; b.tick();
  CHECK(b.p.buy_ammo_calls == 1 && b.p.buy_ammo_param == 100);
  b.tick(); CHECK(b.p.buy_ammo_calls == 1);
  b.tick(10.1); CHECK(b.p.buy_ammo_calls == 2);
  b.p.ref.shots = 300; b.p.ref.hp = 150; b.p.ref.out_of_combat = 1;
  b.tick(); CHECK(b.p.buy_hp_calls == 1 && b.p.buy_hp_param == 1);
  b.p.ref.hp = 0; b.p.ref.revive_available = 2; b.p.ref.revive_cost = 120;
  b.p.ref.coins = 150; b.tick(); CHECK(b.p.buy_revive_calls == 1);
  b.p.ref.coins = 119; b.tick(10.1); CHECK(b.p.buy_revive_calls == 1);
  b.p.ref.revive_available = 1; b.p.ref.coins = -1;
  b.tick(); CHECK(b.p.buy_revive_calls == 2); // Free revive does not require known coins.
  b.cfg.enable_auto_revive = false;
  b.tick(10.1); CHECK(b.p.buy_revive_calls == 2);
  b.cfg.enable_auto_buy = false; b.p.ref.shots = 0; b.p.ref.hp = 100;
  const int hp = b.p.buy_hp_calls, ammo = b.p.buy_ammo_calls;
  b.tick(10.1); CHECK(b.p.buy_hp_calls == hp && b.p.buy_ammo_calls == ammo);
  b.p.op = {2, 1, 100}; b.tick();
  b.p.op = {3, 2, 100}; b.tick();
  b.p.op = {4, 3, 0}; b.tick();
  CHECK(b.p.buy_hp_calls == hp && b.p.buy_ammo_calls == ammo && b.p.buy_revive_calls == 2);
}

static void test_tunnel_disabled_and_home_unavailable()
{
  auto c = local(); c.regions.clear(); c.region_geometry.clear();
  for (auto& p : c.points) p.region = -1;
  Bench b(c); b.tick(); CHECK(b.target(5, 5));
  b.arrive(5, 5); b.tick(); CHECK(b.target(15, 5) && !b.ex.tunnel.active());
  b.p.ref.hp = 80; b.tick(); CHECK(b.target(1, 5));
  b.cfg.points.erase(b.cfg.points.begin()); b.tick();
  CHECK(b.plan.home_latched && b.p.stop_count > 0 && b.p.spin_last_w == 0);
}

static void test_unsafe_tunnel_start_waits()
{
  Bench b(corridor()); b.p.pose.x = 10.4; b.p.op = {1, 4, 11};
  b.tick(); CHECK(b.p.goal_count == 0 && b.p.stop_count > 0 && b.p.spin_last_w == 0);
  b.p.fold = kFoldFolded;
  b.tick(b.cfg.tunnel_retry_cooldown_s + .1);
  CHECK(b.until([&] { return b.target(11, 5); }));
  CHECK(b.p.through_count == 1); // Folded recovery heads to exit, not the far entry.
  Bench unknown(corridor()); unknown.p.pose.valid = false; unknown.p.op = {1, 4, 11};
  for (int i = 0; i < 5; ++i) unknown.tick();
  CHECK(unknown.p.goal_count == 0 && unknown.p.fold_cmd_count == 0);
  CHECK(unknown.p.cancel_count == 1 && unknown.p.stop_count == 1);
}

int main()
{
  try
  {
    test_geometry_and_routing();
    test_continuous_center_crossing();
    test_chained_regions_and_region_reset();
    test_patrol_dwell_and_stale_pose();
    test_explicit_patrol_list_preserves_zone_and_order();
    test_resource_latches_and_priority();
    test_match_and_death_contract();
    test_buying_is_independent_and_uses_referee_revive_quote();
    test_tunnel_disabled_and_home_unavailable();
    test_unsafe_tunnel_start_waits();
    std::puts("planner_test: production tree behavior passed");
    return 0;
  }
  catch (const std::exception& e) { std::fprintf(stderr, "%s\n", e.what()); return 1; }
}
