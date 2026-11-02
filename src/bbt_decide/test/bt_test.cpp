// Production behavior-tree tests at the public Ports boundary.  The real XML
// owns workflow order; tests supply sensor/action feedback and inspect commands.
#include <cmath>
#include <cstdio>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "bbt_decide/bt.hpp"

using namespace bbt_decide;

static int g_failures = 0;
#define CHECK(cond)                                                          \
  do                                                                         \
  {                                                                          \
    if (!(cond))                                                             \
    {                                                                        \
      ++g_failures;                                                          \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);             \
    }                                                                        \
  } while (0)

static bool near(double a, double b) { return std::abs(a - b) < 1e-8; }

// DebugPorts already records outputs. Only hardware acceptance/confirmation is
// controlled here, without a test-only entry point in the production tree.
struct ControlledPorts : DebugPorts
{
  bool accept_goals = true;
  bool accept_through = true;
  bool confirm_fold = true;
  std::vector<Vec2> goals;
  bool queued_commands = false;
  std::deque<OpCmd> commands;

  OpCmd op_cmd() override
  {
    if (!queued_commands) return DebugPorts::op_cmd();
    if (commands.empty()) return {};
    const auto command = commands.front();
    commands.pop_front();
    return command;
  }
  bool nav_set_goal(double x, double y) override
  {
    if (!accept_goals) return false;
    goals.push_back({x, y});
    return DebugPorts::nav_set_goal(x, y);
  }
  bool nav_through_poses(const std::vector<Vec2>& poses) override
  {
    if (!accept_through) return false;
    return DebugPorts::nav_through_poses(poses);
  }
  bool fold_set(bool folding) override
  {
    if (!fold_capable) return false;
    if (confirm_fold) return DebugPorts::fold_set(folding);
    ++(folding ? fold_cmd_count : unfold_cmd_count);
    return true;
  }
};

struct TreeBench
{
  std::shared_ptr<ControlledPorts> ports = std::make_shared<ControlledPorts>();
  std::shared_ptr<const Config> config;
  BT::Blackboard::Ptr blackboard;
  BT::Tree tree;

  explicit TreeBench(const Config& cfg)
  {
    config = std::make_shared<const Config>(cfg);
    PlanState plan;
    plan.current_region = cfg.start_region;
    blackboard = make_blackboard(config, ports, std::move(plan));
    BT::BehaviorTreeFactory factory;
    register_nodes(factory);
    tree = factory.createTreeFromFile(SRC_DIR "/trees/main.xml", blackboard);
    ports->pose = {2.0, 5.0, 0.0, 0, true};
    ports->ref.game_progress = 4;
    ports->ref.hp = 400;
    ports->ref.hp_max = 400;
    ports->ref.shots = 200;
  }

  void tick(double dt = 0.05)
  {
    ports->time_s += dt;
    ports->pose.stamp_ms = static_cast<uint64_t>(ports->time_s * 1000.0);
    tree.tickOnce();
  }
  void repeat(int count = 5) { for (int i = 0; i < count; ++i) tick(); }
  bool until(const std::function<bool()>& ready, int max_ticks = 20)
  {
    for (int i = 0; i < max_ticks && !ready(); ++i) tick();
    return ready();
  }
  void click(double x, double y, uint32_t seq = 1)
  {
    ports->op = {seq, 4, -1, x, y};
  }
  void arrive(double x, double y)
  {
    ports->pose.x = x;
    ports->pose.y = y;
    ports->goal = kGoalSuccess;
  }
  bool target(double x, double y) const
  {
    return ports->goal_count > 0 && near(ports->goal_x, x) && near(ports->goal_y, y);
  }
};

static Config local_cfg()
{
  Config c;
  c.enable_tunnel = false;
  c.enable_outpost = false;
  c.switch_confirm_frames = 1;
  c.switch_dwell_s = 0.0;
  c.patrol_dwell_s = 1000.0;
  c.regions = {"local"};
  c.region_geometry = {{"local", {{0, 0}, {20, 0}, {20, 10}, {0, 10}}}};
  c.points = {{1, {1.0, 5.0}, kZoneHome, 0},
              {10, {5.0, 5.0}, kZonePatrol, 0},
              {11, {15.0, 5.0}, kZonePatrol, 0}};
  return c;
}

static void test_no_tunnel_without_regions_accepts_finite_original_coordinates()
{
  auto cfg = local_cfg();
  cfg.regions.clear();
  cfg.region_geometry.clear();
  for (auto& point : cfg.points) point.region = -1;
  TreeBench b(cfg);
  b.click(27.125, -3.75);
  b.tick();
  CHECK(b.target(27.125, -3.75));
  CHECK(b.ports->through_count == 0);
  const int goals = b.ports->goal_count;
  const int cancelled = b.ports->cancel_count;
  for (const Vec2 invalid : {Vec2{std::numeric_limits<double>::infinity(), 2},
                             Vec2{2, std::numeric_limits<double>::quiet_NaN()}})
  {
    b.click(invalid.x, invalid.y, ++b.ports->op.seq);
    b.tick();
    CHECK(b.target(27.125, -3.75));
    CHECK(b.ports->goal_count == goals && b.ports->cancel_count == cancelled);
  }

  // Optional geometry still constrains clicks even when no tunnels are configured.
  TreeBench bounded(local_cfg());
  bounded.click(7.25, 7.5);
  bounded.tick();
  const int bounded_goals = bounded.ports->goal_count;
  bounded.click(27.125, -3.75, 2);
  bounded.tick();
  CHECK(bounded.target(7.25, 7.5));
  CHECK(bounded.ports->goal_count == bounded_goals);
}

static void test_operator_burst_preserves_last_valid_navigation_command()
{
  TreeBench b(local_cfg());
  b.click(7.25, 7.5);
  b.tick();
  CHECK(b.target(7.25, 7.5));
  b.ports->queued_commands = true;
  b.ports->commands = {
      {2, 4, -1, 8.25, 7.5},
      {3, 1, 100, 0, 0}, // Human purchase never replaces a movement command.
      {4, 4, -1, 40, 40}, // Invalid position is skipped independently.
      {5, 4, -1, 9.25, 7.5},
      {6, 4, -1, 9.25, 7.5},
      {7, 4, -1, std::numeric_limits<double>::quiet_NaN(), 5}};
  const int goals = b.ports->goal_count;
  b.tick();
  CHECK(b.target(9.25, 7.5));
  CHECK(b.ports->goal_count == goals + 1); // Only the final valid intent executes this tick.
  CHECK(b.ports->commands.empty());

  b.ports->commands = {{8, 2, 1, 0, 0}, {9, 4, -1, 10.25, 7.5},
                       {10, 4, -1, 40, 40}};
  b.tick();
  CHECK(b.target(10.25, 7.5));
  CHECK(b.ports->commands.empty());
  b.ports->ref.out_of_combat = 1;
  b.arrive(10.25, 7.5);
  b.tick();
  b.tick(19);
  const int arrived_goals = b.ports->goal_count;
  b.ports->commands = {{11, 4, -1, 10.25, 7.5}, {12, 3, 1, 0, 0},
                       {13, 4, -1, std::numeric_limits<double>::infinity(), 0}};
  b.tick();
  CHECK(b.ports->goal_count == arrived_goals);
  b.tick(1.1);
  CHECK(!b.target(10.25, 7.5)); // Duplicate never restarts the twenty-second hold.
}

static Config corridor_cfg()
{
  Config c = local_cfg();
  c.enable_tunnel = true;
  c.regions = {"left", "right"};
  c.region_geometry = {
      {"left", {{0, 0}, {9.5, 0}, {9.5, 10}, {0, 10}}},
      {"right", {{10.5, 0}, {20, 0}, {20, 10}, {10.5, 10}}}};
  c.start_region = 0;
  c.points[2].region = 1;
  TunnelDef t;
  t.name = "corridor_hole";
  t.align[0] = {9.0, 5.0}; t.align_region[0] = 0;
  t.align[1] = {11.0, 5.0}; t.align_region[1] = 1;
  t.center = {10.0, 5.0}; t.has_center = true;
  t.corners = {{9.25, 4.5}, {10.75, 4.5}, {10.75, 5.5}, {9.25, 5.5}};
  c.tunnels = {t};
  return c;
}

static void test_standby_is_quiet_and_end_stops_once()
{
  TreeBench b(local_cfg());
  b.ports->ref.game_progress = 0;
  b.repeat();
  CHECK(b.ports->goal_count == 0 && b.ports->cancel_count == 0);
  CHECK(b.ports->stop_count == 0 && b.ports->spin_count == 0);
  b.ports->ref.game_progress = 4;
  b.click(7.25, 7.5);
  b.tick();
  CHECK(b.target(7.25, 7.5));
  const int cancelled = b.ports->cancel_count;
  const int stopped = b.ports->stop_count;
  b.ports->ref.game_progress = 5;
  b.tick();
  CHECK(b.ports->cancel_count == cancelled + 1);
  CHECK(b.ports->stop_count == stopped + 1);
  CHECK(b.ports->spin_last_w == 0.0);
  CHECK(b.ports->attack_last_fire == 0);
  const int goals = b.ports->goal_count;
  const int spins = b.ports->spin_count;
  b.repeat();
  CHECK(b.ports->goal_count == goals && b.ports->spin_count == spins);
  CHECK(b.ports->cancel_count == cancelled + 1);
  CHECK(b.ports->stop_count == stopped + 1);
  // A cached click must not reappear as a new task on a new match.
  b.ports->ref.game_progress = 4;
  b.tick();
  CHECK(b.target(5.0, 5.0));
}

static void test_arbitrary_click_not_snapped_and_no_distance_arrival()
{
  auto c = local_cfg();
  c.nav_timeout_s = 1000;
  TreeBench b(c);
  b.click(7.25, 7.5);  // farther than 1.5m from every configured point
  b.tick();
  CHECK(b.target(7.25, 7.5));
  const int goals = b.ports->goal_count;
  b.ports->pose.x = 7.25; b.ports->pose.y = 7.5;
  b.ports->ref.out_of_combat = 1;
  b.ports->ref.hp = 10;  // even standing on the point, no success means still travelling
  b.tick(121.0);
  CHECK(b.target(7.25, 7.5) && b.ports->goal_count == goals);
  b.arrive(7.25, 7.5);
  b.tick();
  b.tick(19.9);
  CHECK(b.target(7.25, 7.5) && b.ports->goal_count == goals);
  b.tick(0.2);
  CHECK(b.target(1.0, 5.0));
}

static void test_duplicate_does_not_resend_or_reset_hold()
{
  TreeBench b(local_cfg());
  b.click(7.25, 7.5);
  b.tick();
  const int goals = b.ports->goal_count;
  const int cancels = b.ports->cancel_count;
  b.click(7.25, 7.5, 2);
  b.tick();
  CHECK(b.ports->goal_count == goals && b.ports->cancel_count == cancels);
  b.arrive(7.25, 7.5);
  b.ports->ref.out_of_combat = 1;
  b.tick();
  b.tick(15.0);
  b.click(7.25, 7.5, 3);
  b.tick();
  CHECK(b.ports->goal_count == goals && b.ports->cancel_count == cancels);
  CHECK(b.ports->spin_last_w == b.config->spin_w);
  CHECK(b.ports->attack_last_fire == 1);
  b.tick(5.0);
  CHECK(!b.target(7.25, 7.5));
  const int after_release = b.ports->goal_count;
  b.repeat();
  CHECK(b.ports->goal_count == after_release);  // cached seq cannot resurrect a consumed task
}

static void test_combat_clear_timer_and_unknown_feedback()
{
  TreeBench b(local_cfg());
  b.click(7.25, 7.5);
  b.tick();
  b.ports->ref.out_of_combat = 1;
  b.arrive(7.25, 7.5);
  b.tick();
  const int goals = b.ports->goal_count;
  b.tick(18.0);
  b.ports->ref.out_of_combat = 0;
  b.tick(0.0);
  b.tick(7.0);
  b.ports->ref.out_of_combat = 1;
  b.tick(0.0);
  b.tick(5.0);
  b.ports->ref.out_of_combat = 1;  // a fresh repeated true must not restart the clock
  b.tick(0.0);
  b.tick(4.9);
  CHECK(b.ports->goal_count == goals);
  b.tick(0.2);
  CHECK(b.ports->goal_count > goals);

  TreeBench unknown(local_cfg());
  unknown.click(7.25, 7.5);
  unknown.tick();
  unknown.arrive(7.25, 7.5);
  unknown.tick();
  const int held = unknown.ports->goal_count;
  unknown.tick(40.0);
  CHECK(unknown.ports->goal_count == held);  // unknown is not a combat-clear observation
  unknown.ports->ref.out_of_combat = 1;
  unknown.tick(0.0);
  unknown.tick(9.0);
  unknown.ports->ref.out_of_combat = -1;
  unknown.tick(0.0);
  unknown.ports->ref.out_of_combat = 1;
  unknown.tick(0.0);
  unknown.tick(9.9);
  CHECK(unknown.ports->goal_count == held);
  unknown.tick(0.2);
  CHECK(unknown.ports->goal_count > held);
}

static void test_home_priority_buying_and_no_return_to_consumed_click()
{
  TreeBench b(local_cfg());
  b.click(7.25, 7.5);
  b.ports->ref.hp = 100;
  b.ports->ref.shots = 10;
  b.ports->ref.coins = 1000;
  b.ports->ref.out_of_combat = 1;
  b.tick();
  CHECK(b.target(7.25, 7.5));
  CHECK(b.ports->buy_ammo_calls == 1 && b.ports->buy_hp_calls == 1);
  b.arrive(7.25, 7.5);
  b.tick();
  b.tick(19.9);
  CHECK(b.target(7.25, 7.5));
  b.tick(0.2);
  CHECK(b.target(1, 5));  // home wins when both release conditions become true
  b.ports->ref.hp = 400;
  b.ports->ref.shots = 200;
  b.tick();
  CHECK(b.target(5, 5));
  CHECK(!b.target(7.25, 7.5));

  TreeBench restored(local_cfg());
  restored.click(7.25, 7.5);
  restored.ports->ref.hp = 100;
  restored.tick();
  restored.arrive(7.25, 7.5);
  restored.tick();
  restored.ports->ref.hp = 400;
  restored.tick(20.0);
  CHECK(restored.target(7.25, 7.5)); // unknown combat keeps holding; old low-HP event is gone
  restored.ports->ref.out_of_combat = 1;
  restored.tick();
  restored.tick(10.1);
  CHECK(restored.target(5, 5));
}

static void test_new_coordinates_replace_old_and_get_fresh_timers()
{
  TreeBench b(local_cfg());
  b.click(7.25, 7.5);
  b.tick();
  b.arrive(7.25, 7.5);
  b.ports->ref.out_of_combat = 1;
  b.tick();
  b.tick(19.0);
  const int cancelled = b.ports->cancel_count;
  b.click(8.25, 8.5, 2);
  b.tick();
  CHECK(b.target(8.25, 8.5));
  CHECK(b.ports->cancel_count == cancelled + 1);
  b.arrive(8.25, 8.5);
  b.tick();
  b.tick(19.9);
  CHECK(b.target(8.25, 8.5));
  b.tick(0.2);
  CHECK(!b.target(8.25, 8.5));
}

static void test_hold_durations_are_configurable()
{
  auto c = local_cfg();
  c.op_min_hold_s = 3.0;
  c.op_combat_clear_s = 1.0;
  TreeBench b(c);
  b.click(7.25, 7.5);
  b.tick();
  b.arrive(7.25, 7.5);
  b.ports->ref.out_of_combat = 1;
  b.tick();
  b.tick(2.9);
  CHECK(b.target(7.25, 7.5));
  b.tick(0.2);
  CHECK(!b.target(7.25, 7.5));
}

static void test_failure_waits_two_seconds_and_duplicate_does_not_clear_retry()
{
  TreeBench b(local_cfg());
  b.click(7.25, 7.5);
  b.tick();
  const int goals = b.ports->goal_count;
  const int stops = b.ports->stop_count;
  b.ports->goal = kGoalFailure;
  b.tick();
  CHECK(b.ports->stop_count == stops + 1 && b.ports->spin_last_w == 0);
  const int cancelled = b.ports->cancel_count;
  b.tick(1.0);
  b.click(7.25, 7.5, 2);
  b.tick(0.0);
  b.tick(0.9);
  CHECK(b.ports->goal_count == goals);
  CHECK(b.ports->cancel_count == cancelled);
  b.tick(0.2);
  CHECK(b.ports->goal_count == goals + 1 && b.target(7.25, 7.5));
  CHECK(b.ports->spin_last_w == b.config->spin_w);
  // A new target is not trapped by the failed target's cooldown.
  b.ports->goal = kGoalFailure;
  b.tick();
  b.click(8.25, 8.5, 3);
  b.tick();
  CHECK(b.target(8.25, 8.5));
}

static void test_navigation_timeout_is_per_attempt_not_task_lifetime()
{
  TreeBench b(local_cfg());
  b.click(7.25, 7.5);
  b.tick();
  const int goals = b.ports->goal_count;
  b.tick(24.9);
  CHECK(b.ports->goal_count == goals);
  b.click(7.25, 7.5, 2);  // same coordinates cannot restart the 25s deadline
  b.tick(0.0);
  b.tick(0.2);
  CHECK(b.ports->spin_last_w == 0 && b.ports->stop_count > 0);
  b.tick(1.9);
  CHECK(b.ports->goal_count == goals);
  b.tick(0.2);
  CHECK(b.ports->goal_count == goals + 1 && b.target(7.25, 7.5));
  // Successful goals are not retried after their previous deadline.
  b.arrive(7.25, 7.5);
  b.tick();
  const int after_success = b.ports->goal_count;
  b.tick(40.0);
  CHECK(b.ports->goal_count == after_success);
}

static void test_rejected_navigation_and_empty_target_stop()
{
  TreeBench b(local_cfg());
  b.ports->accept_goals = false;
  b.click(7.25, 7.5);
  b.tick();
  CHECK(b.ports->stop_count > 0 && b.ports->spin_last_w == 0);
  b.ports->accept_goals = true;
  b.tick(1.9);
  CHECK(b.ports->goal_count == 0);
  b.tick(0.2);
  CHECK(b.target(7.25, 7.5));

  auto c = local_cfg();
  c.points.clear();
  TreeBench empty(c);
  empty.tick();
  CHECK(empty.ports->goal_count == 0 && empty.ports->stop_count == 1);
  CHECK(empty.ports->spin_last_w == 0);
  const int cancels = empty.ports->cancel_count;
  empty.repeat();
  CHECK(empty.ports->cancel_count == cancels && empty.ports->stop_count == 1);
}

static void start_crossing(TreeBench& b)
{
  b.ports->op = {1, 4, 11};
  b.tick();
  CHECK(b.target(9, 5));
  b.arrive(9, 5);
  CHECK(b.until([&] { return b.ports->through_count == 1; }));
  CHECK(b.ports->fold == kFoldFolded);
  CHECK(b.ports->through_poses.size() == 2);
  CHECK(b.target(11, 5));
}

static void test_region_geometry_free_click_and_invalid_click_preserves_task()
{
  TreeBench b(corridor_cfg());
  b.click(17.25, 7.5);
  b.tick();
  CHECK(b.target(9, 5));
  b.arrive(9, 5);
  CHECK(b.until([&] { return b.ports->through_count == 1; }));
  b.arrive(11, 5);
  CHECK(b.until([&] { return b.target(17.25, 7.5); }));
  const int goals = b.ports->goal_count;
  const int cancels = b.ports->cancel_count;
  for (const Vec2 bad : {Vec2{10, 5}, Vec2{25, 5},
                        Vec2{std::numeric_limits<double>::quiet_NaN(), 5}})
  {
    b.click(bad.x, bad.y, ++b.ports->op.seq);
    b.tick();
    CHECK(b.target(17.25, 7.5));
    CHECK(b.ports->goal_count == goals && b.ports->cancel_count == cancels);
  }
}

static void test_new_click_during_crossing_finishes_safe_transaction()
{
  TreeBench b(corridor_cfg());
  start_crossing(b);
  const int goals = b.ports->goal_count;
  const int cancels = b.ports->cancel_count;
  b.click(17.25, 7.5, 2);
  b.tick();
  CHECK(b.ports->goal_count == goals && b.ports->cancel_count == cancels);
  CHECK(b.ports->through_count == 1 && b.ports->fold == kFoldFolded);
  b.arrive(11, 5);
  CHECK(b.until([&] { return b.target(17.25, 7.5); }));
  CHECK(b.ports->unfold_cmd_count == 1);

  // Even a target back on the entrance side cannot reverse an active crossing.
  TreeBench reverse(corridor_cfg());
  start_crossing(reverse);
  const int through = reverse.ports->through_count;
  const int cancelled = reverse.ports->cancel_count;
  reverse.click(6.25, 7.5, 2);
  reverse.tick();
  CHECK(reverse.target(11, 5));
  CHECK(reverse.ports->through_count == through);
  CHECK(reverse.ports->cancel_count == cancelled);
  reverse.arrive(11, 5);
  CHECK(reverse.until([&] { return reverse.ports->unfold_cmd_count == 1; }));
  CHECK(reverse.ports->fold == kFoldUnfolded);
  CHECK(!reverse.target(6.25, 7.5));
}

static void test_arbitrary_click_crosses_multiple_tunnels_then_original_goal()
{
  auto c = corridor_cfg();
  c.tunnels[0].center = {9.8, 5.2};  // measured center differs from align midpoint
  c.regions.push_back("far");
  c.region_geometry.push_back({"far", {{20.5, 0}, {30, 0}, {30, 10}, {20.5, 10}}});
  c.region_geometry[1].polygon = {{10.5, 0}, {19.5, 0}, {19.5, 10}, {10.5, 10}};
  auto t = c.tunnels[0];
  t.name = "second_hole";
  t.align[0] = {19, 5}; t.align_region[0] = 1;
  t.align[1] = {21, 5}; t.align_region[1] = 2;
  t.center = {20.3, 4.8};
  t.corners = {{19.25, 4.5}, {20.75, 4.5}, {20.75, 5.5}, {19.25, 5.5}};
  c.tunnels.push_back(t);
  TreeBench b(c);
  b.click(27.25, 7.5);
  b.tick();
  CHECK(b.target(9, 5));
  b.arrive(9, 5);
  CHECK(b.until([&] { return b.ports->through_count == 1; }));
  CHECK(b.ports->through_poses.size() == 2);
  if (b.ports->through_poses.size() == 2)
  {
    CHECK(near(b.ports->through_poses[0].x, c.tunnels[0].center.x));
    CHECK(near(b.ports->through_poses[0].y, c.tunnels[0].center.y));
  }
  b.arrive(11, 5);
  CHECK(b.until([&] { return b.target(19, 5); }));
  b.arrive(19, 5);
  CHECK(b.until([&] { return b.ports->through_count == 2; }));
  CHECK(b.ports->through_poses.size() == 2);
  if (b.ports->through_poses.size() == 2)
  {
    CHECK(near(b.ports->through_poses[0].x, c.tunnels[1].center.x));
    CHECK(near(b.ports->through_poses[0].y, c.tunnels[1].center.y));
  }
  CHECK(b.target(21, 5));
  b.arrive(21, 5);
  CHECK(b.until([&] { return b.target(27.25, 7.5); }));
  CHECK(b.ports->unfold_cmd_count == 2);
}

static void test_measured_center_is_preserved_in_both_crossing_directions()
{
  for (int from_region : {0, 1})
  {
    auto c = corridor_cfg();
    c.start_region = from_region;
    c.tunnels[0].center = {9.75, 5.2};  // neither coordinate is the align midpoint
    TreeBench b(c);
    b.ports->pose.x = from_region == 0 ? 2.0 : 18.0;
    b.ports->op = {1, 4, from_region == 0 ? 11 : 10};
    b.tick();
    const Vec2 entry = c.tunnels[0].align[from_region];
    const Vec2 exit = c.tunnels[0].align[1 - from_region];
    CHECK(b.target(entry.x, entry.y));
    b.arrive(entry.x, entry.y);
    CHECK(b.until([&] { return b.ports->through_count == 1; }));
    CHECK(b.ports->through_poses.size() == 2);
    if (b.ports->through_poses.size() == 2)
    {
      CHECK(near(b.ports->through_poses[0].x, c.tunnels[0].center.x));
      CHECK(near(b.ports->through_poses[0].y, c.tunnels[0].center.y));
      CHECK(near(b.ports->through_poses[1].x, exit.x));
      CHECK(near(b.ports->through_poses[1].y, exit.y));
    }
    const int stops = b.ports->stop_count;
    b.ports->pose.x = c.tunnels[0].center.x;
    b.ports->pose.y = c.tunnels[0].center.y;
    b.tick();
    CHECK(b.ports->goal == kGoalRunning && b.ports->stop_count == stops);
    CHECK(b.ports->through_count == 1 && b.ports->unfold_cmd_count == 0);
    b.arrive(exit.x, exit.y);
    CHECK(b.until([&] { return b.target(from_region == 0 ? 15 : 5, 5); }));
    CHECK(b.ports->unfold_cmd_count == 1);
  }
}

static void test_tunnel_center_is_continuous_and_final_goal_still_required()
{
  TreeBench b(corridor_cfg());
  start_crossing(b);
  const int goals = b.ports->goal_count;
  const int stops = b.ports->stop_count;
  const int cancels = b.ports->cancel_count;
  b.ports->pose.x = 10; b.ports->pose.y = 5;
  b.repeat();
  CHECK(b.ports->goal == kGoalRunning);
  CHECK(b.ports->goal_count == goals && b.ports->through_count == 1);
  CHECK(b.ports->stop_count == stops && b.ports->cancel_count == cancels);
  CHECK(b.ports->unfold_cmd_count == 0);
  b.arrive(11, 5);
  CHECK(b.until([&] { return b.target(15, 5); }));
  CHECK(b.ports->unfold_cmd_count == 1);
  CHECK(b.ports->fold == kFoldUnfolded);
  // The exit's success cannot start the 20s operator hold at the final point.
  b.ports->ref.hp = 100;
  b.ports->ref.out_of_combat = 1;
  b.tick(20.1);
  CHECK(b.target(15, 5));
}

static void test_tunnel_waits_for_confirmed_fold_with_explicit_stop()
{
  TreeBench b(corridor_cfg());
  b.ports->confirm_fold = false;
  b.ports->op = {1, 4, 11};
  b.tick();
  b.arrive(9, 5);
  CHECK(b.until([&] { return b.ports->fold_cmd_count == 1; }));
  CHECK(b.ports->goal_count == 1 && b.ports->through_count == 0);
  CHECK(b.ports->stop_count > 0 && b.ports->spin_last_w == 0);
  CHECK(b.ports->attack_last_fire == 0);
  b.tick(1.0);
  CHECK(b.ports->goal_count == 1 && b.ports->through_count == 0);
  b.ports->fold = kFoldFolded;
  CHECK(b.until([&] { return b.ports->through_count == 1; }));
  CHECK(b.target(11, 5));
}

static void test_tunnel_recovery_preserves_actual_center_progress()
{
  for (bool past_center : {false, true})
  {
    TreeBench b(corridor_cfg());
    start_crossing(b);
    if (past_center)
    {
      b.ports->pose.x = 10.2;
      b.tick();
    }
    b.ports->goal = kGoalFailure;
    const int stops = b.ports->stop_count;
    b.tick();
    CHECK(b.ports->stop_count == stops + 1 && b.ports->spin_last_w == 0);
    CHECK(b.ports->attack_last_fire == 0);
    b.tick(4.9);
    CHECK(b.ports->through_count == 1);
    b.tick(0.2);
    CHECK(b.until([&] { return b.ports->through_count == 2; }));
    CHECK(b.ports->through_poses.size() == (past_center ? 1u : 2u));
    if (!b.ports->through_poses.empty())
      CHECK(near(b.ports->through_poses.front().x, past_center ? 11.0 : 10.0));
    CHECK(b.ports->fold == kFoldFolded && b.ports->unfold_cmd_count == 0);
    b.arrive(11, 5);
    CHECK(b.until([&] { return b.target(15, 5); }));
  }
}

static void test_exit_unfold_failure_does_not_cross_again()
{
  TreeBench b(corridor_cfg());
  start_crossing(b);
  b.ports->confirm_fold = false;
  b.arrive(11, 5);
  CHECK(b.until([&] { return b.ports->unfold_cmd_count == 1; }));
  CHECK(b.ports->spin_last_w == 0 && b.ports->attack_last_fire == 0);
  const int through = b.ports->through_count;
  b.tick(3.1);
  b.tick(4.9);
  CHECK(b.ports->unfold_cmd_count == 1);
  b.tick(0.2);
  CHECK(b.until([&] { return b.ports->unfold_cmd_count == 2; }));
  CHECK(b.ports->through_count == through);
  b.ports->fold = kFoldUnfolded;
  CHECK(b.until([&] { return b.target(15, 5); }));
}

static void test_match_end_during_crossing_preserves_fold_and_clears_task()
{
  TreeBench b(corridor_cfg());
  start_crossing(b);
  const int stopped = b.ports->stop_count;
  const int cancelled = b.ports->cancel_count;
  b.ports->ref.game_progress = 5;
  b.tick();
  CHECK(b.ports->stop_count == stopped + 1);
  CHECK(b.ports->cancel_count == cancelled + 1);
  CHECK(b.ports->spin_last_w == 0 && b.ports->attack_last_fire == 0);
  CHECK(b.ports->fold == kFoldFolded && b.ports->unfold_cmd_count == 0);
  CHECK(!b.blackboard->get<ExecState>("execution").tunnel.active());
  const int goals = b.ports->goal_count;
  b.repeat();
  CHECK(b.ports->goal_count == goals && b.ports->stop_count == stopped + 1);
}

static void test_no_pose_unknown_fold_and_missing_action_never_go_direct()
{
  TreeBench unknown_pose(corridor_cfg());
  unknown_pose.ports->pose.valid = false;
  unknown_pose.ports->op = {1, 4, 11};
  unknown_pose.tick();
  CHECK(unknown_pose.ports->goal_count == 0);
  CHECK(unknown_pose.ports->stop_count > 0 && unknown_pose.ports->spin_last_w == 0);

  TreeBench unknown_fold(corridor_cfg());
  unknown_fold.ports->fold = kFoldUnknown;
  unknown_fold.ports->op = {1, 4, 11};
  unknown_fold.tick();
  CHECK(unknown_fold.ports->goal_count == 0);
  CHECK(unknown_fold.ports->stop_count > 0 && unknown_fold.ports->spin_last_w == 0);

  TreeBench missing_action(corridor_cfg());
  missing_action.ports->accept_through = false;
  missing_action.ports->op = {1, 4, 11};
  missing_action.tick();
  missing_action.arrive(9, 5);
  missing_action.repeat(10);
  CHECK(missing_action.ports->goal_count == 1);
  CHECK(missing_action.ports->through_count == 0);
  CHECK(missing_action.ports->stop_count > 0 && missing_action.ports->spin_last_w == 0);
}

static void test_unsubmitted_crossing_can_be_replaced_outside_but_not_inside()
{
  TreeBench outside(corridor_cfg());
  outside.ports->accept_through = false;
  outside.ports->op = {1, 4, 11};
  outside.tick();
  outside.arrive(9, 5);
  outside.repeat(4);
  CHECK(outside.ports->through_count == 0);
  CHECK(outside.ports->stop_count > 0);
  outside.click(6.25, 7.5, 2);
  outside.tick();
  CHECK(outside.target(6.25, 7.5)); // No accepted request, so the outside route is replaceable.
  CHECK(outside.ports->through_count == 0);

  TreeBench inside(corridor_cfg());
  inside.ports->pose.x = 10.2;
  inside.ports->fold = kFoldFolded;
  inside.ports->accept_through = false;
  inside.ports->op = {1, 4, 11};
  inside.tick();
  inside.click(6.25, 7.5, 2);
  inside.tick();
  CHECK(!inside.target(6.25, 7.5)); // Actual tunnel occupancy still owns safe recovery.
  CHECK(inside.ports->spin_last_w == 0);
  inside.ports->accept_through = true;
  inside.tick(5.1);
  CHECK(inside.target(11, 5));
  CHECK(inside.ports->through_count == 1);
}

static void test_autonomous_rules_still_own_patrol_and_outpost()
{
  auto c = local_cfg();
  c.enable_outpost = true;
  c.points.push_back({20, {6, 8}, kZoneRetake, 0});
  c.points.push_back({21, {8, 8}, kZoneOutpost, 0});
  c.patrol_dwell_s = 15.0;
  TreeBench b(c);
  b.tick();
  CHECK(b.target(5, 5));
  b.tick(15.1);  // arrival not required for configured patrol rotation
  CHECK(b.target(15, 5));
  b.ports->ref.our_outpost_hp = 500;
  b.ports->ref.our_outpost_ctrl = kCtrlNone;
  b.ports->ref.enemy_outpost_hp = 500;
  b.ports->ref.enemy_outpost_ctrl = kCtrlNone;
  b.tick();
  CHECK(b.target(6, 8));
  b.ports->ref.our_outpost_ctrl = kCtrlFull;
  b.tick();
  CHECK(b.target(8, 8));
  CHECK(b.ports->attack_last_type == 2 && b.ports->attack_last_id == 1);
  CHECK(b.ports->attack_last_fire == 1);
  // Death does not add a new stop branch: lower controller owns lockout.
  b.ports->ref.hp = 0;
  b.tick();
  CHECK(b.target(1, 5));
  CHECK(b.ports->spin_last_w == c.spin_w);
}

static void test_home_hysteresis_switch_debounce_and_missing_home()
{
  auto c = local_cfg();
  c.switch_dwell_s = 2.0;
  c.switch_confirm_frames = 3;
  TreeBench b(c);
  b.tick();
  b.ports->ref.hp = 100;
  b.ports->ref.shots = 10;
  b.repeat(3);
  CHECK(b.target(5, 5));  // confirmed candidate still respects target dwell
  b.tick(2.0);
  CHECK(b.target(1, 5));
  b.ports->ref.hp = 400;
  b.repeat();
  CHECK(b.target(1, 5));  // HP alone cannot clear low-ammunition latch
  b.ports->ref.shots = 200;
  b.repeat(3);
  b.tick(2.0);
  CHECK(b.target(5, 5));

  c.points.erase(c.points.begin());
  TreeBench missing(c);
  missing.ports->ref.hp = 100;
  missing.tick();
  CHECK(missing.ports->goal_count == 0);
  CHECK(missing.ports->stop_count == 1 && missing.ports->spin_last_w == 0);
}

static void test_manual_purchases_ignored_and_auto_revive_uses_referee()
{
  TreeBench manual(local_cfg());
  for (int type : {1, 2, 3})
  {
    manual.ports->op = {static_cast<uint32_t>(type), type, 100};
    manual.tick();
  }
  CHECK(manual.ports->buy_ammo_calls == 0 && manual.ports->buy_hp_calls == 0);
  CHECK(manual.ports->buy_revive_calls == 0);
  for (int qualification : {0, 1, 2, -1})
  {
    for (int coins : {-1, 0, 119, 120, 150, 1000})
    {
      TreeBench b(local_cfg());
      b.ports->ref.hp = 0;
      b.ports->ref.coins = coins;
      b.ports->ref.revive_available = qualification;
      b.ports->ref.revive_cost = 120;
      b.tick();
      const bool allowed = qualification == 1 || (qualification == 2 && coins >= 120);
      CHECK(b.ports->buy_revive_calls == (allowed ? 1 : 0));
      b.repeat();
      CHECK(b.ports->buy_revive_calls == (allowed ? 1 : 0));
    }
  }
  auto c = local_cfg();
  c.enable_auto_buy = false;
  TreeBench off(c);
  off.ports->ref.hp = 0;
  off.ports->ref.revive_available = 1;
  off.tick();
  CHECK(off.ports->buy_revive_calls == 0);
  c.enable_auto_buy = true;
  c.enable_auto_revive = false;
  TreeBench revive_off(c);
  revive_off.ports->ref.hp = 0;
  revive_off.ports->ref.revive_available = 1;
  revive_off.tick();
  CHECK(revive_off.ports->buy_revive_calls == 0);

  TreeBench unknown_price(local_cfg());
  unknown_price.ports->ref.hp = 0;
  unknown_price.ports->ref.coins = 1000;
  unknown_price.ports->ref.revive_available = 2;
  unknown_price.tick();
  CHECK(unknown_price.ports->buy_revive_calls == 0);
}

int main()
{
  test_operator_burst_preserves_last_valid_navigation_command();
  test_no_tunnel_without_regions_accepts_finite_original_coordinates();
  test_standby_is_quiet_and_end_stops_once();
  test_arbitrary_click_not_snapped_and_no_distance_arrival();
  test_duplicate_does_not_resend_or_reset_hold();
  test_combat_clear_timer_and_unknown_feedback();
  test_home_priority_buying_and_no_return_to_consumed_click();
  test_new_coordinates_replace_old_and_get_fresh_timers();
  test_hold_durations_are_configurable();
  test_failure_waits_two_seconds_and_duplicate_does_not_clear_retry();
  test_navigation_timeout_is_per_attempt_not_task_lifetime();
  test_rejected_navigation_and_empty_target_stop();
  test_region_geometry_free_click_and_invalid_click_preserves_task();
  test_new_click_during_crossing_finishes_safe_transaction();
  test_arbitrary_click_crosses_multiple_tunnels_then_original_goal();
  test_measured_center_is_preserved_in_both_crossing_directions();
  test_tunnel_center_is_continuous_and_final_goal_still_required();
  test_tunnel_waits_for_confirmed_fold_with_explicit_stop();
  test_tunnel_recovery_preserves_actual_center_progress();
  test_exit_unfold_failure_does_not_cross_again();
  test_match_end_during_crossing_preserves_fold_and_clears_task();
  test_no_pose_unknown_fold_and_missing_action_never_go_direct();
  test_unsubmitted_crossing_can_be_replaced_outside_but_not_inside();
  test_autonomous_rules_still_own_patrol_and_outpost();
  test_home_hysteresis_switch_debounce_and_missing_home();
  test_manual_purchases_ignored_and_auto_revive_uses_referee();
  std::printf("bt_test: %d failure(s)\n", g_failures);
  return g_failures == 0 ? 0 : 1;
}
