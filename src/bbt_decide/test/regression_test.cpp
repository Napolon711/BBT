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
      {"safety", "hp_home_ratio"}, {"safety", "hp_resume_ratio"}, {"safety", "spin_w"},
      {"safety", "odom_stale_ms"}, {"safety", "patrol_dwell_s"}, {"safety", "switch_dwell_s"},
      {"safety", "gate_clear_m"}, {"safety", "op_goal_timeout_s"}, {"safety", "nav_timeout_s"},
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
      {"safety", "patrol_dwell_s"}, {"safety", "switch_dwell_s"},
      {"safety", "nav_retry_s"}, {"auto_buy", "cooldown_s"}};
  for (const auto& field : nonnegative_fields)
    invalid_config(minimal_yaml + field.section + ": {" + field.name + ": -0.1}\n");
  for (const auto* timeout : {"nav_timeout_s", "fold_timeout_s", "fold_stale_ms",
                            "odom_stale_ms", "op_goal_timeout_s", "tunnel_retry_cooldown_s"})
    for (const auto* value : {"0", "-0.1"})
      invalid_config(minimal_yaml + "safety: {" + timeout + ": " + value + "}\n");
  invalid_config(minimal_yaml + "auto_buy: {hp_ratio: 1.1}\n");

  const auto zero = valid_config(minimal_yaml +
      "field_to_map: {enabled: true, dx: -1, dy: -2, dyaw_deg: -90}\n"
      "safety: {hp_home_ratio: 0, hp_resume_ratio: 0, gate_clear_m: 0, in_tunnel_margin_m: 0, "
      "arrive_radius_m: 0, patrol_dwell_s: 0, switch_dwell_s: 0, nav_retry_s: 0, "
      "spin_w: -1.5}\n"
      "outpost: {snap_m: 0}\n"
      "auto_buy: {hp_ratio: -1, cooldown_s: 0}\n");
  CHECK(zero.gate_clear_m == 0 && zero.in_tunnel_margin_m == 0 && zero.arrive_radius_m == 0);
  CHECK(zero.op_snap_m == 0 && zero.patrol_dwell_s == 0 && zero.switch_dwell_s == 0);
  CHECK(zero.nav_retry_s == 0 && zero.buy_cooldown_s == 0);
  CHECK(zero.hp_buy_ratio == -1 && zero.spin_w == -1.5);
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

static Config corridor()
{
  Config c;
  c.regions = {"left", "right"}; c.enable_auto_buy = false; c.enable_outpost = false;
  c.switch_dwell_s = 0; c.switch_confirm_frames = 1; c.patrol_dwell_s = 100;
  c.points = {{1,{1,5},kZoneHome,0}, {10,{5,5},kZonePatrol,0}, {11,{15,5},kZonePatrol,1}};
  TunnelDef t; t.align[0]={9,5}; t.align[1]={11,5}; t.align_region[0]=0; t.align_region[1]=1;
  c.tunnels={t}; return c;
}
struct FeedbackPorts : DebugPorts
{
  bool fold_set(bool fold_request) override
  {
    ++(fold_request ? fold_cmd_count : unfold_cmd_count);
    return true; // No synthetic completion: the test supplies actual feedback.
  }
};
// Drive the real executor through entry and folding to the crossing phase.
static void enter(FeedbackPorts& p, const Config& c, PlanState& plan, ExecState& ex)
{
  Sense s; s.match_running=true; s.pos_valid=true; s.pos={5,5};
  Decision d; d.target_point=11;
  CHECK(tunnel_handled(p,c,s,d,plan,ex,0));
  p.goal=kGoalSuccess; ex.tunnel.step(p,c,plan,.1);
  ex.tunnel.step(p,c,plan,.2); p.fold=kFoldFolded;
  ex.tunnel.step(p,c,plan,.3); ex.tunnel.step(p,c,plan,.4);
  CHECK(ex.tunnel.phase==TunnelRunner::kWaitExit);
}
static void unknown_control()
{
  Config c=corridor(); c.enable_tunnel=false; c.enable_outpost=true;
  c.points.push_back({20,{6,6},kZoneRetake,0});
  DebugPorts p; p.ref.game_progress=4; p.ref.hp=p.ref.hp_max=400;
  p.ref.our_outpost_hp=500; p.ref.enemy_outpost_hp=500;
  PlanState plan;
  CHECK(decide_tree(sense(p,c,0),c,plan,0).target_point==10);
  p.ref.our_outpost_ctrl=3; plan=PlanState{};
  CHECK(decide_tree(sense(p,c,0),c,plan,0).target_point==10);
}
static void navigation_timeout()
{
  Config c=corridor(); DebugPorts p; p.ref.game_progress=4; p.ref.hp=50; p.ref.hp_max=400;
  PlanState plan; ExecState ex;
  step(p,c,plan,ex,0); step(p,c,plan,ex,26);
  CHECK(p.cancel_count==1 && p.stop_count==1 && ex.applied_goal==-2);
  step(p,c,plan,ex,27); CHECK(p.goal_count==1);
  step(p,c,plan,ex,29); CHECK(p.goal_count==2 && p.goal_x==1);
}
static void crossing_feedback_loss()
{
  Config c=corridor(); FeedbackPorts p; PlanState plan; ExecState ex;
  enter(p,c,plan,ex); p.fold=kFoldUnknown;
  const int cancels_before = p.cancel_count;
  ex.tunnel.step(p,c,plan,.5);
  CHECK(ex.tunnel.active() && p.cancel_count==cancels_before+1 && ex.tunnel.crossing_started);
  Sense s; s.match_running=true; Decision home; home.target_point=1;
  const int count=p.goal_count;
  CHECK(tunnel_handled(p,c,s,home,plan,ex,1)); CHECK(p.goal_count==count);
  CHECK(tunnel_handled(p,c,s,home,plan,ex,7)); CHECK(p.goal_count==count);
  p.fold=kFoldFolded;
  CHECK(p.fold_cmd_count==2); // Recovery reissues folding instead of waiting forever.
  CHECK(tunnel_handled(p,c,s,home,plan,ex,7.1));
  CHECK(tunnel_handled(p,c,s,home,plan,ex,7.2));
  CHECK(p.goal_x==11 && p.goal_count==count+1); // Finish chosen crossing before reversing.
}
static void unfolding_timeout()
{
  Config c=corridor(); FeedbackPorts p; PlanState plan; ExecState ex;
  enter(p,c,plan,ex); p.goal=kGoalSuccess; ex.tunnel.step(p,c,plan,.5);
  CHECK(plan.current_region==1); // Arrival, independent of unfold acknowledgment.
  ex.tunnel.step(p,c,plan,.6); ex.tunnel.step(p,c,plan,4);
  CHECK(ex.tunnel.active() && ex.tunnel.exit_reached && plan.current_region==1);
  Sense s; s.match_running=true; Decision home; home.target_point=1;
  const int count=p.goal_count;
  p.fold=kFoldUnfolded; // Late acknowledgment must not let ordinary navigation bypass a tunnel.
  CHECK(tunnel_handled(p,c,s,home,plan,ex,4.1)); CHECK(p.goal_count==count);
  CHECK(tunnel_handled(p,c,s,home,plan,ex,10));
  CHECK(tunnel_handled(p,c,s,home,plan,ex,10.1));
  CHECK(!ex.tunnel.active() && plan.current_region==1);
}
static void old_goal_after_round_trip()
{
  Config c=corridor(); FeedbackPorts p; PlanState plan; ExecState ex;
  Sense s; s.match_running=true; bool force=false;
  nav_cruise(p,c,s,1,ex,0,force); enter(p,c,plan,ex);
  CHECK(ex.applied_goal==-2);
  p.goal=kGoalSuccess; ex.tunnel.step(p,c,plan,.5); ex.tunnel.step(p,c,plan,.6);
  p.fold=kFoldUnfolded; ex.tunnel.step(p,c,plan,.7);
  s.pos_valid=true; s.pos={15,5}; Decision home; home.target_point=1;
  CHECK(tunnel_handled(p,c,s,home,plan,ex,1));
  CHECK(p.goal_x==11);
  p.goal=kGoalSuccess; ex.tunnel.step(p,c,plan,1.1); ex.tunnel.step(p,c,plan,1.2);
  p.fold=kFoldFolded; ex.tunnel.step(p,c,plan,1.3); ex.tunnel.step(p,c,plan,1.4);
  p.goal=kGoalSuccess; ex.tunnel.step(p,c,plan,1.5); ex.tunnel.step(p,c,plan,1.6);
  p.fold=kFoldUnfolded; ex.tunnel.step(p,c,plan,1.7);
  CHECK(!ex.tunnel.active() && plan.current_region==0);
  s.nav_state=p.goal; nav_cruise(p,c,s,1,ex,2,force);
  CHECK(p.goal_x==1 && ex.applied_goal==1);
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
  CHECK(std::abs(find_point(c,1)->pos.x-25.39)<1e-8);
  CHECK(std::abs(find_point(c,1)->pos.y-13.48)<1e-8);
  CHECK(find_point(c,20)->region==c.start_region); // Preserve manually assigned own-side region.

}
static void review_regressions()
{
  auto c=corridor(); DebugPorts p; PlanState plan; ExecState ex;
  p.ref.game_progress=4;p.ref.hp=p.ref.hp_max=400;
  p.pose={5,5,0,0,true};p.op={1,4,10};
  step(p,c,plan,ex,0);
  CHECK(p.goal_count==1 && !plan.op_fresh && plan.op_goal==10);
  p.op={2,4,10};step(p,c,plan,ex,.1);
  CHECK(p.goal_count==2); // Fresh same-point click must resend even inside arrival radius.
  c.regions.push_back("island");c.points.push_back({99,{25,5},kZonePatrol,2});
  p.op={3,4,99};handle_operator(p,c,plan);CHECK(plan.op_goal==10);
  plan.op_goal=11;plan.op_since_s=0;plan.op_fresh=false;
  auto sensed=sense(p,c,121);decide_tree(sensed,c,plan,121);CHECK(plan.op_goal==-1);
  auto disabled=c.tunnels[0];disabled.align_region[0]=disabled.align_region[1]=-1;
  c.tunnels={disabled};CHECK(!too_close_to_gate(c,{10,5}));
  TunnelRunner t;CHECK(!t.start(p,c,-1,0,nullptr,c.points[0],0));
  CHECK(!t.start(p,c,999,0,nullptr,c.points[0],0));

  c=corridor();p=DebugPorts{};ex=ExecState{};Sense s;bool resend=false;
  for(int i=0;i<200;++i) nav_cruise(p,c,s,-1,ex,i*.05,resend);
  CHECK(p.cancel_count==1 && p.stop_count==1);
  p=DebugPorts{};p.fold=kFoldUnknown;ex=ExecState{};plan=PlanState{};
  Decision d;d.target_point=11;s.match_running=true;
  for(int i=0;i<90;++i)tunnel_handled(p,c,s,d,plan,ex,i*.05);
  CHECK(p.cancel_count==1 && p.stop_count==1);

}

int main()
{
  review_regressions();
  operator_topic_defaults(); configuration_validation();
  unknown_control(); navigation_timeout(); crossing_feedback_loss(); unfolding_timeout();
  old_goal_after_round_trip(); uptime_and_config();
  std::puts("PASS: unknown control, navigation timeout, crossing recovery, goal ownership, uptime, manual map configuration");
}
