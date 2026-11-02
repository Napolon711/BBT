#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <thread>
#include "bbt_decide/ros_ports.hpp"
using namespace bbt_decide;
#define CHECK(x) do { if (!(x)) throw std::runtime_error("line " + std::to_string(__LINE__) + ": " #x); } while (0)
class ClockPorts : public RosPorts
{
public:
  using RosPorts::RosPorts;
  double offset=0;
  double now_s() override { return RosPorts::now_s()+offset; }
};
void check_team(int own_faction)
{
  auto node=std::make_shared<rclcpp::Node>("bbt_ports_test_"+std::to_string(own_faction), rclcpp::NodeOptions().use_intra_process_comms(true));
  RosPortsConfig cfg; cfg.own_faction=own_faction; cfg.referee_2027=true; cfg.referee_stale_ms=2000;
  const bool red=own_faction==1;
  CHECK(cfg.op_cmd_topic=="/sentry/operator_point");
  CHECK(cfg.op_cmd_xy_topic=="/referee/operator_cmd");
  ClockPorts ports(node.get(),cfg);
  auto refpub=node->create_publisher<std_msgs::msg::Float64MultiArray>(cfg.referee_topic,10);
  auto foldpub=node->create_publisher<std_msgs::msg::Int32>(cfg.fold_state_topic,10);
  auto statuspub=node->create_publisher<rm2027_interfaces::msg::RobotStatus>(cfg.robot_status_topic,10);
  auto hppub=node->create_publisher<rm2027_interfaces::msg::RobotHP>(cfg.robot_hp_topic,10);
  auto eventpub=node->create_publisher<rm2027_interfaces::msg::FieldEvent>(cfg.field_event_topic,10);
  auto infopub=node->create_publisher<rm2027_interfaces::msg::SentryInfo>(cfg.sentry_info_topic,10);
  std::vector<rm2027_interfaces::msg::SentryCommand> commands;
  std::vector<int> modes;
  auto sub=node->create_subscription<rm2027_interfaces::msg::SentryCommand>(cfg.purchase_topic,10,
      [&](rm2027_interfaces::msg::SentryCommand::SharedPtr m){commands.push_back(*m);});
  auto modesub=node->create_subscription<std_msgs::msg::Int32>(cfg.chassis_mode_topic,10,
      [&](std_msgs::msg::Int32::SharedPtr m){modes.push_back(m->data);});
  const auto pump=[&](){
    for(int i=0;i<15;++i){rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  };
  const auto deliver=[&](auto pub,const auto& msg){
    for(int i=0;i<100 && pub->get_subscription_count()==0;++i) pump();
    CHECK(pub->get_subscription_count()>0); pub->publish(msg); pump();
  };
  std_msgs::msg::Float64MultiArray ref;
  ref.data={4,400,400,150,500,500,0,1,300,10000,5000,300,0,600};
  deliver(refpub,ref);
  // A UC summary is already in the configured team's perspective; no identity is required.
  CHECK(ports.referee().our_outpost_hp==500 && ports.referee().enemy_outpost_hp==600);
  CHECK(ports.referee().robot_id==0);
  CHECK(ports.referee().our_outpost_ctrl==0);
  ref.data[9]=4000000000.0; deliver(refpub,ref);
  CHECK(ports.referee().hp==400); // uint32 source timestamp exceeds signed-int range.
  for(int size:{9,10,11,12,13,14})
  {
    auto optional=ref;optional.data.resize(size);
    deliver(refpub,optional);
    const auto snapshot=ports.referee();
    CHECK(snapshot.hp==400 && snapshot.our_outpost_hp==500 && snapshot.our_outpost_ctrl==0);
    CHECK(snapshot.our_base_hp==(size>10?5000:-1));
    CHECK(snapshot.ally_ctrl_deg==(size>11?300:-1));
    CHECK(snapshot.enemy_outpost_hp==(size>13?600:-1));
  }
  for(int size:{0,8,15,16,17,18})
  {
    std_msgs::msg::Float64MultiArray bad; bad.data.resize(size,0);
    deliver(refpub,bad); CHECK(ports.referee().hp==400);
  }
  auto bad=ref; bad.data[1]=std::numeric_limits<double>::quiet_NaN();
  deliver(refpub,bad); CHECK(ports.referee().hp==400);
  std_msgs::msg::Float64MultiArray legacy;
  legacy.data={4,320,400,100,500,501,0,601,2,300,5000,4000,10,20,7,1,100,300,1};
  deliver(refpub,legacy); CHECK(ports.referee().hp==320 && ports.referee().enemy_outpost_ctrl==2);
  deliver(refpub,ref);

  std_msgs::msg::Int32 folded; folded.data=kFoldFolded;
  deliver(foldpub,folded); CHECK(ports.fold_state()==kFoldFolded);
  ports.fold_set(true); CHECK(ports.fold_state()==kFoldUnknown);
  deliver(foldpub,folded); CHECK(ports.fold_state()==kFoldFolded);
  ports.offset+=1.1; CHECK(ports.fold_state()==kFoldUnknown);
  CHECK(!ports.nav_set_goal(1,2)); // no Action server: never execute an unchecked topic goal
  bool counted=false;
  for(const auto& link:ports.iface_report()) if(link.name=="nav_goal") {CHECK(link.count==0);counted=true;}
  CHECK(counted);
  ports.base_spin(1.5);pump();ports.base_spin(0);pump();
  CHECK(modes.size()==2 && modes[0]==2 && modes[1]==1);

  rm2027_interfaces::msg::RobotStatus status;
  rm2027_interfaces::msg::RobotHP hp;hp.outpost_r_hp=501;hp.outpost_b_hp=601;
  rm2027_interfaces::msg::FieldEvent event;event.outpost_r_control=0;event.outpost_b_control=2;
  rm2027_interfaces::msg::SentryInfo info;
  info.remote_allowance_exchange_times=3;info.remote_hp_exchange_times=2;
  info.team_allowance_exchangeable=300;info.out_of_combat=true;info.posture=1;
  const auto refresh=[&](){deliver(refpub,ref);deliver(infopub,info);};
  refresh();deliver(hppub,hp);deliver(eventpub,event);
  CHECK(ports.referee().our_outpost_hp==(red?501:601));
  CHECK(ports.referee().enemy_outpost_ctrl==(red?2:0));
  ports.offset += 2.1;
  CHECK(ports.referee().out_of_combat == 1); // slow referee messages preserve the last combat fact
  CHECK(ports.referee().revive_available == -1); // purchase qualification remains freshness-gated
  info.out_of_combat = false;
  deliver(infopub,info);
  CHECK(ports.referee().out_of_combat == 0);
  info.out_of_combat = true;
  refresh();deliver(hppub,hp);deliver(eventpub,event);
  // No RobotStatus message has been delivered: it must not gate purchases.
  ports.buy_ammo(100);pump();CHECK(!commands.empty());
  CHECK(commands.back().remote_allowance_times==4 && commands.back().remote_hp_times==2);
  CHECK(commands.back().exchange_allowance_total==info.exchanged_projectile_allowance); // Remote request must not also buy locally.
  for(int robot_id:{red?107:7,0,red?7:107})
  {
    status.robot_id=robot_id;deliver(statuspub,status);
    CHECK(ports.referee().robot_id==robot_id);
    CHECK(ports.referee().our_outpost_hp==(red?501:601));
    CHECK(ports.referee().our_outpost_ctrl==(red?0:2));
    CHECK(ports.referee().enemy_outpost_ctrl==(red?2:0));
  }
  auto count=commands.size();ports.buy_ammo(100);ports.buy_hp(1);pump();
  for(size_t i=count;i<commands.size();++i) CHECK(commands[i].remote_allowance_times==4 && commands[i].remote_hp_times==2);
  ports.offset+=13;refresh();ports.service_purchase();count=commands.size();
  ports.buy_ammo(100);pump();CHECK(commands.size()==count); // Unacknowledged request stays latched.
  info.remote_allowance_exchange_times=4;deliver(infopub,info);ports.service_purchase();
  ports.buy_hp(1);pump();CHECK(commands.size()==count); // Delivery delay after acknowledgment.
  ports.offset+=6.1;refresh();ports.buy_hp(1);pump();
  CHECK(ports.referee().robot_id==0); // Stale identity remains diagnostic only.
  CHECK(commands.back().remote_allowance_times==4 && commands.back().remote_hp_times==3);
  info.remote_hp_exchange_times=3;deliver(infopub,info);ports.service_purchase();
  ports.offset+=6.1;ref.data[1]=0;ref.data[4]=0;info.can_free_respawn=true;refresh();
  ports.buy_revive();pump();CHECK(commands.back().confirm_respawn && !commands.back().confirm_buy_respawn);
  ref.data[1]=400;deliver(refpub,ref);ports.service_purchase();pump();
  CHECK(!commands.back().confirm_respawn && !commands.back().confirm_buy_respawn);
  auto intpub=node->create_publisher<std_msgs::msg::Int32MultiArray>(cfg.op_cmd_topic,10);
  auto xypub=node->create_publisher<std_msgs::msg::Float64MultiArray>(cfg.op_cmd_xy_topic,10);
  std_msgs::msg::Int32MultiArray integer;integer.data={4,10,1000};deliver(intpub,integer);
  const auto first=ports.op_cmd();CHECK(first.param==10);
  std_msgs::msg::Float64MultiArray xy;xy.data={4,-1,1,2,3};deliver(xypub,xy);
  const auto second=ports.op_cmd();
  CHECK(second.seq!=first.seq && second.param==-1);
  deliver(intpub,integer);CHECK(ports.op_cmd().seq==0);
  xy.data[2]=1.5;deliver(xypub,xy);CHECK(ports.op_cmd().seq==0);
  bool counted_bad=false;
  for(const auto& link:ports.iface_report())if(link.name=="op_cmd_xy")counted_bad=link.dropped==1;
  CHECK(counted_bad);
  xy.data={4,-1,2,4,5};deliver(xypub,xy); // valid B
  integer.data={1,100,1001};deliver(intpub,integer); // manual purchase must not erase B
  xy.data={4,-1,3,999,999};deliver(xypub,xy); // geometric-invalid C, handled by the tree
  const auto b = ports.op_cmd();
  const auto manual_buy = ports.op_cmd();
  const auto c = ports.op_cmd();
  CHECK(b.type == 4 && b.x == 4 && b.y == 5);
  CHECK(manual_buy.type == 1 && manual_buy.seq > b.seq);
  CHECK(c.type == 4 && c.x == 999 && c.seq > manual_buy.seq);
  CHECK(ports.op_cmd().seq == 0);
  ports.enable_data_source_guard();
  auto sourcepub=node->create_publisher<std_msgs::msg::String>(cfg.data_source_topic,10);
  std_msgs::msg::String source;source.data="sim";deliver(sourcepub,source);refresh();
  CHECK(ports.referee().game_progress==0);count=commands.size();ports.buy_ammo(100);pump();CHECK(commands.size()==count);
}
void check_nav2_route()
{
  using Action=nav2_msgs::action::NavigateThroughPoses;
  using Handle=rclcpp_action::ServerGoalHandle<Action>;
  auto node=std::make_shared<rclcpp::Node>("bbt_nav2_route_test");
  RosPortsConfig cfg;cfg.own_faction=1;cfg.nav_through_poses_action="/test_through_poses";
  RosPorts ports(node.get(),cfg);
  CHECK(!ports.nav_through_poses({{10,5},{11,5}})); // no stop-at-centre fallback
  std::vector<std::shared_ptr<Handle>> handles;
  auto server=rclcpp_action::create_server<Action>(node,cfg.nav_through_poses_action,
      [](const rclcpp_action::GoalUUID&,std::shared_ptr<const Action::Goal>){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](std::shared_ptr<Handle>){return rclcpp_action::CancelResponse::ACCEPT;},
      [&](std::shared_ptr<Handle> handle){handles.push_back(handle);});
  auto status=node->create_publisher<std_msgs::msg::Int32>(cfg.nav_status_topic,10);
  int single_goals=0;
  auto goals=node->create_subscription<geometry_msgs::msg::PoseStamped>(cfg.nav_goal_topic,10,
      [&](geometry_msgs::msg::PoseStamped::SharedPtr){++single_goals;});
  const auto pump=[&](){for(int i=0;i<30;++i){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(2));}};
  bool sent=false;
  for(int i=0;i<30 && !sent;++i){pump();sent=ports.nav_through_poses({{10,5},{11,5}});}
  CHECK(sent);pump();CHECK(handles.size()==1 && single_goals==0);
  CHECK(handles[0]->get_goal()->poses.size()==2);
  CHECK(handles[0]->get_goal()->poses[0].pose.position.x==10);
  CHECK(handles[0]->get_goal()->poses[1].pose.position.x==11);
  auto feedback=std::make_shared<Action::Feedback>();feedback->number_of_poses_remaining=1;
  handles[0]->publish_feedback(feedback);
  std_msgs::msg::Int32 stale;stale.data=kGoalSuccess;status->publish(stale);pump();
  CHECK(ports.nav_state()==kGoalRunning && single_goals==0); // via point does not finish
  handles[0]->succeed(std::make_shared<Action::Result>());pump();
  CHECK(ports.nav_state()==kGoalSuccess);
  CHECK(ports.nav_through_poses({{10,5},{11,5}}));pump();CHECK(handles.size()==2);
  ports.nav_cancel();CHECK(ports.nav_through_poses({{15,5}}));pump();
  // A server may complete successfully while the cancel request is in flight.
  handles[1]->succeed(std::make_shared<Action::Result>());
  pump();CHECK(ports.nav_state()==kGoalRunning && single_goals==0); // stale route result ignored
  CHECK(handles.back()->get_goal()->poses.size()==1); // folded exit-only recovery stays authorized
}
void check_single_goal_requires_identity_and_server()
{
  auto node = std::make_shared<rclcpp::Node>("bbt_single_identity_test");
  RosPortsConfig cfg;
  cfg.own_faction = 1;
  cfg.nav_status_topic = "/single_identity/status";
  RosPorts ports(node.get(), cfg);
  // A legacy status has no goal identity and cannot acknowledge a new request.
  CHECK(!ports.nav_set_goal(3.25, 7.75));
  auto status = node->create_publisher<std_msgs::msg::Int32>(cfg.nav_status_topic, 10);
  for (int i = 0; i < 40; ++i)
  {
    std_msgs::msg::Int32 msg;
    msg.data = kGoalSuccess;
    status->publish(msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  CHECK(ports.nav_state() != kGoalSuccess);
}

void check_single_goal_checked_path_and_stale_results()
{
  using Compute = nav2_msgs::action::ComputePathToPose;
  using ComputeServer = rclcpp_action::ServerGoalHandle<Compute>;
  using Follow = nav2_msgs::action::FollowPath;
  using FollowServer = rclcpp_action::ServerGoalHandle<Follow>;
  auto node = std::make_shared<rclcpp::Node>("bbt_checked_navigation_test");
  RosPortsConfig cfg;
  cfg.own_faction = 1;
  cfg.nav_compute_path_action = "/checked/compute";
  cfg.nav_follow_path_action = "/checked/follow";
  cfg.nav_status_topic = "/checked/status";
  cfg.odom_topic = "/checked/odom";
  TunnelDef tunnel;
  tunnel.corners = {{1, 0}, {2, 0}, {2, 2}, {1, 2}};
  cfg.nav_tunnels = {tunnel};
  cfg.nav_regions = {"outer", "middle"};
  cfg.nav_region_geometry = {
      {"outer", {{-1, -2}, {.5, -2}, {.5, 5}, {-1, 5}}},
      {"outer", {{2.5, -2}, {5, -2}, {5, 5}, {2.5, 5}}},
      {"outer", {{-1, -2}, {5, -2}, {5, -.5}, {-1, -.5}}},
      {"outer", {{-1, 2.5}, {5, 2.5}, {5, 5}, {-1, 5}}},
      {"middle", {{.5, -.5}, {2.5, -.5}, {2.5, 2.5}, {.5, 2.5}}}};
  std::vector<std::shared_ptr<ComputeServer>> plans;
  std::vector<std::shared_ptr<FollowServer>> follows;
  bool reject_follow = false;
  auto compute_server = rclcpp_action::create_server<Compute>(node, cfg.nav_compute_path_action,
      [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Compute::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }, [](std::shared_ptr<ComputeServer>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [&](std::shared_ptr<ComputeServer> handle) { plans.push_back(handle); });
  auto follow_server = rclcpp_action::create_server<Follow>(node, cfg.nav_follow_path_action,
      [&](const rclcpp_action::GoalUUID&, std::shared_ptr<const Follow::Goal>) {
        return reject_follow ? rclcpp_action::GoalResponse::REJECT :
            rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }, [](std::shared_ptr<FollowServer>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [&](std::shared_ptr<FollowServer> handle) { follows.push_back(handle); });
  RosPorts ports(node.get(), cfg);
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(cfg.odom_topic, 10);
  auto status_pub = node->create_publisher<std_msgs::msg::Int32>(cfg.nav_status_topic, 10);
  const auto pump = [&]() {
    for (int i = 0; i < 25; ++i)
    {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  };
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 0;
  odom.pose.pose.position.y = 1;
  odom.pose.pose.orientation.w = 1;
  for (int i = 0; i < 10 && !ports.base_pose().valid; ++i) { odom_pub->publish(odom); pump(); }
  CHECK(ports.base_pose().valid);
  const auto request = [&](double x, double y) {
    bool sent = false;
    for (int i = 0; i < 20 && !sent; ++i) { pump(); sent = ports.nav_set_goal(x, y); }
    CHECK(sent);
    pump();
    CHECK(ports.nav_state() == kGoalRunning);
  };
  const auto result = [](std::initializer_list<Vec2> positions) {
    auto value = std::make_shared<Compute::Result>();
    value->path.header.frame_id = "map";
    for (const auto& p : positions)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.orientation.w = 1;
      value->path.poses.push_back(pose);
    }
    return value;
  };

  request(3.25, 1);
  CHECK(plans.size() == 1);
  CHECK(plans.back()->get_goal()->goal.pose.position.x == 3.25);
  plans.back()->succeed(result({{0, 1}, {3.25, 1}})); // direct path enters an unauthorized tunnel
  pump();
  CHECK(ports.nav_state() == kGoalFailure && follows.empty());

  request(3.25, 1);
  plans.back()->succeed(result({{0, 1}, {0, 6}, {3.25, 6}, {3.25, 1}})); // uncovered space
  pump();
  CHECK(ports.nav_state() == kGoalFailure && follows.empty());

  request(3.25, 1);
  // All sampled poses belong to "outer", but the sparse middle segment leaves
  // it through another region without touching the tunnel rectangle.
  plans.back()->succeed(result({{0, 1}, {0, 2.25}, {3.25, 2.25}, {3.25, 1}}));
  pump();
  CHECK(ports.nav_state() == kGoalFailure && follows.empty());

  request(3.25, 1);
  plans.back()->succeed(result({{0, 1}, {0, -1}, {3.25, -1}, {3.25, 1}}));
  pump();
  CHECK(follows.size() == 1 && ports.nav_state() == kGoalRunning);
  CHECK(follows.back()->get_goal()->path.poses.size() == 4); // execute exactly the checked detour
  std_msgs::msg::Int32 stale;
  stale.data = kGoalSuccess;
  status_pub->publish(stale);
  pump();
  CHECK(ports.nav_state() == kGoalRunning); // uncorrelated telemetry cannot acknowledge arrival
  follows.back()->succeed(std::make_shared<Follow::Result>());
  pump();
  CHECK(ports.nav_state() == kGoalSuccess);

  request(3.25, 1);
  auto old_plan = plans.back();
  request(3.25, 3);
  CHECK(old_plan->is_canceling());
  old_plan->succeed(result({{0, 1}, {3.25, 1}})); // result races the accepted cancellation
  pump();
  CHECK(ports.nav_state() == kGoalRunning && follows.size() == 1);
  plans.back()->succeed(result({{0, 1}, {0, 3}, {3.25, 3}}));
  pump();
  CHECK(follows.size() == 2);
  auto old_follow = follows.back();
  ports.nav_cancel();
  request(0, 4);
  CHECK(old_follow->is_canceling());
  old_follow->succeed(std::make_shared<Follow::Result>());
  pump();
  CHECK(ports.nav_state() == kGoalRunning);
  plans.back()->succeed(result({{0, 1}, {0, 3.5}})); // wrong final coordinate
  pump();
  CHECK(ports.nav_state() == kGoalFailure && follows.size() == 2);

  request(0, 4);
  reject_follow = true;
  plans.back()->succeed(result({{0, 1}, {0, 4}}));
  pump();
  CHECK(ports.nav_state() == kGoalFailure); // controller rejection is not arrival

  // Legacy/demo configurations without a geometric partition keep their
  // ordinary navigation interface; they still cannot invent an arbitrary region.
  auto demo_cfg = cfg;
  demo_cfg.nav_region_geometry.clear();
  demo_cfg.nav_tunnels.clear();
  RosPorts demo_ports(node.get(), demo_cfg);
  odom_pub->publish(odom);
  pump();
  bool sent = false;
  for (int i = 0; i < 20 && !sent; ++i) { pump(); sent = demo_ports.nav_set_goal(3.25, 1); }
  CHECK(sent);
  pump();
  reject_follow = false;
  plans.back()->succeed(result({{0, 1}, {3.25, 1}}));
  pump();
  CHECK(demo_ports.nav_state() == kGoalRunning && follows.size() == 3);
  follows.back()->succeed(std::make_shared<Follow::Result>());
  pump();
  CHECK(demo_ports.nav_state() == kGoalSuccess);
}

void check_successful_tunnel_exit_clearance()
{
  using Compute = nav2_msgs::action::ComputePathToPose;
  using ComputeServer = rclcpp_action::ServerGoalHandle<Compute>;
  using Follow = nav2_msgs::action::FollowPath;
  using FollowServer = rclcpp_action::ServerGoalHandle<Follow>;
  using Through = nav2_msgs::action::NavigateThroughPoses;
  using ThroughServer = rclcpp_action::ServerGoalHandle<Through>;
  auto node = std::make_shared<rclcpp::Node>("bbt_exit_clearance_test");
  RosPortsConfig cfg;
  cfg.own_faction = 1;
  cfg.nav_compute_path_action = "/exit_clearance/compute";
  cfg.nav_follow_path_action = "/exit_clearance/follow";
  cfg.nav_through_poses_action = "/exit_clearance/through";
  cfg.odom_topic = "/exit_clearance/odom";
  cfg.referee_topic = "/exit_clearance/referee";
  TunnelDef tunnel;
  tunnel.align[0] = {9.32, 3.5}; tunnel.align[1] = {9.32, 1};
  tunnel.center = {9.32, 2.15}; tunnel.has_center = true;
  tunnel.corners = {{9.82, 3.5}, {8.82, 3.5}, {8.82, 1}, {9.82, 1}};
  auto other = tunnel;
  other.corners = {{10.5, .8}, {10.7, .8}, {10.7, .95}, {10.5, .95}};
  other.align[0] = {10.4, .875}; other.align[1] = {10.8, .875};
  other.center = {10.6, .875};
  cfg.nav_tunnels = {tunnel, other};
  cfg.nav_regions = {"mid"};
  cfg.nav_region_geometry = {{"mid", {{0, 0}, {28, 0}, {28, 15}, {0, 15}}}};
  std::vector<std::shared_ptr<ComputeServer>> plans;
  std::vector<std::shared_ptr<FollowServer>> follows;
  std::vector<std::shared_ptr<ThroughServer>> crossings;
  auto compute = rclcpp_action::create_server<Compute>(node, cfg.nav_compute_path_action,
      [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Compute::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }, [](std::shared_ptr<ComputeServer>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [&](std::shared_ptr<ComputeServer> handle) { plans.push_back(handle); });
  auto follow = rclcpp_action::create_server<Follow>(node, cfg.nav_follow_path_action,
      [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Follow::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }, [](std::shared_ptr<FollowServer>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [&](std::shared_ptr<FollowServer> handle) { follows.push_back(handle); });
  auto through = rclcpp_action::create_server<Through>(node, cfg.nav_through_poses_action,
      [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Through::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }, [](std::shared_ptr<ThroughServer>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [&](std::shared_ptr<ThroughServer> handle) { crossings.push_back(handle); });
  RosPorts ports(node.get(), cfg);
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(cfg.odom_topic, 10);
  auto ref_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(cfg.referee_topic, 10);
  const auto pump = [&]() {
    for (int i = 0; i < 25; ++i)
    {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  };
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 9.320000293611336;
  odom.pose.pose.position.y = 1.0202299563309136; // real physical case's successful exit residual
  odom.pose.pose.orientation.w = 1;
  for (int i = 0; i < 10 && !ports.base_pose().valid; ++i) { odom_pub->publish(odom); pump(); }
  CHECK(ports.base_pose().valid);
  std_msgs::msg::Float64MultiArray playing; playing.data = {4, 400, 400, 100, 0, -1, -1, 0, 0};
  ref_pub->publish(playing); pump();
  const auto request = [&]() {
    bool sent = false;
    for (int i = 0; i < 20 && !sent; ++i) { pump(); sent = ports.nav_set_goal(11.37, .66); }
    CHECK(sent); pump();
  };
  const auto complete_plan = [&](std::initializer_list<Vec2> positions) {
    auto result = std::make_shared<Compute::Result>();
    result->path.header.frame_id = "map";
    for (const auto& p : positions)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = p.x; pose.pose.position.y = p.y;
      pose.pose.orientation.w = 1;
      result->path.poses.push_back(pose);
    }
    plans.back()->succeed(result); pump();
  };
  const auto authorize_exit = [&]() {
    CHECK(ports.nav_through_poses({{9.32, 2.15}, {9.32, 1}})); pump();
    crossings.back()->succeed(std::make_shared<Through::Result>()); pump();
    CHECK(ports.nav_state() == kGoalSuccess);
    ports.nav_cancel(); // normal successful-exit stop must retain the physical clearance evidence
  };
  request();
  complete_plan({{9.320000293611336, 1.0202299563309136}, {9.32, .45}, {11.37, .66}});
  CHECK(ports.nav_state() == kGoalFailure && follows.empty()); // no crossing evidence
  authorize_exit();
  request();
  complete_plan({{9.320000293611336, 1.0202299563309136}, {9.32, .45}, {11.37, .66}});
  CHECK(ports.nav_state() == kGoalRunning && follows.size() == 1);
  ports.nav_cancel();
  request();
  complete_plan({{9.320000293611336, 1.0202299563309136}, {9.32, 1.1}, {9.32, .45}, {11.37, .66}});
  CHECK(ports.nav_state() == kGoalFailure && follows.size() == 1); // backtracking inside the mouth
  request();
  complete_plan({{9.320000293611336, 1.0202299563309136}, {9.32, .45}, {9.32, 1.01}, {11.37, .66}});
  CHECK(ports.nav_state() == kGoalFailure && follows.size() == 1); // no re-entry after clearance
  request();
  complete_plan({{9.320000293611336, 1.0202299563309136}, {9.32, .45}, {10.6, .875}, {11.37, .66}});
  CHECK(ports.nav_state() == kGoalFailure && follows.size() == 1); // another tunnel stays forbidden
  CHECK(ports.nav_through_poses({{9.32, 1.02}})); pump();
  crossings.back()->succeed(std::make_shared<Through::Result>()); pump();
  ports.nav_cancel();
  request();
  complete_plan({{9.320000293611336, 1.0202299563309136}, {9.32, .45}, {11.37, .66}});
  CHECK(ports.nav_state() == kGoalFailure && follows.size() == 1); // success at an unconfigured point grants nothing
  authorize_exit();
  odom.pose.pose.position.y = 2.2; odom_pub->publish(odom); pump();
  request();
  complete_plan({{9.320000293611336, 2.2}, {9.32, .45}, {11.37, .66}});
  CHECK(ports.nav_state() == kGoalFailure && follows.size() == 1); // not yet past the real center
  odom.pose.pose.position.y = 1.0202299563309136; odom_pub->publish(odom); pump();
  authorize_exit();
  CHECK(ports.nav_through_poses({{9.32, 2.15}, {9.32, 1}})); pump();
  request(); // a new crossing revokes the old exit evidence before its success
  complete_plan({{9.320000293611336, 1.0202299563309136}, {9.32, .45}, {11.37, .66}});
  CHECK(ports.nav_state() == kGoalFailure && follows.size() == 1);
  authorize_exit();
  odom.pose.pose.position.y = .9; odom_pub->publish(odom); pump();
  odom.pose.pose.position.y = 1.0202299563309136; odom_pub->publish(odom); pump();
  request(); // leaving the body consumes physical clearance; moving back cannot reuse it
  complete_plan({{9.320000293611336, 1.0202299563309136}, {9.32, .45}, {11.37, .66}});
  CHECK(ports.nav_state() == kGoalFailure && follows.size() == 1);
  authorize_exit();
  CHECK(ports.nav_through_poses({{9.32, 2.15}, {9.32, 1}})); pump();
  std_msgs::msg::Float64MultiArray ended; ended.data = {5, 400, 400, 100, 0, -1, -1, 0, 0};
  ref_pub->publish(ended); pump();
  crossings.back()->succeed(std::make_shared<Through::Result>()); pump();
  ports.nav_cancel(); // even an accepted request's late success cannot restore match-end authority
  request();
  complete_plan({{9.320000293611336, 1.0202299563309136}, {9.32, .45}, {11.37, .66}});
  CHECK(ports.nav_state() == kGoalFailure && follows.size() == 1); // match end revokes clearance
}
int main(int argc,char**argv)
{
  rclcpp::init(argc,argv);
  {
    auto node=std::make_shared<rclcpp::Node>("bbt_missing_team_test");
    for(int own_faction:{0,3})
    {
      RosPortsConfig cfg;cfg.own_faction=own_faction;
      bool rejected=false;
      try { RosPorts ports(node.get(),cfg); }
      catch(const std::invalid_argument&) { rejected=true; }
      CHECK(rejected);
    }
  }
  check_team(1);
  check_team(2);
  check_single_goal_requires_identity_and_server();
  check_single_goal_checked_path_and_stale_results();
  check_successful_tunnel_exit_clearance();
  check_nav2_route();
  rclcpp::shutdown();
  std::puts("PASS: manual red/blue, missing/wrong/stale identity, summary layouts, freshness, modes, purchases and intra-process communication");
}
