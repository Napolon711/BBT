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
  CHECK(ports.nav_set_goal(1,2)); // ordinary navigation needs only the UC topic executor
  bool counted=false;
  for(const auto& link:ports.iface_report()) if(link.name=="nav_goal") {CHECK(link.count==1);counted=true;}
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
  auto old_route = handles.back();
  ports.nav_cancel();
  CHECK(ports.nav_set_goal(20, 7));pump();
  CHECK(single_goals==1 && ports.nav_state()==kGoalRunning);
  old_route->succeed(std::make_shared<Action::Result>());pump();
  CHECK(ports.nav_state()==kGoalRunning); // old tunnel result cannot finish a topic task
  status->publish(stale);pump();
  CHECK(ports.nav_state()==kGoalSuccess); // ordinary task uses the UC topic result
}
void check_ordinary_goal_topic_contract()
{
  auto node = std::make_shared<rclcpp::Node>("bbt_topic_navigation_test",
      rclcpp::NodeOptions().use_intra_process_comms(true));
  RosPortsConfig cfg;
  cfg.own_faction = 1;
  cfg.nav_goal_topic = "/topic_navigation/goal";
  cfg.nav_status_topic = "/topic_navigation/status";
  cfg.nav_cancel_topic = "/topic_navigation/cancel";
  RosPorts ports(node.get(), cfg);
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  int cancels = 0;
  auto goal_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(cfg.nav_goal_topic, 10,
      [&](geometry_msgs::msg::PoseStamped::SharedPtr m) { goals.push_back(*m); });
  auto cancel_sub = node->create_subscription<std_msgs::msg::Empty>(cfg.nav_cancel_topic, 10,
      [&](std_msgs::msg::Empty::SharedPtr) { ++cancels; });
  auto status_pub = node->create_publisher<std_msgs::msg::Int32>(cfg.nav_status_topic, 10);
  const auto pump = [&]() {
    for (int i = 0; i < 15; ++i) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  };
  const auto report = [&](int state) {
    std_msgs::msg::Int32 message;
    message.data = state;
    status_pub->publish(message);
    pump();
  };
  // Unsolicited topic statuses do not create a task. No planner/controller
  // Action server or odometry is required to submit an ordinary final goal.
  report(kGoalSuccess);
  CHECK(ports.nav_state() == kGoalIdle);
  CHECK(ports.nav_set_goal(3.25, 7.75));
  pump();
  CHECK(goals.size() == 1 && ports.nav_state() == kGoalRunning);
  CHECK(goals.back().header.frame_id == "map");
  CHECK(goals.back().pose.position.x == 3.25 && goals.back().pose.position.y == 7.75);
  CHECK(goals.back().pose.orientation.w == 1.0);
  report(99);
  CHECK(ports.nav_state() == kGoalRunning);
  report(kGoalSuccess);
  CHECK(ports.nav_state() == kGoalSuccess);
  CHECK(!ports.nav_set_goal(std::numeric_limits<double>::quiet_NaN(), 1));
  pump();
  CHECK(goals.size() == 1 && ports.nav_state() == kGoalSuccess);
  CHECK(ports.nav_set_goal(4.5, 6.5));
  pump();
  CHECK(goals.size() == 2 && ports.nav_state() == kGoalRunning);
  report(kGoalFailure);
  CHECK(ports.nav_state() == kGoalFailure);
  ports.nav_cancel();
  pump();
  CHECK(cancels == 1 && ports.nav_state() == kGoalIdle);
  report(kGoalSuccess);
  CHECK(ports.nav_state() == kGoalIdle);
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
  check_ordinary_goal_topic_contract();
  check_nav2_route();
  rclcpp::shutdown();
  std::puts("PASS: manual red/blue, missing/wrong/stale identity, summary layouts, freshness, modes, purchases and intra-process communication");
}
