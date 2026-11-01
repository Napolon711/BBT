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
int main(int argc,char**argv)
{
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("bbt_ports_test", rclcpp::NodeOptions().use_intra_process_comms(true));
  RosPortsConfig cfg; cfg.referee_2027=true; cfg.referee_stale_ms=2000;
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
  CHECK(ports.referee().our_outpost_hp==-1 && ports.referee().enemy_outpost_hp==-1);
  rm2027_interfaces::msg::RobotStatus identity;identity.robot_id=7;deliver(statuspub,identity);
  CHECK(ports.referee().our_outpost_hp==500 && ports.referee().enemy_outpost_hp==600);
  CHECK(ports.referee().our_outpost_ctrl==0);
  ref.data[9]=4000000000.0; deliver(refpub,ref);
  CHECK(ports.referee().hp==400); // uint32 source timestamp exceeds signed-int range.
  for(int size:{0,8,13,15,16,17,18})
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
  ports.nav_set_goal(1,2);
  bool counted=false;
  for(const auto& link:ports.iface_report()) if(link.name=="nav_goal") {CHECK(link.count==1 && link.rate_hz>0);counted=true;}
  CHECK(counted);
  ports.base_spin(1.5);pump();ports.base_spin(0);pump();
  CHECK(modes.size()==2 && modes[0]==2 && modes[1]==1);

  rm2027_interfaces::msg::RobotStatus status;status.robot_id=7;
  rm2027_interfaces::msg::RobotHP hp;hp.outpost_r_hp=501;hp.outpost_b_hp=601;
  rm2027_interfaces::msg::FieldEvent event;event.outpost_r_control=0;event.outpost_b_control=2;
  rm2027_interfaces::msg::SentryInfo info;
  info.remote_allowance_exchange_times=3;info.remote_hp_exchange_times=2;
  info.team_allowance_exchangeable=300;info.out_of_combat=true;info.posture=1;
  const auto refresh=[&](){deliver(refpub,ref);deliver(statuspub,status);deliver(infopub,info);};
  refresh();deliver(hppub,hp);deliver(eventpub,event);
  CHECK(ports.referee().our_outpost_hp==501 && ports.referee().enemy_outpost_ctrl==2);
  status.robot_id=107;deliver(statuspub,status);
  CHECK(ports.referee().our_outpost_hp==601 && ports.referee().our_outpost_ctrl==2);
  status.robot_id=7;deliver(statuspub,status);
  ports.buy_ammo(100);pump();CHECK(!commands.empty());
  CHECK(commands.back().remote_allowance_times==4 && commands.back().remote_hp_times==2);
  CHECK(commands.back().exchange_allowance_total==info.exchanged_projectile_allowance); // Remote request must not also buy locally.
  auto count=commands.size();ports.buy_ammo(100);ports.buy_hp(1);pump();
  for(size_t i=count;i<commands.size();++i) CHECK(commands[i].remote_allowance_times==4 && commands[i].remote_hp_times==2);
  ports.offset+=13;refresh();ports.service_purchase();count=commands.size();
  ports.buy_ammo(100);pump();CHECK(commands.size()==count); // Unacknowledged request stays latched.
  info.remote_allowance_exchange_times=4;deliver(infopub,info);ports.service_purchase();
  ports.buy_hp(1);pump();CHECK(commands.size()==count); // Delivery delay after acknowledgment.
  ports.offset+=6.1;refresh();ports.buy_hp(1);pump();
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
  CHECK(ports.op_cmd().seq!=first.seq && ports.op_cmd().param==-1);
  const auto second=ports.op_cmd();deliver(intpub,integer);CHECK(ports.op_cmd().seq==second.seq);
  xy.data[2]=1.5;deliver(xypub,xy);CHECK(ports.op_cmd().seq==second.seq);
  bool counted_bad=false;
  for(const auto& link:ports.iface_report())if(link.name=="op_cmd_xy")counted_bad=link.dropped==1;
  CHECK(counted_bad);
  ports.enable_data_source_guard();
  auto sourcepub=node->create_publisher<std_msgs::msg::String>(cfg.data_source_topic,10);
  std_msgs::msg::String source;source.data="sim";deliver(sourcepub,source);refresh();
  CHECK(ports.referee().game_progress==0);count=commands.size();ports.buy_ammo(100);pump();CHECK(commands.size()==count);
  rclcpp::shutdown();
  std::puts("PASS: summary layouts, malformed input, feedback freshness, stats, modes, typed referee and purchase acknowledgment");
}
