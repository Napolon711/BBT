// BBT 决策节点：定时器驱动行为树（trees/main.xml），本身不做决策。
//
// 数据通路（唯一通路，仿真与实车共用）：
//   订阅（进）：/odom /nav_status /gimbal_fold_state /referee/summary
//               /referee/operator_cmd（操作手指令：只消费"指点"，买类忽略）
//   发布（出）：/goal_pose /nav_cancel /cmd_vel /move_mode /cmd_spin /gimbal_fold_cmd
//               /sentry/buy_ammo /sentry/buy_hp /sentry/buy_revive（自动购买下单）
//               /sentry/iface_status（1 Hz 数据流看板）
// Arena 仿真由 tools/run_arena.py 经 PTY/ros2_libxr 发布同一套消息——
// 换掉桥，接上实车电控/导航/裁判，决策层一行不用改。
// 比赛外完全静默：结束沿一次性收尾（取消导航+停自旋+终止过洞），之后不发任何消息
//（自动买也一样，比赛外不下单）；开始沿切一次底盘模式。
//
// 参数表：
//   config_file — string，必填，decision.yaml 路径（区域表/点表/洞表/阈值都在里面）
//   tree_file   — string，""，行为树 xml 路径；空 = 用包内 trees/main.xml
//   tick_hz     — double，20.0，决策频率，[1,200]
//   data_source — string，"real"，"sim" = 接仿真数据（只是话题表不同）；
//                 real 模式下一旦听到 /sentry/data_source = "sim" 立即待机（硬保护）
//   referee_protocol — string，默认 "2027"；旧摘要仿真可显式选择 "legacy"。
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "bbt_decide/bt.hpp"
#include "bbt_decide/ros_ports.hpp"
#include "bbt_decide/types.hpp"

namespace
{

using namespace bbt_decide;

class BbtNode : public rclcpp::Node
{
public:
  BbtNode() : Node("bbt_decide")
  {
    // 决策频率
    const double hz = declare_parameter<double>("tick_hz", 20.0);
    if (!std::isfinite(hz) || hz < 1.0 || hz > 200.0)
    {
      throw std::invalid_argument("tick_hz must be finite and in [1,200]");
    }

    // 行为树文件：没显式给就用包安装目录里的默认树
    std::string tree_file = declare_parameter<std::string>("tree_file", "");
    if (tree_file.empty())
    {
      tree_file = ament_index_cpp::get_package_share_directory("bbt_decide") +
                  "/trees/main.xml";
    }

    // 配置：必填，演示配置会绕开真实点表，正是要禁止的东西
    const std::string config_file = declare_parameter<std::string>("config_file", "");
    if (config_file.empty())
    {
      throw std::invalid_argument(
          "config_file 必填：给出 decision.yaml 路径（区域表/点表/洞表都由人手填在里面）");
    }
    Config loaded;
    std::string err;
    std::vector<std::string> warns;
    if (!load_config(config_file, loaded, err, warns))
    {
      throw std::runtime_error("配置加载失败: " + err);
    }
    for (const auto& w : warns) RCLCPP_WARN(get_logger(), "配置告警: %s", w.c_str());
    ctx_ = std::make_shared<Context>();
    ctx_->cfg = loaded;
    RCLCPP_INFO(get_logger(), "配置已加载: %s（点 %zu 个，洞 %zu 条）", config_file.c_str(),
                ctx_->cfg.points.size(), ctx_->cfg.tunnels.size());

    // 数据源开关：只决定用哪张话题表，数据一律经真实 ROS 话题进出
    const std::string data_source = declare_parameter<std::string>("data_source", "real");
    if (data_source != "sim" && data_source != "real")
    {
      throw std::invalid_argument("data_source 只能是 sim 或 real，收到: " + data_source);
    }
    data_source_ = data_source;
    const TopicMap& tm = (data_source == "sim") ? ctx_->cfg.topics_sim : ctx_->cfg.topics_real;
    {
      std::string bad;
      if (!tm.all_named(bad))
      {
        throw std::runtime_error("话题表 topics." + data_source + " 的字段 " + bad + " 为空");
      }
    }

    RosPortsConfig rc;
    rc.nav_goal_topic = tm.nav_goal;
    rc.nav_status_topic = tm.nav_status;
    rc.nav_cancel_topic = tm.nav_cancel;
    rc.odom_topic = tm.odom;
    rc.cmd_vel_topic = tm.cmd_vel;
    rc.chassis_mode_topic = tm.chassis_mode;
    rc.spin_topic = tm.spin;
    rc.fold_cmd_topic = tm.fold_cmd;
    rc.fold_state_topic = tm.fold_state;
    rc.referee_topic = tm.referee;
    rc.op_cmd_topic = tm.op_cmd;
    rc.op_cmd_xy_topic = tm.op_cmd_xy;
    rc.buy_ammo_topic = tm.buy_ammo;
    rc.buy_hp_topic = tm.buy_hp;
    rc.buy_revive_topic = tm.buy_revive;
    rc.attack_cmd_topic = tm.attack_cmd;
    rc.data_source_topic = tm.data_source;
    rc.mode_independent = ctx_->cfg.mode_independent;
    const std::string referee_protocol = declare_parameter<std::string>("referee_protocol", "2027");
    if (referee_protocol != "2027" && referee_protocol != "legacy")
      throw std::invalid_argument("referee_protocol 只能是 2027 或 legacy");
    if (data_source == "real" && referee_protocol != "2027")
      throw std::invalid_argument("实车必须使用 2027 裁判协议");
    rc.referee_2027 = referee_protocol == "2027";
    rc.fold_stale_ms = ctx_->cfg.fold_stale_ms;
    ports_ = std::make_shared<RosPorts>(this, rc);
    ctx_->ports = ports_;

    if (data_source == "sim")
    {
      RCLCPP_WARN(get_logger(),
                  "==== 数据源 = sim（仿真/模拟）==== 本进程按仿真数据决策，不得用于实车。"
                  "上实车前把 data_source 改回 real。");
    }
    else
    {
      // 实车模式硬保护：听到任何 "sim" 声明立即作废裁判快照 → 决策树待机分支停车
      ports_->enable_data_source_guard();
      RCLCPP_INFO(get_logger(),
                  "==== 数据源 = real（实车）==== 仿真数据守卫已开启（%s 声明 \"sim\" 即停车）",
                  tm.data_source.c_str());
    }

    // 底盘模式不再开机就发（比赛外一条消息不发）：比赛开始的上升沿切一次
    //（见 bt.cpp IsMatchRunning / planner.cpp run_executor 的沿检测）。

    // 区域记账从开局区域起步（开赛沿会再重置；过洞完成写出口对正点的区域）。
    // enable_tunnel = false 时 regions 可以为空（region 与过洞耦合，整套不需要）。
    ctx_->plan.current_region = ctx_->cfg.start_region;
    if (!ctx_->cfg.regions.empty())
    {
      RCLCPP_INFO(get_logger(), "区域表 %zu 个，开局区域 = %s（下标 %d）",
                  ctx_->cfg.regions.size(),
                  ctx_->cfg.regions[ctx_->cfg.start_region].c_str(),
                  ctx_->cfg.start_region);
    }
    else
    {
      RCLCPP_INFO(get_logger(), "enable_tunnel = false：不过洞，region 整套未配置");
    }

    // 建树：注册节点 + 从 xml 载入（节点逻辑 = planner.cpp 纯函数，树只做编排）
    register_nodes(factory_, ctx_);
    factory_.registerBehaviorTreeFromFile(tree_file);
    tree_ = factory_.createTree("Main");

    // 1 Hz 接口数据流看板：每条链路的帧数/频率/年龄/丢弃数，
    // 用来确认"仿真/实车的数据确实从真实接口进来了"
    iface_pub_ = create_publisher<std_msgs::msg::String>(tm.iface_status, 10);
    iface_timer_ = create_wall_timer(std::chrono::milliseconds(1000),
                                     [this]() { publish_iface_status(); });

    // 决策主循环
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / hz)),
        [this]() { tick(); });

    RCLCPP_INFO(get_logger(), "bbt_decide started: tree=%s hz=%.1f data_source=%s",
                tree_file.c_str(), hz, data_source.c_str());
  }

private:
  // 推一拍树 + 目标点变化时打一条日志。
  void tick()
  {
    if (data_source_ == "real" && ctx_->cfg.require_map_alignment && !ctx_->cfg.field_to_map.enabled)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                          "等待 field_to_map 标定；确认恒等变换也须显式 enabled=true");
      return;
    }
    try
    {
      tree_.tickOnce();
    }
    catch (const std::exception& e)
    {
      timer_->cancel();  // 树挂了就停 tick，别每 50 ms 刷同一条错误
      tree_.haltTree();
      ports_->nav_cancel();
      ports_->base_stop();
      ports_->base_spin(0.0);
      ports_->attack_cmd(0, 0, 0, 0);
      RCLCPP_ERROR(get_logger(), "tree stopped: %s", e.what());
      return;
    }
    const int acting =
        ctx_->ex.tunnel.active() ? ctx_->ex.tunnel.after_point : ctx_->decision.target_point;
    if (acting != last_acting_)
    {
      const double now = ports_->now_s();
      const PointDef* p = acting >= 0 ? find_point(ctx_->cfg, acting) : nullptr;
      if (ctx_->ex.tunnel.active() && ctx_->ex.tunnel.tunnel >= 0 &&
          static_cast<size_t>(ctx_->ex.tunnel.tunnel) < ctx_->cfg.tunnels.size())
      {
        RCLCPP_INFO(get_logger(), "[%.1fs] 过洞中：%s → 对正点(%.2f, %.2f) → 之后去点 %d",
                    now, ctx_->cfg.tunnels[ctx_->ex.tunnel.tunnel].name.c_str(),
                    ctx_->ex.tunnel.entry.x, ctx_->ex.tunnel.entry.y,
                    ctx_->ex.tunnel.after_point);
      }
      else if (p != nullptr)
      {
        RCLCPP_INFO(get_logger(), "[%.1fs] 目标点 → %d (%.2f, %.2f)", now, acting, p->pos.x,
                    p->pos.y);
      }
      else
      {
        RCLCPP_INFO(get_logger(), "[%.1fs] 待机（比赛外静默，不发消息）", now);
      }
      last_acting_ = acting;
    }
  }

  // 接口数据流统计 → 一行 JSON。
  void publish_iface_status()
  {
    const auto links = ports_->iface_report();
    const bool sim_seen = ports_->sim_source_seen();
    const bool guard_triggered = (data_source_ == "real") && sim_seen;
    std::string js = "{\"data_source\":\"" + data_source_ +
                     "\",\"sim_source_seen\":" + (sim_seen ? "true" : "false") +
                     ",\"guard_triggered\":" + (guard_triggered ? "true" : "false") +
                     ",\"links\":[";
    for (size_t i = 0; i < links.size(); ++i)
    {
      const auto& l = links[i];
      if (i) js += ",";
      char buf[512];
      std::snprintf(buf, sizeof(buf),
                    "{\"name\":\"%s\",\"topic\":\"%s\",\"type\":\"%s\",\"dir\":\"%s\","
                    "\"count\":%llu,\"rate_hz\":%.2f,\"age_s\":%.3f,\"dropped\":%llu}",
                    l.name.c_str(), l.topic.c_str(), l.msg_type.c_str(),
                    l.outbound ? "out" : "in", static_cast<unsigned long long>(l.count),
                    l.rate_hz, l.last_age_s, static_cast<unsigned long long>(l.dropped));
      js += buf;
    }
    js += "]}";
    std_msgs::msg::String msg;
    msg.data = js;
    iface_pub_->publish(msg);
  }

  std::shared_ptr<Context> ctx_;      // 全树共享黑板
  std::shared_ptr<RosPorts> ports_;   // 端口适配层（唯一数据进出口）
  BT::BehaviorTreeFactory factory_;   // BT 工厂：注册节点 + 从 xml 建树
  BT::Tree tree_;                     // 主行为树，每 tick 调一次 tickOnce()
  int last_acting_ = -3;              // 上一 tick 生效目标（-3 保证启动时打一条日志）
  std::string data_source_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr iface_pub_;
  rclcpp::TimerBase::SharedPtr iface_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try
  {
    rclcpp::spin(std::make_shared<BbtNode>());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("bbt_decide"), "%s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
