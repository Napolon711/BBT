// BBT 行为树节点实现：只做编排，逻辑全在 planner.cpp 的纯函数里。
// 约定：所有节点无状态（跨 tick 状态放 Context；ReactiveSequence 会 halt 兄弟节点）。
#include "bbt_decide/bt.hpp"

namespace bbt_decide
{
namespace
{
using BT::NodeStatus;

// ------------------------------------------------------------ 门控 + 感知
// IsMatchRunning：处理操作手指令 + 自动购买、抓一帧感知存进 ctx->sense，并做比赛门控。
// 门控只看比赛状态：game_progress == 4（进行中）→ SUCCESS；未开始/已结束 →
// FAILURE（外层 ReactiveFallback 落到 Standby：比赛外不发任何消息）。
// 不做"有效性+新鲜度"判定——能收到比赛状态和血量就行（使用者口径）。
// 操作手指令只消费"指点"（买类忽略——那是操作手和裁判客户端的事）；
// 自动购买在比赛进行中由决策层自己下单（弹/血低、金币够、冷却过）。
// 比赛开始的上升沿切一次底盘模式（比赛外不发）。
BT::NodeStatus tick_gate(Context& ctx)
{
  const double now = ctx.ports->now_s();
  ctx.sense = sense(*ctx.ports, ctx.cfg, now);
  if (ctx.sense.match_running) update_match_gate(*ctx.ports, ctx.cfg, ctx.sense, ctx.plan, ctx.ex);
  handle_operator(*ctx.ports, ctx.cfg, ctx.plan);
  run_auto_buy(*ctx.ports, ctx.cfg, ctx.sense, ctx.plan, now);
  if (!ctx.sense.match_running) return NodeStatus::FAILURE;
  return NodeStatus::SUCCESS;
}

// ------------------------------------------------------------ 决策树
// DecideTarget：跑保底决策树（待机/阵亡→-1；血量锁存→家；巡逻轮换+防抖）。
// nav_goal_for 告诉决策树"执行器当前导航目标对应哪个点"：过洞中是 -2，
// 防止对正点的 success 被误当成巡逻到点而跳点。
BT::NodeStatus tick_decide(Context& ctx)
{
  const double now = ctx.ports->now_s();
  const int nav_goal_for = ctx.ex.tunnel.active() ? -2 : ctx.ex.applied_goal;
  ctx.decision = decide_tree(ctx.sense, ctx.cfg, ctx.plan, now, nav_goal_for);
  return NodeStatus::SUCCESS;
}

// ------------------------------------------------------------ 过洞执行
// TunnelCross：过洞五段状态机。目标在别的区域 → 启动/推进（独占，RUNNING）；
// 跨区但被拒/冷却/不可达 → 停车等待（RUNNING，绝不把跨区目标放给常规导航）；
// 与过洞无关（同区/空目标）→ FAILURE 让位给 NavCruise。
// 有状态动作：状态在 ctx->ex.tunnel（Context 黑板），节点成员不存东西。
class TunnelCross : public BT::StatefulActionNode
{
public:
  TunnelCross(const std::string& name, const BT::NodeConfig& config,
              std::shared_ptr<Context> ctx)
    : BT::StatefulActionNode(name, config), ctx_(std::move(ctx))
  {
  }
  static BT::PortsList providedPorts() { return {}; }

private:
  NodeStatus onStart() override { return step(); }
  NodeStatus onRunning() override { return step(); }
  // 被 halt（比赛结束切 Standby / 上层换树）：只清状态机记账，**不发任何消息**——
  // 一次性收尾（取消导航 + 停自旋）由 Standby 的下降沿统一做，这里做了会双发。
  // 不进冷却、不动折叠——正常退出 ≠ 过洞失败，且退出不等于离开洞口。
  void onHalted() override
  {
    ctx_->ex.tunnel.phase = TunnelRunner::kOff;
    ctx_->ex.tunnel.tunnel = -1;
    ctx_->ex.tunnel.after_point = -1;
  }
  NodeStatus step()
  {
    const double now = ctx_->ports->now_s();
    return tunnel_handled(*ctx_->ports, ctx_->cfg, ctx_->sense, ctx_->decision,
                          ctx_->plan, ctx_->ex, now)
               ? NodeStatus::RUNNING
               : NodeStatus::FAILURE;
  }
  std::shared_ptr<Context> ctx_;
};

// ------------------------------------------------------------ 常规导航
// NavCruise：同区域目标的常规导航（换点先取消旧 Goal；失败按 nav_retry_s 重试；
// 操作手重指点 = 目标更新，取消重发；空目标/点不存在 → 停车）。
// 恒发受击旋转 spin_w（本段 = 非过洞段）；恒 RUNNING（换不换点由决策层下 tick 定）。
class NavCruise : public BT::StatefulActionNode
{
public:
  NavCruise(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<Context> ctx)
    : BT::StatefulActionNode(name, config), ctx_(std::move(ctx))
  {
  }
  static BT::PortsList providedPorts() { return {}; }

private:
  NodeStatus onStart() override { return step(); }
  NodeStatus onRunning() override { return step(); }
  void onHalted() override {}  // 换点/中止的收尾在 nav_cruise 与 Standby 里，这里不重复
  NodeStatus step()
  {
    nav_cruise(*ctx_->ports, ctx_->cfg, ctx_->sense, ctx_->decision.target_point,
               ctx_->ex, ctx_->ports->now_s(), ctx_->plan.op_fresh);
    return NodeStatus::RUNNING;
  }
  std::shared_ptr<Context> ctx_;
};

// ------------------------------------------------------------ 待机
// Standby：比赛外静默。下降沿（比赛刚结束）做**一次性**收尾——终止过洞流程
// （只清状态，不动折叠）、取消导航（在跑的导航节点就此终止）、停自旋（我们开的
// 我们关掉）；**不**发停车指令（不为比赛外停车专门留一条给电控的口——取消导航
// 后底盘自己停）。收尾之后完全静默：一条消息都不发，等比赛开始。
class Standby : public BT::StatefulActionNode
{
public:
  Standby(const std::string& name, const BT::NodeConfig& config,
          std::shared_ptr<Context> ctx)
    : BT::StatefulActionNode(name, config), ctx_(std::move(ctx))
  {
  }
  static BT::PortsList providedPorts() { return {}; }

private:
  NodeStatus onStart() override { return step(); }
  NodeStatus onRunning() override { return step(); }
  void onHalted() override {}
  NodeStatus step()
  {
    update_match_gate(*ctx_->ports, ctx_->cfg, ctx_->sense, ctx_->plan, ctx_->ex);
    return NodeStatus::RUNNING;  // 静默等待比赛开始
  }
  std::shared_ptr<Context> ctx_;
};

}  // namespace

// ------------------------------------------------------------ 注册
// 把全部节点注册进工厂：XML（trees/main.xml）里按名字引用。
void register_nodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<Context> ctx)
{
  factory.registerSimpleCondition(
      "IsMatchRunning", [ctx](BT::TreeNode&) { return tick_gate(*ctx); });
  factory.registerSimpleAction(
      "DecideTarget", [ctx](BT::TreeNode&) { return tick_decide(*ctx); });
  factory.registerNodeType<TunnelCross>("TunnelCross", ctx);
  factory.registerNodeType<NavCruise>("NavCruise", ctx);
  factory.registerNodeType<Standby>("Standby", ctx);
}

}  // namespace bbt_decide
