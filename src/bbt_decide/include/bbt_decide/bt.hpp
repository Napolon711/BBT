// BBT 行为树编排（trees/main.xml）：
// IsMatchRunning：操作手指点（买类忽略）、感知、自动购买和比赛门控。
// DecideTarget：指点 → 回家 → 前哨夺回/进攻 → 巡逻。
// TunnelCross / NavCruise：过洞优先，普通导航让位。
// Standby：比赛结束沿一次性收尾，其后静默。
// 区域只在开局和确认到达洞出口时记账，没有 SyncRegion 节点或外部区域输入。
#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <memory>

#include "bbt_decide/planner.hpp"
#include "bbt_decide/ports.hpp"

namespace bbt_decide
{

// 全树共享黑板：所有节点通过捕获的 ctx 读写，节点自身不存跨 tick 状态
// （ReactiveSequence 会 halt 兄弟节点，成员状态不可靠——同主工作区 Context 约定）。
struct Context
{
  Config cfg;                 // 配置（点表/洞表/阈值），加载后当常量用
  std::shared_ptr<Ports> ports;  // 端口集合（ROS 适配层 / 测试桩）
  PlanState plan;             // 决策记忆：锁存 / 防抖 / 巡逻进度 / 区域记账
  ExecState ex;               // 执行器状态：导航下发记账 + 过洞状态机
  Sense sense;                // 本 tick 感知快照（IsMatchRunning 刷新，后续节点共用）
  Decision decision;          // 本 tick 决策输出（DecideTarget 产出，执行节点消费）
};

// 注册全部节点。参数：factory — BT 工厂（整棵树只调一次，重复注册同名会抛异常）；
//                   ctx — 共享上下文，生命周期必须覆盖整棵树。
// 副作用：注册 2 个简单节点（IsMatchRunning / DecideTarget）和
//         3 个动作节点类型（TunnelCross / NavCruise / Standby）。
void register_nodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<Context> ctx);

}  // namespace bbt_decide
