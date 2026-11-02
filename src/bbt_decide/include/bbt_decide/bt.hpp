// Production behavior tree: condition and I/O leaves share durable blackboard facts.
// main.xml directly expresses target priority, operator lifecycle and tunnel sequencing.
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
  ExecState ex;               // 导航请求归属与真实穿越证据
  Sense sense;                // 本 tick 感知快照（IsMatchRunning 刷新，后续节点共用）
  Decision decision;          // 本 tick 决策输出（DecideTarget 产出，执行节点消费）
};

// 注册全部节点。参数：factory — BT 工厂（整棵树只调一次，重复注册同名会抛异常）；
//                   ctx — 共享上下文，生命周期必须覆盖整棵树。
// 注册条件及单项I/O动作；流程、优先级和重试分支由XML组合。
void register_nodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<Context> ctx);

}  // namespace bbt_decide
