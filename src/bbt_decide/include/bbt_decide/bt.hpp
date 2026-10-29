// BBT 行为树：把"感知 → 区域同步 → 决策树 → 执行"编排在 trees/main.xml 里。
//
// 与主工作区同款分层：BT 节点只做编排，决策/执行逻辑全部是 planner.cpp 的纯函数
// （decide_tree / tunnel_handled / nav_cruise / resolve_region），所以"跑树"和
// "单测直接调 step()"走的是同一份逻辑，行为不会分叉。
//
// 树结构（ReactiveSequence 每 tick 从头重跑，与主工作区 main.xml 同风格）：
//   ReactiveFallback
//     ReactiveSequence
//       IsMatchRunning   操作手指令（买类随时转发）+ 感知 + 门控（只看开始/结束）；
//                        比赛开始沿切一次底盘模式
//       SyncRegion       区域解析（nearest_point 自判 / external + 防抖窗；
//                        enable_tunnel=false 时整套跳过）
//       DecideTarget     决策树：操作手指点（最高优先级）→ 血量/弹量锁存 → 家；
//                        巡逻轮换 + 换点防抖
//       ReactiveFallback
//         TunnelCross    过洞五段状态机（需要时独占；与过洞无关则让位）
//         NavCruise      常规导航（换点/操作手重指点先取消再发；失败按节奏重试；
//                        空目标停车；本段恒发受击旋转 spin_w）
//     Standby            比赛外静默：结束沿一次性收尾（终止过洞 + 取消导航 + 停自旋），
//                        之后一条消息不发，等比赛开始
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
// 副作用：注册 3 个简单节点（IsMatchRunning / SyncRegion / DecideTarget）和
//         3 个动作节点类型（TunnelCross / NavCruise / Standby）。
void register_nodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<Context> ctx);

}  // namespace bbt_decide
