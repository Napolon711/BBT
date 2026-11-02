# 行为树名称、端口与黑板

生产树为 `src/bbt_decide/trees/main.xml`。`BehaviorTree ID="Main"` 标识树定义，root 的 `main_tree_to_execute="Main"` 指定默认入口，运行节点当前也以 `Main` 建树。各叶节点、控制节点和装饰节点都有唯一的语义 `name`，用于日志和工具中区分实例；节点类型仍由 XML 标签决定。

## 数据约定

C++ 在创建树时调用 `make_blackboard()` 注入配置、外部接口与初始值，并把该黑板传给 `createTree()`。运行中的领域状态只保存在 BehaviorTree.CPP 原生黑板中。

| 默认键 | C++ 类型 | 用途 |
|---|---|---|
| `config` | `std::shared_ptr<const Config>` | 已加载并完成坐标部署的只读配置；阈值和点表继续来自 YAML |
| `io` | `std::shared_ptr<Ports>` | ROS 或测试用外部接口；这是依赖对象，区别于行为树数据端口 |
| `sense` | `Sense` | 本 tick 的裁判、位置与导航感知快照 |
| `plan` | `PlanState` | 人工目标、资源锁存、巡逻与防抖等决策记忆 |
| `execution` | `ExecState` | 导航请求归属、重试时间和洞穿越真实进度 |
| `target` | `Decision` | 本 tick 选定并经过防抖确认的目标 |

每个节点只声明自己使用的领域端口，而不是把全部字段展开到 XML。`config`、`io` 是输入依赖；其他端口按节点行为声明为输入、输出或双向。

```xml
<ReadInputs name="read_inputs"
            config="{config}" io="{io}"
            plan="{plan}" sense="{sense}"/>
<MatchRunning name="match_running" sense="{sense}"/>
<SelectOperator name="select_operator"
                io="{io}" plan="{plan}" target="{target}"/>
```

以上为不同位置的实际叶节点示例，不能据此省略完整生产树中的其他条件。`ReadInputs` 读取 config/io、双向更新 plan、输出 sense；`MatchRunning` 只读取 sense；`SelectOperator` 读取 io/plan、输出 target。

XML 属性左侧是节点端口名，花括号内是黑板键。可以写 `sense="{other_sense}"` 将该实例接到另一个 `Sense`，也可通过 `SubTree` 转接领域键。映射后的输入键须以正确的 C++ 类型初始化，输出键可以由生产节点建立。缺少输入或类型不匹配会明确报错；不会退回隐藏的共享上下文。

## 节点实现与状态寿命

节点注册接口为 `register_nodes(factory)`，不接收某棵树的上下文。每个叶的 manifest 声明原生 InputPort、OutputPort 或 BidirectionalPort；叶回调通过 `getInput()` 取得类型化值，修改输出/双向值后通过 `setOutput()` 发布。Action 返回 RUNNING 时也发布状态，因此响应式父节点中断不会丢失人工目标、冷却时间或穿越证据。中止本身不代表物理任务已完成。

一份 factory 可以创建多棵树，只要各树使用独立黑板，就不会共享领域运行状态。`io` 是否共用由调用方显式注入决定。生产节点从黑板读取 plan、execution、target 的只读快照用于日志和 trace，不在每 tick 维护第二份 Context。兼容测试入口 `step()` 仅在调用边界把旧参数写入新黑板、tick后读回。

增加叶节点时，应同时注册准确的端口类型与方向，在 XML 中连接需要的键，并给实例命名。节点可依赖的领域值应由参数明确列出，业务优先级和执行顺序继续在 XML 中表达。

## 验证

`bbt_decide_blackboard_test` 从公开 XML、factory 和黑板接口验证输入/输出/双向重映射、不同树隔离、错误输入、SubTree 及 RUNNING/halt 后持久状态；既有生产树与 ROS 回归继续验证业务行为。本轮三个包构建、10项CTest及红蓝六洞双向24场均通过，原坐标到达后保护与解除时序通过；完整证据见[验证记录](specs/bbt-implementation-validation.md)的原生黑板复验章节。
