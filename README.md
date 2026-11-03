# BBT — 哨兵保底决策树

独立 ROS 2 Jazzy 工作区。纯规则行为树负责导航、回家补给、前哨争夺、操作手指点与折叠过洞，不运行 RL。通信使用 27BT-UC 的相同话题与消息定义，其他组继续发送 UC 的全量数据，BBT 只订阅需要的子集。

## 构建与验证

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rm2027_interfaces ros2_libxr bbt_decide
source install/setup.bash
colcon test --packages-select bbt_decide ros2_libxr
colcon test-result --verbose
python3 tools/merge_compile_commands.py
```

三个包提供裁判消息定义、串口桥和保底决策。测试覆盖纯逻辑、生产行为树、ROS 端口、整节点、PTY 串口及仿真适配，均不打开真实串口。需要核对另一个 UC 工作区时：

```bash
python3 tools/check_uc_interfaces.py --reference /你的路径/27BT-UC
```

检查消息字段、共享话题名、39 个战术点和六洞配置是否与参考工作区一致。

## 开局人工指定红蓝

与 27BT-UC 一样，桥和决策启动时都必须显式传入相同的 `own_team:=red` 或 `own_team:=blue`。不支持 auto，运行中不允许修改。裁判 `robot_id` 是诊断数据，不用于推断、核实或改变己方阵营。购买线帧发送者由人工选择确定为 7/107。

正式点表以红方为基准：家 ID100 为 `(2.61,1.52)`，蓝方部署时绕场心旋转为 `(25.39,13.48)`。全部39点的ID、坐标和region标签、六洞坐标沿用当前27BT-UC；新增区域几何校验它们的实际归属，配置矛盾时拒绝加载，不静默改点。UC 的 defend/harass/hole 在 BBT 中归入 patrol，保留 home/outpost/retake。BBT只按人工阵营解释裁判双方数据，不检测或识别场上敌我机器人；检测、跟踪和瞄准由自瞄系统负责。

当前暂用裁判地图：`field_to_map.enabled: true`、平移和旋转均为零。它表示暂按 map=field 运行，并不表示已完成未来实车导航地图的实测标定。`tools/align_map.py`、测量模板和后续标定流程完整保留，见 [联调文档](docs/field_bringup.md)。

## 实车接线与启动

其他组已经提供 UC 桥/定位/导航话题时，只启动 BBT 决策即可；同一个串口只运行一个桥，同一套导航输出只运行一个决策执行器。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 仅在现场没有现成 UC 串口桥时启动本包桥；示例选红方，实际由操作员决定。
ros2 launch ros2_libxr bridge.launch.py \
  port:=/dev/serial/by-id/你的设备 own_team:=red

# 另一终端，同样先 source，再启动 BBT。
ros2 run bbt_decide bbt_decide_node --ros-args \
  -p config_file:="$PWD/src/bbt_decide/config/decision.yaml" \
  -p data_source:=real -p own_team:=red
```

BBT 使用的共享消息与 UC 的类型/字段含义相同，详见 [UC 接口子集说明](docs/uc_interface_subset.md) 和 [电控接口文档](docs/电控接口文档.md)。普通导航必须补接标准Nav2 `ComputePathToPose`、`FollowPath`，过洞必须补接 `NavigateThroughPoses`；不能仅接旧 `/goal_pose` 执行器。`/goal_pose`保留为目标观测口，不得同时由旧执行器消费并发起导航。不消费的雷达、自瞄、RL 等输入不要求其他组停发，也不会成为 BBT 的启动依赖。BBT 的操作手指点通道是可选输入，没有指点发布者时仍正常执行规则决策。

## 坐标与后续标定

裁判场地为 28×15 m，点表、操作手点击采用固定场地系。当前恒等变换下 `/odom`、导航Action目标与观测用 `/goal_pose` 坐标一致。适配层直接读取 `/odom` 的位置，不查询 TF；改 header 不会变换位置。

以后另建导航地图时，变换方向为：

```text
p_map = R(dyaw_deg) * p_field + [dx,dy]
```

```bash
python3 tools/align_map.py pairs.csv --config src/bbt_decide/config/decision.yaml
python3 tools/align_map.py pairs.csv --config src/bbt_decide/config/decision.yaml --write
python3 tools/plot_points.py
```

CSV 列为 `id,map_x,map_y`，id 为洞名，后两列是洞中心在导航地图中的实测米制坐标，至少两个独立点，建议三个以上。工具不会因为残差超过 5 cm 自动禁止写入，应先核对结果。与 UC 的 `nav_to_field` 方向互逆，不要直接复制参数。开局换边先在场地系旋转战术几何，再输出 map 目标；真实点击坐标只做场地到 map 变换，不随己方阵营再旋转。

## 决策与过洞

目标优先级为操作手指点 → 血量/弹量锁存回家 → 我方前哨夺回 → 对方前哨争夺 → 巡逻。操作手坐标不吸附预设点，由用户已确认的 `region_geometry`命名多边形确认归属；同一region可由多块多边形组成，不同region内部不能重叠。洞体内部、场外、跨region共享边界等归属不明确的点击静默拒绝并保留原任务；入口边界仍可作为入口停车点。区域几何、洞连接图与折叠执行树分别负责判区、选路和过洞，三者均保留。不同region之间必须按洞连接图穿越，可以经mid连续过多个洞，不以普通坡道或绕行替代跨region过洞。

简单地图可关闭过洞并省略region及几何：此时接受有限数值的原始XY，由导航判断可达性；只要配置了区域几何，仍要求唯一归属。人工输入按本地接收顺序排队消费，后到的无效点击或购买类型不会覆盖、吞掉队列中较早的有效坐标。

人工目标不设固定120秒过期。只有最终原坐标的本次导航成功才开始20秒最短保持，不能用接近0.5米、入口/出口成功或旧回执提前计时。到达后连续脱战10秒与20秒保护并行；满20秒时回家锁存仍成立则结束人工任务回家，否则连续脱战满10秒才恢复自主，其他情况继续在点位作战。这两个默认时间分别由 `safety.op_min_hold_s`、`safety.op_combat_clear_s`配置。脱战只采用裁判 `SentryInfo.out_of_combat`，持久保留最后已知值，不由开火、敌人可见性或伤害推断；重复消息和正常慢更新不重启连续脱战计时，新的未脱战值才清零。购买与复活仍独立要求新鲜裁判数据，不能借用这个持久值绕过2秒时效。战术保持允许自旋和攻击，回家后不自动返回旧指点。有效任务内同坐标重复指令不取消重发，也不重置到达、脱战、超时和冷却；不同坐标替换目标，已经启动的洞穿越先安全完成。

回家锁存需要血量和弹量都恢复才解除；家点不可用时停车。前哨血量和控制状态均已知时才参与争夺，控制状态接受0/1/2。巡逻保留原15秒轮换，不增加高地代码限制；正式 `patrol_point_ids`为 `[28,40,70,101]`，按显式名单选择巡逻候选。109、114只供outpost任务使用，全部39点原有zone用途标签保持不变，未列入巡逻名单不等于删除点。业务流程均由树条件、动作和控制节点表达，黑板只保留请求、计时与物理进度事实，不另设阶段状态机。 节点通过显式类型化端口连接原生黑板，XML实例具有唯一语义名称；数据键、端口方向和子树映射见[行为树黑板说明](docs/behavior-tree-blackboard.md)。

配置点继续按 `safety.gate_clear_m: 0.5`检查到洞两端align连线段的距离，当前39点均不会因此被过滤。own road洞最近配置点47为1.502431 m，enemy road洞最近点153为0.914822 m。这个配置点筛选不套用到操作手任意原坐标；人工点仍按洞体内部、数值及区域归属等有效性检查接受或拒绝。

普通导航先调用 `/compute_path_to_pose`，从当前位置起逐段检查返回路径全程位于本次目标region的多边形并集内、且没有进入未经授权的洞体，再将该固定路径交给 `/follow_path`。路径不能穿入其他region后再回来，也不能只检查离散路径点。`FollowPath`不隐式重规划；失败后等2秒，重新规划并重新检查同一目标，无总重试次数上限。单次普通导航25秒超时仍保留，与取消人工任务120秒过期无关。服务缺失、规划拒绝或路径检查失败时停车，不退回旧单点执行。Action名称可通过 `nav_compute_path_action`、`nav_follow_path_action` 配置。

跨区目标通过区域图选择最少过洞次数的路线；多条路线并列时选择最近的首洞入口。新跨区任务在定位尚未有效时停车等待，不提前锁定某条洞。执行顺序为：

```text
入口导航并停车 → 折叠确认 → NavigateThroughPoses([洞中心途经点, 出口目标]) → 出口停车展开 → 原目标导航
```

洞中心是洞表显式配置的连续途经点，不由两端align取中点计算，不单独发布到 `/goal_pose`、不等待中心到达回执。Nav2 Action 默认 `/navigate_through_poses`，可用节点参数 `nav_through_poses_action` 指定命名空间；应启用连续跟踪整条路径的 NavigateThroughPoses 行为树。Action服务未接通时拒绝穿越，不退化为逐点停车。未配置中心的简单走廊只提交出口；有新鲜定位证明已过中心后，恢复同样只提交出口。机构反馈过期或导航失败会停车取消，保留实际进度并在5秒冷却后重新确认折叠；已到出口的展开失败只重试展开。出口到达后更新区域，展开确认后才交回普通导航。全部Action按请求代际及Goal身份隔离，旧 `/nav_status` 不参与完成判定。

正式洞出口位于洞体边界，导航在容差内返回成功时实际位置可能仍在边界内约2厘米。因此保留本次指定洞成功穿越的事实，只授权后续路径开头沿出口向外清出该洞体；不允许反向、清出后再入或穿另一洞，完整region约束不放宽。这是成功穿越证据，不增设阶段状态机。

过洞期间停自旋，普通导航发布 `spin_w`（默认1.5 rad/s）。明确停车统一取消本程序拥有的导航、停自旋并发零 `/cmd_vel`，包括冷却、空目标、机构等待、拒绝过洞及比赛结束；停止后的重复树tick不反复创建取消和新目标。导航执行端必须响应取消并停止旧速度输出。比赛外不发新运动/购买；结束沿另清人工目标、攻击意图与过洞事务。到点战术保持不作为故障停车，不持续发零速度压掉正常自旋。当前保留阵亡由下位机锁车和最后一次比赛状态的既有约定，不新增死亡停车或裁判慢更新停车分支；机构及购买另查数据时效。

## 通信与购买

BBT 接收 UC `/referee/summary`（10–14 元素）及 RobotHP/FieldEvent/SentryInfo，需要 RobotStatus 时只读取其诊断数据，不依赖其身份选择己方。兼容历史 Arena 9 元素和旧 BBT 19 元素布局。19 个 rm2027 消息的字段与 UC 相同。

实际购买与 UC 一致：`/referee/sentry_command` 的 SentryCommand 经桥封装成 `0x0301/0x0120`，由 `ref_tx` 下发。固定买弹100发，回血量参数1表示一次请求。执行层查比赛、摘要和兑换信息时效、脱战、余额、池和次数，确认次数后等待6秒到账窗口。`/sentry/buy_*` 仅作意图观测，不接第二个执行器。

操作手买血、买弹、复活直接作用于裁判系统，不经过BBT；BBT根据裁判回读独立继续自主购买。自主复活优先使用裁判免费资格，免费不检查金币；付费只检查裁判允许及余额足够支付裁判报价，不加200金币策略门槛。`auto_buy.enable`为自主购买总开关，`auto_buy.revive_enable`单独控制其中的自主复活；人工裁判操作不受BBT配置开关影响。

可选 `/referee/operator_cmd` 为 Float64 `[type,param,seq,x,y]`，BBT只消费导航type=4；桥保留 `0x0303` 点击解析。仿真 ID 指点为 `/sentry/operator_point` 的 Int32 `[type,param,seq]`。没有这些输入不影响 UC 标准数据接入。`/sentry/iface_status` 提供接口收发统计；`/sentry/decision_trace` 记录实际 BBT 目标、树执行证据和区域，用于回放验收。

## 六洞仿真与回放

使用本仓库 `tools/run_tunnel_suite.py`，导航控制和禁区约束参考 27BT-UC-simside。默认 Arena 根目录为 `~/rl_workspace/arena1_sim`，也可通过 `--arena-root` 指定。示例演示阵营不代表正式开局选择：

```bash
source /opt/ros/jazzy/setup.bash
build_dir="runs/bbt-acceptance-$(date -u +%Y%m%dT%H%M%SZ)"
python3 tools/build_acceptance_receipt.py --output "$build_dir"
source "$build_dir/install/setup.bash"
python3 tools/run_tunnel_suite.py --build-receipt "$build_dir/build_receipt.json" \
  --team red --jobs 6 --output runs/bbt_six_red
python3 tools/check_tunnel_suite.py runs/bbt_six_red/0_own_road_tunnel_forward_red
python3 tools/package_tunnel_evidence.py runs/bbt_six_red \
  --output runs/bbt_six_red_evidence.zip
```

每洞单独一场：从引擎出生点通过 ROS 和串口物理移动到测试侧，再启动生产 BBT。生成配置只选择既有点为固定任务并设置测试入口区域；不改点 ID、坐标、洞几何或引擎规则。验收检查入口单点目标、一次Nav2中心/出口任务、中心处持续运动且没有成功回执、折叠、真实屋顶穿越、出口展开及原目标继续导航。早期逐中心停车的记录保留为历史证据，不能作为连续途经点语义的验收。

每场输出 `replay.json`（Arena rmuc-replay-v4）、`trajectory.jsonl`、`interfaces.jsonl`、`case.json`、`validation.json` 和配置差异。`suite.json` 汇总各洞结果，`source_manifest.json` 引用构建收据并记录源文件、配置、运行库与二进制哈希。收据记录提交、源码状态、构建器与依赖版本、CMake 配置和安装产物摘要；它是本机执行记录，不是签名证明。验收开始和结束都会校验收据、源码和实际运行库，未提供收据时拒绝启动。

`tools/package_tunnel_evidence.py` 生成可共享的精简 ZIP，保留收据、源清单、验证器、每场轨迹和配置，并在导出前重验轨迹及汇总。ZIP 不含完整 replay、ROS 接口日志或视频；需要深入排查时保留原始 `runs/` 目录。成功重验后可将 ZIP 作为 PR 附件或发布资产提供给审查者。没有构建收据的历史目录会标为未验证，不能追溯证明其二进制来源。

`tools/run_arena.py --six-tunnels` 同样转到上述真实六洞验收，不再生成虚构的七区域链。普通短局可用 `tools/run_arena.py --team red --seconds 60 --output runs/bbt_smoke`；`--acceptance` 关闭前哨任务并提高买弹阈值以触发购买。

Arena 内部坐标经适配层转换为裁判系 `(x,15-y)`，红蓝采用同一坐标转换；战术换边只在配置部署时旋转一次。几何重建值及模拟导航不能替代实车实际间隙、导航回执归属、取消输出、速度仲裁和固件独立看门狗验收。

当前原生黑板版本已通过三包构建、10项CTest，以及红蓝六洞正反向24/24物理仿真。原坐标(11.49,0.58)准确到达并满足20秒保护；源/二进制哈希一致、0坏包。三项准备导航各用一次显式停稳重试。证据路径、失败保留与实机边界见[本轮验证记录](docs/specs/bbt-implementation-validation.md)。

## 完整混合策略比赛

六洞验收通过后，可运行标准420秒比赛。只有 `--own-team` 指定一方的哨兵运行BBT，对方哨兵及其余机器人各自从本地现有、与机型匹配的2027策略池随机分配。双方独立抽取，随机种子、候选池和每台机器人的具体策略写入 `policy_assignments.json`。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/run_bbt_match.py --own-team red --seconds 420 --seed 2027 \
  --domain 208 --output runs/bbt_full_match_red
```

策略来源默认使用Arena本地已缓存的RMUC2027策略目录，可用 `--policy-root` 指定已有兼容策略库。红方2773哨兵的移动、模式、折叠、购买来自BBT实际ROS+PTY链路；自瞄替身按BBT开火/目标意图瞄准。每台机器人仅接收引擎提供的自身视野及正常队内通信。其余机器人的控制权只属于抽到的策略，不借用它们的哨兵动作来驱动BBT。

回放保留每个物理tick的实际动作和完整结果，并通过重放全部输入比较物理帧与终局摘要来验证。原版Arena3D查看器使用隔离记录目录，不修改已有Arena比赛记录：

```bash
/home/chengdong/rl_workspace/.venv/bin/python tools/view_arena_replays.py \
  runs/bbt_full_match_red --port 8878
```

打开 `http://127.0.0.1:8878/`，在比赛与回放中选择播放。实际完整比赛结果、策略分配和确定性重放验证分别位于运行目录中的 `result.json`、`policy_assignments.json` 和 `validation.json`。

历史已完成记录为 `runs/bbt_full_match_20261101_v2`：种子2027，红方哨兵BBT，其余九台独立策略；第146秒红方基地被摧毁，蓝方获胜。比赛按引擎终局提前结束，全部2920个物理tick的确定性重放核验通过。当时的六洞验收记录为 `runs/bbt_six_red_20261101` 和 `runs/bbt_six_blue_20261101`，共12项通过。这些记录保留原始证据，不证明本文新增区域几何、人工目标生命周期及规划审核链已通过验证；新实现须以自己的源/二进制哈希和测试报告验收。

本局哨兵在多车接触下未能展开、没有开火，运行和回放校验通过不代表战术流程正常；证据与归因见[展开受阻诊断](docs/match_v2_diagnosis.md)。

单哨兵功能演示（60秒红方占领全部前哨，依次展示巡逻、买血、买弹、回家、复活和指点）见[演示说明](docs/feature_demo.md)。洞中心已修正为连续途经点，Nav2接口和验收见[途经点说明](docs/nav2_waypoints.md)。
