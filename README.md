# BBT — 哨兵保底决策树（独立工作区）

从主工作区 `27BT-UC/sentry_decide` 的保底分支裁剪出来的**独立版**：纯规则决策，
**不用 RL**，只保留哨兵导航、电控（底盘/折叠云台）、裁判系统三类接口。
雷达 / 自瞄 / RL 仲裁全部不要（自瞄完全交给电控）。购买分两路：**决策层自己
自动买**（弹/血低了、金币够就下单）；**操作手的买指令不经决策层**（裁判客户端/
电控直接办，决策只从裁判数据里看到账）——决策层唯一接的操作手输入是"指个临时点"。

## 五条硬规则（对应需求）

1. **随意换地图，region 和点全由人手填**：区域是 `decision.yaml` 里你自己定义的
   名字表（走廊可以就 `regions: [left, right]`，正式场地写 RM 五区），每个点声明
   属于哪个区域，每条洞的两个对正点各自声明它服务的区域。换地图 = 换这张表 +
   重标归属，代码一行不动。
2. **过洞按 region 判（与主工作区同款语义），且和 region 耦合在一个总开关上**：
   `enable_tunnel: true`（默认）→ 车当前区域与目标点区域不同时，在"区域图"上
   BFS 出要依次过的洞；入口 = 绑定车当前区域的对正点，出口 = 绑定目标区域的
   对正点，**不做几何猜测**；过完一条写记账区域、重新评估，多洞自动串联。
   `enable_tunnel: false` → **region 整套不需要**（regions 表、点的 region 字段、
   洞表全删掉），所有点互相直连，任何目标直接发导航（见 `plain_demo.yaml`）。
   车当前区域 = **纯记账**（不比最近点、不订阅区域话题，零区域接口）：
   开赛沿写 `start_region`；之后**唯一**的更新点是过洞完成写出口对正点的 region。
   记账为什么自洽：车只按决策发的导航目标移动，跨区移动必走"过洞流程"、过洞
   完成必写出口 → 不存在车绕开决策换区的路径。（比赛中被外力推过洞/搬走记账
   不会自纠——靠洞口"未折叠拒绝裸穿"保护和下一次开赛沿重置兜底。）
3. **地图和建图对齐**：点表按场地图纸系（STL）填；导航建图系若与图纸系不重合，
   用 `tools/align_map.py`（标几组"洞中心：场地系 ↔ 建图系实测"对应点）解算
   `field_to_map`（旋转 + 平移）写进 yaml，**加载时自动把全部点/对正点搬到建图系**，
   之后发给导航的 `/goal_pose` 与收到的 `/odom` 在同一系。仿真（Arena）不需要：
   桥自己做了镜像，保持 `enabled: false`。
4. **数据一律从真实 ROS 话题进出**：唯一进出口是 `RosPorts`，没有进程内注水通道。
   Arena 仿真数据由主工作区 `tools/arena_ros_bridge.py` 按同一套话题发进来，走的
   就是实车同一条数据通路；`/sentry/iface_status`（1 Hz）能看到每条接口的帧数/
   频率/年龄，用来确认数据确实在真实流动。
5. **比赛外完全静默**：没开赛一条消息不发。比赛结束的那一拍做**一次性收尾**
   （终止过洞流程 + 取消导航 + 停自旋——我们启动的我们关掉），**不给电控发停车**
   （取消导航后底盘自己停，没必要为此专门留一条口）；之后闭嘴等比赛开始，
   开始的那一拍切一次底盘模式。操作手的买类指令不受此门控（备赛阶段就能花钱）。

## 行为树（trees/main.xml）

执行层由 BehaviorTree.CPP 驱动（`tree_file` 参数可换自己的树），树节点只做编排，
决策/执行逻辑全部是 planner.cpp 的纯函数——跑树和单测直接调函数走的是同一份逻辑：

```
ReactiveFallback
├─ ReactiveSequence
│  ├─ IsMatchRunning   操作手指令（只消费指点）+ 自动购买 + 感知 + 门控
│  │                   （game_progress==4）；开始沿：切一次底盘模式 + 区域记账
│  │                   重置为 start_region；未开始/已结束 → 落到 Standby
│  ├─ DecideTarget     决策树（见下）
│  └─ ReactiveFallback
│     ├─ TunnelCross   过洞五段状态机（跨区域目标独占；无关则让位；关闭时恒让位）
│     └─ NavCruise     常规导航（换点/操作手重指点先取消再发；失败按节奏重试；
│                      空目标停车；本段恒发受击旋转 spin_w）
└─ Standby             比赛外静默：结束沿一次性收尾（终止过洞+取消导航+停自旋），
                       之后一条消息不发，等比赛开始
```

DecideTarget 内部的决策树（每 tick 自上而下，命中即止）：

```
0. 操作手指点（最高优先级，人比规则大）→ 去操作手给的点，到达即清除、恢复下述决策
1. 血量 < 30% 或 弹量 < 50（锁存；血量 ≥ 60% 且弹量 ≥ 150 同时满足才解除）
   → 回家（回血/补给都在家；"补了血没补弹就出门"会被弹量阈值立刻拉回来）
2. 其余 ──────────────────────────→ 巡逻：按 yaml 书写顺序环形轮换，
                                      到达（导航 success / 站进 0.5 m）或驻留满
                                      patrol_dwell_s 换下一个（孤岛区域的点自动跳过）
```

没有阵亡分支、也不做裁判数据新鲜度判定：
- 死了下位机锁死、发什么都动不了——血量 0 本来就低于回家阈值，锁存自然保持；
  残血复活 → 从原地继续回家；满血复活 → 锁存解除恢复巡逻。由血量自然衔接。
- 门控只看比赛状态（开始/结束）；裁判数据断流时按最后缓存的比赛状态继续打。

执行器把目标点翻译成动作；目标在别的区域时**必须先过洞**（绝不裸穿）：

```
入口对正点（绑定车当前区域）→ 折叠云台(/gimbal_fold_cmd=1) → 出口对正点
（绑定目标区域）→ 展开(=2) → 记账区域=出口区域 → 交回常规导航
```

保护逻辑：车区域与洞两端都对不上 → 拒绝穿洞（停车等区域修正）；未折叠却已在
洞口 → 停车拒绝 + 5 s 冷却；过洞中比赛结束 → 收尾终止（折叠保持、区域不记账）；
导航失败 → 2 s 节奏重试（驻留超时后决策自然换点）。

**受击旋转（小陀螺）**：所有非过洞导航段恒发 `spin_w`（含巡逻段、回家段、操作手
指点段）；过洞五段全程恒发 0（折叠穿洞时姿态必须稳定）；比赛结束收尾发一次 0。

**自动购买（决策层自己下单）**：判定只用裁判数据 + 金币门槛 + 冷却，比赛外不下单：

| 类别 | 触发（都在比赛进行中） | 下单 |
|---|---|---|
| 买弹 | 弹量 < `ammo_min_shots`(30) 且金币 ≥ `ammo_min_coins`(200) | `/sentry/buy_ammo` 100 发 |
| 买血 | **活着**且血量比例 < `hp_ratio`(0.40，故意比回家锁存 30% 高：先远程补一口) 且金币 ≥ 300 | `/sentry/buy_hp` 200 血 |
| 买复活 | hp == 0（=阵亡，不需要单独的阵亡判定）且金币 ≥ 200 | `/sentry/buy_revive` |

远程兑换有延迟：每类下单后冷却 `cooldown_s`(10 s) 才允许重试；**买没买到看裁判
数据**——血量/弹量涨了触发条件自然消失，没涨（钱不够/失败）冷却过了会再下单。
金币未知（裁判没发）一律不买（宁可不花不错花）。各条阈值写负数可单独关掉一类，
`enable: false` 三类全关。

**操作手指令**（`/referee/operator_cmd`，Int32MultiArray `[type, param, seq]`，
seq 每条 +1 用于去重）。**只消费指点（type 4）**；买类（1–3）不是决策层的事，
收到即忽略——操作手经裁判客户端/电控直接办，到账与否从裁判数据里看：

| type | 含义 | 决策层行为 |
|---|---|---|
| 4 | 指点（param=点表 id） | 最高优先级导航目标；**重指同一个点 = 导航目标更新**（取消旧 Goal 重发，卡死能拉起）；到达即清除、恢复巡逻；点表外的 id 静默忽略 |
| 1/2/3 | 操作手买弹/买血/买复活 | 忽略（记下 seq，不转发不处理） |

## 目录

```
BBT/
├── tools/align_map.py            # 地图对齐：解算 field_to_map 并写入 yaml
├── tools/plot_points.py          # 把 yaml 的点/洞画成 png（仿主工作区 candidates.png）
├── docs/                         # 点位图：decision.png / corridor_demo.png / plain_demo.png
├── src/bbt_decide/
│   ├── include/bbt_decide/
│   │   ├── types.hpp      # 区域表/点/洞/裁判快照/操作手指令/地图对齐（纯数据）
│   │   ├── ports.hpp      # 抽象接口：导航+电控+裁判+操作手+时钟（DebugPorts 供测试）
│   │   ├── planner.hpp    # 决策树 + region 路由(find/route_tunnels) + 过洞五段状态机
│   │   ├── bt.hpp         # 行为树上下文与节点注册（编排层）
│   │   └── ros_ports.hpp  # ROS 适配层（唯一数据进出口，话题契约见文件头）
│   ├── src/
│   │   ├── planner.cpp    # 决策/执行实现（纯逻辑，不依赖 ROS）
│   │   ├── bt.cpp         # BT 节点：薄包装 planner 纯函数
│   │   ├── config.cpp     # decision.yaml 加载 + 校验告警 + field_to_map 搬点
│   │   ├── ros_ports.cpp  # 话题收发实现（含 /current_region、操作手指令）
│   │   └── main.cpp       # 节点：20 Hz tickOnce 树 + 1 Hz 接口看板
│   ├── trees/main.xml     # 行为树（可用 Groot 打开 / tree_file 参数替换）
│   ├── config/
│   │   ├── decision.yaml       # RM 2027 场地配置（五区域 + 6 洞，Arena 已验证坐标）
│   │   ├── corridor_demo.yaml  # 走廊演示：自定义 left/right 两区域 + 中间一个洞
│   │   └── plain_demo.yaml     # 无洞演示：enable_tunnel=false，region 整套不需要
│   └── test/                   # planner_test（纯逻辑 17 组）+ bt_test（树驱动 4 组）
```

## 配置方法（decision.yaml）

```yaml
enable_tunnel: true               # 过洞总开关：false 时下面 regions/region/tunnels 全不需要
regions: [own_low, own_high, mid, enemy_low, enemy_high]  # 你自己的区域表，随意换
start_region: own_low             # 开局区域（开赛沿把区域记账重置成它）

points:                           # id 全场唯一；region 过洞开着时必填、关着时可省
  - {id: 1,  x: 25.39, y: 13.48, zone: home,   region: own_low}
  - {id: 10, x: 22.69, y: 9.42,  zone: patrol, region: own_low}

tunnels:                          # 每洞两个对正点，各带 region 绑定
  - name: own_road_tunnel
    center: [18.68, 12.85]        # 可选：地图对齐工具的基准
    align:
    - {p: [18.68, 11.50], region: own_low}   # 车在 own_low 从这个点进洞
    - {p: [18.68, 14.00], region: mid}       # 在 mid 从这个点进（双向）

safety:
  hp_home_ratio: 0.30             # 血量 < 30% → 回家
  hp_resume_ratio: 0.60           # 血量 ≥ 60% 参与解除锁存
  ammo_home_min: 50               # 弹量 < 50 同样锁存回家（-1 = 不按弹量回家）
  ammo_resume_min: 150            # 弹量 ≥ 150 才算恢复（血+弹同时恢复才出门）
  spin_w: 1.5                     # 受击旋转 rad/s（非过洞段恒转，过洞段恒 0）

auto_buy:                         # 决策层自己下单（详见上表）
  enable: true
  ammo_min_shots: 30              # 弹量 < 30 → 买弹 100 发（金币 ≥ 200 才买）
  hp_ratio: 0.40                  # 活着且血量 < 40% → 买血 200（金币 ≥ 300 才买）
  revive_min_coins: 200           # hp==0 且金币 ≥ 200 → 买复活
  cooldown_s: 10                  # 同类购买的下单间隔

field_to_map: {enabled: false, dx: 0, dy: 0, dyaw_deg: 0}   # 建图对齐，见下
```

走廊里搭洞的最小配置就两行区域、四个点、一个洞（见 `corridor_demo.yaml`）；
不过洞的配置连 regions 都不用写（见 `plain_demo.yaml`）。

校验与告警（加载时自动做，不致命但打日志）：区域名重复/region 不认识/开局区域
非法（以上仅过洞开着时）→ 直接报错；没配家或巡逻点、孤岛区域（没有任何洞连接）、
对正点重合、点贴闸线（< `gate_clear_m`，会挡过洞路径）→ 告警；过洞关着却写了
洞表 → 告警并忽略。

## 点位可视化

```bash
python3 tools/plot_points.py                                # 画 decision.yaml → docs/decision.png
python3 tools/plot_points.py -c src/bbt_decide/config/corridor_demo.yaml
python3 tools/plot_points.py -c src/bbt_decide/config/plain_demo.yaml
python3 tools/plot_points.py -c xxx.yaml -o docs/xxx.png    # 自定义输出
```

风格仿主工作区 `docs/field/candidates.png`：同一 region 同色、zone 用 marker
（home 方块 / patrol 圆点）、洞画闸轴虚线 + 洞名、对正点白芯圆圈标 region、
坐标范围按点自动外扩（走廊/RM 场地/任何地图都能画）。画的是 yaml 里的场地系
原始坐标（field_to_map 变换前）。

## 地图对齐（场地图纸系 → 导航建图系）

```bash
# 1) 建图后在地图上量几个洞中心在建图系里的坐标（至少 2 个，建议 ≥3 个分散的）
cat > pairs.csv <<'EOF'
id,map_x,map_y
own_road_tunnel,3.42,1.05
enemy_road_tunnel,-2.10,4.77
EOF
# 2) 解算（场地系坐标自动取 yaml 里的 tunnels[].center），看残差（>5cm 查点位）
python3 tools/align_map.py pairs.csv
# 3) 确认残差正常后写进 yaml
python3 tools/align_map.py pairs.csv --write
```

加载时全部点自动搬到建图系，决策层内部从此只用一个坐标系。只解旋转 + 平移
（米制建图不该有尺度差；`--scale` 可诊断尺度漂移）。

## 构建与测试

```bash
cd ~/BBT
source /opt/ros/jazzy/setup.bash
colcon build
colcon test --packages-select bbt_decide   # 或直接跑 ./build/bbt_decide/bbt_decide_tests
```

## 在 Arena 仿真里跑（数据从真实接口进去）

```bash
# 终端 1：Arena 桥（扮演定位/裁判/导航/电控折叠，按 yaml 里的 region 发 /current_region）
python3 ~/27BT-UC/tools/arena_ros_bridge.py --own-team blue --duration 420 --seed 1 \
    --config ~/BBT/src/bbt_decide/config/decision.yaml

# 终端 2：BBT 决策（data_source:=sim 选仿真话题表）
source ~/BBT/install/setup.bash
ros2 run bbt_decide bbt_decide_node --ros-args \
  -p config_file:=$HOME/BBT/src/bbt_decide/config/decision.yaml \
  -p data_source:=sim

# 终端 3：看数据流（确认每条接口都有真实数据）
python3 ~/27BT-UC/tools/iface_monitor.py      # 或 ros2 topic echo /sentry/iface_status

# 想模拟操作手指令（买弹 100 发 / 指点 id10）：
ros2 topic pub --once /referee/operator_cmd std_msgs/msg/Int32MultiArray \
  '{data: [1, 100, 1]}'
ros2 topic pub --once /referee/operator_cmd std_msgs/msg/Int32MultiArray \
  '{data: [4, 10, 2]}'
```

区域不需要桥发任何东西：纯记账（开赛沿 = start_region，过洞完成写出口），
桥发不发 `/current_region` 都无所谓。走廊演示（不走 Arena）：把 `config_file`
换成 `corridor_demo.yaml`，手动 `ros2 topic pub /odom /nav_status /referee/summary`
喂状态即可。

## 上实车

- 话题名不同的话改 `decision.yaml` 的 `topics.real:` 段（逐项 TO_FILL 已标注），
  节点参数 `data_source` 用默认 `real`。
- 区域判定零输入：纯记账（开赛沿 `start_region` + 过洞完成写出口），谁都不用发区域。
- 操作手三条买指令话题（/sentry/buy_ammo、/sentry/buy_hp、/sentry/buy_revive）与
  指令入口（/referee/operator_cmd）是预留口，实车话题名确认后改 topics.real 段。
- real 模式自带硬保护：一旦 `/sentry/data_source` 上听到 `"sim"` 声明，立即作废
  裁判快照 → 决策树落待机分支（静默）。

## 接口清单（全部走真实话题）

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 发 | /goal_pose | PoseStamped | 导航目标点（对正点/巡逻点/家/操作手指的点，已按 field_to_map 搬到建图系） |
| 发 | /nav_cancel | Empty | 取消当前目标（换点 / 操作手重指点 / 比赛结束收尾各一次） |
| 发 | /cmd_vel | Twist | 只发 0 停车；巡航速度归导航 |
| 发 | /move_mode | Int32 | 比赛开始沿切一次 INDEPENDENT（比赛外不发） |
| 发 | /cmd_spin | Float32 | 受击旋转：非过洞段恒 spin_w，过洞段恒 0，比赛结束收尾发一次 0 |
| 发 | /gimbal_fold_cmd | UInt8 | 1 折叠 / 2 展开（过洞用） |
| 发 | /sentry/buy_ammo | Int32 | **自动**买弹下单（决策层自己决定何时买） |
| 发 | /sentry/buy_hp | Int32 | **自动**买血下单 |
| 发 | /sentry/buy_revive | Int32 | **自动**买复活下单（hp==0 触发） |
| 发 | /sentry/iface_status | String(JSON) | 1 Hz 接口数据流统计 |
| 收 | /nav_status | Int32 | 0 idle/1 running/2 success/3 failure |
| 收 | /odom | Odometry | 位姿（x/y/yaw，建图系） |
| 收 | /gimbal_fold_state | Int32 | 0–4，2=已折叠 4=已展开 |
| 收 | /referee/summary | Float64MultiArray | [0]进度 [1]hp [2]hp_max [3]弹 [4]金币 [8]阶段剩余。自动买的判定与到账确认都看它 |
| 收 | /referee/operator_cmd | Int32MultiArray | 操作手指令 [type, param, seq]：只消费 type=4 指点，买类(1–3)忽略 |
| 收 | /sentry/data_source | String | "sim"/"real"（real 模式硬保护） |

注意：/nav_status 必须反映**当前目标**的状态——导航侧收到新目标后应回到 running，
否则决策层会把旧目标的 success 当作新目标已到达。

## 数据通路审计（所有数据必须走真实接口）

决策核心（planner）读入口全部经 `Ports` 抽象落到真实话题：
`referee()`←/referee/summary、`base_pose()`←/odom、`nav_state()`←/nav_status、
`fold_state()`←/gimbal_fold_state、`op_cmd()`←/referee/operator_cmd。
区域**没有任何输入接口**：纯记账（开赛沿重置 + 过洞完成写出口）。测试桩（StubPorts/DebugPorts）只存在于
test/，生产代码零引用；节点进程内没有任何向决策层注数据的旁路。

新鲜度保护（过期数据不当真数据用）：

| 数据 | 保护 | 行为 |
|---|---|---|
| 裁判 | 无新鲜度判定（按使用人口径：能收到比赛状态/血量/弹量即可） | 断流按最后缓存值继续；从未收到 → game_progress=0 → 待机（静默） |
| 位姿 | odom_stale_ms（默认 1 s） | 过期 → 位姿按无效：区域保持记账、到达只认导航 success、洞口保护不判（全部朝安全方向退化） |
| 折叠状态 | 无独立超时 | 未知(0) → 拒绝穿洞；指令后 3 s 没到位 → 中止过洞进冷却 |
| 导航状态 | 无独立超时 | 依赖各段 nav_timeout_s（25 s）兜底 |

如实标注的两处**本机乐观改写**（非话题来源，主工作区同款契约）：
`nav_set_goal` 发出后本机先把 nav_state 置 running、`nav_cancel` 后置 idle——占位
契约没有 ack，以 /nav_status 后续回调为准。因此 **/nav_status 是硬依赖**：它断流时
过洞流程靠 25 s 超时中止、巡逻靠驻留超时换点，行为能兜住但功能瘫痪；上实车前
确认导航侧一定回报状态。

## 与主工作区的差异

| | sentry_decide（主） | bbt_decide（本包） |
|---|---|
| 决策 | RL + 保底 + 仲裁 + Guard | 纯决策树（无 RL） |
| 区域 | 固定 5 区枚举 | 用户自定义区域表（任意数量），纯记账判定，随 enable_tunnel 整体开关 |
| 接口 | 导航/电控/裁判/雷达/自瞄/买卖 | 导航 + 电控 + 裁判 + 操作手指令/购买转发预留口 |
| 回家条件 | 血量 | 血量 OR 弹量（双条件解除） |
| 购买 | 操作手指令 + 决策建议 + 仲裁 | 决策层自动下单（操作手的买不经决策层） |
| 待机 | — | 比赛外完全静默（结束沿一次性收尾） |
| 地图对齐 | field_to_map + align_map.py | 同款（tools/align_map.py，按洞中心解算） |
| 依赖 | behaviortree_cpp + 自定义消息 | behaviortree_cpp + rclcpp 标准消息 + yaml-cpp |
| 点分类 | 7 种 zone | 2 种（home/patrol） |
