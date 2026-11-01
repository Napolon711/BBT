# BBT — 哨兵保底决策树

独立 ROS 2 Jazzy 工作区。行为树负责导航、回家补给、前哨争夺、操作手指点和折叠过洞；不运行 RL。通信以 `27BT-UC` 当前原始裁判帧方案为准。

## 构建与验证

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rm2027_interfaces ros2_libxr bbt_decide
source install/setup.bash
colcon test --packages-select bbt_decide ros2_libxr
colcon test-result --verbose
python3 tools/merge_compile_commands.py
```

三个包分别提供裁判消息定义、串口桥和决策。测试包括纯逻辑、实际行为树、ROS 适配、整节点、伪终端串口及 Arena 适配器；测试不会打开真实串口。

## 实车运行

现场按 [坐标约定与联调步骤](docs/field_bringup.md) 操作；当前直接使用裁判坐标，无需另做地图变换。另建导航地图时可使用 [测量 CSV 模板](docs/calibration_pairs.csv.example)。

`src/bbt_decide/config/decision.yaml` 已设置 `map=field` 的恒等变换，位置输入与导航均采用裁判场地坐标，启动：

```bash
source install/setup.bash
ros2 launch ros2_libxr bridge.launch.py port:=/dev/serial/by-id/你的设备
ros2 run bbt_decide bbt_decide_node --ros-args \
  -p config_file:=$PWD/src/bbt_decide/config/decision.yaml -p data_source:=real
```

正式配置保留 `require_map_alignment`，当前用 `field_to_map.enabled: true` 和零平移、零旋转确认两系重合。以后接入独立导航地图时再标定；将 enabled 设为 false 会使 real 模式等待。

决策只使用“我方／敌方”：原人工点表的 ID、坐标及 region 保持不变，不按红蓝切换战术几何，也不因缺少阵营 ID 阻止导航。协议中的红蓝字段只在通信适配层转换成我方/敌方快照。家点仍为 `(25.39,13.48)`。前哨几何坐标按 27BT-UC：我方 `[10.98,3.60]`、敌方 `[17.02,11.40]`；两个战术 outpost 点为 `(15.69,10.90)` 和 `(13.65,11.15)`，分别写入点 ID 21、22；retake 点 ID 20 保留。地图见 [referee_map_20261101.png](docs/referee_map_20261031.png)，可用 `outpost.enable: false` 关闭该分支。

## 地图与自定义场地

`field_to_map` 的方向是：

```text
p_map = R(dyaw_deg) * p_field + [dx,dy]
```

当前 map 与裁判场地系相同，因此 `dx=0, dy=0, dyaw_deg=0`，操作手点击、点表、导航目标和 `/odom` 使用同一套米制坐标。若以后使用独立导航地图，变换与参考工程的 `nav_to_field` 方向互逆，不能直接复制同一组参数。加载时点/洞统一变换到导航 map 系，odom 必须提供 map 系位置。

```bash
python3 tools/align_map.py pairs.csv --write
python3 tools/plot_points.py
```

以上标定命令仅在另建导航地图时使用。`pairs.csv` 使用 `id,map_x,map_y` 列，id 是洞名，坐标是该洞中心在导航地图中的实测值。

自定义场地按自己的地图填写点位及敌我归属。`enable_tunnel: false` 时区域和洞表均可省略；开启时使用 `regions`、`start_region`、各点及洞两端的 region 路由。演示配置为 `corridor_demo.yaml` 和 `plain_demo.yaml`。

## 决策与过洞

目标优先级：操作手指点 → 血量/弹量锁存回家 → 我方前哨夺回 → 对方前哨争夺 → 巡逻。指点同样检查可达性与洞口距离；重指同一点会强制下发一次，即使已在到达半径内。`safety.op_goal_timeout_s` 默认 120 秒，到期恢复规则决策；过洞事务仍须先安全结束。前哨血量和控制状态必须同时已知，控制状态只接受 0/1/2。回家锁存需要血量和弹量都恢复才解除；家点不可用时停车，不再降级去进攻/巡逻。

跨区域目标通过区域图 BFS 选择隧道。执行顺序：入口导航 → 折叠确认 → 出口导航 → 展开确认 → 普通导航。到达出口即更新区域记账，机构恢复单独等待：

- 每次折叠/展开指令都清除旧的完成值，等待后续真实反馈；反馈默认 1 秒失效。
- 穿洞期间持续要求“已折叠”。反馈失效或导航失败时取消运动，保留出口方向和事务，冷却后重新发送折叠请求，只有收到新的折叠到位反馈才继续。
- 已到出口但展开失败时保持停车并重试展开，不把执行权交给普通导航。
- 过洞接管时清除旧导航目标记录，防止往返后停在对正点而漏发最终目标。
- 普通导航与洞口导航都使用 `nav_timeout_s`（默认 25 秒）；普通导航失败后按 `nav_retry_s` 重试。

冷却、重试、空目标窗口只发送一次停止/取消；重新开始运动后解除停止锁存。配置字段转换错误返回中文错误及行号，购买单位、超时和锁存阈值在加载时检查。

比赛未开始时不发运动/购买指令；比赛结束沿一次性取消导航、停止旋转并清理过洞事务。区域按开局和已确认出口记账，不做几何区域推断。裁判比赛状态继续采用最后缓存值；折叠和购买执行通路分别检查反馈/源数据时效。

非过洞段发布 `spin_w`，实车适配层切换 ROTOR 模式；过洞段发送零旋转并切回 INDEPENDENT。底盘速度来自导航，桥按原样发送 x/y/w，单位 m/s、rad/s，断流默认 200 ms 全部归零。

## 通信和购买

完整契约见 [电控接口文档](docs/电控接口文档.md)。桥接收 `ref_frame/chassis_state/ahrs_quaternion/yawmotor_angle`，发送 `ref_tx/chassis_data/chassis_mode/fold_cmd`。旧 `sentry_ref/ref_buy/gimbal_fold` 串口契约不再使用。

实车摘要为 14 元素，并通过 `RobotStatus/RobotHP/FieldEvent/SentryInfo` 补齐阵营、双方前哨控制和兑换信息。兼容旧 BBT 完整 19 元素及 Arena 的 9/12 元素摘要；截断长度、NaN/Inf 等非法输入丢弃。`/sentry/iface_status` 提供收发数量、频率、年龄和丢弃统计。

自动买弹固定一次 100 发；`hp_amount=1` 表示一次远程回血请求，恢复量由裁判规则决定。实际购买由 `/referee/sentry_command` 转为 `ref_tx` 的 `0x0301/0x0120`。适配层检查有效裁判数据、脱战状态、余额、剩余兑换池与次数上限；按回执次数 +1 重发，未确认不会跳号，确认后再等 6 秒到账窗口。复活使用裁判免费/付费资格及实际金币需求，复活后撤销请求位。

`/sentry/buy_ammo`、`buy_hp`、`buy_revive` 是意图观测口，不能再接第二个执行器重复购买。操作手买类指令仍不经过决策层。

桥解析 `0x0303` 小地图点击并发布 `/referee/operator_cmd` 的 Float64 数组 `[type,param,seq,x,y]`，按内容去除服务器重发；完全相同的再次点击无法与服务器定频重发区分。仿真指点 ID 使用 `/sentry/operator_point` 的 Int32 `[type,param,seq]`；`/referee/operator_cmd` 保留 Float64 原始小地图点击，避免同名话题混用类型。两路独立去重，递增 seq 可重发同一点。点击坐标吸附到 `op_snap_m` 内的点表点。

## 仿真与联调边界

仿真使用 `data_source:=sim`，默认 `referee_protocol` 为字符串 `2027`，完整裁判消息、购买执行和底盘模式与实车一致；来源守卫只在 real 模式开启。旧式直接摘要仿真可显式使用 `referee_protocol:=legacy`，实车禁止 legacy。模拟器必须与手填点表使用同一套坐标；决策不做阵营镜像。

Arena 的 `rmuc2027` 场地为 28×15 m。内部坐标经适配层统一转换为裁判系：`x_field=x_arena, y_field=15-y_arena`，角度和 y 速度同时反号；两个阵营使用相同转换。BBT 接收的仿真 `/odom` 与点表均为裁判坐标，`field_to_map` 保持恒等；原始 Arena JSON/画面中的 y 方向与 BBT 不同。这里确认的是坐标接线一致，未标注尺寸和部分高度仍是模拟器重建，不能据此认定全部实场几何完全相同。

本仓库提供独立的 Arena-1 启动与验收脚本，无需训练、RL 或雷达消息包：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/run_arena.py --acceptance --seconds 60 --output runs/arena_check
python3 tools/check_arena_run.py runs/arena_check --tunnel --purchase

# 过洞期间丢失机构反馈，检查停车、重新折叠和恢复
python3 tools/run_arena.py --acceptance --seconds 50 --fault-chassis 16:20 \
  --domain 185 --output runs/arena_fault_check
python3 tools/check_arena_run.py runs/arena_fault_check --tunnel --purchase --recovery
```

默认引擎目录为 `~/.zcode/workspace/default/Arena-1`，可用 `--arena-root` 或 `ARENA_ROOT` 指定。输出目录不能复用已有结果；`replay.json` 可在 Arena 观战页载入，`interfaces.jsonl`、`physics.json`、`result.json` 和 `validation.json` 保留接口与物理证据。默认 DDS 域 186，仅本机发现。

`--acceptance` 只改运行目录中的配置副本：关闭未标定的前哨点，并把买弹触发阈值改成 350，从真实开局 300 发触发一次远程购买；不修改物理引擎、金币、弹量或裁判数据。省略该参数即可运行原配置。默认选择 Arena 的 blue 槽以对应原点表的我方半场；队名仅用于仿真适配及裁判身份，不旋转任何战术点。

链路为 Arena → 官方裁判帧/SharedTopic → PTY → 本仓库 ros2_libxr → BBT；导航速度、折叠和购买也走反向串口链路。折叠完成读取引擎真实变形进度。导航是 SDK 适配替身，洞口要求位置误差 ≤8 cm 且速度 ≤0.12 m/s 才报告成功，穿洞路径持续到该目标结束。仿真按实时速度运行以匹配生产节点单调时钟。

已验证一次公路隧道完整通行、出洞后继续运动、买弹实际到账、重发幂等，以及机构断流后的恢复。没有把六洞双向通行或实际导航性能作为已完成结论。Arena 的免费复活由引擎自动读条触发；付费复活请求已映射，但本次短局未覆盖死亡/复活。

真实导航侧仍须实现 `/goal_pose`、`/nav_cancel`、`/nav_status`（0 idle/1 running/2 success/3 failure），确保回执属于当前目标；目前没有 Goal UUID。取消必须停止导航输出，零速仲裁和固件独立控制看门狗仍需实车验收。自瞄消费 `/sentry/attack_cmd` 并自行执行跟踪、弹道和断流禁火。
