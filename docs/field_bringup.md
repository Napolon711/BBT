# 坐标约定与实车联调

以下命令从工作区根目录执行。当前按用户确认直接使用裁判系统场地坐标，配置已设置恒等变换，无需采集标定对应点。已有仿真记录只证明记录中的场景，实车结果另行验收。

## 1. 当前直接使用裁判坐标

`decision.yaml` 当前配置为：

```yaml
field_to_map:
  enabled: true
  dx: 0.0
  dy: 0.0
  dyaw_deg: 0.0
```

此时 map 与场地系相同，操作手点击、点表、导航目标和位置输入均使用裁判坐标，单位米。`require_map_alignment: true` 保留，enabled 表示已确认采用这一坐标约定。

Arena 内部和 BBT 使用的裁判坐标关系为：

```text
x_field = x_arena
y_field = 15 - y_arena
yaw_field = -yaw_arena
(vx_field, vy_field) = (vx_arena, -vy_arena)
```

仿真适配层已经完成上述换算，红蓝阵营使用同一公式，BBT 内无需再翻转。Arena 原始 JSON/画面中的坐标不能直接当成 BBT 坐标抄入点表。例如内部蓝前哨 `(17.024, 3.6)` 对应裁判系 `(17.024, 11.4)`，内部红前哨 `(10.976, 11.4)` 对应 `(10.976, 3.6)`；这两个位置是建筑位置，不是战术导航点。

仿真坐标已统一，但地图中未标注尺寸和部分高度属于重建值，洞口对正点也由配置声明。实车几何和通行性仍需现场核对。仿真默认 blue 槽对应当前高 x 半场家点；切换 team 不会自动改点表。

### 以后另建导航地图时再标定

点表和洞坐标属于场地系，导航目标属于 map 系。变换方向是：

```text
p_map = R(dyaw_deg) * p_field + [dx, dy]
```

复制 [测量表模板](calibration_pairs.csv.example)，填入洞中心在导航地图中的坐标，单位米：

```bash
cp docs/calibration_pairs.csv.example /tmp/bbt_calibration_pairs.csv
```

`id` 必须与 `decision.yaml` 中的洞名一致。保留至少 2 个不同的洞中心，建议测量 3 个以上、分布在不同位置；删除未测行，不要留空或填零代替未知值。场地系坐标由工具从 `tunnels[].center` 读取，表中只填导航 map 系坐标。

可在 RViz 的 map 坐标系中量取洞中心。先确认测到的是同一个物理位置，坐标单位均为米、地图尺度正确。两点只能确定旋转和平移，第三点及额外检查点可以帮助发现错配。重复测同一个洞不能增加独立约束。

先解算并查看残差：

```bash
python3 tools/align_map.py /tmp/bbt_calibration_pairs.csv \
  --config src/bbt_decide/config/decision.yaml
```

逐点检查残差。工具对超过 5 cm 的点给出提示，这是复查提示，不能替代洞口实际间隙要求；确认无误后再写入：

```bash
python3 tools/align_map.py /tmp/bbt_calibration_pairs.csv \
  --config src/bbt_decide/config/decision.yaml --write
git diff -- src/bbt_decide/config/decision.yaml
```

写入会设置 `field_to_map.enabled: true`。只有确认两个坐标系重合时，才能采用 `dx=0, dy=0, dyaw_deg=0`。`--scale` 只用于尺度诊断，本配置只支持旋转和平移；写入时不要使用该选项。

## 2. 核对点位、区域和位姿输入

| 项目 | 现场确认内容 |
|---|---|
| `start_region` | 与每次比赛开始时车实际所在区域相符；当前为 `own_low` |
| 家点 ID 1 | 场地系 `(25.39, 13.48)` 对应实际补给位置，导航可达 |
| retake 点 ID 20 | 当前保留 `(12, 4)`；确认其符合实际夺回任务和区域归属 |
| outpost 点 ID 21、22 | 当前为 `(15.69, 10.90)`、`(13.65, 11.15)`；确认能执行任务且可达，待确认时在联调配置中设 `outpost.enable: false` |
| 洞两端 `align` | 对正点分别位于声明的区域，机构完全通过后才到出口目标 |
| `/odom` | position 已处于导航 map 系；仅修改 `header.frame_id` 不会变换坐标 |
| 操作手点击 | 点击坐标和点表使用同一场地系，才能正确吸附到点表点 |

当前适配层直接使用 `/odom` 的位置数值，不查询 TF。若导航原始里程计在 odom 系，应由定位/导航侧提供 map 系位姿并接到配置的 `topics.real.odom`；同时检查图上的实际位置与变换后的家点、洞口是否吻合。

点位或变换有修改后，重启决策节点重新加载。`field_to_map` 只在配置加载时应用一次。

## 3. 构建和接口检查

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rm2027_interfaces ros2_libxr bbt_decide
source install/setup.bash
colcon test --packages-select bbt_decide ros2_libxr
colcon test-result --verbose
```

导航进程由实际导航工作区启动，本仓库只提供目标和回执接口。确认串口设备，再在一个终端启动唯一的桥进程：

```bash
ls -l /dev/serial/by-id/
ros2 launch ros2_libxr bridge.launch.py port:=/dev/serial/by-id/你的设备
```

其他终端也先 source ROS 和本工作区，再分别检查：

```bash
ros2 topic info /referee/operator_cmd --verbose
ros2 topic echo /referee/summary --once
ros2 topic echo /referee/robot_status --once
ros2 topic echo /odom --once
ros2 topic hz /gimbal_fold_state
```

| 输入/输出 | 应观察到的契约 |
|---|---|
| `/referee/operator_cmd` | `std_msgs/msg/Float64MultiArray`，`[type,param,seq,x,y]`；现场点击后有数据 |
| `/sentry/operator_point` | `std_msgs/msg/Int32MultiArray`，仿真 ID 指点；默认正式配置关闭此订阅 |
| `/referee/summary` | 实车 14 元素摘要；比赛进行中第 0 项为 4 |
| `/referee/robot_status` | 实际哨兵身份 7 或 107；用于裁判双方映射及购买 |
| `/gimbal_fold_state` | 0 未知 / 1 折叠中 / 2 已折叠 / 3 展开中 / 4 已展开；持续真实刷新，建议至少 10 Hz |
| `/nav_status` | `std_msgs/msg/Int32`，0 idle / 1 running / 2 success / 3 failure；属于当前导航目标 |
| `/nav_cancel` | `std_msgs/msg/Empty`；导航侧收到后停止当前目标的控制输出 |

## 4. 启动决策并观察

当前正式配置已确认裁判系恒等变换。导航及位置输入采用同一坐标后，启动：

```bash
ros2 run bbt_decide bbt_decide_node --ros-args \
  -p config_file:="$PWD/src/bbt_decide/config/decision.yaml" \
  -p data_source:=real
```

比赛进行中会按当前配置自主选点，非过洞段默认 `spin_w: 1.5`，自动购买默认开启。首次只验导航或机构时，可另存一份联调配置，按试验目的设置 `auto_buy.enable: false`、`outpost.enable: false` 或 `safety.spin_w: 0`，并把 `config_file` 指向该文件。

另外观察：

```bash
ros2 topic echo /sentry/iface_status
```

检查输入 `count` 是否持续增加、`last_age_s` 是否合理、`dropped` 是否增加。比赛状态没有进入 4 时不发导航；real 模式收到 `/sentry/data_source` 的 `sim` 声明后会锁存保护，需要先停止仿真发布源再重启决策节点。默认约定会保留最后一次比赛状态，裁判断流不会自动变成比赛结束。

## 5. 逐项记录实车结果

先验普通导航，再验单洞，最后扩大到需要使用的各洞及反向通行。

| 项目 | 通过依据 | 现场结果 |
|---|---|---|
| 普通导航 | map 系目标正确，到达回执属于当前目标 | 待填 |
| 取消及速度轴 | 取消后停止当前导航输出；x/y/w 单位和正负方向正确 | 待填 |
| 自旋 | ROTOR 正负和零角速度符合预期；切回 INDEPENDENT 正常 | 待填 |
| 折叠/展开 | 到位状态来自真实机构；新指令后有新的反馈 | 待填 |
| 单洞穿越 | 入口到位→折叠到位→出口到位→展开到位→继续普通导航 | 待填 |
| 机构反馈中断 | 洞内反馈过期后停止导航；恢复后重新折叠确认再继续出口方向 | 待填 |
| 操作手指点 | 用真实小地图点击，吸附到预期的可达点；完成后恢复规则决策 | 待填 |
| 购买 | `/referee/sentry_command` 有请求、裁判次数确认、实际血/弹到账；重复发送不重复执行 | 待填 |
| 比赛结束 | 一次性取消导航并停自旋；导航侧和底盘实际停止 | 待填 |
| 控制断流 | 桥默认 cmd_vel 断流 200 ms 下发全零；固件独立掉线保护实际生效 | 待填 |

故障项先在台架或已具备停车控制的条件下验证。购买意图 `/sentry/buy_*` 仅作观测，不再接第二个购买执行器。现有短局记录未替代六洞双向通行及实车导航验收。

需要协助解算时，提供测量 CSV 的文本内容即可；需要排查联调问题时，附对应配置、节点日志及失败项的目标/状态/反馈记录。
