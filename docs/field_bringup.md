# 坐标约定与实车联调

以下命令从工作区根目录执行。当前暂用裁判场地坐标和恒等变换，未来实车地图仍可标定，工具完整保留。开局与27BT-UC一样人工指定own_team=red/blue，不从裁判ID推断。已有仿真记录只证明记录中的场景，实车结果另行验收。

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

仿真坐标已统一，但地图中未标注尺寸和部分高度属于重建值，洞口对正点也由配置声明。点表现在与UC统一为红方基准；blue启动时将全部战术点和洞绕场心旋转180度，ID和region语义保持不变。与Arena内部Y翻转是两个不同步骤，不能重复换算。

洞center是洞表中的显式坐标，不是两端align的中点。测量与标定时应使用该物理位置，不能重新按中点推算。

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
| 家点 ID 100 | red为 `(2.61,1.52)`，blue为 `(25.39,13.48)`；与UC点表一致 |
| retake 点 ID 30、86 | red基准为 `(13.69,3.67)`、`(11.89,9.50)`，blue按场心旋转；确认任务位置可达 |
| outpost 点 ID 109、114 | red基准为 `(15.69,10.90)`、`(13.65,11.15)`，blue按场心旋转；待验收时可在联调配置中关闭前哨任务 |
| 巡逻名单 | 正式 `patrol_point_ids: [28,40,70,101]`；109、114仅outpost，其他点与原zone标签保留 |
| 区域几何 | 已确认的region多边形与实车地图一致；跨region必须按洞连接图通行，可经mid过多个洞 |
| 洞两端 `align` | 对正点对应声明的区域；出口可能处于洞体边界，核对导航到达容差与实际机构位置 |
| `/odom` | position 已处于导航 map 系；仅修改 `header.frame_id` 不会变换坐标 |
| 操作手点击 | 采用固定场地系原始XY，变换一次后导航；不吸附点表，配置几何时须有唯一region归属 |

当前适配层直接使用 `/odom` 的位置数值，不查询 TF。若导航原始里程计在 odom 系，应由定位/导航侧提供 map 系位姿并接到配置的 `topics.real.odom`；同时检查图上的实际位置与变换后的家点、洞口是否吻合。

点位或变换有修改后，重启决策节点重新加载。配置几何在加载时应用一次 `field_to_map`；操作手原始坐标在接收后应用一次，不能再随阵营旋转。

正式配置的 `safety.gate_clear_m: 0.5`只筛选配置点，当前39个点均保留；不套用到人工任意原坐标。洞体内部、非法数值或无法唯一归属的点击静默拒绝且保留原任务。简单地图关闭过洞并合法省略region/几何时，有限XY交导航判断可达性。

## 3. 构建和接口检查

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rm2027_interfaces ros2_libxr bbt_decide
source install/setup.bash
colcon test --packages-select bbt_decide ros2_libxr
colcon test-result --verbose
```

导航进程由实际导航工作区启动，必须提供标准Nav2 `ComputePathToPose`、`FollowPath`与 `NavigateThroughPoses` Action。BBT普通导航先获取路径，从当前位置起逐段确认路径完整位于目标region多边形并集内、且不穿未经授权的洞体，再用FollowPath执行固定路径；不能穿其他region再返回，也不能隐式重规划绕过审核。失败后由BBT冷却、重新规划并重新审核。

`/goal_pose`与 `/nav_status`仅作兼容观测，不能作为单点执行入口或任务成功依据。必须停止消费旧 `/goal_pose`并自行导航的执行器，避免与BBT持有的Action并行发速度。结果与取消按请求代际、Goal身份隔离；导航端取消后须停止旧任务输出。默认Action名称可通过 `nav_compute_path_action`、`nav_follow_path_action`、`nav_through_poses_action`配置，详见[导航契约](nav2_waypoints.md)。

如果其他组已启动UC桥，直接接入即可，不再启动第二个桥。否则确认串口设备，再启动唯一的桥；以下示例选red，正式阵营由操作员指定：

```bash
ls -l /dev/serial/by-id/
ros2 launch ros2_libxr bridge.launch.py port:=/dev/serial/by-id/你的设备 own_team:=red
```

其他终端也先 source ROS 和本工作区，再分别检查：

```bash
ros2 topic info /referee/operator_cmd --verbose
ros2 topic echo /referee/summary --once
ros2 topic echo /referee/robot_status --once
ros2 topic echo /odom --once
ros2 topic hz /gimbal_fold_state
ros2 action list -t
```

| 输入/输出 | 应观察到的契约 |
|---|---|
| `/referee/operator_cmd` | `std_msgs/msg/Float64MultiArray`，`[type,param,seq,x,y]`；现场点击后有数据 |
| `/sentry/operator_point` | `std_msgs/msg/Int32MultiArray`，仿真 ID 指点；默认正式配置关闭此订阅 |
| `/referee/summary` | 实车 14 元素摘要；比赛进行中第 0 项为 4 |
| `/referee/robot_status` | ID仅诊断；阵营和购买发送ID由own_team固定，不核实裁判ID |
| `/gimbal_fold_state` | 0 未知 / 1 折叠中 / 2 已折叠 / 3 展开中 / 4 已展开；持续真实刷新，建议至少 10 Hz |
| `/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose`，返回map系完整路径；不执行速度控制 |
| `/follow_path` | `nav2_msgs/action/FollowPath`，只执行BBT审核通过的固定路径，不自行重规划 |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses`，折叠确认后一次提交中心与出口，中心不停留 |
| `/goal_pose`、`/nav_status` | 仅观测；后者0 idle / 1 running / 2 success / 3 failure，不用于Action完成判定 |
| `/nav_cancel` | `std_msgs/msg/Empty`兼容取消通知；BBT另取消其持有的Action，导航侧停止对应旧任务速度 |

## 4. 启动决策并观察

当前正式配置已确认裁判系恒等变换。导航及位置输入采用同一坐标后，启动：

```bash
ros2 run bbt_decide bbt_decide_node --ros-args \
  -p config_file:="$PWD/src/bbt_decide/config/decision.yaml" \
  -p data_source:=real -p own_team:=red
```

比赛进行中会按当前配置自主选点，非过洞段默认 `spin_w: 1.5`，自动购买默认开启。首次只验导航或机构时，可另存一份联调配置，按试验目的设置 `auto_buy.enable: false`、`outpost.enable: false` 或 `safety.spin_w: 0`，并把 `config_file` 指向该文件。

另外观察：

```bash
ros2 topic echo /sentry/iface_status
```

检查输入 `count` 是否持续增加、`last_age_s` 是否合理、`dropped` 是否增加。比赛状态没有进入 4 时不发导航；real 模式收到 `/sentry/data_source` 的 `sim` 声明后会锁存保护，需要先停止仿真发布源再重启决策节点。默认约定会保留最后一次比赛状态，裁判断流不会自动变成比赛结束。

人工输入按本地接收顺序排队消费，无效点击或购买类型不能吞掉此前收到的有效导航坐标。有效任务内重复相同坐标不取消重发、不重置计时；不同坐标替换目标，已开始的洞穿越先安全完成。人工购买直达裁判，BBT仍独立自主购买，不重复执行人工订单。

只有最终原坐标对应的导航成功才启动20秒最短保持；入口/出口成功、进入0.5米半径或旧任务回执均不算。20秒保护与到达后连续脱战10秒并行计算：满20秒且回家锁存成立则结束指点回家，否则连续脱战满10秒才恢复自主，其他情况继续在点位自旋和作战。脱战使用持久的最后已知裁判 `out_of_combat`，慢更新不重置；新的未脱战值清零。购买和复活仍独立要求默认2秒新鲜裁判数据。无固定120秒人工任务过期，普通导航单次25秒超时、2秒冷却和自动重试仍保留。

过洞一次提交显式 `[center,exit]`，中心只有连续运动的途经语义，无独立到位回执；出口成功后停车展开。失败按真实进度恢复，已过中心只重发出口。正式出口在洞体边界，导航成功时实际位置可能仍稍在洞内；仅匹配的成功穿越事实允许后续路径开头沿该出口向外清出，不可反向、离开后再入或穿其他洞，region约束保持。现场须核对这个余量与真实机构空间，不以仿真代替间隙验收。

## 5. 逐项记录实车结果

先验普通导航，再验单洞，最后扩大到需要使用的各洞及反向通行。

| 项目 | 通过依据 | 现场结果 |
|---|---|---|
| 普通导航 | map系原目标正确；ComputePath完整路径逐段通过region/洞体审核后才交FollowPath；旧结果不完成新目标 | 待填 |
| 路径约束 | 拒绝跨其他region再返回、未经授权穿洞及审核后隐式重规划；合法同region路径正常执行 | 待填 |
| 取消及速度轴 | 取消后停止当前导航输出；x/y/w 单位和正负方向正确 | 待填 |
| 自旋 | ROTOR 正负和零角速度符合预期；切回 INDEPENDENT 正常 | 待填 |
| 折叠/展开 | 到位状态来自真实机构；新指令后有新的反馈 | 待填 |
| 单洞穿越 | 入口到位→折叠到位→一次提交中心/出口、中心持续运动且无独立成功→出口成功停车→展开到位→继续普通导航 | 待填 |
| 出口容差 | 指定洞成功后仅允许开头向外清出；反向、再入、其他洞路径仍拒绝，实际机构间隙足够 | 待填 |
| 机构反馈中断 | 洞内反馈过期后停止导航；恢复后重新折叠确认再继续出口方向 | 待填 |
| 操作手指点 | 真实点击不在点表的XY，必要时过洞后准确导航至原坐标；无效输入不覆盖有效目标，同点重复不重发 | 待填 |
| 到点作战 | 最终成功后20秒内不离点；20秒后回家需求优先，否则连续脱战10秒才恢复自主，期间允许自旋攻击 | 待填 |
| 购买 | `/referee/sentry_command` 有请求、裁判次数确认、实际血/弹到账；重复发送不重复执行 | 待填 |
| 比赛结束 | 一次性取消导航、零 `/cmd_vel`、停自旋、清攻击与人工任务；导航侧和底盘实际停止 | 待填 |
| 控制断流 | 桥默认 cmd_vel 断流 200 ms 下发全零；固件独立掉线保护实际生效 | 待填 |

故障项先在台架或已具备停车控制的条件下验证。购买意图 `/sentry/buy_*` 仅作观测，不再接第二个购买执行器。现有短局记录未替代六洞双向通行及实车导航验收。

需要协助解算时，提供测量 CSV 的文本内容即可；需要排查联调问题时，附对应配置、节点日志及失败项的目标/状态/反馈记录。
