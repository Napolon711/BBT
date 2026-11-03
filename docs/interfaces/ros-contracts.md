# ROS 与导航交接清单

审计基线 `10e0910`。普通导航同时列出历史行为和用户确认的修复边界；其余表按基线源码实现记录。来源：[RosPorts](../../src/bbt_decide/src/ros_ports.cpp)、[节点](../../src/bbt_decide/src/main.cpp)、[默认配置](../../src/bbt_decide/config/decision.yaml)、[生产串口桥](../../src/ros2_libxr/src/bridge_node.cpp)。完整自定义字段见 [消息附录](message-definitions.md)。下列绝对话题名以默认配置、无额外 namespace/remap 为准；桥代码中 `referee/*`、`chassis/state`、`serial/gimbal_joint_state` 使用相对名，有 namespace 时应随之解析。

## 导航与定位

| 方向（相对 BBT） | 名称 / 精确 ROS 类型 | 字段、单位及确认语义 |
|---|---|---|
| 输出 | `/goal_pose` — `geometry_msgs/msg/PoseStamped` | `header.stamp`、`header.frame_id="map"`、`pose.position.x/y`（m）、`orientation.w=1`，其余 pose 分量默认零。确认边界：普通任务只发最终目标；洞入口仍是洞流程目标。基线此 topic 仅观察，实际普通执行走两个 Action。 |
| 输入 | `/nav_status` — `std_msgs/msg/Int32` | `data`：0 idle、1 running、2 success、3 failure。确认边界：普通导航状态回执；基线收到仅统计，所有值均不确认完成。此接口没有 goal ID/UUID，执行端须保证替换/取消后不把旧完成事件当作新目标结果。 |
| 输出 | `/nav_cancel` — `std_msgs/msg/Empty` | 无数据字段；确认边界：取消普通导航。基线同时按 Action UUID 取消与发此消息。取消 topic 没有请求身份，外部消费者语义须一致。 |
| 输入 | `/odom` — `nav_msgs/msg/Odometry` | 只消费 `pose.pose.position.x/y` 与 `pose.pose.orientation.x/y/z/w` 解算 yaw；忽略 `header.frame_id/stamp`、`child_frame_id`、covariance 与 twist，不做 TF。调用方须提供与目标一致的 map 系米制位姿。以本机收件时间计新鲜度，默认 `odom_stale_ms=1000` 后决策视为无效。生产串口桥不发布它。 |
| 输出 Action | `/navigate_through_poses` — `nav2_msgs/action/NavigateThroughPoses` | `Goal.poses: PoseStamped[]`，`behavior_tree` 默认空；洞任务为 `[中心,出口]`（map）。读取绑定请求 UUID 的结果，要求 Action `SUCCEEDED` 且 `error_code=0`；不消费 feedback。**类型标准，实机 server 未确认**。 |
| 历史输出 Action | `/compute_path_to_pose` — `nav2_msgs/action/ComputePathToPose` | 基线普通规划发送 map 目标、`use_start=false`，取 `Result.path/error_code`。BBT 校验终点和每段区域/洞体后才执行。用户已确认撤掉此 BBT 依赖。 |
| 历史输出 Action | `/follow_path` — `nav2_msgs/action/FollowPath` | 基线执行通过门禁的 `nav_msgs/Path`，三个 controller/checker ID 默认空；成功读取绑定请求 UUID 的结果且 `error_code=0`。用户已确认撤掉此 BBT 依赖。 |

BBT 自身没有普通路径规划器；`/goal_pose` 不包含路径。生产 `ros2_libxr_bridge` 也不实现上述 Action servers。基线仓库中的三个服务器都在 `tools/arena/bridge.py`，是仿真替身，模拟路径来自 `NavController.plan()` 和 Arena SDK 图搜索。图上的折线顶点不能据此称为实车导航提供的数据。

## BBT 的其他输入

普通 pub/sub 均使用 ROS 默认可靠、volatile profile、history depth 10；BBT 默认 tick 20 Hz。2027 专用输入仅在启用 2027 协议时创建。

| 名称 / 精确 ROS 类型 | 消费字段及行为 |
|---|---|
| `/gimbal_fold_state` — `std_msgs/msg/Int32` | `data` 0未知/1折叠中/2已折叠/3展开中/4已展开。新折叠命令先清缓存，须之后收到独立反馈；默认 `fold_stale_ms=1000` 后返回未知。 |
| `/referee/summary` — `std_msgs/msg/Float64MultiArray` | 读取 `data`，忽略 `layout`；允许长度 9…14 或 19，finite 且各字段在代码数值范围内。生产桥固定 14 项，布局见下表。 |
| `/referee/robot_status` — `rm2027_interfaces/msg/RobotStatus` | BBT 仅取 `robot_id` 作遥测，默认 2 s freshness；不用于纠正或否决人工队伍。 |
| `/referee/robot_hp` — `rm2027_interfaces/msg/RobotHP` | 按人工队伍映射 `outpost_r_hp/outpost_b_hp`；还消费 `ally_base_hp/enemy_base_hp`、`ally_control_degree/enemy_control_degree`；默认 2 s freshness。 |
| `/referee/field_event` — `rm2027_interfaces/msg/FieldEvent` | 消费 `outpost_r_control/outpost_b_control`，按队伍映成己方/对方；默认 2 s freshness。 |
| `/referee/sentry_info` — `rm2027_interfaces/msg/SentryInfo` | 消费复活资格/价格、剩余可兑弹量、累计兑换量与兑换次数，做购买门控和回执；消费 `out_of_combat`。购买要求新鲜，战斗状态读取逻辑不因 freshness 超时直接重置为战斗中。 |
| `/referee/operator_cmd` — `std_msgs/msg/Float64MultiArray` | `data=[type,param,seq,x,y]`，至少5项，全部finite；type整数1…5、param整数≥-1、seq为1…UINT32_MAX整数；按seq去重。当前只执行type=4；基线把原始x/y直接作为场地m处理。裁判0303到此数组的适配是本桥实现；用户补充点击为红方原点坐标，蓝方map是否需旋转仍待确认，不能把基线原样透传当作最终认可坐标合同。 |
| `/sentry/operator_point` — `std_msgs/msg/Int32MultiArray` | `data=[type,param,seq]`，仿真ID指点；至少3项，type 1…5、seq非零，两路输入独立去重后按本机收件顺序合并。real配置为空，不创建该订阅。 |
| `/sentry/data_source` — `std_msgs/msg/String` | real模式输入保护；收到 `data="sim"` 后本进程永久将 `game_progress` 压为0而待机。发布此字符串不等于鉴别出真实硬件。 |

## BBT 的其他输出

| 名称 / 精确 ROS 类型 | 字段/单位 | 已有执行端与限制 |
|---|---|---|
| `/cmd_vel` — `geometry_msgs/msg/Twist` | `linear/ angular: Vector3`；BBT只发全零停车 | 导航执行端另提供正常速度；生产桥消费 x/y/z角速度，见电控文档。多个发布者无自动仲裁。 |
| `/move_mode` — `std_msgs/msg/Int32` | `data` 0 RELAX/1 INDEPENDENT/2 ROTOR/3 FOLLOW | 桥转单字节，无执行ack；比赛开始切独立，spin启停可切ROTOR/独立。 |
| `/cmd_spin` — `std_msgs/msg/Float32` | `data` 角速度 rad/s，默认 `spin_w=1.5` | 桥仅ROTOR模式使用，200 ms freshness；被查电控快照忽略此角速度。 |
| `/gimbal_fold_cmd` — `std_msgs/msg/UInt8` | `data` 1折叠/2展开 | 桥原字节转 `fold_cmd`；发出不证明机构到位。 |
| `/sentry/attack_cmd` — `std_msgs/msg/Int32MultiArray` | `data=[target_type,target_id,fire,free_fire]`；前哨 `[2,1,1,0]`、普通 `[0,0,1,1]`、停止全零 | 生产串口桥无此订阅。需外部自瞄/发射消费者，数组枚举仅是本仓库约定，没有现场兼容证据。 |
| `/sentry/buy_ammo` — `std_msgs/msg/Int32` | `data=rounds`，2027任务请求100发 | 2027下只有下单门控通过才发布；桥不消费，属意图观测。 |
| `/sentry/buy_hp` — `std_msgs/msg/Int32` | `data=amount`，2027请求1次 | 同上。 |
| `/sentry/buy_revive` — `std_msgs/msg/Int32` | `data=1` | 同上。若另有旧消费者执行buy_*，应排除与新购买路径重复下单。 |
| `/referee/sentry_command` — `rm2027_interfaces/msg/SentryCommand` | 绝对累计兑换量、买弹/回血次数、复活确认、姿态与能量确认，完整字段见附录 | 2027实际购买输入；桥编码0x0301/0x0120经 `ref_tx` 下发。BBT约200 ms检查回执，最多12 s重发、6 s cooldown；发ROS消息不代表裁判成功。 |
| `/sentry/iface_status` — `std_msgs/msg/String` | `data` 1 Hz JSON，链路count/rate/age/dropped | 诊断，只能证明本进程收发调用。 |
| `/sentry/decision_trace` — `std_msgs/msg/String` | `data` 每tick JSON | 诊断，不是控制或传感接口；Arena使用它展示决策。 |

## 生产桥发布的裁判数据与机构数据

以下类型缩写均为 `rm2027_interfaces/msg/…`。命令长度是decoder最小数据段长度，非等长检查；桥按收到的有效帧发布typed数据，不自行保证上游频率。

| 裁判cmd / 最小数据字节 | ROS名称 | 类型 / depth | BBT消费 |
|---|---|---|---|
| 原始帧 | `referee/raw` | `RefereeFrame` /20 | 否；publish_raw=true才发 |
| 0x0001 /11 | `referee/game_status` | `GameStatus` /1 | 通过summary间接读取progress/time |
| 0x0002 /1 | `referee/game_result` | `GameResult` /1 | 否 |
| 0x0003 /24 | `referee/robot_hp` | `RobotHP` /1 | 部分字段 |
| 0x0101 /8 | `referee/field_event` | `FieldEvent` /1 | R/B前哨控制 |
| 0x0104 /3 | `referee/warning` | `RefereeWarning` /5 | 否 |
| 0x0201 /17 | `referee/robot_status` | `RobotStatus` /1 | robot_id遥测；summary间接HP |
| 0x0202 /14 | `referee/power_heat` | `PowerHeat` /1 | 否；仅投影payload偏移8/10/12的三个u16 |
| 0x0203 /12 | `referee/robot_pos` | `RobotPosition` /1 | 否；裁判x/y米、angle度，不能代替odom |
| 0x0204 /7 | `referee/buff` | `RobotBuff` /1 | 否 |
| 0x0206 /7 | `referee/damage` | `RobotDamage` /10 | 否 |
| 0x0208 /8 | `referee/projectile_allowance` | `ProjectileAllowance` /1 | 通过summary间接弹量/金币 |
| 0x0209 /6 | `referee/rfid` | `RfidStatus` /1 | 否 |
| 0x020B /40 | `referee/ground_robot_position` | `GroundRobotPosition` /1 | 否；只读重型工程、标准3/4坐标，valid实际直接true |
| 0x020D /14 | `referee/sentry_info` | `SentryInfo` /1 | 部分字段 |
| 0x0303 /12 | `/referee/operator_cmd` | `std_msgs/msg/Float64MultiArray` /10 | 实车指点；前两个float为x/y，p[9]==0映type4/param=-1，否则type5/param=p[9]；相同12B payload去重，本地生成seq |
| timer | `/referee/summary` | `std_msgs/msg/Float64MultiArray` /1 | 是，默认20 Hz |
| MCU yaw | `/yawmotor_angle` | `std_msgs/msg/Float32` /50 | 否；data=角度rad |
| MCU chassis | `/gimbal_fold_state` | `std_msgs/msg/Int32` /1 | 是；data=fold_state |
| MCU AHRS+yaw | `serial/gimbal_joint_state` | `sensor_msgs/msg/JointState` /20 | 否；name=`[gimbal_pitch_joint,gimbal_yaw_joint]`、position=`[pitch,yaw]` rad，velocity/effort为空，需AHRS和yaw都收到 |
| MCU chassis | `chassis/state` | `ChassisState` /5 | 不直接消费，只用fold投影 |

`referee/raw` 在publish_raw=true时也发布内部裁判帧校验失败的数据，此时cmd_id=0；只有校验通过才继续typed解码。它不是“所有raw均已通过裁判CRC”的保证。`GroundRobotPosition.valid` 注释称保留字段异常时false，实际decoder固定true；不能将该标志当作实现了坐标有效性检查。`DecisionInput` 和 `RefereeTx` 虽有消息定义，当前生产桥/BBT没有对应ROS数据收发端；`RefereeTx` 的同名串口wire struct也不是ROS消息传输。

## summary逐索引契约

生产桥固定14项 `Float64MultiArray.data`，一般未知填-1，索引7初值0。长度不是版本号；BBT兼容布局的语义不得混用。

| index | 生产14项（亦是9…14项兼容布局的对应索引） | 历史19项布局 |
|---:|---|---|
| 0 | game_progress：0未开始/1准备/2自检/3倒计时/4比赛/5结束 | 同左 |
| 1 | 当前HP | 同左 |
| 2 | 最大HP | 同左 |
| 3 | 17mm允许发弹量 | 同左 |
| 4 | 剩余金币 | 同左 |
| 5 | 己方前哨HP | 同左 |
| 6 | 己方前哨控制原始码：0全控/1临时/2未控 | 己方前哨控制 |
| 7 | 控制码有效0/1 | 对方前哨HP |
| 8 | 比赛剩余秒 | 对方前哨控制 |
| 9 | bridge steady ms派生的最老必需源收件时间估计 | 比赛剩余秒 |
| 10 | 己方基地HP | 己方基地HP |
| 11 | 己方累计控制度（协议未注明单位，不按百分比解释） | 对方基地HP |
| 12 | 最老必需源age，ms | 己方累计控制度 |
| 13 | 对方前哨HP | 对方累计控制度 |
| 14 | — | robot_id |
| 15 | — | 复活资格 |
| 16 | — | 复活价格 |
| 17 | — | 队伍可兑弹量 |
| 18 | — | 脱战状态 |

桥用0x0001/0x0201/0x0208三个必需源的最大age，默认超过2000 ms就停止summary；其他typed话题仍是有效帧到达即发。0x0003/0x0101过期时其派生summary字段保持未知。summary是latest-state，不是事件队列。BBT 2027模式下会用新鲜typed HP/FieldEvent/SentryInfo覆盖相应summary字段。

## 生产桥的ROS输入与参数

桥只订阅 `/cmd_vel` Twist、`/move_mode` Int32、`/cmd_spin` Float32、`/gimbal_fold_cmd` UInt8、相对名 `referee/sentry_command` SentryCommand；depth均10。**没有导航、attack_cmd、buy_*消费者**。实际wire映射和未接通项见 [电控清单](electrical-wire.md)。

默认参数：`port=/dev/ttyACM0`、`baud=115200`、`own_team`必须red/blue、`publish_raw=true`、`zero_vel_timeout_ms=200`、`referee_timeout_ms=2000`、`summary_rate_hz=20`、`sentry_cmd_rate_hz=5`、`sentry_cmd_hold_s=3`。BBT与桥各自设置队伍，不相互核实；桥sender ID为红7/蓝107。

## 标准消息的字段展开

这里列出上表使用的标准结构，避免数组或嵌套类型只留一个名字。只消费哪些字段仍以上表为准。

| ROS类型 | 字段结构 |
|---|---|
| `std_msgs/msg/Int32`、`UInt8`、`Float32`、`String` | 分别为 `int32 data`、`uint8 data`、`float32 data`、`string data` |
| `std_msgs/msg/Empty` | 无字段 |
| `std_msgs/msg/Header` | `builtin_interfaces/Time stamp`（`int32 sec`、`uint32 nanosec`），`string frame_id` |
| `geometry_msgs/msg/Point`、`Vector3` | 均为 `float64 x,y,z` |
| `geometry_msgs/msg/Quaternion` | `float64 x,y,z,w` |
| `geometry_msgs/msg/Pose` | `Point position`、`Quaternion orientation` |
| `geometry_msgs/msg/PoseStamped` | `Header header`、`Pose pose` |
| `geometry_msgs/msg/Twist` | `Vector3 linear`、`Vector3 angular` |
| `nav_msgs/msg/Odometry` | `Header header`、`string child_frame_id`、`geometry_msgs/PoseWithCovariance pose`（`Pose pose`、`float64[36] covariance`）、`geometry_msgs/TwistWithCovariance twist`（`Twist twist`、`float64[36] covariance`） |
| `nav_msgs/msg/Path` | `Header header`、`geometry_msgs/PoseStamped[] poses`；仅修复前普通规划Action使用 |
| `std_msgs/msg/Float64MultiArray`、`Int32MultiArray` | `MultiArrayLayout layout`，分别为 `float64[] data`、`int32[] data`；layout含 `MultiArrayDimension[] dim`、`uint32 data_offset`，每个dim含 `string label`、`uint32 size`、`uint32 stride`。本仓库上述数组接口不读取layout。 |
| `sensor_msgs/msg/JointState` | `Header header`、`string[] name`、`float64[] position`、`float64[] velocity`、`float64[] effort` |

诊断String的基线JSON字段：`iface_status`含 `data_source/sim_source_seen/guard_triggered/links[]`，每条link为 `name/topic/type/dir/count/rate_hz/age_s/dropped`；`decision_trace`含 `seq/time_s/source/own_team/current_region/tunnel/applied_goal/target_point/match_running/operator_target/target_generation/crossing_started/center_reached/exit_reached`，目标有效时追加 `target_xy`，洞流程有效时追加 `entry/center/exit`。它们用于观察，不提供导航或机构执行确认。
