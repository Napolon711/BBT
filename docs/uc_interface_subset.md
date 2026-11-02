# BBT 消费 27BT-UC 的接口子集

导航执行契约：当前检出的UC只有单点 `/goal_pose`，不足以保证任意坐标的安全过洞和回执归属。BBT普通导航先调用标准 `ComputePathToPose`，逐段审核返回路径没有进入未经授权的洞体，再用 `FollowPath`执行这条固定路径；过洞使用 `NavigateThroughPoses`一次提交中心与出口。三项标准Nav2 Action均须补接，不能只接旧UC单点执行器。共享裁判消息、话题字段和电控线格式继续兼容UC；导航执行方式有上述明确扩展。

其他组保持 27BT-UC 的全量数据输出。BBT 直接连接其中需要的话题，不要求发送方改消息类型、重排字段或裁剪载荷。两个决策程序使用相同输出口，应选一个运行；同一物理串口只开一个桥。

## 人工阵营

桥和 BBT 启动参数均为必填只读 `own_team:=red` 或 `own_team:=blue`，由操作员开局指定。裁判 RobotStatus 的 robot_id 只作诊断：缺失、为0、与人工选择相反或过期时，不会切换己方，不会单凭此禁止导航/购买。HP、比赛和兑换数据自身仍按原有有效性/时效要求处理。

正式点表以red为基准，保留UC的39个点ID/坐标/region及六洞坐标；BBT新增用户已确认的区域多边形，预设点及洞口的region须与几何一致，矛盾配置拒绝加载。blue运行时绕场心 `(14,7.5)` 旋转180度，敌我语义和ID不变。当前暂用恒等地图变换，后续实测标定工具保留。己方/对方只表示人工阵营下的裁判数据归属，不要求BBT识别场上敌我机器人。

BBT原有zone用途标签保持不变，正式巡逻通过 `patrol_point_ids: [28,40,70,101]`显式选择；109、114只用于outpost任务。不同region之间必须经洞连接图过洞，允许经mid多洞转接，不使用普通坡道或绕行跨区。洞center取显式配置，不取两端align的中点。配置点0.5米闸线筛选不用于任意坐标点击，当前39个配置点均未被此条件过滤。

## 标准输入

| UC 话题 | 消息 | BBT 取用内容 |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | map系位置，接收时刻用于位姿时效 |
| `/nav_status` | `std_msgs/Int32` | 兼容观测的0空闲/1运行/2到达/3失败；不作为Action完成依据 |
| `/gimbal_fold_state` | `std_msgs/Int32` | 0未知/1折叠中/2折叠完成/3展开中/4展开完成 |
| `/referee/summary` | `std_msgs/Float64MultiArray` | UC的10–14元素摘要，额外字段按长度取用 |
| `/referee/robot_hp` | `rm2027_interfaces/RobotHP` | 双方前哨HP、基地、控制度；人工阵营选敌我 |
| `/referee/field_event` | `rm2027_interfaces/FieldEvent` | 双方前哨控制状态；人工阵营选敌我 |
| `/referee/sentry_info` | `rm2027_interfaces/SentryInfo` | 兑换次数/池、脱战、复活资格与费用 |
| `/referee/robot_status` | `rm2027_interfaces/RobotStatus` | 诊断ID；不作为己方阵营来源 |
| `/sentry/data_source` | `std_msgs/String` | real下听到sim会锁存来源保护 |

其余 UC 雷达、自瞄跟踪、地面机器人位置、RFID、功率、RL等消息可继续发送；BBT不订阅的输入不影响运行。缺少雷达/RL不阻塞保底决策。

UC标准摘要基础布局：`[game_progress,hp,hp_max,shots,coins,own_outpost_hp,own_control,control_valid,stage_remain_s,source_stamp_ms]`；可选第10项基地HP，第11项控制度，第12项源年龄ms，第13项对方前哨HP。BBT同时兼容历史9元素/19元素布局，但现场发送方使用现行UC布局即可。

## 标准输出

| 话题 | 消息/含义 |
|---|---|
| `/goal_pose` | `geometry_msgs/PoseStamped`，map目标观测；不得另接旧单点导航执行器 |
| `/nav_cancel` | `std_msgs/Empty`，兼容取消通知；BBT另外取消自己持有的Action Goal |
| `/cmd_vel` | `geometry_msgs/Twist`，BBT只发停车，巡航速度由导航发 |
| `/move_mode` | `std_msgs/Int32`，0 RELAX / 1 INDEPENDENT / 2 ROTOR / 3 FOLLOW |
| `/cmd_spin` | `std_msgs/Float32`，rad/s |
| `/gimbal_fold_cmd` | `std_msgs/UInt8`，1折叠/2展开 |
| `/sentry/attack_cmd` | `std_msgs/Int32MultiArray(4)`，`[type,id,fire,free_fire]` |
| `/referee/sentry_command` | `rm2027_interfaces/SentryCommand`，唯一实际购买执行口 |
| `/sentry/buy_ammo`、`/sentry/buy_hp` | `std_msgs/Int32`，仅意图观测 |

桥与UC共用SharedTopic的ref_frame/ref_tx/chassis_state/chassis_data/chassis_mode/fold_cmd及其线格式，不需要下位机另增BBT私有载荷。19个rm2027消息定义按字段、顺序与常量比较和UC相同。

## 必需的导航Action

| 默认名称 | 标准类型 | 契约与节点参数 |
|---|---|---|
| `/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | 规划普通目标；`nav_compute_path_action`可改名 |
| `/follow_path` | `nav2_msgs/action/FollowPath` | 只跟踪审核通过的固定路径；`nav_follow_path_action`可改名 |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | 只用于已批准的当前洞穿越；`nav_through_poses_action`可改名 |

普通路径从当前位置起每一段都检查位于目标region的多边形并集内、且不与未授权洞体相交，不能只检查离散路径点或终点region，也不能穿出本region后再返回。`FollowPath`不得在执行时绕过审核另行重规划；失败后由BBT冷却、重新调用规划并重新审核。Action服务缺失、拒绝、路径为空或检查不通过时停车，不回退到 `/goal_pose`执行。入口及最终原坐标也走这条普通导航链。

出口在洞体边界且导航有到达容差，成功回执时实际位置可能仍稍在洞内。仅凭当前指定洞的成功穿越事实，允许后续路径的开头沿该出口向外清出；反向、离开后再入、穿过其他洞仍拒绝，region整段约束保持。此授权保存的是物理完成证据，不是新阶段状态机。

过洞先确认折叠，再提交 `[center,exit]`；中心不停留、不产生单独成功。已确认过中心后的恢复只提交 `[exit]`，无中心的简单走廊也只提交出口。取消和回执按Action Goal身份及任务代际隔离，旧规划、旧跟踪或旧穿越结果不得完成新目标。`/nav_status`迟到成功不会覆盖这些Action结果。必须关闭旧 `/goal_pose`执行器，避免它与BBT持有的Action任务并行输出速度；导航端响应取消后须停止旧任务速度。

BBT明确停车时取消其导航、发零 `/cmd_vel`并停自旋；战术到点保持仍允许自旋和攻击，不按故障停车处理。入口折叠等待、出口展开等待、失败冷却、空目标及比赛结束均属于明确停车。

## 可选输入与诊断扩展

保留BBT已有的 `/referee/operator_cmd` Float64 `[type,param,seq,x,y]`坐标指点、`/sentry/operator_point` Int32点ID测试输入。它们是可选功能，不需要其他组为了基础保底决策另外发布。BBT桥保留对0x0303小地图点击的解析，决策只处理导航type=4。任意坐标不要求存在于点表，不吸附到1.5米内的预设点；有效归属由区域几何确定，必要时穿洞后继续去原坐标。洞体内部或归属不明确的终点静默拒绝。

人工输入按本地接收顺序排队消费，无效坐标或购买类型不能吞掉前一个有效导航坐标。关闭过洞并合法省略region/几何时，有限数值原始XY直接交给导航判断可达性；配置了几何时仍严格要求唯一归属。

人工任务没有固定120秒过期；最终导航成功后至少保持20秒，之后回家锁存优先，否则满足连续脱战10秒才解除。两个计时从最终到达后并行进行，脱战取裁判SentryInfo字段；同坐标重复指令不重发也不复位计时。操作手购买直接进入裁判系统，不经过BBT；BBT仍独立执行自主购买与复活，指点不关闭这些规则。免费复活无金币要求，付费复活只检查裁判资格和实际报价，不叠加200金币门槛。

到点保持使用持久的最后已知 `out_of_combat`，正常慢更新或重复消息不重置连续脱战计时，后续未脱战值清零。该移动决策约定不改变购买/复活的默认2秒裁判新鲜度要求。

`/sentry/buy_revive` 仍仅为意图观测；实际复活也使用标准SentryCommand执行。`/sentry/iface_status` 为JSON统计，`/sentry/decision_trace` 为JSON逐帧记录，包含人工阵营、目标、区域与物理执行证据；不参与控制输入。行为流程由决策树表达，诊断字段不构成独立阶段状态机。

核对另一个UC检出版本：

```bash
python3 tools/check_uc_interfaces.py --reference /路径/27BT-UC
```

此检查验证消息定义、默认共享话题及几何；实际ROS和PTY测试另行验证收发行为，不能用字段比较代替实车联调。
