# BBT 消息原文附录（审计基线 10e0910）

下方为审计基线全部 19 个 `.msg` 的逐字原文，随后是本机 ROS Jazzy 的标准 Action 定义。类型名、字段顺序、数组和常量全部保留。`std_msgs/Header` 包含 `builtin_interfaces/Time stamp`（`int32 sec`、`uint32 nanosec`）与 `string frame_id`；`geometry_msgs/Point` 包含 `float64 x/y/z`。注释只是仓库声明，执行差异见 [ROS清单](ros-contracts.md) 和 [电控清单](electrical-wire.md)，特别是 `GroundRobotPosition.valid`、`RefereeTx` 的组帧责任。`DecisionInput`、`RefereeTx` 消息定义存在，不代表生产ROS节点已收发它们。


## rm2027_interfaces/msg/ChassisState

来源：[src/rm2027_interfaces/msg/ChassisState.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/ChassisState.msg:1)

```text
# 下位机上报的底盘/机构状态（不是裁判系统协议，而是 STM32 通过 SharedTopic 发上来的）
#
# 用途：给决策层提供"车实际在动什么"的反馈，用于动作确认与超时保护；
#       折叠状态是过隧道流程的必要反馈（裁判系统不提供任何折叠信息）。
std_msgs/Header header

float32 vx              # 底盘实际前进速度，车体系，单位 m/s
float32 vy              # 底盘实际左移速度，车体系，单位 m/s
float32 wz              # 底盘实际旋转角速度，单位 rad/s（逆时针为正）
uint8 fold_state        # 0 未知 / 1 折叠中 / 2 已折叠 / 3 展开中 / 4 已展开
uint8 posture           # 哨兵姿态：0 未知 / 1 进攻 / 2 防御 / 3 移动 / 4 强化进攻 / 5 强化防御 / 6 强化移动
uint16 buffer_energy_j  # 超级电容缓冲能量，单位 J
uint16 power_w          # 当前底盘功率，单位 W
uint32 tick_ms          # 下位机本地毫秒计数（开机起算）
```


## rm2027_interfaces/msg/DecisionInput

来源：[src/rm2027_interfaces/msg/DecisionInput.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/DecisionInput.msg:1)

```text
# Atomic policy input: observation and mask always belong to the same sequence.
std_msgs/Header header
uint32 sequence
string schema_hash
float64[] observation
int32[] mask
```


## rm2027_interfaces/msg/FieldEvent

来源：[src/rm2027_interfaces/msg/FieldEvent.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/FieldEvent.msg:1)

```text
# 0x0101 场地事件数据（服务器 → 全体机器人，1 Hz，内容数据段 8 字节）
# 协议表 1-12：event_data(uint32) + event_data_2(uint32)。
# 两个原始字照原样保留；下面的解码字段只是把位段拆开，方便业务层直接用。
# 所有 *_state 字段的取值含义见各自注释，未知/不适用时按原始字判断。
std_msgs/Header header

uint32 event_data            # 原始 event_data
uint32 event_data_2          # 原始 event_data_2

# ---------- event_data ----------
# bit0 己方补给区激活状态：0 未激活，1 己方激活
uint8 supply_zone_active
# bit1-2 己方小能量机关激活状态：0 未激活，1 已激活，2 正在激活
uint8 small_energy_state
# bit3-4 己方大能量机关激活状态：0 未激活，1 已激活，2 正在激活
uint8 big_energy_state
# bit5-7 R 堡垒增益点：0 未激活，1 己方激活，2 对方激活，3 双方同时激活，
#       4 = 己方未控制该阵地（此时该字段无意义）
uint8 fortress_r_state
# bit8-10 B 堡垒增益点，含义同 fortress_r_state
uint8 fortress_b_state
# bit11-13 R 基地增益点：0 未激活，1 己方激活，2 对方激活，3 双方同时激活，
#        4 = 己方未控制该阵地
uint8 base_gain_r_state
# bit14-16 B 基地增益点，含义同 base_gain_r_state
uint8 base_gain_b_state
# bit17 己方制高点增益点：0 未激活，1 己方激活
bool own_highland_gain
# bit18-19 R 中央高地增益点：0 未激活，1 己方激活，2 对方激活，
#        3 = 己方未控制该阵地（此时该字段无意义）
uint8 central_highland_r_state
# bit20-21 B 中央高地增益点，含义同 central_highland_r_state
uint8 central_highland_b_state
# bit22-24 R 前哨增益点：0 未激活，1 己方激活，2 对方激活，3 双方同时激活，
#        4 = 己方未控制该阵地
uint8 outpost_gain_r_state
# bit25-27 B 前哨增益点，含义同 outpost_gain_r_state
uint8 outpost_gain_b_state
# bit28-29 R 前哨阵地控制状态：0 己方完全控制，1 己方临时控制，2 己方未控制
# ★ 这是"阵地控制"而不是 0~100 的控制度，只有三档
uint8 outpost_r_control
# bit30-31 B 前哨阵地控制状态：0 己方完全控制，1 己方临时控制，2 己方未控制
uint8 outpost_b_control

# ---------- event_data_2 ----------
# bit0-1 R 基地阵地控制状态：0 己方完全控制，1 己方临时控制，2 己方未控制
uint8 base_r_control
# bit2-3 B 基地阵地控制状态：0 己方完全控制，1 己方临时控制，2 己方未控制
uint8 base_b_control
# bit4-12 对方飞镖最后一次击中己方前哨站或基地的时间，单位 s（0~420，0 表示尚未击中）
uint16 enemy_dart_last_hit_time_s
# bit13-15 对方飞镖最后击中目标：0 开局默认值，1 前哨站，2 基地固定目标，
#          3 基地随机固定目标，4 基地随机移动目标，5 基地末端移动目标，6 基地末端移动目标
uint8 enemy_dart_last_hit_target
# bit16-17 R 高地控制区激活状态（仅 RMUL）：0 未激活，1 己方激活，2 对方激活
uint8 rmul_r_highland_activate
# bit18-19 R 高地控制区控制状态（仅 RMUL）：0 己方完全控制，1 己方临时控制，2 己方未控制
uint8 rmul_r_highland_control
# bit20-21 B 高地控制区激活状态（仅 RMUL）
uint8 rmul_b_highland_activate
# bit22-23 B 高地控制区控制状态（仅 RMUL）
uint8 rmul_b_highland_control
# bit24-25 中央控制区激活状态（仅 RMUL）
uint8 rmul_center_activate
# bit26-27 中央控制区控制状态（仅 RMUL）
uint8 rmul_center_control
```


## rm2027_interfaces/msg/GameResult

来源：[src/rm2027_interfaces/msg/GameResult.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/GameResult.msg:1)

```text
# 0x0002 比赛结果数据（服务器 → 全体机器人，比赛结束后发送，内容数据段 1 字节）
# 协议表 1-2
std_msgs/Header header

uint8 winner
# winner：0 平局，1 红方胜，2 蓝方胜
```


## rm2027_interfaces/msg/GameStatus

来源：[src/rm2027_interfaces/msg/GameStatus.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/GameStatus.msg:1)

```text
# 0x0001 比赛状态数据（服务器 → 全体机器人，1 Hz，内容数据段 11 字节）
# 协议表 1-1
std_msgs/Header header

uint8 GAME_TYPE_RMUL = 1
uint8 GAME_TYPE_RMUC = 2
uint8 GAME_TYPE_RMUT = 3
uint8 GAME_TYPE_RMUA = 4
uint8 GAME_TYPE_RMUL_3V3 = 5
uint8 GAME_TYPE_RMUL_1V1 = 6

uint8 PROGRESS_NOT_START = 0
uint8 PROGRESS_PREPARATION = 1
uint8 PROGRESS_SELF_CHECKING = 2
uint8 PROGRESS_COUNT_DOWN = 3
uint8 PROGRESS_RUNNING = 4
uint8 PROGRESS_GAME_OVER = 5

# 比赛类型，见 GAME_TYPE_*
uint8 game_type
# 当前比赛阶段，见 PROGRESS_*
uint8 game_progress
# 当前阶段剩余时间，单位 s（四舍五入，0~420）
uint16 stage_remain_time
# 本机器人对应的服务器 UNIX 时间戳，单位 us（用于和下位机本地时间对齐）
uint64 sync_time_stamp
```


## rm2027_interfaces/msg/GroundRobotPosition

来源：[src/rm2027_interfaces/msg/GroundRobotPosition.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/GroundRobotPosition.msg:1)

```text
# 0x020B 地面机器人位置数据（服务器 → 对应机器人，1 Hz，内容数据段 40 字节）
# 协议表 1-21。★ 只包含"己方"3 台地面机器人，单位 m。
# 2027 版把 2026 的 1/2 号步兵改成了"重装 + 3 号 + 4 号"，并保留 4 个 4 字节保留位。
# 保留位在下位机转发时被丢弃，不进入本消息。
std_msgs/Header header

geometry_msgs/Point heavy_engineer   # 己方重装机器人位置（x, y），单位 m
geometry_msgs/Point standard_3       # 己方 3 号步兵机器人位置，单位 m
geometry_msgs/Point standard_4       # 己方 4 号步兵机器人位置，单位 m
bool valid                           # 三台位置是否全部有效（任一保留位异常时由下位机置 false）
```


## rm2027_interfaces/msg/PowerHeat

来源：[src/rm2027_interfaces/msg/PowerHeat.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/PowerHeat.msg:1)

```text
# 0x0202 实时功率热量数据（服务器 → 对应机器人，10 Hz，内容数据段 14 字节）
# 协议表 1-13。
# ★ 2027 版前 8 字节是保留位（2026 版这里直接是缓冲能量），真正有用的字段从偏移 8 开始。
#   下位机若沿用 2026 的 6 字节结构体会整帧拒绝。
std_msgs/Header header

uint16 buffer_energy      # 偏移 8  缓冲能量，单位 J
uint16 shooter_17mm_heat  # 偏移 10 17mm 发射机构的射击热量
uint16 shooter_42mm_heat  # 偏移 12 42mm 发射机构的射击热量
```


## rm2027_interfaces/msg/ProjectileAllowance

来源：[src/rm2027_interfaces/msg/ProjectileAllowance.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/ProjectileAllowance.msg:1)

```text
# 0x0208 弹丸允许发弹量（服务器 → 己方重装/步兵/哨兵，10 Hz，内容数据段 8 字节）
# 协议表 1-18
std_msgs/Header header

uint16 projectile_allowance_17mm      # 机器人自身剩余的 17mm 允许发弹量
uint16 projectile_allowance_42mm      # 42mm 允许发弹量
uint16 remaining_gold_coin            # 剩余金币数量
uint16 projectile_allowance_fortress  # 堡垒储备允许发弹量
```


## rm2027_interfaces/msg/RefereeFrame

来源：[src/rm2027_interfaces/msg/RefereeFrame.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/RefereeFrame.msg:1)

```text
# 裁判系统原始帧（不是协议里的命令，而是转接桥的调试/兜底通道）
#
# 下位机把两条链路上收到的**每一个 CRC 校验通过**的裁判系统帧原样转发上来，
# 桥在这里按 2027 协议解析。这样协议新增/改动的命令不需要下位机重新烧录。
#
# 帧格式（两字节长度，见协议 1.2 节）：
#   偏移 0  1B  SOF = 0xA5
#   偏移 1  2B  data_length = 内容数据段字节数 n（整帧长度 - 9，不含 cmd_id/CRC）
#   偏移 3  1B  seq
#   偏移 4  1B  CRC8（覆盖 0..3）
#   偏移 5  2B  cmd_id
#   偏移 7  nB  内容数据段
#   偏移 7+n  2B CRC16（覆盖 0..7+n-1）
std_msgs/Header header

uint8 BUS_MAIN = 0     # 常规链路（电源管理模块 UART，115200）
uint8 BUS_VIDEO = 1    # 图传链路（UART，921600）

uint8 bus              # 来源链路，见 BUS_*
uint8 seq              # 下位机侧递增序号，0-255 回绕，仅用于统计丢帧
uint32 tick_ms         # 下位机收到该帧时的本地毫秒计数（开机起算）
uint16 len             # 整帧字节数，9..（含 SOF 与 CRC16）
uint16 cmd_id          # 桥解析出的命令码；长度非法时置 0
uint8[] raw            # 原始帧字节，从 SOF(0xA5) 到最后 2 字节 CRC16
```


## rm2027_interfaces/msg/RefereeTx

来源：[src/rm2027_interfaces/msg/RefereeTx.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/RefereeTx.msg:1)

```text
# 上位机要求下位机向裁判系统发送一帧（上行原始帧）
#
# 桥负责组帧：SOF + data_length + seq + CRC8 + cmd_id + data + CRC16。
# 下位机只负责把 data 原样放进内容数据段，按 bus 选择串口发出。
# 这样 0x0301（机器人交互数据）下面的子内容 0x0120 / 0x0121 等都由上位机组包，
# 协议改动不需要重新烧录下位机。
std_msgs/Header header

uint8 BUS_MAIN = 0
uint8 BUS_VIDEO = 1

uint16 cmd_id          # 命令码，例如 0x0301
uint8 bus              # 从哪条链路发出，默认 BUS_MAIN
uint8[] data           # 内容数据段（不含帧头/帧尾/CRC）
```


## rm2027_interfaces/msg/RefereeWarning

来源：[src/rm2027_interfaces/msg/RefereeWarning.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/RefereeWarning.msg:1)

```text
# 0x0104 裁判警告数据（服务器 → 对应机器人，1/2/3 Hz，内容数据段 3 字节）
# 协议表 1-13
std_msgs/Header header

uint8 level                 # 警告等级：1 黄牌，2 红牌，3 判负
uint8 offending_robot_id    # 违规机器人 ID，见附录二
uint8 count                 # 触犯次数（用于区分同一机器人同类警告的第几次）
```


## rm2027_interfaces/msg/RfidStatus

来源：[src/rm2027_interfaces/msg/RfidStatus.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/RfidStatus.msg:1)

```text
# 0x0209 机器人 RFID 模块状态（服务器 → 对应机器人，3 Hz，内容数据段 6 字节）
# 协议表 1-19：rfid_status(uint32) + rfid_status_2(uint16)，共 39 个有效位。
# ★ 2026 版只有 4 字节 / 32 位，2027 版拆成两个字段并且新增了隧道增益点。
# 每个 bool 为 true 表示"已检测到该增益点 RFID 卡"。
std_msgs/Header header

uint32 rfid_status      # 原始字 1（bit0-30 有效）
uint16 rfid_status_2    # 原始字 2（bit0-8 有效）

# ---------- rfid_status ----------
bool supply_zone_own                    # bit0  己方补给区
bool assembly_own                       # bit1  己方装配区增益点
bool base_gain_r                        # bit2  R 基地增益点
bool base_gain_b                        # bit3  B 基地增益点
bool fortress_r                         # bit4  R 堡垒增益点
bool fortress_b                         # bit5  B 堡垒增益点
bool central_highland_r                 # bit6  R 中央高地增益点
bool central_highland_b                 # bit7  B 中央高地增益点
bool outpost_r                          # bit8  R 前哨增益点
bool outpost_b                          # bit9  B 前哨增益点
bool highland_own                       # bit10 己方制高点增益点
bool rmul_r_highland                    # bit11 R 高地控制区（兼增益点，仅 RMUL）
bool rmul_b_highland                    # bit12 B 高地控制区（兼增益点，仅 RMUL）
bool rmul_center                        # bit13 中央控制区（兼增益点，仅 RMUL）
bool terrain_cross_own_center_down      # bit14 己方地形跨越增益点（中央高地下方）
bool terrain_cross_own_center_up        # bit15 己方地形跨越增益点（中央高地上方）
bool terrain_cross_enemy_center_down    # bit16 对方地形跨越增益点（中央高地下方）
bool terrain_cross_enemy_center_up      # bit17 对方地形跨越增益点（中央高地上方）
bool terrain_cross_own_highway_down     # bit18 己方地形跨越增益点（公路下方）
bool terrain_cross_own_highway_up       # bit19 己方地形跨越增益点（公路上方）
bool terrain_cross_enemy_highway_down   # bit20 对方地形跨越增益点（公路下方）
bool terrain_cross_enemy_highway_up     # bit21 对方地形跨越增益点（公路上方）
bool tunnel_own_highway_down            # bit22 己方隧道（靠近己方公路一侧 下方）
bool tunnel_own_highway_mid             # bit23 己方隧道（靠近己方公路一侧 中间）
bool tunnel_own_highway_up              # bit24 己方隧道（靠近己方公路一侧 上方）
bool tunnel_near_own_trapezoid_low      # bit25 隧道（靠近中央高地，靠近己方梯形高地较低处）
bool tunnel_near_own_trapezoid_mid      # bit26 隧道（靠近中央高地，靠近己方梯形高地较中间处）
bool tunnel_near_own_trapezoid_high     # bit27 隧道（靠近中央高地，靠近己方梯形高地较高处）
bool tunnel_far_own_trapezoid_far       # bit28 隧道（远离中央高地，距己方梯形高地较远处）
bool tunnel_far_own_trapezoid_mid       # bit29 隧道（远离中央高地，距己方梯形高地中间处）
bool tunnel_far_own_trapezoid_near      # bit30 隧道（远离中央高地，距己方梯形高地较近处）

# ---------- rfid_status_2 ----------
bool tunnel_enemy_highway_down          # bit0  对方隧道（靠近对方公路一侧 下方）
bool tunnel_enemy_highway_mid           # bit1  对方隧道（靠近对方公路一侧 中间）
bool tunnel_enemy_highway_up            # bit2  对方隧道（靠近对方公路一侧 上方）
bool tunnel_near_enemy_trapezoid_low    # bit3  隧道（靠近中央高地，靠近对方梯形高地较低处）
bool tunnel_near_enemy_trapezoid_mid    # bit4  隧道（靠近中央高地，靠近对方梯形高地较中间处）
bool tunnel_near_enemy_trapezoid_high   # bit5  隧道（靠近中央高地，靠近对方梯形高地较高处）
bool tunnel_far_enemy_trapezoid_far     # bit6  隧道（远离中央高地，距对方梯形高地较远处）
bool tunnel_far_enemy_trapezoid_mid     # bit7  隧道（远离中央高地，距对方梯形高地中间处）
bool tunnel_far_enemy_trapezoid_near    # bit8  隧道（远离中央高地，距对方梯形高地较近处）
```


## rm2027_interfaces/msg/RobotBuff

来源：[src/rm2027_interfaces/msg/RobotBuff.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/RobotBuff.msg:1)

```text
# 0x0204 机器人增益数据（服务器 → 对应机器人，3 Hz，内容数据段 7 字节）
# 协议表 1-15
std_msgs/Header header

uint8 recovery_buff       # 回血增益，百分比（10 表示每秒恢复血量上限的 10%）
uint16 cooling_buff       # 射击热量冷却增益具体值（直接值，x 表示冷却增加 x/s）
uint8 defence_buff        # 防御增益，百分比
uint8 vulnerability_buff  # 负防御增益，百分比（30 表示 -30%）
uint16 attack_buff        # 攻击增益，百分比（50 表示 50%）
```


## rm2027_interfaces/msg/RobotDamage

来源：[src/rm2027_interfaces/msg/RobotDamage.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/RobotDamage.msg:1)

```text
# 0x0206 机器人受伤害数据（服务器 → 对应机器人，即时发送，内容数据段 7 字节）
# 协议表 1-16。本机裁判系统本地判定，最终是否生效以服务器判定为准。
std_msgs/Header header

uint8 armor_id             # 装甲模块（或测速模块）ID 编号，其他原因扣血时为 0
uint8 hp_deduction_reason  # 0 弹丸攻击，1 装甲模块离线，2 超级电容管理模块离线，5 受撞击
uint16 hit_sequence        # 命中事件序号，每发命中递增 1
int16 hit_position_x       # 命中位置 x，单位 mm（装甲检测面左下角为原点）
int16 hit_position_y       # 命中位置 y，单位 mm
```


## rm2027_interfaces/msg/RobotHP

来源：[src/rm2027_interfaces/msg/RobotHP.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/RobotHP.msg:1)

```text
# 0x0003 机器人血量数据（服务器 → 全体机器人，3 Hz，内容数据段 24 字节）
# 协议表 1-8。
# ★ 2027 版是"己方/对方"视角，不是红蓝视角：字段名里的 ally 就是本机器人所属方，
#   所以不需要先知道自己是红还是蓝就能直接用。2 号位在 2027 是保留位。
# ★ 双方血量都是真值，不需要雷达；damage_difference 与累计控制度也在这里。
std_msgs/Header header

uint16 ally_1_robot_hp      # 偏移 0  己方 1 号重装机器人血量（未上场/被罚下为 0）
uint16 reserved             # 偏移 2  保留位
uint16 ally_3_robot_hp      # 偏移 4  己方 3 号步兵机器人血量
uint16 ally_4_robot_hp      # 偏移 6  己方 4 号步兵机器人血量
int16 damage_difference     # 偏移 8  己方全队总伤害与对方全队总伤害之差
uint16 ally_7_robot_hp      # 偏移 10 己方 7 号哨兵机器人血量（就是自己）
uint16 outpost_r_hp         # 偏移 12 R 前哨血量
uint16 ally_base_hp         # 偏移 14 己方基地血量
uint16 outpost_b_hp         # 偏移 16 B 前哨血量
uint16 enemy_base_hp        # 偏移 18 对方基地血量
uint16 ally_control_degree  # 偏移 20 己方累计控制度
uint16 enemy_control_degree # 偏移 22 对方累计控制度
```


## rm2027_interfaces/msg/RobotPosition

来源：[src/rm2027_interfaces/msg/RobotPosition.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/RobotPosition.msg:1)

```text
# 0x0203 机器人位置数据（服务器 → 对应机器人，1 Hz，内容数据段 12 字节）
# 协议表 1-14。坐标系：场地围挡在红方补给站附近的交点为原点，
# 沿场地长边指向蓝方为 x 正方向，沿场地短边指向红方停机坪为 y 正方向。
# ★ 这是裁判系统给的独立定位源，与导航 odom 互校核；单位 m / 度。
std_msgs/Header header

float32 x       # 本机器人位置 x，单位 m
float32 y       # 本机器人位置 y，单位 m
float32 angle   # 本机器人测速模块朝向，单位 度，正北为 0 度
```


## rm2027_interfaces/msg/RobotStatus

来源：[src/rm2027_interfaces/msg/RobotStatus.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/RobotStatus.msg:1)

```text
# 0x0201 机器人状态数据（服务器 → 对应机器人，10 Hz，内容数据段 17 字节）
# 协议表 1-10
std_msgs/Header header

uint8 robot_id                    # 本机器人 ID，见附录二（7 = 红方哨兵，107 = 蓝方哨兵）
uint8 robot_level                 # 机器人等级
uint16 current_hp                 # 当前血量
uint16 maximum_hp                 # 血量上限
uint16 shooter_barrel_cooling_value   # 射击热量冷却速率，单位 /s
uint16 shooter_barrel_heat_limit      # 射击热量上限
uint16 chassis_power_limit            # 底盘功率上限，单位 W
float32 shooter_initial_speed         # 偏移 12 机器人射击初速度上限，单位 m/s（2027 新增，float）
bool power_gimbal_output              # 云台输出：0 不输出，1 输出
bool power_chassis_output             # 底盘输出：0 不输出，1 输出
bool power_launcher_output            # 发射机构输出：0 不输出，1 输出
```


## rm2027_interfaces/msg/SentryCommand

来源：[src/rm2027_interfaces/msg/SentryCommand.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/SentryCommand.msg:1)

```text
# 子内容 0x0120 哨兵自主决策指令（机器人 → 裁判系统服务器）
#
# 它不是一个独立的 cmd_id，而是 0x0301"机器人交互数据"里 data_cmd_id = 0x0120 的子内容，
# 内容数据段 4 字节。桥会自动补上 0x0301 的 6 字节子头
# （data_cmd_id + sender_id + receiver_id=0x8080），再交给下位机发帧。
#
# ★ 所有"次数"类和"发弹量"类字段都要求单调递增，桥会自己维护单调计数器，
#   本消息里的字段是**绝对值**（连续发同样的值 = 不重复生效），不是增量。
std_msgs/Header header

# bit0 是否确认复活：0 确认不复活（即使读条完成），1 确认复活
bool confirm_respawn
# bit1 是否确认兑换立即复活
bool confirm_buy_respawn
# bit2-12 将要兑换的允许发弹量累计值（开局 0，只能单调递增）
uint16 exchange_allowance_total
# bit13-16 远程兑换发弹量的请求次数（开局 0，每次只能 +1）
uint8 remote_allowance_times
# bit17-20 远程兑换血量的请求次数（开局 0，每次只能 +1）
uint8 remote_hp_times
# bit21-23 姿态：1 进攻，2 防御，3 移动，4 强化进攻，5 强化防御，6 强化移动
uint8 posture
# bit24 是否确认让能量机关进入正在激活状态
bool activate_energy_mechanism
```


## rm2027_interfaces/msg/SentryInfo

来源：[src/rm2027_interfaces/msg/SentryInfo.msg](/home/chengdong/BBT/src/rm2027_interfaces/msg/SentryInfo.msg:1)

```text
# 0x020D 哨兵自主决策相关信息同步（服务器 → 哨兵，1 Hz，内容数据段 14 字节）
# 协议表 1-23：sentry_info(uint32) + sentry_info_2(uint16) + sentry_info_3(uint64)
std_msgs/Header header

uint32 sentry_info
uint16 sentry_info_2
uint64 sentry_info_3

# ---------- sentry_info ----------
# bit0-10 除远程兑换外，本局已成功兑换的允许发弹量（开局 0）
uint16 exchanged_projectile_allowance
# bit11-14 已成功远程兑换允许发弹量的次数（开局 0）
uint8 remote_allowance_exchange_times
# bit15-18 已成功远程兑换血量的次数（开局 0）
uint8 remote_hp_exchange_times
# bit19 当前是否可以确认免费复活
bool can_free_respawn
# bit20 当前是否可以兑换立即复活
bool can_buy_respawn
# bit21-30 当前兑换立即复活需要花费的金币数
uint16 respawn_cost_coin

# ---------- sentry_info_2 ----------
# bit0 是否处于脱战状态：1 脱战。★ 这是"是否在战斗"的唯一权威来源
bool out_of_combat
# bit1-11 队伍 17mm 允许发弹量的剩余可兑换数
uint16 team_allowance_exchangeable
# bit12-13 当前姿态：1 进攻，2 防御，3 移动
uint8 posture
# bit14 己方能量机关当前是否可以进入正在激活状态
bool energy_mechanism_activatable
# bit15 当前姿态是否为强化姿态
bool posture_strengthened

# ---------- sentry_info_3 ----------
# bit0-7 进攻姿态弱化前剩余可持续时长，s
uint8 attack_pose_remain_s
# bit8-15 防御姿态弱化前剩余可持续时长，s
uint8 defence_pose_remain_s
# bit16-23 移动姿态弱化前剩余可持续时长，s
uint8 move_pose_remain_s
# bit32-39 强化进攻姿态剩余可持续时长，s
uint8 strengthened_attack_remain_s
# bit40-47 强化防御姿态剩余可持续时长，s
uint8 strengthened_defence_remain_s
# bit48-55 强化移动姿态剩余可持续时长，s
uint8 strengthened_move_remain_s
```


# 本机安装的 Nav2 Action 定义

`nav2_msgs` 版本 1.3.11（ROS 2 Jazzy）。`ComputePathToPose/FollowPath` 在本次审计中保留为修复前接口记录，用户已确认普通导航撤掉这两个依赖。`NavigateThroughPoses` 保留用于洞内任务，但实机server是否接通尚未确认。标准定义本身不能证明服务器存在。


## nav2_msgs/action/ComputePathToPose

来源：[ComputePathToPose.action](/opt/ros/jazzy/share/nav2_msgs/action/ComputePathToPose.action:1)

```text
#goal definition
geometry_msgs/PoseStamped goal
geometry_msgs/PoseStamped start
string planner_id
bool use_start # If false, use current robot pose as path start, if true, use start above instead
---
#result definition

# Error codes
# Note: The expected priority order of the errors should match the message order
uint16 NONE=0
uint16 UNKNOWN=200
uint16 INVALID_PLANNER=201
uint16 TF_ERROR=202
uint16 START_OUTSIDE_MAP=203
uint16 GOAL_OUTSIDE_MAP=204
uint16 START_OCCUPIED=205
uint16 GOAL_OCCUPIED=206
uint16 TIMEOUT=207
uint16 NO_VALID_PATH=208

nav_msgs/Path path
builtin_interfaces/Duration planning_time
uint16 error_code
string error_msg
---
#feedback definition
```


## nav2_msgs/action/FollowPath

来源：[FollowPath.action](/opt/ros/jazzy/share/nav2_msgs/action/FollowPath.action:1)

```text
#goal definition
nav_msgs/Path path
string controller_id
string goal_checker_id
string progress_checker_id
---
#result definition

# Error codes
# Note: The expected priority order of the errors should match the message order
uint16 NONE=0
uint16 UNKNOWN=100
uint16 INVALID_CONTROLLER=101
uint16 TF_ERROR=102
uint16 INVALID_PATH=103
uint16 PATIENCE_EXCEEDED=104
uint16 FAILED_TO_MAKE_PROGRESS=105
uint16 NO_VALID_CONTROL=106
uint16 CONTROLLER_TIMED_OUT=107

std_msgs/Empty result
uint16 error_code
string error_msg
---
#feedback definition
float32 distance_to_goal
float32 speed
```


## nav2_msgs/action/NavigateThroughPoses

来源：[NavigateThroughPoses.action](/opt/ros/jazzy/share/nav2_msgs/action/NavigateThroughPoses.action:1)

```text

#goal definition
geometry_msgs/PoseStamped[] poses
string behavior_tree
---
#result definition

# Error codes
# Note: The expected priority order of the errors should match the message order
uint16 NONE=0

uint16 error_code
string error_msg
---
#feedback definition
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
float32 distance_remaining
int16 number_of_poses_remaining
```
