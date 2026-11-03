# 电控 SharedTopic 与裁判 wire 核查

审计基线 `10e0910`；相对 NUC 定义方向。NUC 是运行ROS的上位机，MCU 是电控下位机。本机“UC”在本报告中仅指 `/home/chengdong/27BT-UC` 这个上位机工程目录，不能把这个目录当成已接通的硬件。

来源：[SharedTopic编解码](../../src/ros2_libxr/src/shared_topic.cpp)、[wire结构](../../src/ros2_libxr/include/ros2_libxr/referee_2027.hpp)、[桥收发与注册](../../src/ros2_libxr/src/bridge_node.cpp)，以及电控缓存 `/home/chengdong/27BT-UC/.cache/audit_20261031/daohangxiaoche` 的 `User/xrobot_main.hpp`、`Modules/HostData/HostData.hpp`、`Modules/Chassis/Omni.hpp`。补充文件哈希见 [证据清单](wire-source-evidence-20261103.json)。

## 先核对实际注册方向

| 方向 | BBT桥 `init_topics()` 要求的名称 | 被查电控入口实际注册的名称 |
|---|---|---|
| MCU→NUC（桥RX） | `ref_frame`、`chassis_state`、`ahrs_quaternion`、`yawmotor_angle` | `SharedTopicClient` 只上送 `ahrs_quaternion`、`yawmotor_angle`、`referee_state` |
| NUC→MCU（桥TX） | `chassis_data`、`chassis_mode`、`ref_tx`、`fold_cmd` | `SharedTopic` 只接收 `chassis_data`、`chassis_mode`、`referee_start` |

两边只有四个名称相交。**该电控构建入口没有注册 `ref_frame/ref_tx/chassis_state/fold_cmd`**，旧的 `referee_start/referee_state` 不能代替它们。该事实仅限已查看的源码快照；当前机器人烧录物、其他分支和固件版本未核实。PTY假下位机验证的是模拟收发约定。

`HostData` 内虽还建立 `target_euler`、`fire_notify` 回调，但上述串口接收注册表未包含它们；不能据“有回调”断言它们已经从这个串口入口接通。当前BBT桥也没有对应ROS→wire映射。

## SharedTopic外层字节协议

payload为裸结构体内存拷贝；不是ROS序列化。主机/STM32须共同满足小端、IEEE-754 float32与相同packed布局，协议没有版本字段。

| offset | 大小 | 编码 |
|---:|---:|---|
| 0 | 1 | `0x5A` |
| 1 | 3 | payload长度n，24-bit big-endian |
| 4 | 4 | topic原文CRC32（不含结尾NUL），原样拷贝主机u32；当前平台little-endian |
| 8 | 8 | timestamp_us，big-endian；NUC发送使用steady_clock微秒；桥接收不据此拒绝旧包 |
| 16 | 1 | CRC8覆盖 `[0,16)` |
| 17 | n | 裸payload |
| 17+n | 1 | CRC8覆盖 `[0,17+n)` |

总长 `18+n`。CRC8反射多项式 `0x8C`、初值 `0xFF`、无final XOR；CRC32反射多项式 `0xEDB88320`、初值 `0xFFFFFFFF`、无final XOR。长度/时间戳与payload内部字节序不同，不能按整帧统一字节序解释。

## 每个wire payload

表中 `u8/u16/u32/f32` 为固定宽度类型，`@`后为payload字节偏移；多字节payload字段均little-endian。CRC列可供抓包对照。

| 名称 / CRC32 | 方向、长度 | 字段与语义 |
|---|---|---|
| `ref_frame` / `0x39CCA6F0` | MCU→NUC，136 B | `bus:u8@0`、`seq:u8@1`、`len:u16@2`、`tick_ms:u32@4`、`raw:u8[128]@8`。bus 0常规裁判UART/1图传；len为raw里完整裁判帧长度（9…128）；tick为MCU本地ms。raw必须包含0xA5头和CRC。 |
| `chassis_state` / `0xA2AB738A` | MCU→NUC，22 B | `vx:f32@0`、`vy:f32@4`、`wz:f32@8`、`fold_state:u8@12`、`posture:u8@13`、`buffer_energy_j:u16@14`、`power_w:u16@16`、`tick_ms:u32@18`。声明速度m/s、rad/s、能量J、功率W；fold 0未知/1折叠中/2已折叠/3展开中/4已展开；posture 0未知/1进攻/2防御/3移动/4…6强化。快照未上送，真实性/单位无实车证据。 |
| `ahrs_quaternion` / `0xA6351AEA` | MCU→NUC，16 B | `x,y,z,w` 四个f32，意图为Eigen quaternion coeffs顺序。桥只用于pitch和joint-state。 |
| `yawmotor_angle` / `0xB66ECCA5` | MCU→NUC，4 B | f32，约定rad；桥不做finite校验。 |
| `chassis_data` / `0x4149F404` | NUC→MCU，12 B | `vx:f32@0`、`vy:f32@4`、`wz:f32@8`。桥取 `/cmd_vel.linear.x/y`、`.angular.z`，车体系x前/y左，按m/s与rad/s直传；ROTOR时角速度由新鲜 `/cmd_spin` 覆盖。电控实现单位冲突见下节。 |
| `chassis_mode` / `0x7B7040CC` | NUC→MCU，1 B | u8：0 RELAX/1 INDEPENDENT/2 ROTOR/3 FOLLOW。模式值匹配不等于各模式速度行为匹配。 |
| `ref_tx` / `0x35258E05` | NUC→MCU，136 B | `cmd_id:u16@0`、`len:u16@2`、`bus:u8@4`、`rsv:u8[3]@5`、`data:u8[128]@8`。rsv全0；data仅裁判内容段，当前购买cmd_id=0x0301、len=10、bus=0；由MCU组官方外帧并发送，快照没有接收者。 |
| `fold_cmd` / `0xA150712C` | NUC→MCU，1 B | u8：1折叠/2展开；桥原样发送，不校验枚举，不证明执行。需后续独立fold_state反馈。 |

wire大小在NUC头文件有136/136/22/12/16的 `static_assert`。这只验证NUC构建布局，不会替电控验证ABI。topic名参与CRC，一个字符或结构字段顺序的变化也须同步双方。

## 旧电控接口及速度语义冲突

`User/xrobot_main.hpp` 为 `HostData` 传入 `target_euler/chassis_data/fire_notify/chassis_mode`。其回调类型如下；串口实际注册范围仍以上表为准。

| 旧名称 | MCU回调结构 | BBT当前桥情况 |
|---|---|---|
| `target_euler` | 9个float：`rol,pit,yaw,rol_dot,pit_dot,yaw_dot,rol_ddot,pit_ddot,yaw_ddot` | 无该wire发送和ROS映射；不是 `/sentry/attack_cmd` 的别名 |
| `chassis_data` | `HostChassisTarget {float vx,vy,w;}` | 名称/数量相同，但执行尺度不一致 |
| `fire_notify` | `LauncherCMD {bool isfire;}` | 无该wire发送和ROS映射；attack意图未由本桥接入发射闭环 |
| `chassis_mode` | `HostChassisMode {uint8_t mode;}` | 0…3模式枚举相同，无执行ack |
| `referee_start` | `uint8_t`：4开始、0重置 | MCU `SimReferee` 演示模块输入，不是真实裁判帧；当前BBT桥不发送 |
| `referee_state` | 16B `SimRefereeState`，字段见下表 | MCU `SimReferee` 演示模块输出，不是新raw/typed裁判接口；当前BBT桥不接收 |

被查入口明确实例化 `SimReferee`：每100 ms发布、比赛420 s、开始60 s后己方前哨HP归零，初始HP=400、弹量=300、前哨HP=1500、team_color=0（红）。因此这两个旧topic接的是**电控侧裁判演示状态**，不能据它们证明真实裁判接通。`Modules/SimReferee/SimReferee.hpp` 固定以下16B布局，并有size/offset静态断言：

| offset | 字段 / 类型 |
|---:|---|
| 0 | `game_progress: uint8_t`（0未开始/4比赛/5结束） |
| 1 | `reserved_0: uint8_t[3]`（全0） |
| 4 | `stage_remain_time: int32_t`（s） |
| 8 | `current_hp: uint16_t` |
| 10 | `projectile_allowance_17mm: uint16_t` |
| 12 | `red_outpost_hp: uint16_t` |
| 14 | `blue_outpost_hp: uint16_t` |

`Modules/Chassis/Omni.hpp::UpdateCMD()` 的直接实现：

- INDEPENDENT角速度为 `max_v * cmd_data.z / wheel_to_center`；平移以 `|x|+|y|` 做菱形限幅，再乘 `sqrt(2)*max_v`，即按归一化输入缩放，不是SI速度直通。
- ROTOR不使用输入z；旋转基准为 `-max_v/wheel_to_center`，再根据平移/功率系数降速。因此NUC的spin目标值（包括0）不直接控制该模式转速。
- ROTOR/FOLLOW平移还按yaw和max_v变换；桥本身没有做这些单位缩放或TF。

`HostData` 写入 `last_chassis_time_/last_gimbal_time_/last_fire_time_`，但该文件没有使用它们做超时停控，`OnMonitor()`为空。本次未发现快照中的HostData独立断流看门狗；这不等于已经穷尽所有硬件保护。NUC桥在自身正常运行、定时器正常、串口仍可写时，`/cmd_vel` 200 ms超时会每10 ms发送全零。桥退出、USB断开或MCU不接收时，NUC不能发送最终零包。

## 0x0301 / 0x0120购买与复活下行

生产输入为相对ROS名 `referee/sentry_command`、类型 `rm2027_interfaces/msg/SentryCommand`。桥没有以 `rm2027_interfaces/msg/RefereeTx` 创建ROS出站topic，而是使用私有 `RefTxWire`。

| `ref_tx.data` offset | 类型/值 |
|---:|---|
| 0 | u16 LE：data_cmd_id=0x0120 |
| 2 | u16 LE：sender_id，人工own_team红7/蓝107；不根据裁判robot_id交叉确认 |
| 4 | u16 LE：receiver_id=0x8080 |
| 6 | u32 LE bitfield：见下表 |

| bit | 内容 |
|---:|---|
| 0 | confirm_respawn |
| 1 | confirm_buy_respawn |
| 2…12 | exchange_allowance_total，11bit累计量 |
| 13…16 | remote_allowance_times，4bit累计次数 |
| 17…20 | remote_hp_times，4bit累计次数 |
| 21…23 | posture，3bit；有效1…6 |
| 24 | activate_energy_mechanism |
| 25…31 | 0 |

桥将这10B内容放进136B `RefTxWire`。MCU还需生成官方外帧：`A5 + data_length:u16 LE + seq:u8 + header CRC8 + cmd_id:u16 LE + data[n] + CRC16 LE`，总长 `n+9`；此命令应为19B。NUC的 `BuildRefFrame()` 虽能组整帧，当前发送函数没有调用；组SOF/len/seq/CRC的责任留给MCU，缓存固件没有此消费者。

默认桥重发5 Hz（配置上限10 Hz），保留最新命令3 s；要求新鲜0x020D、0x0001且game_progress=4，累计数单调不减、次数最多增1，posture=0只用裁判/底盘回读补足，未知则拒发。ROS收件、串口write成功、重复累计请求均不等于裁判回执。`RefereeTx.msg` 注释称“桥组帧”，与当前发送实现不符，应以此处的实际执行链为审查依据。

## 当前能下的结论

BBT与本机UC的 `shared_topic.hpp/.cpp`、`referee_2027.hpp/.cpp` 四文件逐字节相同；证明的是两份上位机编解码实现相同。旧电控注册表缺新接口、速度尺度不同、ROTOR忽略w，是已查看代码可复核的差异。当前烧录物哈希、真实串口抓包、裁判回执、折叠机构反馈和实际底盘尺度尚无本次审计证据。
