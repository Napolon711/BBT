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
