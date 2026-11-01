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
