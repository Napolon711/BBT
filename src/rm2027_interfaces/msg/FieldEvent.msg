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
