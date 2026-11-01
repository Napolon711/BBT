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
