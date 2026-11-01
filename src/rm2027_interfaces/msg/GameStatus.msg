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
