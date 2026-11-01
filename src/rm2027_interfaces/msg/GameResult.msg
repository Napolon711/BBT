# 0x0002 比赛结果数据（服务器 → 全体机器人，比赛结束后发送，内容数据段 1 字节）
# 协议表 1-2
std_msgs/Header header

uint8 winner
# winner：0 平局，1 红方胜，2 蓝方胜
