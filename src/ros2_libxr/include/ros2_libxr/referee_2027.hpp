// 裁判系统串口协议 V1.0.0（20261029）帧编解码 + 与下位机约定的 SharedTopic 线结构
//
// 帧格式（协议 1.2 节）：
//   SOF(0xA5,1B) data_length(2B) seq(1B) CRC8(1B) cmd_id(2B) data(nB) CRC16(2B)
//   - data_length = data 的字节数 n（**不含** CRC16），整帧长度 = n + 9
//   - CRC8 覆盖前 4 字节；CRC16 覆盖除最后 2 字节以外的全部字节，均按小端存放
#pragma once

#include <cstddef>
#include <cstdint>

namespace ros2_libxr
{

constexpr std::uint8_t kRefSof = 0xA5;
constexpr std::size_t kRefHeaderSize = 5;

std::uint8_t RefCrc8(const void * data, std::size_t len);
std::uint16_t RefCrc16(const void * data, std::size_t len);

struct RefFrameView
{
  std::uint16_t cmd_id{0};
  std::uint8_t seq{0};
  const std::uint8_t * payload{nullptr};
  std::uint16_t payload_len{0};
};

// 校验并解析一帧；raw_len 必须是整帧长度。返回 false 表示帧不合法。
bool ParseRefFrame(const std::uint8_t * raw, std::size_t raw_len, RefFrameView & out);

// 组帧，返回总字节数；容量不足返回 0。
std::size_t BuildRefFrame(std::uint16_t cmd_id, std::uint8_t seq, const std::uint8_t * data,
                          std::uint16_t data_len, std::uint8_t * dst, std::size_t cap);

// ---------------------------------------------------------------------------
// 下位机 ↔ 上位机的 SharedTopic 线结构（全部 [[gnu::packed]] 小端，零序列化）
// 每个话题的 payload 就是下面这些结构体的原始字节。
// ---------------------------------------------------------------------------

// 下行 payload 上限：常规链路最长帧 0x0301 = 118+9 = 127B，图传链路 0x0311 = 60+9 = 69B，
// 128 字节足够容纳所有**下行**命令。上行 0x0310（300B）不走这条通道。
constexpr std::size_t kRefRawCap = 128;

// 下行话题 ref_frame：下位机把两条链路上每个 CRC 通过的帧原样转发上来
struct [[gnu::packed]] RefFrameWire
{
  std::uint8_t bus;    // 0 常规链路（电源管理模块 UART）1 图传链路
  std::uint8_t seq;    // 下位机侧递增序号，仅用于统计
  std::uint16_t len;   // 整帧字节数（9..128）
  std::uint32_t tick_ms;  // 下位机本地毫秒计数
  std::uint8_t raw[kRefRawCap];
};

// 上行话题 ref_tx：上位机要求下位机发一帧
struct [[gnu::packed]] RefTxWire
{
  std::uint16_t cmd_id;
  std::uint16_t len;   // data 有效字节数
  std::uint8_t bus;
  std::uint8_t rsv[3];  // 对齐填充，必须置 0
  std::uint8_t data[kRefRawCap];
};

// 下行话题 chassis_state
struct [[gnu::packed]] ChassisStateWire
{
  float vx;
  float vy;
  float wz;
  std::uint8_t fold_state;   // 0 未知 / 1 折叠中 / 2 已折叠 / 3 展开中 / 4 已展开
  std::uint8_t posture;      // 0 未知 / 1 进攻 / 2 防御 / 3 移动 / 4..6 强化姿态
  std::uint16_t buffer_energy_j;
  std::uint16_t power_w;
  std::uint32_t tick_ms;
};

// 上行话题 chassis_data（沿用下位机已有话题名与结构）
struct [[gnu::packed]] MoveVecWire
{
  float vx;
  float vy;
  float wz;
};

// 下行话题 ahrs_quaternion：LibXR::Quaternion<float> 继承 Eigen::Quaternion<float>，
// 内存布局就是 Eigen 的 coeffs 顺序 (x, y, z, w)。
struct [[gnu::packed]] QuatWire
{
  float x;
  float y;
  float z;
  float w;
};

static_assert(sizeof(RefFrameWire) == 136, "ref_frame payload 必须是 136 字节");
static_assert(sizeof(RefTxWire) == 136, "ref_tx payload 必须是 136 字节");
static_assert(sizeof(ChassisStateWire) == 22, "chassis_state payload 必须是 22 字节");
static_assert(sizeof(MoveVecWire) == 12, "chassis_data payload 必须是 12 字节");
static_assert(sizeof(QuatWire) == 16, "ahrs_quaternion payload 必须是 16 字节");

}  // namespace ros2_libxr
