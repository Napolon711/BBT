#include "ros2_libxr/referee_2027.hpp"

#include <cstring>
#include <mutex>

namespace ros2_libxr
{
namespace
{
std::uint8_t g_tab8[256];
std::uint16_t g_tab16[256];
std::once_flag g_tables_once;

void init_tables()
{
  std::call_once(g_tables_once, [] {
  for (int i = 0; i < 256; ++i)
  {
    std::uint8_t c8 = static_cast<std::uint8_t>(i);
    for (int j = 0; j < 8; ++j)
    {
      c8 = (c8 & 0x01U) ? static_cast<std::uint8_t>((c8 >> 1) ^ 0x8CU)
                        : static_cast<std::uint8_t>(c8 >> 1);
    }
    g_tab8[i] = c8;

    std::uint16_t c16 = static_cast<std::uint16_t>(i);
    for (int j = 0; j < 8; ++j)
    {
      c16 = (c16 & 1U) ? static_cast<std::uint16_t>((c16 >> 1) ^ 0x8408U)
                       : static_cast<std::uint16_t>(c16 >> 1);
    }
    g_tab16[i] = c16;
  }
  });
}
}  // namespace

std::uint8_t RefCrc8(const void * data, std::size_t len)
{
  init_tables();
  const auto * p = static_cast<const std::uint8_t *>(data);
  std::uint8_t crc = 0xFFU;
  while (len-- > 0)
  {
    crc = g_tab8[(crc ^ *p++) & 0xFFU];
  }
  return crc;
}

std::uint16_t RefCrc16(const void * data, std::size_t len)
{
  init_tables();
  const auto * p = static_cast<const std::uint8_t *>(data);
  std::uint16_t crc = 0xFFFFU;
  while (len-- > 0)
  {
    crc = static_cast<std::uint16_t>(g_tab16[(crc ^ *p++) & 0xFFU] ^ (crc >> 8));
  }
  return crc;
}

bool ParseRefFrame(const std::uint8_t * raw, std::size_t raw_len, RefFrameView & out)
{
  if (raw == nullptr || raw_len < 9 || raw_len > 0xFFFF)
  {
    return false;
  }
  if (raw[0] != kRefSof)
  {
    return false;
  }
  const std::uint16_t data_len =
      static_cast<std::uint16_t>(raw[1] | (static_cast<std::uint16_t>(raw[2]) << 8));
  if (static_cast<std::size_t>(data_len) + 9 != raw_len)
  {
    return false;
  }
  if (RefCrc8(raw, 4) != raw[4])
  {
    return false;
  }
  const std::uint16_t crc16 = static_cast<std::uint16_t>(
      raw[raw_len - 2] | (static_cast<std::uint16_t>(raw[raw_len - 1]) << 8));
  if (RefCrc16(raw, raw_len - 2) != crc16)
  {
    return false;
  }
  out.seq = raw[3];
  out.cmd_id = static_cast<std::uint16_t>(raw[5] | (static_cast<std::uint16_t>(raw[6]) << 8));
  out.payload = raw + 7;
  out.payload_len = data_len;
  return true;
}

std::size_t BuildRefFrame(std::uint16_t cmd_id, std::uint8_t seq, const std::uint8_t * data,
                          std::uint16_t data_len, std::uint8_t * dst, std::size_t cap)
{
  const std::size_t total = static_cast<std::size_t>(data_len) + 9;
  if (dst == nullptr || cap < total)
  {
    return 0;
  }
  dst[0] = kRefSof;
  dst[1] = static_cast<std::uint8_t>(data_len & 0xFFU);
  dst[2] = static_cast<std::uint8_t>((data_len >> 8) & 0xFFU);
  dst[3] = seq;
  dst[4] = RefCrc8(dst, 4);
  dst[5] = static_cast<std::uint8_t>(cmd_id & 0xFFU);
  dst[6] = static_cast<std::uint8_t>((cmd_id >> 8) & 0xFFU);
  if (data_len > 0 && data != nullptr)
  {
    std::memcpy(&dst[7], data, data_len);
  }
  const std::uint16_t crc16 = RefCrc16(dst, total - 2);
  dst[total - 2] = static_cast<std::uint8_t>(crc16 & 0xFFU);
  dst[total - 1] = static_cast<std::uint8_t>((crc16 >> 8) & 0xFFU);
  return total;
}

}  // namespace ros2_libxr
