#include "ros2_libxr/shared_topic.hpp"

#include <algorithm>
#include <cstring>
#include <mutex>

namespace ros2_libxr
{
namespace
{
std::uint8_t g_crc8_tab[256];
std::uint32_t g_crc32_tab[256];
std::once_flag g_tables_once;

void init_tables()
{
  std::call_once(g_tables_once, [] {
  for (int i = 0; i < 256; ++i)
  {
    // CRC8: poly 0x8C reflected, init handled per call
    std::uint8_t c8 = static_cast<std::uint8_t>(i);
    for (int j = 0; j < 8; ++j)
    {
      c8 = (c8 & 0x01U) ? static_cast<std::uint8_t>((c8 >> 1) ^ 0x8CU)
                        : static_cast<std::uint8_t>(c8 >> 1);
    }
    g_crc8_tab[i] = c8;

    std::uint32_t c32 = static_cast<std::uint32_t>(i);
    for (int j = 0; j < 8; ++j)
    {
      c32 = (c32 & 1U) ? ((c32 >> 1) ^ 0xEDB88320U) : (c32 >> 1);
    }
    g_crc32_tab[i] = c32;
  }
  });
}
}  // namespace

std::uint8_t Crc8(const void * data, std::size_t len)
{
  init_tables();
  const auto * p = static_cast<const std::uint8_t *>(data);
  std::uint8_t crc = 0xFFU;
  while (len-- > 0)
  {
    crc = g_crc8_tab[(crc ^ *p++) & 0xFFU];
  }
  return crc;
}

std::uint32_t Crc32(const void * data, std::size_t len)
{
  init_tables();
  const auto * p = static_cast<const std::uint8_t *>(data);
  std::uint32_t crc = 0xFFFFFFFFU;
  while (len-- > 0)
  {
    crc = g_crc32_tab[(crc ^ *p++) & 0xFFU] ^ (crc >> 8);
  }
  return crc;
}

std::uint32_t TopicNameCrc32(const std::string & name)
{
  return Crc32(name.data(), name.size());
}

std::size_t EncodeSharedTopic(std::uint32_t topic_crc32, const void * payload,
                              std::size_t payload_len, std::uint64_t timestamp_us,
                              std::uint8_t * dst, std::size_t cap)
{
  const std::size_t total = kStBaseSize + payload_len;
  if (dst == nullptr || cap < total || payload_len > 0xFFFFFFU)
  {
    return 0;
  }
  std::memset(dst, 0, kStHeaderSize);
  dst[0] = kStPrefix;
  dst[1] = static_cast<std::uint8_t>((payload_len >> 16) & 0xFFU);
  dst[2] = static_cast<std::uint8_t>((payload_len >> 8) & 0xFFU);
  dst[3] = static_cast<std::uint8_t>(payload_len & 0xFFU);
  std::memcpy(&dst[4], &topic_crc32, sizeof(topic_crc32));
  for (int i = 0; i < 8; ++i)
  {
    dst[8 + i] = static_cast<std::uint8_t>((timestamp_us >> ((7 - i) * 8)) & 0xFFU);
  }
  dst[16] = Crc8(dst, 16);
  if (payload_len > 0 && payload != nullptr)
  {
    std::memcpy(&dst[kStHeaderSize], payload, payload_len);
  }
  dst[kStHeaderSize + payload_len] = Crc8(dst, kStHeaderSize + payload_len);
  return total;
}

SharedTopicParser::SharedTopicParser(std::size_t max_buffer) : cap_(max_buffer)
{
  buf_ = new std::uint8_t[cap_];
}

SharedTopicParser::~SharedTopicParser() { delete[] buf_; }

void SharedTopicParser::push(const std::uint8_t * data, std::size_t len)
{
  if (data == nullptr || len == 0)
  {
    return;
  }
  std::size_t offset = 0;
  while (offset < len)
  {
    if (len_ == cap_)
    {
      // 缓冲区满但还没有完整帧：丢掉最旧的字节，保证不卡死。
      const std::size_t drop = cap_ / 2;
      std::memmove(buf_, buf_ + drop, cap_ - drop);
      len_ -= drop;
      bytes_dropped_ += drop;
    }
    const std::size_t room = cap_ - len_;
    const std::size_t take = std::min(room, len - offset);
    std::memcpy(buf_ + len_, data + offset, take);
    len_ += take;
    offset += take;
    parse();
  }
}

void SharedTopicParser::parse()
{
  while (len_ >= kStBaseSize)
  {
    // 1) 同步到 0x5A
    if (buf_[0] != kStPrefix)
    {
      std::size_t i = 1;
      while (i < len_ && buf_[i] != kStPrefix)
      {
        ++i;
      }
      bytes_dropped_ += i;
      std::memmove(buf_, buf_ + i, len_ - i);
      len_ -= i;
      if (len_ < kStBaseSize)
      {
        return;
      }
    }

    // 2) 帧头 CRC8
    if (Crc8(buf_, 16) != buf_[16])
    {
      std::memmove(buf_, buf_ + 1, len_ - 1);
      --len_;
      bytes_dropped_ += 1;
      ++frames_bad_;
      continue;
    }

    const std::size_t payload_len =
        (static_cast<std::size_t>(buf_[1]) << 16) |
        (static_cast<std::size_t>(buf_[2]) << 8) | static_cast<std::size_t>(buf_[3]);
    const std::size_t total = kStBaseSize + payload_len;
    if (payload_len > cap_ - kStBaseSize)
    {
      // 长度离谱（多半是误同步），跳过这个 0x5A
      std::memmove(buf_, buf_ + 1, len_ - 1);
      --len_;
      bytes_dropped_ += 1;
      ++frames_bad_;
      continue;
    }
    if (len_ < total)
    {
      return;  // 等更多字节
    }

    // 3) 整帧 CRC8
    if (Crc8(buf_, kStHeaderSize + payload_len) != buf_[kStHeaderSize + payload_len])
    {
      std::memmove(buf_, buf_ + 1, len_ - 1);
      --len_;
      bytes_dropped_ += 1;
      ++frames_bad_;
      continue;
    }

    Frame frame;
    std::memcpy(&frame.topic_crc32, &buf_[4], sizeof(frame.topic_crc32));
    std::uint64_t ts = 0;
    for (int i = 0; i < 8; ++i)
    {
      ts = (ts << 8) | static_cast<std::uint64_t>(buf_[8 + i]);
    }
    frame.timestamp_us = ts;
    frame.payload = buf_ + kStHeaderSize;
    frame.payload_len = payload_len;

    if (handler_)
    {
      handler_(frame);
    }
    ++frames_ok_;
    bytes_consumed_ += total;
    std::memmove(buf_, buf_ + total, len_ - total);
    len_ -= total;
  }
}

}  // namespace ros2_libxr
