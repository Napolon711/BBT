// SharedTopic / SharedTopicClient 线上协议编解码（与下位机 LibXR 保持一致）
//
// 一帧 = 18 字节头尾 + payload：
//   偏移 0   1B  prefix = 0x5A
//   偏移 1   3B  payload 长度，24 位大端
//   偏移 4   4B  crc32(话题名)，本机小端
//   偏移 8   8B  时间戳，微秒，64 位大端（下位机开机起算）
//   偏移 16  1B  CRC8，覆盖 [0,16)
//   偏移 17  nB  payload（裸 packed 结构体，零序列化）
//   偏移 17+n 1B CRC8，覆盖 [0, 17+n)
//
// CRC8：多项式 0x8C（反射后的 0x31），初值 0xFF，无最终取反
// CRC32：多项式 0xEDB88320，初值 0xFFFFFFFF，**无最终取反**（与 zlib 差一次 ^0xFFFFFFFF）
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

namespace ros2_libxr
{

constexpr std::uint8_t kStPrefix = 0x5A;
constexpr std::size_t kStHeaderSize = 17;  // prefix..crc8
constexpr std::size_t kStBaseSize = 18;    // header + 尾部 crc8

std::uint8_t Crc8(const void * data, std::size_t len);
std::uint32_t Crc32(const void * data, std::size_t len);
std::uint32_t TopicNameCrc32(const std::string & name);

// 组帧，返回写入字节数；容量不足返回 0。
std::size_t EncodeSharedTopic(std::uint32_t topic_crc32, const void * payload,
                              std::size_t payload_len, std::uint64_t timestamp_us,
                              std::uint8_t * dst, std::size_t cap);

// 增量解析 SharedTopic 字节流（自动重同步、丢弃坏帧）。
class SharedTopicParser
{
public:
  struct Frame
  {
    std::uint32_t topic_crc32{0};
    std::uint64_t timestamp_us{0};
    const std::uint8_t * payload{nullptr};
    std::size_t payload_len{0};
  };
  using Handler = std::function<void(const Frame &)>;

  explicit SharedTopicParser(std::size_t max_buffer = 8192);
  ~SharedTopicParser();
  SharedTopicParser(const SharedTopicParser &) = delete;
  SharedTopicParser & operator=(const SharedTopicParser &) = delete;

  void set_handler(Handler handler) { handler_ = std::move(handler); }
  void push(const std::uint8_t * data, std::size_t len);

  std::uint64_t frames_ok() const { return frames_ok_; }
  std::uint64_t frames_bad() const { return frames_bad_; }
  std::uint64_t bytes_dropped() const { return bytes_dropped_; }
  std::uint64_t bytes_consumed() const { return bytes_consumed_; }

private:
  void parse();

  std::uint8_t * buf_{nullptr};
  std::size_t cap_{0};
  std::size_t len_{0};
  Handler handler_;
  std::uint64_t frames_ok_{0};
  std::uint64_t frames_bad_{0};
  std::uint64_t bytes_dropped_{0};
  std::uint64_t bytes_consumed_{0};
};

}  // namespace ros2_libxr
