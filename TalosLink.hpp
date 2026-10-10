#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>

namespace Pldx::Talos {
enum class Color : uint8_t {
  RED = 0,
  BLUE = 1,
};

#pragma pack(push, 1)

struct HeaderFrame {
  static constexpr uint8_t SoF() { return 0x5A; }

  static constexpr uint8_t EoF() { return 0xA5; }

  uint8_t sof;
  uint8_t len;
  uint8_t id;
};

struct ReceiveImuData {
  HeaderFrame header;
  uint32_t time_stamp;
  struct {
    Color self_color;
    float bullet_speed;
    float yaw;
    float pitch;
    float roll;
    float yaw_vel;
    float pitch_vel;
    float roll_vel;
  } data;
  uint8_t eof;
};

struct ReceiveCapabilitiesData {
  HeaderFrame header;
  uint32_t time_stamp;
  struct {
    uint8_t following;

    uint8_t power_rune;

    uint8_t quanta;
  } data;
  uint8_t eof;
};

struct SendQuantaData {
  HeaderFrame header;
  uint32_t time_stamp;
  struct {
    uint16_t custom_byte_block_len;
    uint8_t custom_byte_block[298];
  } data;
  uint8_t eof;
};

struct ReceiveSimpleImuData {
  HeaderFrame header;
  uint32_t time_stamp;
  struct {
    Color self_color;
    float yaw;
    float pitch;
    float roll;
  } data;
  uint8_t eof;
};

struct SendVisionData {
  HeaderFrame header;
  struct {
    bool fire_advice;
    float target_yaw;
    float target_pitch;
    float ref_yaw_v;
    float ref_pitch_v;
    float ref_yaw_a;
    float ref_pitch_a;
    float distance;
  } data;
  uint8_t eof;
};

static_assert(sizeof(bool) == sizeof(uint8_t));

struct SendSimpleVisionData {
  HeaderFrame header;
  struct {
    float target_yaw;
  } data;
  uint8_t eof;
};

#pragma pack(pop)
static_assert(sizeof(HeaderFrame) == 3);
static_assert(sizeof(ReceiveImuData) == 37);
static_assert(sizeof(ReceiveCapabilitiesData) == 11);
static_assert(sizeof(ReceiveSimpleImuData) == 21);
static_assert(sizeof(SendVisionData) == 33);
static_assert(sizeof(SendSimpleVisionData) == 8);
static_assert(sizeof(SendQuantaData) == 308);

// MCU endpoint. Feed transport chunks; callbacks execute synchronously.
class TalosLink {
 public:
  using VisionCallback = void (*)(void*, const SendVisionData&);
  using SimpleVisionCallback = void (*)(void*, const SendSimpleVisionData&);

  TalosLink(void* context, VisionCallback vision,
            SimpleVisionCallback simple_vision)
      : context_(context), vision_(vision), simple_vision_(simple_vision) {}

  void Feed(std::span<const uint8_t> bytes) {
    for (auto byte : bytes) {
      buffer_[size_++] = byte;
      Parse();
    }
  }

  // Feedback includes timestamp; len excludes header, timestamp and tail.
  template <typename Frame>
  static auto Encode(Frame frame) {
    static_assert(std::is_same_v<Frame, ReceiveImuData> ||
                  std::is_same_v<Frame, ReceiveSimpleImuData> ||
                  std::is_same_v<Frame, ReceiveCapabilitiesData>);
    frame.header.sof = HeaderFrame::SoF();
    frame.header.len = sizeof(Frame) - 8;
    frame.header.id = std::is_same_v<Frame, ReceiveImuData> ? 0x01 : 0x03;
    frame.eof = HeaderFrame::EoF();
    std::array<uint8_t, sizeof(Frame)> bytes{};
    std::memcpy(bytes.data(), &frame, sizeof(Frame));
    return bytes;
  }

 private:
  void Drop() {
    --size_;
    std::memmove(buffer_.data(), buffer_.data() + 1, size_);
  }

  void Parse() {
    while (size_) {
      if (buffer_[0] != HeaderFrame::SoF()) {
        Drop();
        continue;
      }
      if (size_ < 3) return;
      // Vision frames have no timestamp. ID 0x04 here is simple vision.
      size_t frame_size = 0;
      if (buffer_[2] == 0x02 && buffer_[1] == 29) {
        frame_size = sizeof(SendVisionData);
      } else if (buffer_[2] == 0x04 && buffer_[1] == 4) {
        frame_size = sizeof(SendSimpleVisionData);
      } else {
        Drop();
        continue;
      }
      if (size_ < frame_size) return;
      if (buffer_[frame_size - 1] != HeaderFrame::EoF()) {
        Drop();
        continue;
      }
      if (frame_size == sizeof(SendVisionData)) {
        // Validate bool representation before copying into a C++ bool.
        if (buffer_[3] <= 1) {
          SendVisionData frame{};
          std::memcpy(&frame, buffer_.data(), sizeof(frame));
          const auto& DATA = frame.data;
          if (std::isfinite(DATA.target_yaw) &&
              std::isfinite(DATA.target_pitch) &&
              std::isfinite(DATA.ref_yaw_v) &&
              std::isfinite(DATA.ref_pitch_v) &&
              std::isfinite(DATA.ref_yaw_a) &&
              std::isfinite(DATA.ref_pitch_a) && std::isfinite(DATA.distance) &&
              DATA.distance >= 0 && vision_) {
            vision_(context_, frame);
          }
        }
      } else {
        SendSimpleVisionData frame{};
        std::memcpy(&frame, buffer_.data(), sizeof(frame));
        if (std::isfinite(frame.data.target_yaw) && simple_vision_) {
          simple_vision_(context_, frame);
        }
      }
      size_ = 0;
    }
  }

  std::array<uint8_t, sizeof(SendVisionData)> buffer_{};
  size_t size_ = 0;
  void* context_;
  VisionCallback vision_;
  SimpleVisionCallback simple_vision_;
};
}  // namespace Pldx::Talos
