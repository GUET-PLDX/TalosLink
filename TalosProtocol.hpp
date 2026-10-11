#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "read_port.hpp"
#include "write_port.hpp"

namespace Pldx::Talos {
enum class Color : uint8_t {
  RED = 0,
  BLUE = 1,
};

LIBXR_PACKED_BEGIN

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

LIBXR_PACKED_END
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
  using VisionCallback = LibXR::Callback<const SendVisionData&>;
  using SimpleVisionCallback = LibXR::Callback<const SendSimpleVisionData&>;

  TalosLink(VisionCallback vision = {}, SimpleVisionCallback simple_vision = {})
      : vision_(vision), simple_vision_(simple_vision) {}

  TalosLink(LibXR::ReadPort& read_port, LibXR::WritePort& write_port,
            VisionCallback vision = {}, SimpleVisionCallback simple_vision = {})
      : read_port_(&read_port),
        write_port_(&write_port),
        vision_(vision),
        simple_vision_(simple_vision) {}

  // Consume a bounded snapshot on the owning thread; never arm a pending read.
  [[nodiscard]] LibXR::ErrorCode Poll() {
    if (!read_port_ || !read_port_->Readable())
      return LibXR::ErrorCode::NOT_SUPPORT;
    size_t remaining = read_port_->Size();
    std::array<uint8_t, sizeof(SendVisionData)> bytes{};
    LibXR::ReadOperation operation;
    while (remaining) {
      const size_t COUNT = remaining < bytes.size() ? remaining : bytes.size();
      const auto RESULT = (*read_port_)({bytes.data(), COUNT}, operation);
      if (RESULT != LibXR::ErrorCode::OK) return RESULT;
      Feed(LibXR::ConstRawData(bytes.data(), COUNT));
      remaining -= COUNT;
    }
    return LibXR::ErrorCode::OK;
  }

  template <typename Frame>
  [[nodiscard]] LibXR::ErrorCode Send(Frame frame,
                                      LibXR::WriteOperation& operation) {
    if (!write_port_) return LibXR::ErrorCode::NOT_SUPPORT;
    const auto BYTES = Encode(frame);
    // WritePort copies bytes into its queue before returning, including async
    // writes.
    return (*write_port_)({BYTES.data(), BYTES.size()}, operation);
  }

  void Feed(LibXR::ConstRawData bytes, bool in_isr = false) {
    if (!bytes.addr_) return;
    const auto* DATA = static_cast<const uint8_t*>(bytes.addr_);
    for (size_t i = 0; i < bytes.size_; ++i) {
      buffer_[size_++] = DATA[i];
      Parse(in_isr);
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

  void Parse(bool in_isr) {
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
              DATA.distance >= 0) {
            vision_.Run(in_isr, frame);
          }
        }
      } else {
        SendSimpleVisionData frame{};
        std::memcpy(&frame, buffer_.data(), sizeof(frame));
        if (std::isfinite(frame.data.target_yaw)) {
          simple_vision_.Run(in_isr, frame);
        }
      }
      size_ = 0;
    }
  }

  std::array<uint8_t, sizeof(SendVisionData)> buffer_{};
  size_t size_ = 0;
  LibXR::ReadPort* read_port_ = nullptr;
  LibXR::WritePort* write_port_ = nullptr;
  VisionCallback vision_;
  SimpleVisionCallback simple_vision_;
};
}  // namespace Pldx::Talos
