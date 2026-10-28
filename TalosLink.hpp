#pragma once
// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Talos visual communication protocol over LibXR UART/USB
depends:
- id: GUET-PLDX/CMD
- id: GUET-PLDX/Referee
=== END MANIFEST === */
// clang-format on

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

#ifndef TALOS_LINK_PROTOCOL_ONLY

#include "CMD.hpp"
#include "Referee.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"
#include "uart.hpp"

class TalosLink {
 public:
  TalosLink(
      CMD& cmd, LibXR::UART& uart, const char* gyro_topic_name,
      const char* euler_topic_name, const char* launcher_topic_name,
      const char* referee_topic_name, uint32_t task_stack_depth,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM)
      : uart_((&uart)),
        cmd_(&cmd),
        gyro_topic_(LibXR::Topic::FindOrCreate<Eigen::Matrix<float, 3, 1>>(
            gyro_topic_name)),
        euler_topic_(LibXR::Topic::FindOrCreate<LibXR::EulerAngle<float>>(
            euler_topic_name)),
        launcher_topic_(LibXR::Topic::FindOrCreate<Referee::LauncherPack>(
            launcher_topic_name)),
        referee_topic_(
            LibXR::Topic::FindOrCreate<Referee::RobotGameRefereePack>(
                referee_topic_name)) {
    vision_callback_ = Pldx::Talos::TalosLink::VisionCallback::Create(
        [](bool, TalosLink* self, const Pldx::Talos::SendVisionData& frame) {
          self->OnVision(frame);
        },
        this);
    simple_callback_ = Pldx::Talos::TalosLink::SimpleVisionCallback::Create(
        [](bool, TalosLink* self,
           const Pldx::Talos::SendSimpleVisionData& frame) {
          self->OnSimpleVision(frame);
        },
        this);
    ASSERT(uart_->read_port_ && uart_->write_port_);
    link_ = Pldx::Talos::TalosLink(*uart_->read_port_, *uart_->write_port_,
                                   vision_callback_, simple_callback_);

    thread_.Create(this, ThreadFunc, "TalosLink", task_stack_depth,
                   thread_priority);
  }

 private:
  static void ThreadFunc(TalosLink* self) {
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro(
        self->gyro_topic_);
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler(
        self->euler_topic_);
    LibXR::Topic::ASyncSubscriber<Referee::LauncherPack> launcher(
        self->launcher_topic_);
    LibXR::Topic::ASyncSubscriber<Referee::RobotGameRefereePack> referee(
        self->referee_topic_);
    gyro.StartWaiting();
    euler.StartWaiting();
    launcher.StartWaiting();
    referee.StartWaiting();
    while (true) {
      const auto RX_RESULT = self->link_.Poll();
      UNUSED(RX_RESULT);
      if (gyro.Available()) {
        self->gyro_ = gyro.GetData();
        gyro.StartWaiting();
      }
      if (euler.Available()) {
        self->euler_ = euler.GetData();
        euler.StartWaiting();
      }
      if (launcher.Available()) {
        self->bullet_speed_ = launcher.GetData().ld.bullet_speed;
        launcher.StartWaiting();
      }
      if (referee.Available()) {
        self->color_ = referee.GetData().robot_status.robot_id >= 101
                           ? Pldx::Talos::Color::BLUE
                           : Pldx::Talos::Color::RED;
        referee.StartWaiting();
      }
      self->ConsumeInput();
      self->SendFeedback();
      self->thread_.Sleep(2);
    }
  }

  void ConsumeInput() {
    CMD::Data data{};
    const bool following = cmd_->GetCtrlMode() == CMD::Mode::CMD_AUTO_CTRL;
    {
      LibXR::Mutex::LockGuard lock(input_mutex_);
      following_ = following;
      const auto now = LibXR::Timebase::GetMilliseconds();
      if (vision_online_ && (now - last_vision_time_).ToMillisecond() <= 150U)
        data = pending_;
      if (stop_pending_) data.launcher.isfire = false;
      stop_pending_ = false;
    }
    data.ctrl_source = CMD::ControlSource::CTRL_SOURCE_AI;
    cmd_->FeedAIGimbal(data);
  }

  void OnVision(const Pldx::Talos::SendVisionData& frame) {
    CMD::Data data{};
    data.ctrl_source = CMD::ControlSource::CTRL_SOURCE_AI;
    data.gimbal.yaw = frame.data.target_yaw;
    data.gimbal.pit = frame.data.target_pitch;
    data.gimbal.yaw_dot = frame.data.ref_yaw_v;
    data.gimbal.pit_dot = frame.data.ref_pitch_v;
    data.gimbal.yaw_ddot = frame.data.ref_yaw_a;
    data.gimbal.pit_ddot = frame.data.ref_pitch_a;
    data.gimbal_online = true;
    data.launcher.isfire = frame.data.fire_advice;
    LibXR::Mutex::LockGuard lock(input_mutex_);
    pending_ = data;
    stop_pending_ |= !data.launcher.isfire;
    last_vision_time_ = LibXR::Timebase::GetMilliseconds();
    vision_online_ = true;
  }
  void OnSimpleVision(const Pldx::Talos::SendSimpleVisionData& frame) {
    CMD::Data data{};
    data.ctrl_source = CMD::ControlSource::CTRL_SOURCE_AI;
    data.gimbal.yaw = frame.data.target_yaw;
    data.gimbal.pit = euler_.Pitch();
    data.gimbal_online = true;
    LibXR::Mutex::LockGuard lock(input_mutex_);
    pending_ = data;
    stop_pending_ |= !data.launcher.isfire;
    last_vision_time_ = LibXR::Timebase::GetMilliseconds();
    vision_online_ = true;
  }
  void SendFeedback() {
    Pldx::Talos::ReceiveImuData frame{};
    frame.time_stamp =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    frame.data.self_color = color_;
    frame.data.bullet_speed = bullet_speed_;
    frame.data.yaw = euler_.Yaw();
    frame.data.pitch = euler_.Pitch();
    frame.data.roll = euler_.Roll();
    frame.data.yaw_vel = gyro_.z();
    frame.data.pitch_vel = gyro_.y();
    frame.data.roll_vel = gyro_.x();
    LibXR::WriteOperation op;
    const auto IMU_RESULT = link_.Send(frame, op);
    UNUSED(IMU_RESULT);
    if (frame.time_stamp - last_capabilities_time_ms_ >= 100U) {
      Pldx::Talos::ReceiveCapabilitiesData capabilities{};
      capabilities.time_stamp = frame.time_stamp;
      {
        LibXR::Mutex::LockGuard lock(input_mutex_);
        capabilities.data.following = following_;
      }
      const auto RESULT = link_.Send(capabilities, op);
      if (RESULT == LibXR::ErrorCode::OK)
        last_capabilities_time_ms_ = frame.time_stamp;
    }
  }

  LibXR::UART* uart_;
  CMD* cmd_;
  LibXR::Topic gyro_topic_, euler_topic_, launcher_topic_, referee_topic_;
  LibXR::Thread thread_;
  LibXR::Callback<const Pldx::Talos::SendVisionData&> vision_callback_;
  LibXR::Callback<const Pldx::Talos::SendSimpleVisionData&> simple_callback_;
  Pldx::Talos::TalosLink link_;
  Eigen::Matrix<float, 3, 1> gyro_{};
  LibXR::EulerAngle<float> euler_{};
  float bullet_speed_{};
  Pldx::Talos::Color color_ = Pldx::Talos::Color::RED;
  LibXR::MillisecondTimestamp last_vision_time_{};
  bool vision_online_ = false;
  LibXR::Mutex input_mutex_;
  CMD::Data pending_{};
  bool following_ = false;
  bool stop_pending_ = false;
  uint32_t last_capabilities_time_ms_ = 0;
};

#endif  // TALOS_LINK_PROTOCOL_ONLY
