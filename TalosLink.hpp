#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Talos visual communication protocol over LibXR UART/USB
depends:
- pldx/CMD@master
- pldx/Referee@master
=== END MANIFEST === */
// clang-format on

#include "CMD.hpp"
#include "Referee.hpp"
#include "TalosProtocol.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"
#include "uart.hpp"

class TalosLink {
 public:
  TalosLink(
      LibXR::UART& uart, CMD& cmd, const char* gyro_topic_name = "gimbal_gyro",
      const char* euler_topic_name = "gimbal_euler",
      const char* launcher_topic_name = "launcher_ref",
      const char* referee_topic_name = "sentry_ref",
      uint32_t task_stack_depth = 1024,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM)
      : uart_(&uart),
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

  void OnMonitor() {}

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
      const auto NOW = LibXR::Timebase::GetMilliseconds();
      if (self->vision_online_ &&
          (NOW - self->last_vision_time_).ToMillisecond() > 150U) {
        CMD::Data offline{};
        offline.ctrl_source = CMD::ControlSource::CTRL_SOURCE_AI;
        self->cmd_->FeedAI(offline);
        self->vision_online_ = false;
      }
      self->SendFeedback();
      self->thread_.Sleep(2);
    }
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
    cmd_->FeedAI(data);
    last_vision_time_ = LibXR::Timebase::GetMilliseconds();
    vision_online_ = true;
  }
  void OnSimpleVision(const Pldx::Talos::SendSimpleVisionData& frame) {
    CMD::Data data{};
    data.ctrl_source = CMD::ControlSource::CTRL_SOURCE_AI;
    data.gimbal.yaw = frame.data.target_yaw;
    data.gimbal.pit = euler_.Pitch();
    data.gimbal_online = true;
    cmd_->FeedAI(data);
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
      capabilities.data.following =
          cmd_->GetCtrlMode() == CMD::Mode::CMD_AUTO_CTRL;
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
  uint32_t last_capabilities_time_ms_ = 0;
};
