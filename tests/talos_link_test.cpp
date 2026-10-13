#include <cassert>
#include <limits>
#include <vector>

#include "TalosLink.hpp"

using namespace Pldx::Talos;
struct State {
  int full = 0;
  int simple = 0;
  float yaw = 0;
};
template <typename Frame>
auto bytes_of(Frame frame) {
  std::array<uint8_t, sizeof(Frame)> bytes{};
  std::memcpy(bytes.data(), &frame, sizeof(frame));
  return bytes;
}
int main() {
  State state;
  const auto VISION = TalosLink::VisionCallback::Create(
      [](bool, State* context, const SendVisionData& FRAME) {
        auto& state = *context;
        ++state.full;
        state.yaw = FRAME.data.target_yaw;
      },
      &state);
  const auto SIMPLE_VISION = TalosLink::SimpleVisionCallback::Create(
      [](bool, State* context, const SendSimpleVisionData& FRAME) {
        auto& state = *context;
        ++state.simple;
        state.yaw = FRAME.data.target_yaw;
      },
      &state);
  TalosLink link(VISION, SIMPLE_VISION);
  SendVisionData frame{};
  frame.header = {0x5A, 29, 0x02};
  frame.eof = 0xA5;
  frame.data.target_yaw = 1.25f;
  const auto FULL = bytes_of(frame);
  link.Feed({FULL.data(), 7});
  assert(state.full == 0);
  link.Feed({FULL.data() + 7, FULL.size() - 7});
  assert(state.full == 1 && state.yaw == 1.25f);
  auto bad = FULL;
  bad.back() = 0;
  std::vector<uint8_t> stream{0x11, 0x5A, 0xFF, 0x02};
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), FULL.begin(), FULL.end());
  stream.insert(stream.end(), FULL.begin(), FULL.end());
  link.Feed({stream.data(), stream.size()});
  assert(state.full == 3);
  bad = FULL;
  bad[3] = 2;
  link.Feed(bad);
  frame.data.distance = std::numeric_limits<float>::quiet_NaN();
  link.Feed(bytes_of(frame));
  assert(state.full == 3);
  SendSimpleVisionData simple{};
  simple.header = {0x5A, 4, 0x04};
  simple.eof = 0xA5;
  simple.data.target_yaw = -0.5f;
  for (auto byte : bytes_of(simple)) link.Feed({&byte, 1});
  assert(state.simple == 1 && state.yaw == -0.5f);
  ReceiveImuData imu{};
  imu.time_stamp = 0x12345678;
  const auto IMU = TalosLink::Encode(imu);
  assert(IMU.size() == 37 && IMU[0] == 0x5A && IMU[1] == 29 && IMU[2] == 1 &&
         IMU[3] == 0x78 && IMU[6] == 0x12 && IMU.back() == 0xA5);
  const auto CAPS = TalosLink::Encode(ReceiveCapabilitiesData{});
  const auto SIMPLE = TalosLink::Encode(ReceiveSimpleImuData{});
  assert(CAPS[1] == 3 && CAPS[2] == 3 && SIMPLE[1] == 13 && SIMPLE[2] == 3);
  std::vector<uint8_t> noise(10000, 0x5A);
  link.Feed({noise.data(), noise.size()});
  link.Feed(FULL);
  assert(state.full == 4);

  LibXR::ReadPort read_port(128);
  LibXR::WritePort write_port(4, 128);
  write_port = [](LibXR::WritePort&, bool) {};
  TalosLink transport(read_port, write_port, VISION, SIMPLE_VISION);
  auto publish = [&](LibXR::ConstRawData data) {
    auto queue = read_port.GetReadQueue();
    assert(queue.PushBatch(static_cast<const uint8_t*>(data.addr_),
                           data.size_) == LibXR::ErrorCode::OK);
    queue.Publish();
  };
  assert(transport.Poll() == LibXR::ErrorCode::OK);
  publish({FULL.data(), 7});
  assert(transport.Poll() == LibXR::ErrorCode::OK && state.full == 4);
  publish({FULL.data() + 7, FULL.size() - 7});
  assert(transport.Poll() == LibXR::ErrorCode::OK && state.full == 5);
  assert(read_port.Size() == 0);
  publish(FULL);
  assert(transport.Poll() == LibXR::ErrorCode::OK && state.full == 6);
  LibXR::WriteOperation operation;
  assert(transport.Send(imu, operation) == LibXR::ErrorCode::OK);
  std::array<uint8_t, sizeof(imu)> sent{};
  {
    auto queue = write_port.GetWriteQueue();
    assert(queue.AvailableSize() == IMU.size());
    queue.PopAll(sent.data());
  }
  assert(sent == IMU && write_port.Size() == 0);
  std::array<uint8_t, 128> filler{};
  assert(write_port(filler, operation) == LibXR::ErrorCode::OK);
  assert(transport.Send(imu, operation) == LibXR::ErrorCode::FULL);
  assert(write_port.Size() == filler.size());
  {
    auto queue = write_port.GetWriteQueue();
    queue.PopAll(filler.data());
  }
  assert(write_port.Size() == 0);
  assert(link.Poll() == LibXR::ErrorCode::NOT_SUPPORT);
  assert(link.Send(imu, operation) == LibXR::ErrorCode::NOT_SUPPORT);
  LibXR::ReadPort unsupported_read(0);
  LibXR::WritePort unsupported_write(0, 0);
  TalosLink unsupported(unsupported_read, unsupported_write);
  assert(unsupported.Poll() == LibXR::ErrorCode::NOT_SUPPORT);
  assert(unsupported.Send(imu, operation) == LibXR::ErrorCode::NOT_SUPPORT);

  bool isr_seen = false;
  TalosLink isr_link(TalosLink::VisionCallback::Create(
      [](bool in_isr, bool* context, const SendVisionData&) {
        *context = in_isr;
      },
      &isr_seen));
  isr_link.Feed({FULL.data(), FULL.size()}, true);
  assert(isr_seen);
  TalosLink empty_link;
  empty_link.Feed({FULL.data(), FULL.size()});
}
