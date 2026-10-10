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
  TalosLink link(
      &state,
      [](void* context, const SendVisionData& FRAME) {
        auto& state = *static_cast<State*>(context);
        ++state.full;
        state.yaw = FRAME.data.target_yaw;
      },
      [](void* context, const SendSimpleVisionData& FRAME) {
        auto& state = *static_cast<State*>(context);
        ++state.simple;
        state.yaw = FRAME.data.target_yaw;
      });
  SendVisionData frame{};
  frame.header = {0x5A, 29, 0x02};
  frame.eof = 0xA5;
  frame.data.target_yaw = 1.25f;
  const auto FULL = bytes_of(frame);
  link.Feed(std::span(FULL).first(7));
  assert(state.full == 0);
  link.Feed(std::span(FULL).subspan(7));
  assert(state.full == 1 && state.yaw == 1.25f);
  auto bad = FULL;
  bad.back() = 0;
  std::vector<uint8_t> stream{0x11, 0x5A, 0xFF, 0x02};
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), FULL.begin(), FULL.end());
  stream.insert(stream.end(), FULL.begin(), FULL.end());
  link.Feed(stream);
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
  for (auto byte : bytes_of(simple)) link.Feed(std::span(&byte, 1));
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
  link.Feed(noise);
  link.Feed(FULL);
  assert(state.full == 4);
}
