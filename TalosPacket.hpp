#pragma once
#include <cstdint>

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
}  // namespace Pldx::Talos
