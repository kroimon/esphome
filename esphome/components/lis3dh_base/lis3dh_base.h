#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace lis3dh_base {

enum class LIS3DHDataRate : uint8_t {
  POWERDOWN = 0b0000,
  NORMAL_1HZ = 0b0001,
  NORMAL_10HZ = 0b0010,
  NORMAL_25HZ = 0b0011,
  NORMAL_50HZ = 0b0100,
  NORMAL_100HZ = 0b0101,
  NORMAL_200HZ = 0b0110,
  NORMAL_400HZ = 0b0111,
  LOWPOWER_1600HZ = 0b1000,
  LOWPOWER_5376HZ = 0b1001
};

class LIS3DHComponent : public PollingComponent {
  SUB_SENSOR(accel_x)
  SUB_SENSOR(accel_y)
  SUB_SENSOR(accel_z)

 public:
  float get_setup_priority() const override;
  void setup() override;
  void dump_config() override;

  void update() override;

 protected:
  virtual bool read_byte(uint8_t a_register, uint8_t *data) = 0;
  virtual bool write_byte(uint8_t a_register, uint8_t data) = 0;
  virtual bool read_bytes(uint8_t a_register, uint8_t *data, size_t len) = 0;
  virtual bool read_byte_16(uint8_t a_register, uint16_t *data) = 0;

  enum ErrorCode {
    NONE = 0,
    COMMUNICATION_FAILED,
    WRONG_CHIP_ID,
  } error_code_{NONE};
};

}  // namespace lis3dh_base
}  // namespace esphome
