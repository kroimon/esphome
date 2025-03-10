#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/lis3dh_base/lis3dh_base.h"

namespace esphome {
namespace lis3dh_i2c {

static const char *const TAG = "lis3dh_i2c.sensor";

class LIS3DHI2CComponent : public esphome::lis3dh_base::LIS3DHComponent, public i2c::I2CDevice {
 public:
  void dump_config() override;

 protected:
  bool read_byte(uint8_t a_register, uint8_t *data) override;
  bool write_byte(uint8_t a_register, uint8_t data) override;
  bool read_bytes(uint8_t a_register, uint8_t *data, size_t len) override;
  bool read_byte_16(uint8_t a_register, uint16_t *data) override;
};

}  // namespace lis3dh_i2c
}  // namespace esphome
