#include <cstddef>
#include <cstdint>

#include "lis3dh_i2c.h"

namespace esphome {
namespace lis3dh_i2c {

static const uint8_t LIS3DH_I2C_AUTO_INCREMENT = 0x80;

void LIS3DHI2CComponent::dump_config() {
  LIS3DHComponent::dump_config();
  LOG_I2C_DEVICE(this);
}

bool LIS3DHI2CComponent::read_byte(uint8_t a_register, uint8_t *data) { return I2CDevice::read_byte(a_register, data); }

bool LIS3DHI2CComponent::write_byte(uint8_t a_register, uint8_t data) {
  return I2CDevice::write_byte(a_register, data);
}

bool LIS3DHI2CComponent::read_bytes(uint8_t a_register, uint8_t *data, size_t len) {
  if (len > 1)
    a_register |= LIS3DH_I2C_AUTO_INCREMENT;
  return I2CDevice::read_bytes(a_register, data, len);
}

bool LIS3DHI2CComponent::read_byte_16(uint8_t a_register, uint16_t *data) {
  return I2CDevice::read_byte_16(a_register | LIS3DH_I2C_AUTO_INCREMENT, data);
}

}  // namespace lis3dh_i2c
}  // namespace esphome
