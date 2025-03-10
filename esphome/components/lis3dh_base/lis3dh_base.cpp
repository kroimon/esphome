#include <cmath>
#include <cstdint>

#include "lis3dh_base.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <esphome/components/sensor/sensor.h>
#include <esphome/core/component.h>

namespace esphome {
namespace lis3dh_base {

static const char *const TAG = "lis3dh.sensor";

static const uint8_t LIS3DH_REGISTER_STATUS_REG_AUX = 0x07;
static const uint8_t LIS3DH_REGISTER_OUT_ADC1_L = 0x08;
static const uint8_t LIS3DH_REGISTER_OUT_ADC1_H = 0x09;
static const uint8_t LIS3DH_REGISTER_OUT_ADC2_L = 0x0A;
static const uint8_t LIS3DH_REGISTER_OUT_ADC2_H = 0x0B;
static const uint8_t LIS3DH_REGISTER_OUT_ADC3_L = 0x0C;
static const uint8_t LIS3DH_REGISTER_OUT_ADC3_H = 0x0D;
static const uint8_t LIS3DH_REGISTER_WHO_AM_I = 0x0F;
static const uint8_t LIS3DH_REGISTER_CTRL_REG0 = 0x1E;
static const uint8_t LIS3DH_REGISTER_TEMP_CFG_REG = 0x1F;
static const uint8_t LIS3DH_REGISTER_CTRL_REG1 = 0x20;
static const uint8_t LIS3DH_REGISTER_CTRL_REG2 = 0x21;
static const uint8_t LIS3DH_REGISTER_CTRL_REG3 = 0x22;
static const uint8_t LIS3DH_REGISTER_CTRL_REG4 = 0x23;
static const uint8_t LIS3DH_REGISTER_CTRL_REG5 = 0x24;
static const uint8_t LIS3DH_REGISTER_CTRL_REG6 = 0x25;
static const uint8_t LIS3DH_REGISTER_REFERENCE = 0x26;
static const uint8_t LIS3DH_REGISTER_STATUS_REG = 0x27;
static const uint8_t LIS3DH_REGISTER_OUT_X_L = 0x28;
static const uint8_t LIS3DH_REGISTER_OUT_X_H = 0x29;
static const uint8_t LIS3DH_REGISTER_OUT_Y_L = 0x2A;
static const uint8_t LIS3DH_REGISTER_OUT_Y_H = 0x2B;
static const uint8_t LIS3DH_REGISTER_OUT_Z_L = 0x2C;
static const uint8_t LIS3DH_REGISTER_OUT_Z_H = 0x2D;
static const uint8_t LIS3DH_REGISTER_FIFO_CTRL_REG = 0x2E;
static const uint8_t LIS3DH_REGISTER_FIFO_SRC_REG = 0x2F;
static const uint8_t LIS3DH_REGISTER_INT1_CFG = 0x30;
static const uint8_t LIS3DH_REGISTER_INT1_SRC = 0x31;
static const uint8_t LIS3DH_REGISTER_INT1_THS = 0x32;
static const uint8_t LIS3DH_REGISTER_INT1_DURATION = 0x33;
static const uint8_t LIS3DH_REGISTER_INT2_CFG = 0x34;
static const uint8_t LIS3DH_REGISTER_INT2_SRC = 0x35;
static const uint8_t LIS3DH_REGISTER_INT2_THS = 0x36;
static const uint8_t LIS3DH_REGISTER_INT2_DURATION = 0x37;
static const uint8_t LIS3DH_REGISTER_CLICK_CFG = 0x38;
static const uint8_t LIS3DH_REGISTER_CLICK_SRC = 0x39;
static const uint8_t LIS3DH_REGISTER_CLICK_THS = 0x3A;
static const uint8_t LIS3DH_REGISTER_TIME_LIMIT = 0x3B;
static const uint8_t LIS3DH_REGISTER_TIME_LATENCY = 0x3C;
static const uint8_t LIS3DH_REGISTER_TIME_WINDOW = 0x3D;
static const uint8_t LIS3DH_REGISTER_ACT_THS = 0x3D;
static const uint8_t LIS3DH_REGISTER_INACT_DUR = 0x3D;

static const uint8_t LIS3DH_WHO_AM_I = 0x33;

union lis3dh_ctrl_reg1 {
  uint8_t raw;
  struct {
    uint8_t Xen : 1;         // X-axis enable
    uint8_t Yen : 1;         // Y-axis enable
    uint8_t Zen : 1;         // Z-axis enable
    uint8_t LPen : 1;        // Low power mode enable
    LIS3DHDataRate ODR : 4;  // Data rate selection
  } __attribute__((packed)) reg;
};

union lis3dh_ctrl_reg2 {
  uint8_t raw;
  struct {
    uint8_t HP_IA1 : 1;   // High-pass filter enabled for AOI function on interrupt 1
    uint8_t HP_IA2 : 1;   // High-pass filter enabled for AOI function on interrupt 2
    uint8_t HPCLICK : 1;  // High-pass filter enabled for CLICK function
    uint8_t FDS : 1;      // Filtered data selection
    uint8_t HPCF : 2;     // High-pass filter cutoff frequency selection
    uint8_t HPM : 2;      // High-pass filter mode selection
  } __attribute__((packed));
};

union lis3dh_ctrl_reg3 {
  uint8_t raw;
  struct {
    uint8_t unused : 1;      // unused
    uint8_t I1_OVERRUN : 1;  // FIFO overrun interrupt on INT1
    uint8_t I1_WTM : 1;      // FIFO watermark interrupt on INT1
    uint8_t IT_321DA : 1;    // 321DA interrupt on INT1
    uint8_t IT_ZYXDA : 1;    // ZYXDA interrupt on INT1
    uint8_t I1_IA2 : 1;      // IA2 interrupt on INT1
    uint8_t I1_IA1 : 1;      // IA1 interrupt on INT1
    uint8_t I1_CLICK : 1;    // Click interrupt on INT1
  } __attribute__((packed));
};

union lis3dh_ctrl_reg4 {
  uint8_t raw;
  struct {
    uint8_t SIM : 1;  // SPI serial interface mode selection
    uint8_t ST : 2;   // Self-test enable
    uint8_t HR : 1;   // High-resolution output mode
    uint8_t FS : 2;   // Full-scale selection
    uint8_t BLE : 1;  // Big/little endian data selection
    uint8_t BDU : 1;  // Block data update
  } __attribute__((packed));
};

union lis3dh_ctrl_reg5 {
  uint8_t raw;
  struct {
    uint8_t D4D_INT2 : 1;  // 4D detection is enabled on INT2
    uint8_t LIR_INT2 : 1;  // Latch interrupt request on INT2_SRC register
    uint8_t D4D_INT1 : 1;  // 4D detection is enabled on INT1
    uint8_t LIR_INT1 : 1;  // Latch interrupt request on INT1_SRC register
    uint8_t unused : 2;    // unused
    uint8_t FIFO_EN : 1;   // FIFO enable
    uint8_t BOOT : 1;      // Reboot memory content
  } __attribute__((packed));
};

union lis3dh_ctrl_reg6 {
  uint8_t raw;
  struct {
    uint8_t unused1 : 1;       // unused
    uint8_t INT_POLARITY : 1;  // INT1 and INT2 pin polarity
    uint8_t unused2 : 1;       // unused
    uint8_t I2_ACT : 1;        // Enable activity interrupt on INT2 pin
    uint8_t I2_BOOT : 1;       // Enable boot on INT2 pin
    uint8_t I2_IA2 : 1;        // Enable interrupt 2 function on INT2 pin
    uint8_t I2_IA1 : 1;        // Enable interrupt 1 function on INT2 pin
    uint8_t I2_CLICK : 1;      // Click interrupt on INT2 pin
  } __attribute__((packed));
};

union lis3dh_status_reg {
  uint8_t raw;
  struct {
    uint8_t XDA : 1;    //  X-axis new data available
    uint8_t YDA : 1;    //  Y-axis new data available
    uint8_t ZDA : 1;    //  Z-axis new data available
    uint8_t ZYXDA : 1;  //  X, Y and Z-axis new data available
    uint8_t XOR : 1;    //  X-axis data overrun
    uint8_t YOR : 1;    //  Y-axis data overrun
    uint8_t ZOR : 1;    //  Z-axis data overrun
    uint8_t ZYXOR : 1;  //  X, Y and Z-axis data overrun
  } __attribute__((packed));
};

float LIS3DHComponent::get_setup_priority() const { return setup_priority::DATA; }

void LIS3DHComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up LIS3DH...");

  // Read chip id
  uint8_t chip_id = 0;
  if (!this->read_byte(LIS3DH_REGISTER_WHO_AM_I, &chip_id)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }
  if (chip_id != LIS3DH_WHO_AM_I) {
    this->error_code_ = WRONG_CHIP_ID;
    this->mark_failed();
    return;
  }

  // Enable all axes and set data rate
  lis3dh_ctrl_reg1 ctrl_reg1 = {.bits = {.Xen = 1, .Yen = 1, .Zen = 1}};
  ctrl_reg1.Xen = 1;
  ctrl_reg1.Yen = 1;
  ctrl_reg1.Zen = 1;
  ctrl_reg1.ODR = LIS3DHDataRate::NORMAL_400HZ;
  if (!this->write_byte(LIS3DH_REGISTER_CTRL_REG1, ctrl_reg1.raw)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }
}

void LIS3DHComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "LIS3DH:");
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with LIS3DH failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
}

}  // namespace lis3dh_base
}  // namespace esphome
