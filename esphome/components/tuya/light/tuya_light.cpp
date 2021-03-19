#include "esphome/core/log.h"
#include "tuya_light.h"

namespace esphome {
namespace tuya {

static const char *TAG = "tuya.light";

void TuyaLight::setup() {
  if (this->dimmer_id_.has_value()) {
    this->parent_->register_listener(*this->dimmer_id_, [this](TuyaDatapoint datapoint) {
      auto call = this->state_->make_call();
      call.set_brightness(float(datapoint.value_uint) / this->max_value_);
      call.perform();
    });
  }
  if (switch_id_.has_value()) {
    this->parent_->register_listener(*this->switch_id_, [this](TuyaDatapoint datapoint) {
      auto call = this->state_->make_call();
      call.set_state(datapoint.value_bool);
      call.perform();
    });
  }
  if (min_value_datapoint_id_.has_value()) {
    parent_->set_datapoint_value(*this->min_value_datapoint_id_, this->min_value_);
  }
}

void TuyaLight::dump_config() {
  ESP_LOGCONFIG(TAG, "Tuya Dimmer:");
  if (this->dimmer_id_.has_value())
    ESP_LOGCONFIG(TAG, "   Dimmer has datapoint ID %u", *this->dimmer_id_);
  if (this->switch_id_.has_value())
    ESP_LOGCONFIG(TAG, "   Switch has datapoint ID %u", *this->switch_id_);
}

light::LightTraits TuyaLight::get_traits() {
  auto traits = light::LightTraits();
  traits.set_supports_brightness(this->dimmer_id_.has_value());
  return traits;
}

void TuyaLight::setup_state(light::LightState *state) { state_ = state; }

void TuyaLight::write_state(light::LightState *state) {
  float brightness;
  state->current_values_as_brightness(&brightness);

  if (brightness == 0.0f) {
    // turning off, first try via switch (if exists), then dimmer
    if (switch_id_.has_value()) {
      parent_->set_datapoint_value(*this->switch_id_, false);
    } else if (dimmer_id_.has_value()) {
      parent_->set_datapoint_value(*this->dimmer_id_, 0);
    }
    return;
  }

  auto brightness_int = static_cast<uint32_t>(brightness * this->max_value_);
  brightness_int = std::max(brightness_int, this->min_value_);

  if (this->dimmer_id_.has_value()) {
    parent_->set_datapoint_value(*this->dimmer_id_, brightness_int);
  }
  if (this->switch_id_.has_value()) {
    parent_->set_datapoint_value(*this->switch_id_, true);
  }
}

}  // namespace tuya
}  // namespace esphome
