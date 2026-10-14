#if defined(ESP32) && defined(ESPFC_IHA_LINK) && !defined(ESP32C3) && !defined(ESP32S2)

#include "Hal/PulseProbe.hpp"
#include <driver/mcpwm.h>
#include <esp_attr.h>
#include <esp_rom_gpio.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_sig_map.h>
#include <soc/gpio_struct.h>

namespace {

struct Channel
{
  volatile uint32_t rise = 0;
  volatile uint32_t width = 0; // APB ticks
  volatile uint32_t pulses = 0;
  uint32_t readPulses = 0;
};

Channel channels[Espfc::Hal::PulseProbe::MAX_PINS];

constexpr mcpwm_unit_t UNIT[] = {MCPWM_UNIT_0, MCPWM_UNIT_0, MCPWM_UNIT_0, MCPWM_UNIT_1};
constexpr mcpwm_capture_channel_id_t CAP[] = {MCPWM_SELECT_CAP0, MCPWM_SELECT_CAP1, MCPWM_SELECT_CAP2,
                                              MCPWM_SELECT_CAP0};
constexpr uint32_t SIGNAL[] = {PWM0_CAP0_IN_IDX, PWM0_CAP1_IN_IDX, PWM0_CAP2_IN_IDX, PWM1_CAP0_IN_IDX};
constexpr uint32_t APB_TICKS_PER_US = 80;

bool IRAM_ATTR onEdge(mcpwm_unit_t, mcpwm_capture_channel_id_t, const cap_event_data_t* e, void* arg)
{
  Channel& c = *static_cast<Channel*>(arg);
  if (e->cap_edge == MCPWM_POS_EDGE)
  {
    c.rise = e->cap_value;
  }
  else
  {
    c.width = e->cap_value - c.rise;
    c.pulses = c.pulses + 1;
  }
  return false;
}

} // namespace

namespace Espfc::Hal {

bool PulseProbe::begin(const int8_t* pins, size_t count)
{
  bool ok = true;
  for (size_t i = 0; i < count && i < MAX_PINS; i++)
  {
    if (pins[i] < 0) continue;
    mcpwm_capture_config_t conf{};
    conf.cap_edge = MCPWM_BOTH_EDGE;
    conf.cap_prescale = 1;
    conf.capture_cb = onEdge;
    conf.user_data = &channels[i];
    ok &= mcpwm_capture_enable_channel(UNIT[i], CAP[i], &conf) == ESP_OK;
    // Not mcpwm_gpio_init(): it would turn the motor output into an input.
    gpio_ll_input_enable(&GPIO, static_cast<gpio_num_t>(pins[i]));
    esp_rom_gpio_connect_in_signal(pins[i], SIGNAL[i], false);
  }
  return ok;
}

PulseProbe::Reading PulseProbe::read(size_t index, uint32_t intervalMs)
{
  if (index >= MAX_PINS || intervalMs == 0) return {0, 0};
  Channel& c = channels[index];
  const uint32_t pulses = c.pulses;
  const uint32_t n = pulses - c.readPulses;
  c.readPulses = pulses;
  if (n == 0) return {0, 0};
  return {static_cast<uint16_t>(c.width / APB_TICKS_PER_US), static_cast<uint16_t>(n * 1000 / intervalMs)};
}

} // namespace Espfc::Hal

#endif
