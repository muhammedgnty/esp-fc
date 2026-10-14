#pragma once

#if defined(ESPFC_IHA_LINK)

#include <cstddef>
#include <cstdint>

namespace Espfc::Hal {

// Measures the pulses on output pins from the pin itself (bench check of the motor signal, works without
// a battery). The pins stay outputs; only their input path is tapped.
class PulseProbe
{
public:
  static constexpr size_t MAX_PINS = 4;

  struct Reading
  {
    uint16_t widthUs; // 0 when no pulse was seen since the last read
    uint16_t hz;
  };

  // Call after the pins are configured as outputs.
  static bool begin(const int8_t* pins, size_t count);
  // Pulse width and rate since the previous read of the same pin.
  static Reading read(size_t index, uint32_t intervalMs);
};

} // namespace Espfc::Hal

#endif
