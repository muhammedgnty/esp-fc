#pragma once

#include "Device/InputDevice.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace Espfc::Device::Input {

// RC channels produced by the phone link task (Connect/IhaLink) on another core.
// A frame read while the next one is being published may mix both; they are 20 ms apart.
class InputIha : public InputDevice
{
public:
  static constexpr size_t CHANNELS = 8;

  void begin();
  void publish(const uint16_t* us, size_t len);

  InputStatus update() override;
  uint16_t get(uint8_t i) const override;
  void get(uint16_t* data, size_t len) const override;
  size_t getChannelCount() const override;
  bool needAverage() const override;

private:
  std::atomic<uint16_t> _shared[CHANNELS]{};
  std::atomic<uint32_t> _writeCount{0};
  uint32_t _readCount = 0;
  uint16_t _channels[CHANNELS]{};
};

} // namespace Espfc::Device::Input
