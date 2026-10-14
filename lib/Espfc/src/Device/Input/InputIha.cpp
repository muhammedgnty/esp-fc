#include "Device/Input/InputIha.hpp"
#include "Hal/FastCode.hpp"

namespace Espfc::Device::Input {

void InputIha::begin()
{
  for (size_t i = 0; i < CHANNELS; i++)
  {
    _channels[i] = i == 2 ? 1000 : 1500;
    _shared[i].store(_channels[i], std::memory_order_relaxed);
  }
  // AUX1 is the arm switch: off until the link says otherwise.
  _channels[4] = 1000;
  _shared[4].store(1000, std::memory_order_relaxed);
}

void InputIha::publish(const uint16_t* us, size_t len)
{
  for (size_t i = 0; i < len && i < CHANNELS; i++)
  {
    _shared[i].store(us[i], std::memory_order_relaxed);
  }
  _writeCount.fetch_add(1, std::memory_order_release);
}

InputStatus FAST_CODE_ATTR InputIha::update()
{
  const uint32_t count = _writeCount.load(std::memory_order_acquire);
  if (count == _readCount) return INPUT_IDLE;
  _readCount = count;
  for (size_t i = 0; i < CHANNELS; i++)
  {
    _channels[i] = _shared[i].load(std::memory_order_relaxed);
  }
  return INPUT_RECEIVED;
}

uint16_t FAST_CODE_ATTR InputIha::get(uint8_t i) const
{
  return _channels[i];
}

void FAST_CODE_ATTR InputIha::get(uint16_t* data, size_t len) const
{
  for (size_t i = 0; i < len; i++)
  {
    data[i] = i < CHANNELS ? _channels[i] : 1500;
  }
}

size_t InputIha::getChannelCount() const
{
  return CHANNELS;
}

bool InputIha::needAverage() const
{
  return false;
}

} // namespace Espfc::Device::Input
