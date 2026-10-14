#pragma once

#if defined(ESPFC_IHA_LINK)

#include "Device/Input/InputIha.hpp"
#include "Hal/IhaNet.hpp"
#include "Model.h"
#include <vehicle.h>
#include <vehicle_protocol.h>

namespace Espfc::Connect {

// The flight controller as seen by the phone protocol: VehicleLogic decides, esp-fc flies.
// Commands become RC channels (iha-link rc_channels.h); telemetry comes from the model.
class FcVehicle : public iha::Vehicle
{
public:
  FcVehicle(Model& model, Device::Input::InputIha& input);

  void step(uint32_t dtMs = kTickMs) override;
  iha::VehicleTelemetry telemetry() const override;

  // Last channels sent to esp-fc.
  const uint16_t* channels() const
  {
    return _channels;
  }

private:
  int measuredThrottle() const;
  bool baroHealthy() const;
  void syncArming(uint32_t dtMs);

  static constexpr uint32_t ARM_TIMEOUT_MS = 500;
  static constexpr float LANDED_VARIO = 0.25f; // m/s
  static constexpr float LANDED_HEIGHT = 1.0f; // m above the arming point

  Model& _model;
  Device::Input::InputIha& _input;
  uint16_t _channels[Device::Input::InputIha::CHANNELS];
  uint32_t _armPendingMs = 0;
  bool _fcArmed = false;
  bool _wasArmed = false;
  float _armHeight = 0.f;
};

// Runs the phone link in its own task: Wi-Fi access point, session, MAVLink, 50 Hz vehicle tick.
class IhaLink
{
public:
  IhaLink(Model& model, Device::Input::InputIha& input);

  // Starts the task; call after everything else is up.
  void begin();

private:
  static void taskMain(void* arg);
  void run();
  void sendDiag(uint64_t now);
  static void onText(void* ctx, const char* text);

  Model& _model;
  FcVehicle _vehicle;
  Hal::IhaNet _net;
  iha::Protocol _protocol;
  uint64_t _lastDiagUs = 0;
};

} // namespace Espfc::Connect

#endif
