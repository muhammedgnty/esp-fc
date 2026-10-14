#if defined(ESPFC_IHA_LINK)

#include "Connect/IhaLink.hpp"
#include "Hal/PulseProbe.hpp"
#include "Hal/Task.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <rc_channels.h>

#define ESPFC_IHA_VERSION "iha 0.1.0"

#ifndef IHA_AP_PASSWORD
#error "IHA_AP_PASSWORD must be set in platformio.ini"
#endif

namespace Espfc::Connect {

namespace {

constexpr uint32_t DIAG_INTERVAL_US = 200000;

iha::ProtocolConfig protocolConfig()
{
  iha::ProtocolConfig c;
  c.firmware = "esp-fc " ESPFC_IHA_VERSION;
  return c;
}

const char* armBlockReason(uint32_t flags)
{
  if (flags & ARMING_DISABLED_NO_GYRO) return "gyro yok";
  if (flags & (ARMING_DISABLED_CALIBRATING | ARMING_DISABLED_ACC_CALIBRATION)) return "kalibrasyon sürüyor";
  if (flags & ARMING_DISABLED_CLI) return "CLI açık";
  if (flags & ARMING_DISABLED_MSP) return "Configurator bağlı";
  if (flags & ARMING_DISABLED_MOTOR_PROTOCOL) return "ESC protokolü kapalı";
  if (flags & ARMING_DISABLED_REBOOT_REQUIRED) return "yeniden başlatın";
  if (flags & ARMING_DISABLED_ANGLE) return "araç eğik";
  if (flags & ARMING_DISABLED_THROTTLE) return "gaz yüksek";
  if (flags & (ARMING_DISABLED_FAILSAFE | ARMING_DISABLED_RX_FAILSAFE)) return "failsafe";
  return nullptr;
}

} // namespace

FcVehicle::FcVehicle(Model& model, Device::Input::InputIha& input): _model(model), _input(input)
{
  const iha::RcChannels idle;
  for (size_t i = 0; i < Device::Input::InputIha::CHANNELS; i++)
  {
    _channels[i] = i < iha::RcChannels::kCount ? idle.us[i] : 1500;
  }
}

int FcVehicle::measuredThrottle() const
{
  if (!_fcArmed) return 0;
  const float thrust = _model.state.output.ch[AXIS_THRUST]; // -1..1
  return std::clamp(static_cast<int>(lrintf((thrust + 1.f) * 500.f)), 0, 1000);
}

bool FcVehicle::baroHealthy() const
{
  return _model.baroActive() && std::isfinite(_model.state.altitude.height) &&
         std::isfinite(_model.state.altitude.vario);
}

void FcVehicle::syncArming(uint32_t dtMs)
{
  _fcArmed = _model.isModeActive(MODE_ARMED);
  if (!isArmed())
  {
    _armPendingMs = 0;
    _wasArmed = false;
    return;
  }
  if (_fcArmed)
  {
    if (!_wasArmed) _armHeight = _model.state.altitude.height;
    _wasArmed = true;
    return;
  }
  if (_wasArmed)
  {
    disarm("Uçuş kontrolcüsü motorları kapattı");
    return;
  }
  _armPendingMs += dtMs;
  if (_armPendingMs < ARM_TIMEOUT_MS) return;
  char text[64];
  const uint32_t flags = _model.state.mode.armingDisabledFlags;
  const char* reason = armBlockReason(flags);
  if (reason)
  {
    snprintf(text, sizeof(text), "Arm reddedildi: %s", reason);
  }
  else
  {
    snprintf(text, sizeof(text), "Arm reddedildi (0x%lx)", static_cast<unsigned long>(flags));
  }
  disarm(text);
}

void FcVehicle::step(uint32_t dtMs)
{
  iha::VehicleSensors sensors;
  sensors.baroOk = baroHealthy();
  sensors.throttle = measuredThrottle();
  sensors.landed = sensors.baroOk && std::fabs(_model.state.altitude.vario) < LANDED_VARIO &&
                   _model.state.altitude.height - _armHeight < LANDED_HEIGHT;
  update(sensors, dtMs);
  syncArming(dtMs);

  iha::RcChannels rc = iha::RcChannels::from(*this);
  // Until esp-fc has switched to ALT HOLD the throttle channel is still absolute: hold the current throttle.
  if (climbRateControl() && !_model.isModeActive(MODE_ALTHOLD))
  {
    rc.us[iha::RcChannels::kThrottle] = iha::RcChannels::throttleUs(sensors.throttle);
  }
  std::copy(rc.us, rc.us + iha::RcChannels::kCount, _channels);
  _input.publish(_channels, Device::Input::InputIha::CHANNELS);
}

iha::VehicleTelemetry FcVehicle::telemetry() const
{
  iha::VehicleTelemetry t;
  if (baroHealthy())
  {
    t.altitudeCm = (_model.state.altitude.height - (_wasArmed ? _armHeight : 0.f)) * 100.f;
    t.varioCms = _model.state.altitude.vario * 100.f;
  }
  // MANUAL: the accumulated throttle the app shows, without esp-fc's input smoothing lag.
  t.throttle = climbRateControl() ? measuredThrottle() : commandThrottle();
  t.batteryMv = std::max(0.f, _model.state.battery.voltage * 1000.f);
  t.armingDisabled = _model.state.mode.armingDisabledFlags;
  return t;
}

IhaLink::IhaLink(Model& model, Device::Input::InputIha& input)
    : _model(model), _vehicle(model, input), _protocol(_vehicle, _net, protocolConfig())
{
}

void IhaLink::begin()
{
  _vehicle.setTextCallback(onText, this);
  // Core 1 next to the gyro task, which preempts it; core 0 runs the pid loop and Wi-Fi.
  Hal::Task::create(taskMain, "ihaLink", 8192, this, Hal::Task::Priority::Low, Hal::CORE_COUNT > 1 ? 1 : 0);
}

void IhaLink::onText(void* ctx, const char* text)
{
  auto* self = static_cast<IhaLink*>(ctx);
  self->_model.logger.info().log("IHA").logln(text);
  iha::Protocol::simText(&self->_protocol, text);
}

void IhaLink::taskMain(void* arg)
{
  static_cast<IhaLink*>(arg)->run();
}

void IhaLink::run()
{
  const bool ok = _net.begin(IHA_AP_PASSWORD);
  _model.logger.info().log("IHA AP").log(_net.ssid()).logln(ok);

  const int8_t* pins = &_model.config.pin[PIN_OUTPUT_0];
  Hal::PulseProbe::begin(pins, Hal::PulseProbe::MAX_PINS);

  constexpr uint64_t tickUs = iha::Vehicle::kTickMs * 1000;
  uint64_t nextTick = Hal::IhaNet::nowUs();
  for (;;)
  {
    const uint64_t now = Hal::IhaNet::nowUs();
    if (now >= nextTick)
    {
      _protocol.tick(now);
      nextTick += tickUs;
      // After a long stall do not try to catch up tick by tick.
      if (now > nextTick + 100000) nextTick = now + tickUs;
      if (now - _lastDiagUs >= DIAG_INTERVAL_US) sendDiag(now);
      continue;
    }
    _net.poll(_protocol, static_cast<uint32_t>((nextTick - now + 999) / 1000));
  }
}

// One JSON line for firmware/tools/iha_diag.py.
void IhaLink::sendDiag(uint64_t now)
{
  const uint32_t intervalMs = static_cast<uint32_t>((now - _lastDiagUs) / 1000);
  _lastDiagUs = now;
  const auto& s = _model.state;
  const uint16_t* ch = _vehicle.channels();
  Hal::PulseProbe::Reading p[Hal::PulseProbe::MAX_PINS];
  for (size_t i = 0; i < Hal::PulseProbe::MAX_PINS; i++) p[i] = Hal::PulseProbe::read(i, intervalMs);
  const iha::VehicleTelemetry t = _vehicle.telemetry();
  char line[512];
  snprintf(line, sizeof(line),
           "{\"ms\":%lu,\"state\":%d,\"mode\":%d,\"fc_armed\":%d,\"fc_modes\":%lu,\"arm_flags\":%lu,"
           "\"rc\":[%u,%u,%u,%u,%u,%u],\"motor\":[%d,%d,%d,%d],"
           "\"pin\":[%d,%d,%d,%d],\"pulse_us\":[%u,%u,%u,%u],\"pulse_hz\":[%u,%u,%u,%u],"
           "\"thr\":%d,\"alt_cm\":%d,\"vario_cms\":%d,\"baro\":%d,\"vbat_mv\":%d,"
           "\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"fresh\":%lu,\"stale\":%lu,"
           "\"press_pa\":%.1f,\"baro_alt\":%.2f,\"temp\":%.2f,\"i2c_err\":%lu}",
           static_cast<unsigned long>(now / 1000), static_cast<int>(_vehicle.state()),
           static_cast<int>(_vehicle.mode()), _model.isModeActive(MODE_ARMED) ? 1 : 0,
           static_cast<unsigned long>(s.mode.mask), static_cast<unsigned long>(s.mode.armingDisabledFlags), ch[0],
           ch[1], ch[2], ch[3], ch[4], ch[5], s.output.us[0], s.output.us[1], s.output.us[2], s.output.us[3],
           _model.config.pin[PIN_OUTPUT_0], _model.config.pin[PIN_OUTPUT_1], _model.config.pin[PIN_OUTPUT_2],
           _model.config.pin[PIN_OUTPUT_3], p[0].widthUs, p[1].widthUs, p[2].widthUs, p[3].widthUs, p[0].hz, p[1].hz,
           p[2].hz, p[3].hz, t.throttle, static_cast<int>(lrintf(t.altitudeCm)), static_cast<int>(lrintf(t.varioCms)),
           _vehicle.baroOk() ? 1 : 0, static_cast<int>(t.batteryMv), Utils::toDeg(s.attitude.euler[AXIS_ROLL]),
           Utils::toDeg(s.attitude.euler[AXIS_PITCH]), Utils::toDeg(s.attitude.euler[AXIS_YAW]),
           static_cast<unsigned long>(_protocol.freshCommands()),
           static_cast<unsigned long>(_protocol.staleCommands()), s.baro.pressureRaw, s.baro.altitudeRaw,
           s.baro.temperature, static_cast<unsigned long>(s.i2cErrorCount));
  _net.sendDiag(line);
}

} // namespace Espfc::Connect

#endif
