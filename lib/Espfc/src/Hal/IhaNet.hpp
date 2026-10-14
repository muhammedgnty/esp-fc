#pragma once

#if defined(ESPFC_IHA_LINK)

#include <cstddef>
#include <cstdint>
#include <vehicle_protocol.h>

namespace Espfc::Hal {

// Network side of the phone link: own Wi-Fi access point, TCP session, MAVLink and diagnostics over UDP.
class IhaNet : public iha::Transport
{
public:
  static constexpr uint16_t DIAG_PORT = 14560;

  // Starts the access point IHA-XXXX (last MAC digits) and opens the sockets.
  bool begin(const char* password);
  const char* ssid() const
  {
    return _ssid;
  }

  // Waits at most timeoutMs for traffic and hands it to the protocol.
  void poll(iha::Protocol& protocol, uint32_t timeoutMs);

  // A line for the PC diagnostics tool, sent to whoever last sent a datagram to DIAG_PORT.
  void sendDiag(const char* line);

  void sendLine(const char* line) override;
  void sendDatagram(const uint8_t* data, size_t len) override;

  static uint64_t nowUs();

private:
  void closeSession(iha::Protocol& protocol);
  void readSession(iha::Protocol& protocol, uint64_t now);

  static constexpr size_t MAX_LINE = 1024;

  char _ssid[16] = {0};
  int _server = -1;
  int _udp = -1;
  int _diag = -1;
  int _client = -1;
  uint32_t _clientIp = 0;
  uint16_t _appPort = 0;
  uint32_t _diagIp = 0;
  uint16_t _diagPort = 0;
  char _line[MAX_LINE + 2];
  size_t _lineLen = 0;
  bool _lineOverflow = false;
};

} // namespace Espfc::Hal

#endif
