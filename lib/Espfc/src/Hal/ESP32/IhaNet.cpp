#if defined(ESP32) && defined(ESPFC_IHA_LINK)

#include "Hal/IhaNet.hpp"
#include <WiFi.h>
#include <cstdio>
#include <cstring>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <lwip/sockets.h>

namespace {

constexpr uint8_t AP_CHANNEL = 6;
constexpr uint8_t AP_MAX_CLIENTS = 3; // phone, a second phone that gets "busy", diagnostics PC

int openSocket(int type, uint16_t port)
{
  const int fd = socket(AF_INET, type, 0);
  if (fd < 0) return -1;
  const int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port);
  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
  {
    close(fd);
    return -1;
  }
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
  return fd;
}

void sendAll(int fd, const char* data, size_t len)
{
  while (len > 0)
  {
    const int n = send(fd, data, len, 0);
    if (n <= 0) return; // a full send buffer means the app is gone; the session check closes it
    data += n;
    len -= n;
  }
}

// Writes to a socket that is about to be closed.
class OneShot : public iha::Transport
{
public:
  explicit OneShot(int fd): _fd(fd) {}
  void sendLine(const char* line) override
  {
    sendAll(_fd, line, strlen(line));
    sendAll(_fd, "\n", 1);
  }
  void sendDatagram(const uint8_t*, size_t) override {}

private:
  int _fd;
};

} // namespace

namespace Espfc::Hal {

uint64_t IhaNet::nowUs()
{
  return static_cast<uint64_t>(esp_timer_get_time());
}

bool IhaNet::begin(const char* password)
{
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  uint8_t mac[6];
  WiFi.softAPmacAddress(mac);
  snprintf(_ssid, sizeof(_ssid), "IHA-%02X%02X", mac[4], mac[5]);
  if (!WiFi.softAP(_ssid, password, AP_CHANNEL, 0, AP_MAX_CLIENTS)) return false;
  // Power save adds tens of ms of latency on the control link.
  esp_wifi_set_ps(WIFI_PS_NONE);
  // Full power (19.5 dBm) disturbs the barometer next to the board: pressure noise 17 Pa (about 1.5 m),
  // 4 Pa at 8.5 dBm (measured with a BMP280 module, 2026-10-14). The pilot is near the drone anyway.
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  _server = openSocket(SOCK_STREAM, iha::ProtocolConfig::kTcpPort);
  _udp = openSocket(SOCK_DGRAM, iha::ProtocolConfig::kUdpPort);
  _diag = openSocket(SOCK_DGRAM, DIAG_PORT);
  if (_server < 0 || _udp < 0 || _diag < 0) return false;
  return listen(_server, 2) == 0;
}

void IhaNet::closeSession(iha::Protocol& protocol)
{
  close(_client);
  _client = -1;
  _appPort = 0;
  protocol.onSessionClosed();
}

void IhaNet::poll(iha::Protocol& protocol, uint32_t timeoutMs)
{
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(_server, &fds);
  FD_SET(_udp, &fds);
  FD_SET(_diag, &fds);
  int maxFd = std::max(_server, std::max(_udp, _diag));
  if (_client >= 0)
  {
    FD_SET(_client, &fds);
    maxFd = std::max(maxFd, _client);
  }
  timeval tv{static_cast<time_t>(timeoutMs / 1000), static_cast<suseconds_t>((timeoutMs % 1000) * 1000)};
  if (select(maxFd + 1, &fds, nullptr, nullptr, &tv) <= 0) return;
  const uint64_t now = nowUs();

  if (FD_ISSET(_server, &fds))
  {
    sockaddr_in peer{};
    socklen_t len = sizeof(peer);
    const int c = accept(_server, reinterpret_cast<sockaddr*>(&peer), &len);
    if (c >= 0)
    {
      // The app behind a silent session crashed or left the Wi-Fi; let the new one in.
      if (_client >= 0 && protocol.sessionIdle(now)) closeSession(protocol);
      if (_client >= 0)
      {
        OneShot busy(c);
        iha::Protocol::rejectBusy(busy);
        close(c);
      }
      else
      {
        const int one = 1;
        setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        fcntl(c, F_SETFL, fcntl(c, F_GETFL, 0) | O_NONBLOCK);
        _client = c;
        _clientIp = peer.sin_addr.s_addr;
        _lineLen = 0;
        _lineOverflow = false;
        protocol.onSessionOpened(now);
      }
    }
  }

  if (_client >= 0 && FD_ISSET(_client, &fds)) readSession(protocol, now);

  if (FD_ISSET(_udp, &fds))
  {
    uint8_t buf[512];
    for (;;)
    {
      const int n = recv(_udp, buf, sizeof(buf), MSG_DONTWAIT);
      if (n <= 0) break;
      protocol.onDatagram(buf, static_cast<size_t>(n), now);
    }
  }

  if (FD_ISSET(_diag, &fds))
  {
    uint8_t buf[64];
    sockaddr_in peer{};
    socklen_t len = sizeof(peer);
    while (recvfrom(_diag, buf, sizeof(buf), MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&peer), &len) > 0)
    {
      _diagIp = peer.sin_addr.s_addr;
      _diagPort = ntohs(peer.sin_port);
      len = sizeof(peer);
    }
  }
}

void IhaNet::readSession(iha::Protocol& protocol, uint64_t now)
{
  char buf[256];
  const int n = recv(_client, buf, sizeof(buf), MSG_DONTWAIT);
  if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
  {
    closeSession(protocol);
    return;
  }
  for (int i = 0; i < n; i++)
  {
    if (buf[i] == '\n')
    {
      if (!_lineOverflow)
      {
        _line[_lineLen] = '\0';
        protocol.onSessionLine(_line, now);
        if (protocol.streaming() && _appPort == 0) _appPort = protocol.appUdpPort();
      }
      _lineLen = 0;
      _lineOverflow = false;
    }
    else if (_lineLen < MAX_LINE)
    {
      _line[_lineLen++] = buf[i];
    }
    else
    {
      _lineOverflow = true; // drop the oversized line
    }
  }
}

void IhaNet::sendLine(const char* line)
{
  if (_client < 0) return;
  // One write, so the line goes out in one segment.
  char out[MAX_LINE + 2];
  const size_t len = std::min(strlen(line), MAX_LINE);
  memcpy(out, line, len);
  out[len] = '\n';
  sendAll(_client, out, len + 1);
}

void IhaNet::sendDatagram(const uint8_t* data, size_t len)
{
  if (_appPort == 0) return;
  sockaddr_in to{};
  to.sin_family = AF_INET;
  to.sin_addr.s_addr = _clientIp;
  to.sin_port = htons(_appPort);
  sendto(_udp, data, len, 0, reinterpret_cast<sockaddr*>(&to), sizeof(to));
}

void IhaNet::sendDiag(const char* line)
{
  if (_diagPort == 0) return;
  sockaddr_in to{};
  to.sin_family = AF_INET;
  to.sin_addr.s_addr = _diagIp;
  to.sin_port = htons(_diagPort);
  sendto(_diag, line, strlen(line), 0, reinterpret_cast<sockaddr*>(&to), sizeof(to));
}

} // namespace Espfc::Hal

#endif
