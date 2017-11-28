#pragma once

#include <vector>
#include <functional>

#include <ESP8266WiFi.h>

#include "Router.h"

class Mesh {
public:
  using Datagram = std::vector<uint8_t>;
  using DatagramHandler = std::function<void(Datagram)>;
  
  Mesh(const DatagramHandler& handler);

  void begin();

  void run();
private:
  static const uint16_t PORT = 1245;
  static const int HELLO_PERIOD = 1000;
  static const int ROUTE_PERIOD = 2500;

  void processWiFiScan(int networkCount);

  DatagramHandler handler;
  Router router;

  WiFiUDP socket;

  unsigned long nextHelloTime;
  unsigned long nextRouteTime;
};
