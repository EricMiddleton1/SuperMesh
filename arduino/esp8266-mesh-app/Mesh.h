#pragma once

#include <vector>
#include <map>
#include <queue>
#include <functional>

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiType.h>
#include <WiFiUdp.h>

extern "C" {
  #include "os_type.h"
}

#include "Router.h"

class Mesh {
public:
  using Datagram = std::vector<uint8_t>;
  using DatagramHandler = std::function<void(Datagram)>;
  
  Mesh(const DatagramHandler& handler);

  void begin();

  void runNetworkTest();

  void run();
private:
  static const uint16_t PORT = 1245;
  static const int HELLO_PERIOD = 1000;
  static const int ROUTE_PERIOD = 5000;
  static const int CONNECT_PERIOD = 15000;
  static const int PING_PERIOD = 1000;
  static const int NEIGHBOR_MAX_TTL = 3;
  static const int VERBOSE_PIN = 2;
  static constexpr char* SSID_START = "esp8266-mesh-";
  static const int CHANNEL = 6;

  enum class Type {
    Hello = 0x00,
    RouteUpdate = 0x01,
    Ping = 0x02,
    Datagram = 0x10
  };

  struct Neighbor {
    Router::Route route;
    int ttl;
  };

  struct SSID {
    Router::ChipID id;
    Router::CostType cost;
  };

  /******WiFi Event Callbacks******/
  /*
  void cbStationModeDisconnected(const WiFiEventStationModeDisconnected&);
  void cbStationModeGotIP(const WiFiEventModeGotIP&);
  void cbSoftAPModeStationConnected(const WiFiEventSoftAPModeStationConnected&);
  void cbSoftAPModeStationDisconnected(const WiFiEventSoftAPModeStationDisconnected&);
  */
  void cbStationScan(void *bssInfo, STATUS status);

  /******Timer Callbacks******/
  void cbHelloTimer();
  void cbRouteTimer();

  void setRandomSubnet();

  void ping();

  void processHelloPacket(const std::vector<uint8_t>& packet);
  void processRoutePacket(const std::vector<uint8_t>& packet);
  void processPingPacket(const std::vector<uint8_t>& packet);

  void sendPacketToNeighbors(const std::vector<uint8_t>& packet, Type type, uint8_t hopCount = 0);
  void sendPacketToNeighbors(const std::vector<uint8_t>& packet, Type type, const Router::ChipID& ignore, uint8_t hopCount = 0);
  void sendPacket(const IPAddress& ip, const std::vector<uint8_t>& packet, Type type, uint8_t hopCount = 0);

  static IPAddress getDefaultGateway();

  static bool ssidToChipID(Router::ChipID& out, const String& ssid);
  static String chipIDToSSID(const Router::ChipID& id);

  Router::ChipID myID;

  DatagramHandler handler;
  Router router;

  std::vector<Neighbor> stationNeighbors;
  Router::Route apNeighbor;

  std::vector<SSID> ssids;

  WiFiUDP socket;

  int status;

  os_timer_t helloTimer, routeTimer, pingTimer;

  unsigned long nextConnectTime;
  unsigned long nextPingTime;

  uint16_t routeSeqNum;

  int pingState;
  std::vector<unsigned long long> pingTimes;
  int totalPingTime;
  std::queue<Router::ChipID> pingQueue;

  std::map<Router::ChipID, uint16_t> routeSeqTable;
};
