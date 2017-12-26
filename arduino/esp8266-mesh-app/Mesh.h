#pragma once

#include <array>
#include <vector>
#include <map>
#include <queue>
#include <functional>

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiType.h>
#include <ESPAsyncUDP.h>

extern "C" {
  #include "os_type.h"
}

#include "Router.h"
#include "Message.h"
#include "Types.h"

class Mesh {
public:
  using MessageHandler = std::function<void(Message)>;
  
  Mesh();

  void begin(const String& meshName);

  void onMessage(const MessageHandler& handler);

  bool send(const Message& msg, const ChipID& destination);

  static String chipIDToString(const ChipID& id);
  static ChipID stringToChipID(const String& str);
  
private:
  static const uint16_t PORT = 1245;
  static const int HELLO_PERIOD = 1000;
  static const int ROUTE_PERIOD = 5000;
  static const int SCAN_PERIOD = 2000;
  static const int CONNECT_TIMEOUT = 15000;
  static const int NEIGHBOR_MAX_TTL = 3;
  static const int CHANNEL = 6;

  static const int TYPE_HELLO = 0x00;
  static const int TYPE_ROUTE = 0x01;
  static const int TYPE_PING = 0x02;
  static const int TYPE_DESTINATION_UNREACHABLE = 0x03;
  static const int TYPE_MESSAGE = 0x10;

  enum WiFiState {
    Disconnected,
    Connecting,
    Connected
  };

  struct Neighbor {
    Router::Route route;
    int ttl;
  };

  struct SSID {
    ChipID id;
    Router::CostType cost;
  };

  /******WiFi Event Callbacks******/
  /*
  void cbSoftAPModeStationConnected(const WiFiEventSoftAPModeStationConnected&);
  void cbSoftAPModeStationDisconnected(const WiFiEventSoftAPModeStationDisconnected&);
  */

  /******Callbacks******/
  void cbStationModeDisconnected(const WiFiEventStationModeDisconnected&);
  void cbStationModeGotIP(const WiFiEventStationModeGotIP&);
  void cbHelloTimer();
  void cbRouteTimer();
  void cbScanTimer();
  void cbStationScan(void *bssInfo, STATUS status);
  void processHelloPacket(Message msg);
  void processRoutePacket(Message msg);

  void startScan();
  void connect(const String& ssid);

  bool send(const Message& msg, const ChipID& destination, uint8_t type);
  void sendToNeighbors(const Message& msg, uint8_t type, const ChipID& except);

  void processPacket(AsyncUDPPacket packet);
  bool isDuplicate(const ChipID& id, uint16_t sequenceNumber) const;

  void registerHandler(uint8_t type, const MessageHandler& handler);

  void setRandomSubnet();

  static IPAddress getDefaultGateway();

  bool ssidToChipID(ChipID& out, const String& ssid);
  String chipIDToSSID(const ChipID& id);

  static bool isBroadcast(const ChipID& id);

  /*
   * Routing and connection-related variables
   */
  Router router;
  ChipID myID;
  std::vector<Neighbor> stationNeighbors;
  Router::Route apNeighbor;
  std::vector<SSID> ssids;
  std::map<ChipID, uint16_t> sequenceTable;
  WiFiState wifiState;
  WiFiEventHandler connectHandler, disconnectHandler;

  String meshName_;

  std::map<uint8_t, MessageHandler> messageHandlers;

  AsyncUDP socket;

  os_timer_t helloTimer, routeTimer, scanTimer;
};
