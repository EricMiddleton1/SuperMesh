#include "Mesh.h"

#include <algorithm>

extern "C" {
  #include "user_interface.h"
  #include "osapi.h"
}

static Mesh* __meshPtr = nullptr;

Mesh::Mesh() {
}

void Mesh::begin(const String& meshName) {
  __meshPtr = this;

  meshName_ = meshName;

  //Load device ID
  WiFi.macAddress(myID.data());  

  //Seed RNG from unique device ID
  int seed = (myID[3] << 16) | (myID[4] << 8) | myID[5];
  srand(seed);

  //Initialize the router subsystem
  router.begin(myID);

  /**************************
  ****Wi-Fi Initialization***
  **************************/
  wifiState = WiFiState::Disconnected;
  disconnectHandler = WiFi.onStationModeDisconnected([this](const WiFiEventStationModeDisconnected& arg) {
    cbStationModeDisconnected(arg);
  });

  connectHandler = WiFi.onStationModeGotIP([this](const WiFiEventStationModeGotIP& arg) {
    cbStationModeGotIP(arg);
  });
  
  WiFi.disconnect();
  WiFi.persistent(false);

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(chipIDToSSID(myID).c_str(), "", CHANNEL);
  setRandomSubnet();

  startScan();

  Serial.print("[Info] SoftAP Default Gateway: ");
  Serial.println(WiFi.softAPIP());

  /**************************
  ****Timer Initialization***
  **************************/
  os_timer_setfn(&helloTimer, [](void* arg) { reinterpret_cast<Mesh*>(arg)->cbHelloTimer(); }, this);
  os_timer_arm(&helloTimer, HELLO_PERIOD, 1);
  os_timer_setfn(&routeTimer, [](void* arg) { reinterpret_cast<Mesh*>(arg)->cbRouteTimer(); }, this);
  os_timer_arm(&routeTimer, ROUTE_PERIOD, 1);
  os_timer_setfn(&scanTimer, [](void* arg) { reinterpret_cast<Mesh*>(arg)->cbScanTimer(); }, this);
  os_timer_arm(&scanTimer, SCAN_PERIOD, 1);
  
  if(socket.listen(PORT)) {
    socket.onPacket([this](AsyncUDPPacket p) { processPacket(std::move(p)); });
  }
  else {
    Serial.println("[Error] Failed to start listening on UDP socket");
  }

  registerHandler(TYPE_HELLO, [this](Message msg) { processHelloPacket(msg); });
  registerHandler(TYPE_ROUTE, [this](Message msg) { processRoutePacket(msg); });
}

void Mesh::onMessage(const MessageHandler& handler) {
  registerHandler(TYPE_MESSAGE, handler);
}

void Mesh::registerHandler(uint8_t type, const MessageHandler& handler) {
  messageHandlers[type] = handler;
}

bool Mesh::send(const Message& msg, const ChipID& destination) {
  send(msg, destination, TYPE_MESSAGE);
}

void Mesh::cbHelloTimer() {
  /*****Prune inactive neighbors*****/
  for(int i = 0; i < stationNeighbors.size(); ++i) {
    stationNeighbors[i].ttl--;
    if(stationNeighbors[i].ttl == 0) {
      Serial.print("[Info] Neighbor has timed out: ");
      Serial.println(chipIDToString(stationNeighbors[i].route.link.target));
      
      stationNeighbors.erase(stationNeighbors.begin() + i);
      --i;
    }
  }

  /*****Send Hello packet to neighbors*****/
  if(wifiState == WiFiState::Connected) {
    //Update link cost
    apNeighbor.link.cost = -WiFi.RSSI();

    Message msg;
    msg.write(apNeighbor.link.cost >> 8);
    msg.write(apNeighbor.link.cost & 0xFF);
    
    send(msg, apNeighbor.link.target, TYPE_HELLO);
  }
}

void Mesh::cbRouteTimer() {
  /*****Update Neighbor Lists*****/
  std::vector<Router::Route> neighbors;
  neighbors.reserve(stationNeighbors.size() + (wifiState == WiFiState::Connected));

  std::vector<Router::Link> links;
  links.reserve(neighbors.size() + (wifiState == WiFiState::Connected));
  
  for(const auto& neighbor : stationNeighbors) {
    neighbors.push_back(neighbor.route);
    links.push_back(neighbor.route.link);
  }
  if(wifiState == WiFiState::Connected) {
    neighbors.push_back(apNeighbor);
    links.push_back(apNeighbor.link);
  }

  /*****Send routing information flood*****/
  /*
  std::vector<uint8_t> routePacket;
  routePacket.reserve(8 + 8*links.size());

  auto linkPtr = reinterpret_cast<uint8_t*>(links.data());
  routePacket.insert(routePacket.end(), myID.begin(), myID.end());
  routePacket.push_back(routeSeqNum >> 8);
  routePacket.push_back(routeSeqNum & 0xFF);
  routePacket.insert(routePacket.end(), linkPtr, linkPtr + 8*links.size());
  sendPacketToNeighbors(routePacket, Type::RouteUpdate);
  routeSeqNum++;
  */

  /*****Update routing table*****/
  router.updateNeighbors(neighbors);
  router.updateRoutingTable();

  router.printNetworkGraph();
  router.printRoutingTable();
  Serial.println();

/*
  if(digitalRead(VERBOSE_PIN) == LOW) {
    router.printNetworkGraph();

    router.printRoutingTable();
    Serial.println();
  }
*/
}

void Mesh::cbStationModeDisconnected(const WiFiEventStationModeDisconnected&) {
  os_timer_disarm(&scanTimer);
  os_timer_arm(&scanTimer, SCAN_PERIOD, 1);
  
  wifiState = WiFiState::Disconnected;
  Serial.println("[Info] Station Mode Disconnected");
}

void Mesh::cbStationModeGotIP(const WiFiEventStationModeGotIP&) {
  os_timer_disarm(&scanTimer);
  os_timer_arm(&scanTimer, SCAN_PERIOD, 1);

  wifiState = WiFiState::Connected;
  Serial.println("[Info] Station Mode Connected");

  apNeighbor.nextHop = getDefaultGateway();

  Serial.print("[Info] AP Neighbor: ");
  Serial.print(apNeighbor.nextHop);
  Serial.print("\t");
  Serial.print(chipIDToString(apNeighbor.link.target));
  Serial.print("\t");
  Serial.println(apNeighbor.link.cost);

  uint8_t mySubnet = WiFi.softAPIP()[2];
  uint8_t apNeighborSubnet = apNeighbor.nextHop[2];

  if(mySubnet == apNeighborSubnet) {
    Serial.println("[Info] AP Neighbor using same subnet. Selecting new subnet");
    setRandomSubnet();
  }
}

void Mesh::startScan() {
  scan_config config {
    .ssid = nullptr,
    .bssid = nullptr,
    .channel = CHANNEL,
    .show_hidden = 0
  };
  wifi_station_scan(&config, [](void* bssInfo, STATUS status) { __meshPtr->cbStationScan(bssInfo, status); });
}

void Mesh::processPacket(AsyncUDPPacket packet) {
  const auto* data = packet.data();
  auto length = packet.length();

  if(length < 16) {
    Serial.println("[Error] Received invalid datagram");
  }
  else {
    ChipID source;
    ChipID destination;
    std::copy(data, data+6, source.begin());
    std::copy(data+6, data+12, destination.begin());
    
    uint8_t sequenceNumber = (data[12] << 8) | data[13];

    if(isDuplicate(source, sequenceNumber)) {
      Serial.print("[Warning] Rejecting duplicate packet from ");
      Serial.print(chipIDToString(source));
      Serial.print(" with sequence number ");
      Serial.println(sequenceNumber);
    }
    else {
      sequenceTable[source] = sequenceNumber;
      
      bool broadcast = isBroadcast(destination);
      uint8_t type = data[14];
  
      if(isBroadcast || destination == myID) {
        auto handler = messageHandlers.find(type);
        if(handler == messageHandlers.end()) {
          Serial.print("[Error] Message of type ");
          Serial.print(static_cast<int>(type));
          Serial.println(" received without registered handler");
        }
        else {
          (handler->second)({data+16, length-16, source, packet.remoteIP()});
        }
      }
      if(isBroadcast) {
        sendToNeighbors({data+16, length-16, source, packet.remoteIP()}, type, myID);
      }
      else if(destination != myID) {
        send({data+16, length-16, source, packet.remoteIP()}, destination, type);
      }
    }
  }
}

bool Mesh::isDuplicate(const ChipID& id, uint16_t sequenceNumber) const {
  auto found = sequenceTable.find(id);
  return (found != sequenceTable.end()) && (found->second == sequenceNumber);
}

bool Mesh::isBroadcast(const ChipID& id) {
  return id == ChipID{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
}

void Mesh::cbStationScan(void *arg, STATUS status) {
  auto* bssInfo = reinterpret_cast<bss_info*>(arg);
  int networkCount;
  
  ssids.clear();
  
  for(networkCount = 0; bssInfo != nullptr; bssInfo = STAILQ_NEXT(bssInfo, next), ++networkCount) {
    ChipID id;

    if(ssidToChipID(id, String(reinterpret_cast<char*>(bssInfo->ssid)))) {
      SSID ssid{id, -bssInfo->rssi};

      auto insertPos = std::find_if(ssids.begin(), ssids.end(), [&ssid](const SSID& other) {
          return other.cost > ssid.cost;
        });
      ssids.insert(insertPos, ssid);
    }
  }

/*
  Serial.print("[Info] WiFi Scan - Found ");
  Serial.print(ssids.size());
  Serial.print(" Mesh Nodes (out of ");
  Serial.print(networkCount);
  Serial.println(" SSIDs scanned)");

  if(!ssids.empty()) {
    Serial.println("\tChipID\t\t\tCost");

    for(const auto& ssid : ssids) {
      Serial.print(String("\t") + chipIDToString(ssid.id) + "\t");
      Serial.println(ssid.cost);
    }
  }
*/

  if(wifiState != WiFiState::Connected && !ssids.empty()) {
    connect(chipIDToSSID(ssids[0].id));

    if(wifiState == WiFiState::Disconnected) {
      wifiState = WiFiState::Connecting;
      
      os_timer_disarm(&scanTimer);
      os_timer_arm(&scanTimer, CONNECT_TIMEOUT, 1);
    }
    
    apNeighbor.link.target = ssids[0].id;
    apNeighbor.link.cost = ssids[0].cost;
  }
}

void Mesh::connect(const String& ssid) {
  Serial.print("[Info] Connecting to SSID ");
  Serial.println(ssid);

  WiFi.begin(ssid.c_str());
}

void Mesh::cbScanTimer() {
  startScan();
}

void Mesh::setRandomSubnet() {
  uint8_t subnet = rand() & 0xFF;
  Serial.print("[Info] Selected new subnet ");
  Serial.println(static_cast<int>(subnet));

  IPAddress newGateway{192, 168, subnet, 1};
  WiFi.softAPConfig(newGateway, newGateway, {255, 255, 255, 0});
}

void Mesh::processHelloPacket(Message msg) {
  if(msg.size() != 2) {
    Serial.print("[Error] Message of type Hello received with invalid length ");
    Serial.println(msg.size());
  }
  else {
    Serial.print("[Info] Received hello packet from ");
    Serial.println(chipIDToString(msg.sender()));
    
    const auto* data = msg.data();
    auto sender = msg.sender();
    uint16_t cost = (data[0] << 8) | (data[1]);

    auto neighbor = std::find_if(stationNeighbors.begin(), stationNeighbors.end(), [&sender](const Neighbor& n) {
        return n.route.link.target == sender;
      });
    if(neighbor == stationNeighbors.end()) {
      Serial.print("[Info] Found new neighbor: ");
      Serial.println(chipIDToString(sender));
      
      stationNeighbors.push_back({{{sender, cost}, msg.senderIP()}, NEIGHBOR_MAX_TTL});
    }
    else {
      neighbor->ttl = NEIGHBOR_MAX_TTL;
      neighbor->route.link.cost = cost;
    }
  }
}

void Mesh::processRoutePacket(Message msg) {
  //Check if payload size is multiple of 8
  if((msg.size() & 0x07) != 0) {
    Serial.print("[Error] Route Update packet received with invalid length (");
    Serial.print(msg.size());
    Serial.println(")");
  }
  else {
    Serial.print("[Info] Received hello packet from ");
    Serial.println(chipIDToString(msg.sender()));
    
    auto sender = msg.sender();
    
    router.processLinkUpdate(sender, reinterpret_cast<const Router::Link*>(msg.data()),
      msg.size() >> 3);
  }
}

bool Mesh::send(const Message& msg, const ChipID& destination, uint8_t type) {
  IPAddress nextHop;
  if(!router.getNextHop(nextHop, destination)) {
    Serial.print("[Error] Mesh::send: No route to destination '");
    Serial.print(chipIDToString(destination));
    Serial.println("'");

    return false;
  }
  else {
    AsyncUDPMessage msgOut(16 + msg.size());
    auto sequenceNumber = sequenceTable[myID]++;
    
    msgOut.write(myID.data(), myID.size());
    msgOut.write(destination.data(), destination.size());
    msgOut.write(sequenceNumber >> 8);
    msgOut.write(sequenceNumber & 0xFF);
    msgOut.write(type);
    msgOut.write(0);
    msgOut.write(msg.data(), msg.size());

    Serial.print("[Info] Sending packet to ");
    Serial.print(chipIDToString(destination));
    Serial.print(" (");
    Serial.print(msg.size());
    Serial.println(" bytes)");
    
    socket.sendTo(msgOut, nextHop, PORT);

    return true;
  }
}

void Mesh::sendToNeighbors(const Message& msg, uint8_t type, const ChipID& except) {
  for(auto& neighbor : stationNeighbors) {
    if(neighbor.route.link.target != except) {
      send(msg, neighbor.route.link.target, type);
    }
  }
  if(wifiState == WiFiState::Connected) {
    if(apNeighbor.link.target != except) {
      send(msg, apNeighbor.link.target, type);
    }
  }
}

IPAddress Mesh::getDefaultGateway() {
  auto myIP = WiFi.localIP();
  myIP[3] = 1;
  
  return myIP;
}

bool Mesh::ssidToChipID(ChipID& out, const String& ssid) {
  if( (ssid.length() == (meshName_.length() + 12+6))
    && (ssid.substring(0, meshName_.length()) == meshName_)) {

    out = stringToChipID(ssid.substring(meshName_.length()+1));
    return true;
  }
  else {
    return false;
  }
}

String Mesh::chipIDToSSID(const ChipID& id) {
  return meshName_ + " " + chipIDToString(id);
}

String Mesh::chipIDToString(const ChipID& id) {
  return String(id[0], HEX) + ":"
    + String(id[1], HEX) + ":"
    + String(id[2], HEX) + ":"
    + String(id[3], HEX) + ":"
    + String(id[4], HEX) + ":"
    + String(id[5], HEX);
}

ChipID Mesh::stringToChipID(const String& str) {
  ChipID id;

  for(int i = 0; i < id.size(); ++i) {
    auto substr = str.substring(3*i, 3*i+2);
    id[i] = strtol(substr.c_str(), nullptr, 16);
  }

  return id;
}

