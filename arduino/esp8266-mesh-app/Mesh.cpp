#include "Mesh.h"

#include <algorithm>

Mesh::Mesh(const DatagramHandler& _handler)
  : handler{_handler}
  , status{-1}
  , nextHelloTime{0}
  , nextRouteTime{0}
  , nextConnectTime{0}
  , nextPingTime{0}
  , pingState{-1}
  , routeSeqNum{0} {
}

void Mesh::begin() {
  pinMode(VERBOSE_PIN, INPUT);
  digitalWrite(VERBOSE_PIN, HIGH);
  
  WiFi.disconnect();
  
  Router::ChipID myID;
  
  WiFi.macAddress(myID.data());
  router.begin(myID);

  int seed = (myID[3] << 16) | (myID[4] << 8) | myID[5];
  srand(seed);

  setRandomSubnet();

  Serial.print("[Info] SoftAP Default Gateway: ");
  Serial.println(WiFi.softAPIP());

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(chipIDToSSID(router.getChipID()).c_str());

  if(socket.begin(PORT) == 0) {
    Serial.println("[Error] Failed to start listening on UDP socket");
  }
}

void Mesh::run() {
  auto curTime = millis();

  if(curTime >= nextHelloTime) {
    auto newStatus = WiFi.status();
    if(newStatus != status) {
      status = newStatus;

      Serial.print("[Info] WiFi Station status update: ");
      if(status == WL_DISCONNECTED)
        Serial.println("Disconnected");
      else if(status == WL_CONNECT_FAILED)
        Serial.println("Connect Failed");
      else if(status == WL_CONNECTED)
        Serial.println("Connected");
      else if(status == WL_IDLE_STATUS)
        Serial.println("Idle");
      else if(status == WL_CONNECTION_LOST)
        Serial.println("Connection Lost");
      else if(status == WL_SCAN_COMPLETED)
        Serial.println("Scan completed");
      else
        Serial.println(status);

      if(status == WL_CONNECTED) {
        apNeighbor.nextHop = getDefaultGateway();

        Serial.print("[Info] AP Neighbor: ");
        Serial.print(apNeighbor.nextHop);
        Serial.print("\t");
        Serial.print(Router::chipIDToString(apNeighbor.link.target));
        Serial.print("\t");
        Serial.println(apNeighbor.link.cost);

        uint8_t mySubnet = WiFi.softAPIP()[2];
        uint8_t apNeighborSubnet = apNeighbor.nextHop[2];

        if(mySubnet == apNeighborSubnet) {
          Serial.println("[Info] AP Neighbor using same subnet. Selecting new subnet");
          setRandomSubnet();
        }
      }
    }
    
    nextHelloTime = (nextHelloTime == 0 ? curTime : nextHelloTime) + HELLO_PERIOD;

    //Prune inactive neighbors
    for(int i = 0; i < stationNeighbors.size(); ++i) {
      stationNeighbors[i].ttl--;
      if(stationNeighbors[i].ttl == 0) {
        Serial.print("[Info] Neighbor has timed out: ");
        Serial.println(Router::chipIDToString(stationNeighbors[i].route.link.target));
        
        stationNeighbors.erase(stationNeighbors.begin() + i);
        --i;
      }
    }

    if(status == WL_CONNECTED) {
      //Update link cost
      apNeighbor.link.cost = -WiFi.RSSI();
      
      //Send Hello packet
      auto id = router.getChipID();
      std::vector<uint8_t> packet(id.begin(), id.end());
      packet.push_back(apNeighbor.link.cost);
      packet.push_back(0);
      
      sendPacket(apNeighbor.nextHop, packet, Type::Hello);
    }
  }

  if(pingState != -1 && curTime >= nextPingTime) {
    nextPingTime = curTime + PING_PERIOD;

    if(pingState > 0) {
      ping();
      pingState--;
    }
    else {
      int responses = std::count(pingTimes.begin(), pingTimes.end(), 0);
      float avgTime = (responses == 0) ? 0.f : totalPingTime / (1000.f*responses);

      Serial.print("Ping Report - ");
      Serial.println(Router::chipIDToString(pingQueue.front()));
      Serial.print("\t");
      Serial.print(pingTimes.size());
      Serial.print(" packets transmitted, ");
      Serial.print(responses);
      Serial.print(" received, ");
      Serial.print(100 - 100*responses/pingTimes.size());
      Serial.print("% packet loss, average rtt = ");
      Serial.print(avgTime);
      Serial.println(" ms\n");

      pingQueue.pop();
      if(pingQueue.empty()) {
        pingState = -1;

        Serial.println("-----Network Test Complete-----");
      }
      else {
        pingTimes.clear();
        totalPingTime = 0;

        Serial.print("Ping - ");
        Serial.println(Router::chipIDToString(pingQueue.front()));
        
        ping();
        pingState = 4;
      }
    }
  }
  
  if(curTime >= nextRouteTime) {
    nextRouteTime = (nextRouteTime == 0 ? curTime : nextRouteTime) + ROUTE_PERIOD;

    std::vector<Router::Route> neighbors;
    neighbors.reserve(stationNeighbors.size() + (status == WL_CONNECTED));

    std::vector<Router::Link> links;
    neighbors.reserve(neighbors.size());
    
    for(const auto& neighbor : stationNeighbors) {
      neighbors.push_back(neighbor.route);
      links.push_back(neighbor.route.link);
    }
    if(status == WL_CONNECTED) {
      neighbors.push_back(apNeighbor);
      links.push_back(apNeighbor.link);
    }

    std::vector<uint8_t> routePacket;
    routePacket.reserve(8 + 8*links.size());

    auto myID = router.getChipID();
    auto linkPtr = reinterpret_cast<uint8_t*>(links.data());
    routePacket.insert(routePacket.end(), myID.begin(), myID.end());
    routePacket.push_back(routeSeqNum >> 8);
    routePacket.push_back(routeSeqNum & 0xFF);
/*
    Serial.println("[Info] Routing update: ");
    for(const auto& element : routePacket) {
      Serial.print((int)element, HEX);
      Serial.print(" ");
    }
    Serial.println();
*/
    routePacket.insert(routePacket.end(), linkPtr, linkPtr + 8*links.size());
/*
    Serial.print("[Info] Sending routing update to neighbors (");
    Serial.print(routeSeqNum);
    Serial.println(")");
*/
    sendPacketToNeighbors(routePacket, Type::RouteUpdate);

    routeSeqNum++;

//    Serial.println("[Info] Updating routing table");
    router.updateNeighbors(neighbors);
    router.updateRoutingTable();

    if(digitalRead(VERBOSE_PIN) == LOW) {
      router.printNetworkGraph();

      router.printRoutingTable();
      Serial.println();
    }
  }

  if(curTime >= nextConnectTime) {
    if(WiFi.status() != WL_CONNECTED) {
      Serial.println("[Info] Performing WiFi Scan");
      
      processWiFiScan(WiFi.scanNetworks());
    }

    nextConnectTime = curTime + CONNECT_PERIOD;
  }

  int length;
  while((length = socket.parsePacket()) > 0) {
    if(length < 4) {
      Serial.println("[Error] Received packet with of length < 4");
    }
    else {
      uint8_t type = socket.read();
      uint8_t hopCount = socket.read();

      //Discard padding
      socket.read();
      socket.read();

      std::vector<uint8_t> packet(socket.available());
      
      if(socket.available() > 0) {
        socket.read(packet.data(), packet.size());
      }

      switch(type) {
        case static_cast<int>(Type::Hello):
          processHelloPacket(packet);
        break;

        case static_cast<int>(Type::RouteUpdate):
          processRoutePacket(packet);
        break;

        case static_cast<int>(Type::Ping):
          processPingPacket(packet);
        break;

        case static_cast<int>(Type::Datagram):
          if(packet.size() >= sizeof(Router::ChipID)) {
            Router::ChipID target;
            std::copy(packet.begin(), packet.begin() + sizeof(Router::ChipID), target.begin());

            if(target == router.getChipID()) {
              packet.erase(packet.begin(), packet.begin() + sizeof(Router::ChipID));
              handler(packet);
            }
            else {
              sendPacketToNeighbors(packet, static_cast<Type>(type), hopCount++);
            }
          }
          else {
            Serial.print("[Error] Packet of type Datagram received with length < ");
            Serial.println(sizeof(Router::ChipID));
          }
        break;

        default:
          Serial.print("[Error] Datagram with invalid type received: ");
          Serial.println(static_cast<int>(type));
        break;
      }
    }
  }
}

void Mesh::runNetworkTest() {
  router.printNetworkGraph();
  router.printRoutingTable();
  Serial.println();
  
  if(pingQueue.empty()) {
    Serial.println("-----Starting Network Test-----");
    
    auto nodes = router.getAllNodes();

    if(!nodes.empty()) {
      for(const auto& node : nodes) {
        pingQueue.push(node);
      }
      pingTimes.clear();
      totalPingTime = 0;
  
      Serial.print("Ping - ");
      Serial.println(Router::chipIDToString(pingQueue.front()));
      
      ping();
      pingState = 4;
    }
    else {
      Serial.println("-----Network Test Complete-----");
    }
  }
}

void Mesh::ping() {
  if(pingQueue.empty()) {
    Serial.println("[Error] Ping(): pingQueue is empty");
  }
  else {
    auto me = router.getChipID();
    const auto& target = pingQueue.front();
    
    std::vector<uint8_t> packet;
  
    packet.push_back(pingTimes.size());
    packet.insert(packet.end(), target.begin(), target.end());
    packet.insert(packet.end(), me.begin(), me.end());

    IPAddress nextHop;
    if(router.getNextHop(nextHop, target)) {
      sendPacket(nextHop, packet, Type::Ping);
      pingTimes.push_back(micros());
    }
    else {
      Serial.print("Ping - No route to device ");
      Serial.println(Router::chipIDToString(target));
    }
  }
}

void Mesh::setRandomSubnet() {
  uint8_t subnet = rand() % 255;
  Serial.print("[Info] Selected new subnet ");
  Serial.println(static_cast<int>(subnet));

  IPAddress newGateway{192, 168, subnet, 1};
  WiFi.softAPConfig(newGateway, newGateway, {255, 255, 255, 0});
}

void Mesh::processHelloPacket(const std::vector<uint8_t>& packet) {
  if(packet.size() != 8) {
    Serial.print("[Error] Packet of type Hello received with invalid length ");
    Serial.println(packet.size());
  }
  else {
    Router::ChipID chipID;
    std::copy(packet.begin(), packet.begin() + 6, chipID.begin());

    uint8_t cost = packet[6];

    auto neighbor = std::find_if(stationNeighbors.begin(), stationNeighbors.end(), [&chipID](const Neighbor& n) {
        return n.route.link.target == chipID;
      });
    if(neighbor == stationNeighbors.end()) {
      Serial.print("[Info] Found new neighbor: ");
      Serial.println(Router::chipIDToString(chipID));
      
      stationNeighbors.push_back({{{chipID, cost}, socket.remoteIP()}, NEIGHBOR_MAX_TTL});
    }
    else {
      neighbor->ttl = NEIGHBOR_MAX_TTL;
      neighbor->route.link.cost = cost;
    }
  }
}

void Mesh::processRoutePacket(const std::vector<uint8_t>& packet) {
/*
  Serial.println("[Info] Received Route Packet: ");
  for(const auto& element : packet) {
    Serial.print((int)element, HEX);
    Serial.print(" ");
  }
  Serial.println();
*/

  if(packet.size() < 8) {
    Serial.print("[Error] Route Update packet received with invalid length (");
    Serial.print(packet.size());
    Serial.println(")");
  }
  else {
    Router::ChipID id;
    std::copy(packet.begin(), packet.begin() + 6, id.begin());
    uint16_t seqNum = (packet[6] << 8) | packet[7];

    bool alreadySeen = false;
    auto lastSeq = routeSeqTable.find(id);
    if(lastSeq == routeSeqTable.end()) {
      routeSeqTable[id] = seqNum;
    }
    else if(lastSeq->second == seqNum) {
      alreadySeen = true;
    }
    else {
      lastSeq->second = seqNum;
    }

    if(alreadySeen) {
/*
      Serial.print("[Info] Discarding stale routing update from ");
      Serial.print(Router::chipIDToString(id));
      Serial.print(" (");
      Serial.print(seqNum);
      Serial.println(")");
*/
    }
    else {
/*
      Serial.print("[Info] Received routing update from ");
      Serial.println(Router::chipIDToString(id));
*/
      router.processLinkUpdate(id, reinterpret_cast<const Router::Link*>(packet.data() + 8),
        packet.size()/8 - 1);

      //Forward to other neighbors
      sendPacketToNeighbors(packet, Type::RouteUpdate, id);
    }
  }
}

void Mesh::processPingPacket(const std::vector<uint8_t>& packet) {
  auto curTime = micros();
  
  Router::ChipID sender, target, me = router.getChipID();

  std::copy(packet.begin()+1, packet.begin()+7, target.begin());
  std::copy(packet.begin()+7, packet.begin()+13, sender.begin());

  bool toForward = true;;

  if(target == me) {
    if(sender == me) {
      toForward = false;
      int seqNum = packet[0];

      if(seqNum >= pingTimes.size()) {
        Serial.println("[Error] Ping: Invalid sequence number");
      }
      else {
        if(pingTimes[seqNum] == 0) {
          Serial.print("[Info] Ping: Received duplicate response (");
          Serial.print((int)seqNum);
          Serial.println(")");
        }
        else {
          auto dt = curTime - pingTimes[seqNum];
          
          Serial.print("\tPing response (");
          Serial.print((int)seqNum);
          Serial.print("): ");
          Serial.print(dt/1000.f);
          Serial.print("ms\n\tRoute: ");

          int hopCount = (packet.size()-7)/6;
          for(int i = 0; i < hopCount; ++i) {
            Router::ChipID hop;
            int offset = 7+6*i;
            std::copy(packet.begin()+offset, packet.begin()+offset+6, hop.begin());

            Serial.print(Router::chipIDToString(hop));
            Serial.print("->");
          }
          Serial.println(Router::chipIDToString(me));
          Serial.println();
  
          pingTimes[seqNum] = 0;
          totalPingTime += dt;
        }
      }
    }
    else {
      target = sender;
      toForward = true;
    }
  }

  if(toForward) {
    std::vector<uint8_t> newPacket = packet;
    std::copy(target.begin(), target.end(), newPacket.begin()+1);
    newPacket.insert(newPacket.end(), me.begin(), me.end());
  
    Serial.print("[Info] Ping: Forwarding packet (from ");
    Serial.print(Router::chipIDToString(sender));
    Serial.print(" to ");
    Serial.print(Router::chipIDToString(target));
    Serial.println(")");
  
    IPAddress nextHop;
    if(router.getNextHop(nextHop, target)) {
      sendPacket(nextHop, newPacket, Type::Ping);
    }
    else {
      Serial.print("Ping Forward - No route to device ");
      Serial.println(Router::chipIDToString(target));
    }
  }
}

void Mesh::processWiFiScan(int networkCount) {
  ssids.clear();
  
  for(int i = 0; i < networkCount; ++i) {
    Router::ChipID id;

    if(ssidToChipID(id, WiFi.SSID(i))) {
      SSID ssid{id, -WiFi.RSSI(i)};

      auto insertPos = std::find_if(ssids.begin(), ssids.end(), [&ssid](const SSID& other) {
          return other.cost > ssid.cost;
        });
      ssids.insert(insertPos, ssid);
    }
  }

  Serial.print("[Info] WiFi Scan - Found ");
  Serial.print(ssids.size());
  Serial.print(" Mesh Nodes (out of ");
  Serial.print(networkCount);
  Serial.println(" SSIDs scanned)");

  if(!ssids.empty()) {
    Serial.println("\tChipID\t\t\tCost");

    for(const auto& ssid : ssids) {
      Serial.print(String("\t") + Router::chipIDToString(ssid.id) + "\t");
      Serial.println(ssid.cost);
    }

    auto connectSSID = chipIDToSSID(ssids[0].id);
    Serial.print("[Info] Connecting to lowest cost device ");
    Serial.print(Router::chipIDToString(ssids[0].id));
    Serial.print(" (");
    Serial.print(connectSSID);
    Serial.println(")");

    WiFi.begin(connectSSID.c_str());
    apNeighbor.link.target = ssids[0].id;
    apNeighbor.link.cost = ssids[0].cost;
  }
}

void Mesh::sendPacketToNeighbors(const std::vector<uint8_t>& packet, Type type, uint8_t hopCount) {
  for(auto& neighbor : stationNeighbors) {
    sendPacket(neighbor.route.nextHop, packet, type, hopCount);
  }
  if(status == WL_CONNECTED) {
    sendPacket(apNeighbor.nextHop, packet, type, hopCount);
  }
}

void Mesh::sendPacketToNeighbors(const std::vector<uint8_t>& packet, Type type, const Router::ChipID& except, uint8_t hopCount) {
  for(auto& neighbor : stationNeighbors) {
    if(neighbor.route.link.target != except) {
      sendPacket(neighbor.route.nextHop, packet, type, hopCount);
    }
  }
  if(status == WL_CONNECTED) {
    if(apNeighbor.link.target != except) {
      sendPacket(apNeighbor.nextHop, packet, type, hopCount);
    }
  }
}

void Mesh::sendPacket(const IPAddress& ip, const std::vector<uint8_t>& packet, Type type, uint8_t hopCount) {
  socket.beginPacket(ip, PORT);

  //Header
  socket.write(static_cast<uint8_t>(type));
  socket.write(hopCount);

  //Padding
  socket.write(static_cast<uint8_t>(0));
  socket.write(static_cast<uint8_t>(0));

  //Packet
  socket.write(packet.data(), packet.size());

  socket.endPacket();
}

IPAddress Mesh::getDefaultGateway() {
  auto myIP = WiFi.localIP();
  myIP[3] = 1;
  
  return myIP;
}

bool Mesh::ssidToChipID(Router::ChipID& out, const String& ssid) {
  String start{SSID_START};
  
  if( (ssid.length() == (start.length() + 12+5))
    && (ssid.substring(0, start.length()) == start)) {

    Serial.print("[Info] SSID String: ");
    Serial.println(ssid.substring(start.length()));

    out = Router::stringToChipID(ssid.substring(start.length()));
    return true;
  }
  else {
    return false;
  }
}

String Mesh::chipIDToSSID(const Router::ChipID& id) {
  return String(SSID_START) + Router::chipIDToString(id);
}

