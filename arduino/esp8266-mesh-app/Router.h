#pragma once

#include <array>
#include <vector>

#include <Arduino.h>
#include <ESP8266WiFi.h>

class Router {
public:
  using ChipID = std::array<uint8_t, 6>;
  using Address = IPAddress;
  using CostType = uint16_t;

  struct Link {
    ChipID target;
    CostType cost;
  };
  
  struct Route {
    Link link;
    Address nextHop;
  };

  Router();

  void begin(const ChipID& id);

  ChipID getChipID() const;

  bool getNextHop(Address& nextHop, const ChipID& id) const;

  //Update information about this device's own neighbor routes
  void updateNeighbors(const std::vector<Route>& neighbors);
  
  //Instructs the router to recreate routing table
  //using Dijkstra's algorithm
  void updateRoutingTable();

  //Process update from neighbor device
  void processLinkUpdate(const ChipID& fromID, const Link* links, size_t length);

  void exportNeighborTable(std::vector<Link>& neighbors) const;

  std::vector<Route> getRoutingTable() const;

  void printRoutingTable() const;
  void printNetworkGraph() const;

  static String chipIDToString(const ChipID& id);
  static ChipID stringToChipID(const String& str);

private:
  struct Node {
    struct Edge {
      size_t nodeIndex;
      CostType cost;
    };

    Node(const ChipID& id);

    ChipID id;
    std::vector<Edge> edges;

    bool mark, processed;
    CostType cost;
    int prev;
  };

  size_t getNodeIndex(const ChipID& id) const;

  ChipID myID;

  std::vector<Node> networkGraph;
  std::vector<Route> routingTable;
  std::vector<Route> neighbors;
};
