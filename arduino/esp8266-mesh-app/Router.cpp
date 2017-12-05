#include "Router.h"

#include <queue>
#include <limits>
#include <algorithm>

using namespace std;

Router::Node::Node(const ChipID& _id)
  : id(_id)
  , mark{false}
  , processed{false}
  , cost{numeric_limits<CostType>::max()}
  , prev{-1} {
}

Router::Router() {
  static_assert(sizeof(Link) == 8, "sizeof(Link) != 8");
}

void Router::begin(const ChipID& id) {
  myID = id;
  networkGraph.emplace_back(myID);
}

Router::ChipID Router::getChipID() const {
  return myID;
}

bool Router::getNextHop(Address& nextHop, const ChipID& id) const {
  auto route = std::find_if(routingTable.begin(), routingTable.end(),
    [&id](const Route& r) {
      return r.link.target == id;
    });

  if(route == routingTable.end()) {
    return false;
  }
  else {
    nextHop = route->nextHop;
    return true;
  }
}

std::vector<Router::ChipID> Router::getAllNodes() const {
  std::vector<ChipID> nodes;

  for(const auto& node : routingTable) {
    nodes.push_back(node.link.target);
  }

  return nodes;
}

void Router::updateNeighbors(const vector<Route>& _neighbors) {
  neighbors = _neighbors;
  
  networkGraph[0].edges.clear();
  
  for(const auto& neighbor : neighbors) {
    auto targetIndex = getNodeIndex(neighbor.link.target);
    if(targetIndex == networkGraph.size()) {
      networkGraph.push_back({neighbor.link.target});
    }

    networkGraph[0].edges.push_back({targetIndex, neighbor.link.cost});
  }
}

void Router::updateRoutingTable() {
  //Clear routing table

  //For each node in graph
    //Clear mark, cost for each node
  //Evaluate Dijkstra's algorithm for that node
    //For each node
      //Add shortest path to that node to routing table
  
  routingTable.clear();

  //Clear mark/cost
  for(auto& node : networkGraph) {
    node.mark = false;
    node.processed = false;
    node.cost = numeric_limits<CostType>::max();
    node.prev = -1;
  }

  struct Compare {
    bool operator()(Node* a, Node* b) {
      return a->cost > b->cost;
    }
  };

  //Evaluate Dijkstra's algorithm
  networkGraph[0].cost = 0;
  networkGraph[0].mark = true;
  priority_queue<Node*, std::vector<Node*>, Compare> q;
  q.push(&networkGraph[0]);

  while(!q.empty()) {
    auto& curNode = *q.top();

    if(!curNode.processed) {

      for(auto& edge : curNode.edges) {
        auto& edgeNode = networkGraph[edge.nodeIndex];

        if(!edgeNode.processed) {
          bool updated = false;

          auto curCost = curNode.cost + edge.cost;
          if(curCost < edgeNode.cost) {
            edgeNode.cost = curCost;
            updated = true;
            edgeNode.prev = q.top() - networkGraph.data();
          }

          if(updated || !edgeNode.mark) {
            edgeNode.mark = true;
            
            q.push(&edgeNode);
          }
        }
      }
      curNode.processed = true;
    }

    q.pop();
  }

  //For each node in graph, fill in routing table
  for(size_t i = 1; i < networkGraph.size(); ++i) {
    int node = i;

    while(networkGraph[node].prev > 0) {
      node = networkGraph[node].prev;
    }

    //If a route exists
    if(networkGraph[node].prev != -1) {
      auto neighbor = std::find_if(neighbors.begin(), neighbors.end(),
        [this, &node](const Route& r) {
          return r.link.target == networkGraph[node].id;
        });

      routingTable.push_back({{networkGraph[i].id, networkGraph[i].cost},
        neighbor->nextHop});
    }
  }
}

void Router::processLinkUpdate(const ChipID& fromID, const Link* links, size_t length) {

  auto nodeIndex = getNodeIndex(fromID);

  if(nodeIndex == networkGraph.size()) {
    //This is a new address
    networkGraph.emplace_back(fromID);
  }

  networkGraph[nodeIndex].edges.clear();

  for(size_t i = 0; i < length; ++i) {
    auto& link = links[i];

    auto edgeIndex = getNodeIndex(link.target);

    if(edgeIndex == networkGraph.size()) {
      //Link references unknown target
      networkGraph.emplace_back(link.target);
    }

    networkGraph[nodeIndex].edges.push_back({edgeIndex, link.cost});
  }
}

void Router::exportNeighborTable(vector<Link>& out) const {
  out.clear();

  for(size_t i = 0; i < neighbors.size(); ++i) {
    out.push_back(neighbors[i].link);
  }
}

std::vector<Router::Route> Router::getRoutingTable() const {
  return routingTable;
}

size_t Router::getNodeIndex(const ChipID& id) const {
  auto found = std::find_if(networkGraph.begin(), networkGraph.end(), [&id](const Node& n) {
      return n.id == id;
    });

  return found - networkGraph.begin();
}

void Router::printRoutingTable() const {
  Serial.print("Routing Table - ");
  Serial.print(routingTable.size());
  Serial.println(" entries");

  if(!routingTable.empty()) {
    Serial.println("\tChipID\t\t\tNext Hop\t\t\tCost");
    for(const auto& route : routingTable) {
      Serial.print("\t");
      Serial.print(chipIDToString(route.link.target));
      Serial.print("\t");
      Serial.print(route.nextHop);
      Serial.print("\t\t\t");
      Serial.println(route.link.cost);
    }
  }
}

void Router::printNetworkGraph() const {
  Serial.print("Network Graph - ");
  Serial.print(networkGraph.size());
  Serial.println(" nodes");

  for(const auto& node : networkGraph) {
    Serial.print("\t");
    Serial.print(chipIDToString(node.id));
    Serial.print("\t\t\t");
    Serial.println(node.cost);
    for(const auto& edge : node.edges) {
      Serial.print("\t\t");
      Serial.print(chipIDToString(networkGraph[edge.nodeIndex].id));
      Serial.print("\t\t\t");
      Serial.println(edge.cost);
    }
  }
}

String Router::chipIDToString(const ChipID& id) {
  return String(id[0], HEX) + ":"
    + String(id[1], HEX) + ":"
    + String(id[2], HEX) + ":"
    + String(id[3], HEX) + ":"
    + String(id[4], HEX) + ":"
    + String(id[5], HEX);
}

Router::ChipID Router::stringToChipID(const String& str) {
  ChipID id;

  for(int i = 0; i < id.size(); ++i) {
    auto substr = str.substring(3*i, 3*i+2);
    id[i] = strtol(substr.c_str(), nullptr, 16);
  }

  return id;
}

