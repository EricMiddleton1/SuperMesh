#include "Router.hpp"

#include <queue>
#include <limits>
#include <algorithm>
#include <sstream>

#include <iostream>

using namespace std;

Router::Node::Node(const ChipID& _id)
	:	id{_id}
	,	mark{false}
	,	processed{false}
	,	cost{numeric_limits<CostType>::max()}
	,	prev{-1} {
}

Router::Router(ChipID _myID)
	:	myID{_myID}	{

	static_assert(sizeof(Link) == 8, "sizeof(Link) != 8");

	networkGraph.emplace_back(myID);
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

	std::cout << "[Info] Router: Starting Dijkstra" << std::endl;

	while(!q.empty()) {
		auto& curNode = *q.top();

		if(!curNode.processed) {
			std::cout << "[Info] Router: Processing " << toString(curNode.id) << std::endl;

			for(auto& edge : curNode.edges) {
				auto& edgeNode = networkGraph[edge.nodeIndex];

				if(!edgeNode.processed) {
					bool updated = false;

					auto curCost = curNode.cost + edge.cost;
					if(curCost < edgeNode.cost) {
						std::cout << "[Info] Router: Updating cost for "
							<< toString(edgeNode.id) << " from " << edgeNode.cost << " to "
							<< curCost << std::endl;

						edgeNode.cost = curCost;
						updated = true;
						edgeNode.prev = q.top() - networkGraph.data();
					}

					if(updated || !edgeNode.mark) {
						edgeNode.mark = true;
						
						q.push(&edgeNode);

						std::cout << "[Info] Router: Pushing " << toString(edgeNode.id) << " into queue"
							<< std::endl;
					}
				}
			}
			curNode.processed = true;
		}
		else {
			std::cout << "[Info] Router: Ignoring already processed node "
				<< toString(curNode.id) << std::endl;
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

void Router::processLinkUpdate(const ChipID& fromID, Link* links, size_t length) {

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

void Router::printNetworkGraph() const {
	std::cout << "Network Graph - " << networkGraph.size() << " nodes\n";

	for(const auto& node : networkGraph) {
		std::cout << "\t" << toString(node.id) << "\t\t\t" << node.cost << "\n";
		for(const auto& edge : node.edges) {
			std::cout << "\t\t" << toString(networkGraph[edge.nodeIndex].id)
				<< "\t\t" << networkGraph[edge.nodeIndex].cost << "\n";
		}
	}

	std::cout << std::flush;
}

std::string Router::toString(const ChipID& id) {
	std::stringstream ss;

	ss << std::hex << (int)id[0] << ":"
		<< (int)id[1] << ":"
		<< (int)id[2] << ":"
		<< (int)id[3] << ":"
		<< (int)id[4] << ":"
		<< (int)id[5];
	
	return ss.str();
}
