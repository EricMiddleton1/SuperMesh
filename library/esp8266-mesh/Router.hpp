#pragma once

#include <array>
#include "vector.hpp"

class Router {
public:
	using ChipID = std::array<uint8_t, 6>;
	using Address = std::string;
	using CostType = uint16_t;

	struct Link {
		ChipID target;
		CostType cost;
	};
	
	struct Route {
		Link link;
		Address nextHop;

		unsigned int ttl;
	};

	Router();

	//Update information about this device's own neighbor routes
	void updateNeighbor(vector<Route>&& neighbors);

	//Process update from neighbor device
	void processLinkUpdate(const Address& from, CostType cost,
		Link* links, size_t length);

	void exportNeighborTable(vector<Link>& routes);

private:
	static constexpr MAX_TTL = 5;

	struct Node {
		struct Edge {
			size_t nodeIndex;
			CostType cost;
		};

		Address addr;
		vector<Edge> edges;
		size_t reverseEdgeCount;

		bool mark;
		int cost;
	};

	vector<Node> networkGraph;
	vector<Route> routingTable;
	vector<Route> neighbors;
};
