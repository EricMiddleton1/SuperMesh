#include "Router.hpp"


Router::Router() {
	static_assert(sizeof(Link) == 8, "sizeof(Link) != 8");
}

void Router::updateNeighbor(vector<Route>&& _neighbors) {
	neighbors = _neighbors;
}

void Router::processLinkUpdate(const Address& from, CostType cost,
	Link* links, size_t length) {

	auto nodeIndex 
	
	for(size_t i = 0; i < length; ++i) {
		auto& link = links[i];
		link.cost += cost;

		auto tableIndex = find_index_if(routingTable, [&link](const Route& r) {
				return r.link.target == link.target;
			});

		if(tableIndex == routingTable.size()) {
			routingTable.push_back({link, from, MAX_TTL});
		}
		else if(link.cost
}

void Router::getNeighborLinks(vector<Link>& links) {
	links.clear();

	for(int i = 0; i < neighbors.size(); ++i) {
		auto& neighbor = neighbors[i];

		neighbor.ttl--;
		if(neighbor.ttl == 0) {
			neighbors.erase(i);
			--i;
		}
		else {
			links.push_back(neighbor.route.link);
		}
	}
}
