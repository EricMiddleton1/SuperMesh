#include <iostream>

#include "Router.hpp"

void printRoutingTable(const Router& r);

int main() {
	Router::ChipID myID{0, 0, 0, 0, 0, 0};
	Router::Route n1{{{0, 0, 0, 0, 0, 1}, 10}, "192.168.1.1"};
	Router::Route n2{{{0, 0, 0, 0, 0, 2}, 15}, "192.168.1.2"};
	Router::Route n3{{{0, 0, 0, 0, 0, 3}, 5}, "192.168.1.3"};
	Router::Route n4{{{0, 0, 0, 0, 0, 4}, 10}, "192.168.1.4"};

	Router router{myID};
	
	std::cout << "[Info] Updating neighbors..." << std::flush;
	router.updateNeighbors({n1, n2, n3, n4});
	std::cout << "done\n[Info] Updating routing table..." << std::flush;
	router.updateRoutingTable();
	std::cout << "done" << std::endl;
	router.printNetworkGraph();
	printRoutingTable(router);

	n1.link.cost = 5;
	n3.link.cost = 100;

	std::cout << "[Info] Updating neighbors..." << std::flush;
	router.updateNeighbors({n1, n2, n3, n4});
	std::cout << "done\n[Info] Updating routing table..." << std::flush;
	router.updateRoutingTable();
	std::cout << "done" << std::endl;
	router.printNetworkGraph();
	printRoutingTable(router);

	std::cout << "[Info] Processing link update from n1..." << std::flush;
	std::vector<Router::Link> links;
	links.push_back({{0,0,0,0,0,5}, 10});
	links.push_back({{0,0,0,0,0,6}, 11});
	links.push_back({{0,0,0,0,0,7}, 9});
	links.push_back({{0,0,0,0,0,8}, 5});
	router.processLinkUpdate(n1.link.target, links.data(), links.size());
	std::cout << "done" << std::endl;

	links.clear();
	links.push_back({{0,0,0,0,0,6}, 100});
	links.push_back({{0,0,0,0,0,7}, 5});
	router.processLinkUpdate(n4.link.target, links.data(), links.size());
	
	links.clear();
	links.push_back({{0,0,0,0,0,10}, 5});
	links.push_back({{0,0,0,0,0,11}, 15});
	links.push_back({{0,0,0,0,0,12}, 5});
	links.push_back({{0,0,0,0,0,9}, 5});
	links.push_back({{0,0,0,0,0,3}, 5});
	router.processLinkUpdate({0,0,0,0,0,5}, links.data(), links.size());

	std::cout << "[Info] Updating routing table..." << std::flush;
	router.updateRoutingTable();
	std::cout << "done" << std::endl;
	router.printNetworkGraph();
	printRoutingTable(router);

	router.updateNeighbors({n2, n4});
	router.updateRoutingTable();
	router.printNetworkGraph();
	printRoutingTable(router);

	Router::ChipID targetID{0,0,0,0,0,5};

	std::cout << "[Info] Getting next hop for target " << Router::toString(targetID)
		<< std::endl;
	
	Router::Address nextHop;
	if(router.getNextHop(nextHop, targetID)) {
		std::cout << "\tNext Hop: " << nextHop << std::endl;
	}
	else {
		std::cout << "\tNo route found" << std::endl;
	}

	targetID = {0,0,0,0,0,20};
	std::cout << "[Info] Getting next hop for target " << Router::toString(targetID)
		<< std::endl;
	
	if(router.getNextHop(nextHop, targetID)) {
		std::cout << "\tNext Hop: " << nextHop << std::endl;
	}
	else {
		std::cout << "\tNo route found" << std::endl;
	}

	return 0;
}

void printRoutingTable(const Router& r) {
	std::cout << "[Info] Routing Table:\n\tTarget\t\tNext Hop\t\tTotal Cost\n";
	for(auto& route : r.getRoutingTable()) {
		std::cout << "\t" << Router::toString(route.link.target) << "\t\t"
			<< route.nextHop << "\t\t" << route.link.cost << std::endl;
	}
}
