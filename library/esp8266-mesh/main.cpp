#include <cstdlib>
#include <cstdint>
#include <iostream>

#include "vector.hpp"


int main() {
	vector<int> v(10);

	v[0] = 1;
	v[2] = 2;
	v[4] = 3;
	v[6] = 4;
	v[8] = 5;

	v.push_back(1);
	v.push_back(2);
	v.push_back(3);
	v.push_back(4);
	v.push_back(5);

	v.erase(0);
	v.erase(5);

	std::cout << "[Info] vector size: " << v.size() << std::endl;
	std::cout << "[Info] Elements: {";
	
	for(uint32_t i = 0; i < v.size(); ++i) {
		std::cout << v[i] << ' ';
	}

	std::cout << "}" << std::endl;

	return 0;
}
