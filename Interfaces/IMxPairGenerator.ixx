export module IMxPairGenerator;

import <vector>;

import "interface.h";

interface IMxPairGenerator {
	virtual ~IMxPairGenerator() = default;
	virtual void GeneratePairs(std::vector<int> bodies) = 0;
};