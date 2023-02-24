#pragma once

#include <ostream>

#include "Common.hpp"

namespace project {
namespace drivers {
namespace local {

enum class EdgeSelection {
	Random,	 // 1
	MinArea, // 2
	MaxArea, // 3
};

std::ostream& operator<<(std::ostream& os, EdgeSelection edgeSelection);

class AlgorithmDriver {
public:
	AlgorithmDriver(std::string pointSetInputFile, std::string pointSetOutputFile);

	virtual ~AlgorithmDriver() = 0;

	virtual Polygon2 run() = 0;

	FT convexHullArea = 0;

protected:
	const std::string pointSetInputFile_;
	const std::string pointSetOutputFile_;

	// Throws Exception Error() if a line is not formatted correctly
	std::vector<Point2> readInput();
};

} // namespace local
} // namespace drivers
} // namespace project