#pragma once

#include "Common.hpp"
#include <string>

namespace project {
namespace drivers {
namespace global {

enum class Polygonization {
	Min,
	Max,
};

enum class Initialisation {
	Incremental,
	ConvexHull,
};

class AlgorithmDriver {
public:
	AlgorithmDriver(std::string pointSetInputFile, std::string pointSetOutputFile, unsigned long L,
					Polygonization polygonization);

	virtual ~AlgorithmDriver() = 0;

	virtual double run(Initialisation init) = 0;

protected:
	const std::string pointSetInputFile_;
	const std::string pointSetOutputFile_;
	const unsigned long L_;
	const Polygonization polygonization_;
	FT convexHullArea_ = 0;

	std::vector<Point2> readInput();
	void writeOutput(const Polygon2& polygon, std::chrono::milliseconds::rep duration) const;
	virtual void writeInfoToOutput(std::ostream& os, const Polygon2& polygon) const = 0;
};

} // namespace global
} // namespace drivers
} // namespace project