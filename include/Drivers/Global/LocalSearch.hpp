#pragma once

#include "Common.hpp"
#include "Drivers/Global/Algorithm.hpp"

namespace project {
namespace drivers {
namespace global {

class LocalSearchDriver : public AlgorithmDriver {
public:
	LocalSearchDriver(std::string pointSetInputFile, std::string pointSetOutputFile, unsigned long L,
					  Polygonization polygonization, double threshold);

	~LocalSearchDriver() override = default;

	double run(Initialisation init) override;

private:
	double threshold_;
	Polygon2 initialPolygon_;

	static Polygon2 withChanges(const Polygon2& polygon, const std::vector<Point2>& path, const Segment2& edge);
	static void bump(const Polygon2& polygon, Polygon2::Vertex_iterator& it);
	static void rewindIfAtEnd(const Polygon2& polygon, Polygon2::Vertex_iterator& it);

	void writeInfoToOutput(std::ostream& os, const Polygon2& polygon) const override;
};

} // namespace global
} // namespace drivers
} // namespace project