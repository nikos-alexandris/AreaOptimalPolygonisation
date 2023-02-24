#pragma once

#include "Common.hpp"
#include "Drivers/Global/Algorithm.hpp"

namespace project {
namespace drivers {
namespace global {

enum class Annealing {
	Local,
	Global,
	Subdivision,
};

class SimulatedAnnealingDriver : public AlgorithmDriver {
public:
	SimulatedAnnealingDriver(std::string pointSetInputFile, std::string pointSetOutputFile, unsigned long L,
							 Polygonization polygonization, Annealing annealing);

	~SimulatedAnnealingDriver() override = default;

	double run(Initialisation init) override;

private:
	Annealing annealing_;
	Polygon2 initialPolygon_;

	Polygon2 localStep(Initialisation init);
	Polygon2 globalStep(Polygon2 polygon, boost::optional<Point2> left, boost::optional<Point2> right);
	Polygon2 subdivision();

	static double energyDelta(FT areaDelta, size_t n, FT chArea);

	void writeInfoToOutput(std::ostream& os, const Polygon2& polygon) const override;
};

} // namespace global
} // namespace drivers
} // namespace project