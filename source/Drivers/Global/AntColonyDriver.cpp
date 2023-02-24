#include "Drivers/Global/AntColony.hpp"

namespace project {
namespace drivers {
namespace global {

AntColonyDriver::AntColonyDriver(std::string pointSetInputFile, std::string pointSetOutputFile, unsigned long L,
								 Polygonization polygonization)
	: AlgorithmDriver(std::move(pointSetInputFile), std::move(pointSetOutputFile), L, polygonization) {}

double AntColonyDriver::run(Initialisation) { return 0.0; }

void AntColonyDriver::writeInfoToOutput(std::ostream&, const Polygon2&) const {}

} // namespace global
} // namespace drivers
} // namespace project