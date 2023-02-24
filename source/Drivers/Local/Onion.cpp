#include "Drivers/Local/Onion.hpp"
#include "Common.hpp"

namespace project {
namespace drivers {
namespace local {

OnionDriver::OnionDriver(std::string pointSetInputFile, std::string pointSetOutputFile, OnionDriver::Ordering ordering)
	: AlgorithmDriver(std::move(pointSetInputFile), std::move(pointSetOutputFile)), ordering_(ordering) {}

Polygon2 OnionDriver::run() { throw NotImplementedError(); }

} // namespace local
} // namespace drivers
} // namespace project