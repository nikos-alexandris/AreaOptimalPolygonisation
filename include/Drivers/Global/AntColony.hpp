#include "Drivers/Global/SimulatedAnnealing.hpp"

namespace project {
namespace drivers {
namespace global {

class AntColonyDriver : public AlgorithmDriver {
public:
	AntColonyDriver(std::string pointSetInputFile, std::string pointSetOutputFile, unsigned long L,
					Polygonization polygonization);

	~AntColonyDriver() override = default;

	double run(Initialisation init) override;

private:
	void writeInfoToOutput(std::ostream& os, const Polygon2& polygon) const override;
};

} // namespace global
} // namespace drivers
} // namespace project