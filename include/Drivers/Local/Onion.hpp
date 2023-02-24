#pragma once

#include "Drivers/Local/Algorithm.hpp"

namespace project {
namespace drivers {
namespace local {

class OnionDriver : public AlgorithmDriver {
public:
	enum class Ordering {
		Random, // 1
		MinX,	// 2
		MaxX,	// 3
		MinY,	// 4
		MaxY	// 5
	};

	OnionDriver(std::string pointSetInputFile, std::string pointSetOutputFile, Ordering ordering);

	~OnionDriver() override = default;

	Polygon2 run() override;

private:
	const Ordering ordering_;
};

} // namespace local
} // namespace drivers
} // namespace project