#include "Drivers/Local/Algorithm.hpp"
#include "Config/Error.hpp"

namespace project {
namespace drivers {
namespace local {

AlgorithmDriver::AlgorithmDriver(std::string pointSetInputFile, std::string pointSetOutputFile)
	: pointSetInputFile_(std::move(pointSetInputFile)), pointSetOutputFile_(std::move(pointSetOutputFile)) {}

AlgorithmDriver::~AlgorithmDriver() = default;

std::vector<Point2> AlgorithmDriver::readInput() {
	// Create a vector for storing the points
	std::vector<Point2> points;

	// Read the input file line by line
	std::ifstream infile(pointSetInputFile_);
	std::string line;
	size_t counter = 0;
	while (getline(infile, line)) {
		// Get ch area
		if (counter == 1) {
			std::string areaStr = line.substr(line.find("area") + 8, line.length() - (line.find("area") + 8) - 2);
			convexHullArea = std::stol(areaStr);
		}
		if (counter < 2) {
			counter++;
			continue;
		}

		// Tokenize the line
		std::string token;
		std::vector<std::string> line_tokens;
		std::stringstream ss(line);
		while (std::getline(ss, token, '\t')) {
			line_tokens.push_back(token);
		}

		// Check if line is formatted correctly
		if (line_tokens.size() != 3) {
			std::cerr << "Wrong input:\n" << line << std::endl;
			throw config::Error();
		}

		// Get the point coordinates
		FT x = std::stoi(line_tokens[1]);
		FT y = std::stoi(line_tokens[2]);

		// Add the point coordinates to the vector
		points.emplace_back(x, y);
	}

	if (points.size() < 3) {
		std::cerr << "Input needs to contain more than 2 points\n";
		throw config::Error();
	}

	return points;
}



std::ostream& operator<<(std::ostream& os, EdgeSelection edgeSelection) {
	switch (edgeSelection) {
		case EdgeSelection::Random:
			os << "1";
			break;
		case EdgeSelection::MinArea:
			os << "2";
			break;
		case EdgeSelection::MaxArea:
			os << "3";
			break;
	}
	return os;
}

} // namespace local
} // namespace drivers
} // namespace project
