#include "Drivers/Global/Algorithm.hpp"
#include "Config/Error.hpp"

namespace project {
namespace drivers {
namespace global {

AlgorithmDriver::AlgorithmDriver(std::string pointSetInputFile, std::string pointSetOutputFile, unsigned long L,
								 Polygonization polygonization)
	: pointSetInputFile_(std::move(pointSetInputFile)), pointSetOutputFile_(std::move(pointSetOutputFile)), L_(L),
	  polygonization_(polygonization) {}

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
			convexHullArea_ = std::stol(areaStr);
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

void AlgorithmDriver::writeOutput(const Polygon2& polygon, std::chrono::milliseconds::rep duration) const {
	// Open the file
	std::ofstream outputFile(pointSetOutputFile_);

	outputFile << "Optimal Area Polygonization" << std::endl;

	// Write the points
	for (const auto& point : polygon.vertices()) {
		outputFile << sprintf_int128(point.x()) << " " << sprintf_int128(point.y()) << std::endl;
	}

	// Write the edges
	for (const auto& edge : polygon.edges()) {
		outputFile << sprintf_int128(edge.source().x()) << " " << sprintf_int128(edge.source().y()) << " "
				   << sprintf_int128(edge.target().x()) << " " << sprintf_int128(edge.target().y()) << std::endl;
	}

	writeInfoToOutput(outputFile, polygon);

	outputFile << "construction time: " << duration << std::endl;

	// Close the file
	outputFile.close();
}

} // namespace global
} // namespace drivers
} // namespace project