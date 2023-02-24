#include "Config/ArgParser.hpp"

#include <iostream>

#include "Config/Error.hpp"

namespace project {
namespace config {

Evaluator ArgParser::parse(int argc, char** argv) {
	for (int i = 1; i < argc;) {
		if (std::string(argv[i]) == "-preprocess") {
			setPreprocess();
			i++;
		} else if (i == argc - 1) {
			std::cerr << "Missing argument" << std::endl;
			throw Error();
		} else {
			setOption(argv[i], argv[i + 1]);
			i += 2;
		}
	}

	checkCompatibility();

	return {pointSetPath_, outputFile_, preprocess_};
}

void ArgParser::showOptions() {
	std::cerr << "Options: " << std::endl;
	std::cerr << "  -i <point set path>" << std::endl;
	std::cerr << "  -o <output file>" << std::endl;
	std::cerr << "  -preprocess <optional>" << std::endl;
}

void ArgParser::setOption(const std::string& option, const std::string& value) {
	if (option == "-i") {
		setPointSetPath(value);
	} else if (option == "-o") {
		setOutputFile(value);
	} else {
		std::cerr << "Unknown option: " << option << std::endl;
		throw Error();
	}
}

void ArgParser::setPointSetPath(const std::string& value) {
	if (initialised_.pointSetPath) {
		std::cerr << "Duplicate option -i is not allowed" << std::endl;
		throw Error();
	}
	initialised_.pointSetPath = true;

	pointSetPath_ = value;
}

void ArgParser::setOutputFile(const std::string& value) {
	if (initialised_.outputFile) {
		std::cerr << "Duplicate option -o is not allowed" << std::endl;
		throw Error();
	}
	initialised_.outputFile = true;

	outputFile_ = value;
}

void ArgParser::setPreprocess() {
	if (initialised_.preprocess) {
		std::cerr << "Duplicate option -preprocess is not allowed" << std::endl;
		throw Error();
	}
	initialised_.preprocess = true;

	preprocess_ = true;
}

void ArgParser::checkCompatibility() const {
	if (!initialised_.pointSetPath) {
		std::cerr << "Option -i is required" << std::endl;
		throw Error();
	}
	if (!initialised_.outputFile) {
		std::cerr << "Option -o is required" << std::endl;
		throw Error();
	}
}

} // namespace config
} // namespace project