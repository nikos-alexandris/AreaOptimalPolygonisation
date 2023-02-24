#pragma once

#include <memory>
#include <string>

#include "Evaluator.hpp"

namespace project {
namespace config {

class ArgParser {
public:
	// Throws config::Error if the arguments are invalid.
	Evaluator parse(int argc, char** argv);

	static void showOptions();

private:
	struct Initialised {
		bool pointSetPath = false;
		bool outputFile = false;
		bool preprocess = false;
	};
	Initialised initialised_;

	std::string pointSetPath_;
	std::string outputFile_;
	bool preprocess_ = false;

	void checkCompatibility() const;

	void setOption(const std::string& option, const std::string& value);

	void setPointSetPath(const std::string& value);

	void setOutputFile(const std::string& value);

	void setPreprocess();
};

} // namespace config
} // namespace project