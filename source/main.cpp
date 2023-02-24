#include "Config/ArgParser.hpp"
#include "Config/Error.hpp"
#include "Evaluator.hpp"

using namespace project;

int main(int argc, char** argv) {
	Evaluator evaluator;

	try {
		evaluator = config::ArgParser().parse(argc, argv);
	} catch (const config::Error&) {
		config::ArgParser::showOptions();
		return 1;
	}

	evaluator.evaluate();

	return 0;
}
