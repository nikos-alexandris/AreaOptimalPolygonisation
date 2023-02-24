#pragma once

#include <map>
#include <string>
#include <vector>

#include <boost/optional.hpp>

namespace project {

class Evaluator {
public:
	Evaluator();

	Evaluator(std::string pointSetPath, std::string outputFile, bool preprocess);

	void evaluate();

private:
	struct Score {
		double minScore = 0.0f;
		double maxScore = 0.0f;
		double minBound = 0.0f;
		double maxBound = 1.0f;
	};

	struct Parameters {
		int localSearchL = 10;
		double localSearchThreshold = 0.25;
		int localStepL = 5000;
		int globalStepL = 5000;
		int subdivisionL = 5000;
	} parameters_;

	std::string pointSetPath_;
	std::string outputFile_;
	bool preprocess_;

	void preprocess(std::string const& filename);
	std::map<int, std::vector<std::string>> getFilesBySize() const;

	static const int NUM_OF_ALGORITHMS = 7;
	static void updateScores(std::array<Score, NUM_OF_ALGORITHMS>& scores, boost::optional<double>* results);
	static int getSizeFromFileName(std::string const& fileName);
	static void printHeader(FILE* file);
	static void printScores(FILE* file, std::array<Score, NUM_OF_ALGORITHMS> const& scores);
	static void alarmHandler(int);
};

} // namespace project