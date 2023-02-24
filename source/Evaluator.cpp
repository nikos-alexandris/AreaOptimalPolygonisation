#include "Evaluator.hpp"
#include "Drivers/Global/LocalSearch.hpp"
#include "Drivers/Global/SimulatedAnnealing.hpp"

#include <cstdio>
#include <dirent.h>
#include <iostream>
#include <sys/stat.h>

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <wait.h>

namespace project {

using boost::optional;
using namespace boost::interprocess;
using namespace drivers::global;

Evaluator::Evaluator() : Evaluator("", "", false) {}

Evaluator::Evaluator(std::string pointSetPath, std::string outputFile, bool preprocess)
	: pointSetPath_(std::move(pointSetPath)), outputFile_(std::move(outputFile)), preprocess_(preprocess) {}

void Evaluator::evaluate() {

	std::map<int, std::vector<std::string>> map = getFilesBySize();

	FILE* statFile = fopen(outputFile_.c_str(), "w");
	if (!statFile) {
		std::cerr << "Could not open file " << outputFile_ << std::endl;
		exit(1);
	}

	printHeader(statFile);

	if (preprocess_) {
		preprocess(map.begin()->second.front());
	}

	for (const auto& entry : map) {
		int size = entry.first;
		fprintf(statFile, "%8d||", size);
		int seconds = (size * 500) / 1000;

		std::array<Score, NUM_OF_ALGORITHMS> scores;

		// Loop for every file with the same size
		for (auto const& filename : entry.second) {
			std::cout << "Running all algorithms with " << filename << std::endl;

			signal(SIGALRM, alarmHandler);

			// Remove shared memory on construction and destruction
			struct ShmRemover {
				ShmRemover() { shared_memory_object::remove("MySharedMemory"); }
				~ShmRemover() { shared_memory_object::remove("MySharedMemory"); }
			} remover;
			shared_memory_object shm(create_only, "MySharedMemory", read_write);
			shm.truncate((NUM_OF_ALGORITHMS * 2) * sizeof(optional<double>));
			mapped_region region(shm, read_write);

			// Sets everything to empty
			std::memset(region.get_address(), 0, region.get_size());

			// 1. Local Search with Incremental Min
			if (fork() == 0) {
				LocalSearchDriver driver(filename, "temp.txt", parameters_.localSearchL, Polygonization::Min,
										 parameters_.localSearchThreshold);
				alarm(seconds);
				double result = driver.run(Initialisation::Incremental);
				signal(SIGALRM, SIG_IGN);
				static_cast<optional<double>*>(region.get_address())[0] = {result};
				_exit(0);
			}

			// 2. Local Search with Incremental Max
			if (fork() == 0) {
				LocalSearchDriver driver(filename, "temp.txt", parameters_.localSearchL, Polygonization::Max,
										 parameters_.localSearchThreshold);
				alarm(seconds);
				double result = driver.run(Initialisation::Incremental);
				signal(SIGALRM, SIG_IGN);
				static_cast<optional<double>*>(region.get_address())[1] = {result};
				_exit(0);
			}

			// 3. Local Search with Convex Hull Min
			if (fork() == 0) {
				LocalSearchDriver driver(filename, "temp.txt", parameters_.localSearchL, Polygonization::Min,
										 parameters_.localSearchThreshold);
				alarm(seconds);
				double result = driver.run(Initialisation::ConvexHull);
				signal(SIGALRM, SIG_IGN);
				static_cast<optional<double>*>(region.get_address())[2] = {result};
				_exit(0);
			}

			// 4. Local Search with Convex Hull Max
			if (fork() == 0) {
				LocalSearchDriver driver(filename, "temp.txt", parameters_.localSearchL, Polygonization::Max,
										 parameters_.localSearchThreshold);
				alarm(seconds);
				double result = driver.run(Initialisation::ConvexHull);
				signal(SIGALRM, SIG_IGN);
				static_cast<optional<double>*>(region.get_address())[3] = {result};
				_exit(0);
			}

			// 5. Local Step with Incremental Min
			if (fork() == 0) {
				SimulatedAnnealingDriver driver(filename, "temp.txt", parameters_.localStepL, Polygonization::Min,
												Annealing::Local);
				alarm(seconds);
				double result = driver.run(Initialisation::Incremental);
				signal(SIGALRM, SIG_IGN);
				static_cast<optional<double>*>(region.get_address())[4] = {result};
				_exit(0);
			}

			// 6. Local Step with Incremental Max
			if (fork() == 0) {
				SimulatedAnnealingDriver driver(filename, "temp.txt", parameters_.localStepL, Polygonization::Max,
												Annealing::Local);
				alarm(seconds);
				double result = driver.run(Initialisation::Incremental);
				signal(SIGALRM, SIG_IGN);
				static_cast<optional<double>*>(region.get_address())[5] = {result};
				_exit(0);
			}

			// 7. Local Step with Convex Hull Min
			if (fork() == 0) {
				SimulatedAnnealingDriver driver(filename, "temp.txt", parameters_.localStepL, Polygonization::Min,
												Annealing::Local);
				alarm(seconds);
				double result = driver.run(Initialisation::ConvexHull);
				signal(SIGALRM, SIG_IGN);
				static_cast<optional<double>*>(region.get_address())[6] = {result};
				_exit(0);
			}

			// 8. Local Step with Convex Hull Max
			if (fork() == 0) {
				SimulatedAnnealingDriver driver(filename, "temp.txt", parameters_.localStepL, Polygonization::Max,
												Annealing::Local);
				alarm(seconds);
				double result = driver.run(Initialisation::ConvexHull);
				signal(SIGALRM, SIG_IGN);
				static_cast<optional<double>*>(region.get_address())[7] = {result};
				_exit(0);
			}

			// 9. Global Step with Incremental Min
			if (fork() == 0) {
				SimulatedAnnealingDriver driver(filename, "temp.txt", parameters_.globalStepL, Polygonization::Min,
												Annealing::Global);
				alarm(seconds);
				double result = driver.run(Initialisation::Incremental);
				signal(SIGALRM, SIG_IGN);
				static_cast<optional<double>*>(region.get_address())[8] = {result};
				_exit(0);
			}

			// 10. Global Step with Incremental Max
			if (fork() == 0) {
				SimulatedAnnealingDriver driver(filename, "temp.txt", parameters_.globalStepL, Polygonization::Max,
												Annealing::Global);
				alarm(seconds);
				double result = driver.run(Initialisation::Incremental);
				signal(SIGALRM, SIG_IGN);
				static_cast<optional<double>*>(region.get_address())[9] = {result};
				_exit(0);
			}

			// 11. Global Step with Convex Hull Min
			if (fork() == 0) {
				SimulatedAnnealingDriver driver(filename, "temp.txt", parameters_.globalStepL, Polygonization::Min,
												Annealing::Global);
				alarm(seconds);
				double result = driver.run(Initialisation::ConvexHull);
				signal(SIGALRM, SIG_IGN);
				static_cast<optional<double>*>(region.get_address())[10] = {result};
				_exit(0);
			}

			// 12. Global Step with Convex Hull Max
			if (fork() == 0) {
				SimulatedAnnealingDriver driver(filename, "temp.txt", parameters_.globalStepL, Polygonization::Max,
												Annealing::Global);
				alarm(seconds);
				double result = driver.run(Initialisation::ConvexHull);
				signal(SIGALRM, SIG_IGN);
				static_cast<optional<double>*>(region.get_address())[11] = {result};
				_exit(0);
			}

			// 13. Subdivision Min
			if (fork() == 0) {
				SimulatedAnnealingDriver driver(filename, "temp.txt", parameters_.subdivisionL, Polygonization::Min,
												Annealing::Subdivision);
				alarm(seconds);
				double result = driver.run(Initialisation::ConvexHull);
				signal(SIGALRM, SIG_IGN);
				static_cast<optional<double>*>(region.get_address())[12] = {result};
				_exit(0);
			}

			// 14. Subdivision Max
			if (fork() == 0) {
				SimulatedAnnealingDriver driver(filename, "temp.txt", parameters_.subdivisionL, Polygonization::Max,
												Annealing::Subdivision);
				alarm(seconds);
				double result = driver.run(Initialisation::ConvexHull);
				signal(SIGALRM, SIG_IGN);
				static_cast<optional<double>*>(region.get_address())[13] = {result};
				_exit(0);
			}

			while (wait(nullptr) != -1)
				;

			updateScores(scores, static_cast<optional<double>*>(region.get_address()));
		}
		printScores(statFile, scores);
	}
	fclose(statFile);
}

void Evaluator::preprocess(std::string const& filename) {
	printf("Preprocessing...\n");
	double best;

	// Local Search L
	best = 1.0f;
	for (int i = 2; i <= 10; i++) {
		LocalSearchDriver driver(filename, "temp.txt", i, Polygonization::Min, parameters_.localSearchThreshold);
		double score = driver.run(Initialisation::Incremental);
		if (score < best) {
			best = score;
			parameters_.localSearchL = i;
		}
	}

	// Local Search Threshold
	best = 1.0f;
	for (float t = 0.25; t <= 0.75; t += 0.25) {
		LocalSearchDriver driver(filename, "temp.txt", parameters_.localSearchL, Polygonization::Min, t);
		double score = driver.run(Initialisation::Incremental);
		if (score < best) {
			best = score;
			parameters_.localSearchThreshold = t;
		}
	}

	// Local step L
	best = 1.0f;
	for (int i = 1000; i <= 5000; i += 1000) {
		SimulatedAnnealingDriver driver(filename, "temp.txt", i, Polygonization::Min, Annealing::Local);
		double score = driver.run(Initialisation::Incremental);
		if (score < best) {
			best = score;
			parameters_.localStepL = i;
		}
	}

	// Global step L
	best = 1.0f;
	for (int i = 1000; i <= 5000; i += 1000) {
		SimulatedAnnealingDriver driver(filename, "temp.txt", i, Polygonization::Min, Annealing::Global);
		double score = driver.run(Initialisation::Incremental);
		if (score < best) {
			best = score;
			parameters_.globalStepL = i;
		}
	}

	// Subdivision L
	best = 1.0f;
	for (int i = 1000; i <= 5000; i += 1000) {
		SimulatedAnnealingDriver driver(filename, "temp.txt", i, Polygonization::Min, Annealing::Subdivision);
		double score = driver.run(Initialisation::Incremental);
		if (score < best) {
			best = score;
			parameters_.subdivisionL = i;
		}
	}
}

std::map<int, std::vector<std::string>> Evaluator::getFilesBySize() const {
	// Check if the directory exists
	struct stat info {};
	if (stat(pointSetPath_.c_str(), &info) != 0) {
		std::cout << "Cannot access directory \"" << pointSetPath_ << "\"" << std::endl;
		exit(1);
	} else if (!(info.st_mode & S_IFDIR)) {
		std::cout << "\"" << pointSetPath_ << "\" is not a directory" << std::endl;
		exit(1);
	}

	// Get a list of all the files in the directory
	// and categorize them by their size
	DIR* dir;
	std::map<int, std::vector<std::string>> map;

	if ((dir = opendir(pointSetPath_.c_str())) != nullptr) {
		for (struct dirent const* diread; (diread = readdir(dir)) != nullptr;) {
			std::string filename(diread->d_name);
			// Don't insert if the dir name is . or ..
			if (filename == "." || filename == "..") {
				continue;
			}
			map[getSizeFromFileName(filename)].emplace_back(pointSetPath_ + "/" + filename);
		}
		closedir(dir);
	} else {
		perror("opendir");
		exit(1);
	}

	return map;
}

void Evaluator::updateScores(std::array<Score, NUM_OF_ALGORITHMS>& scores, boost::optional<double>* results) {
	for (int i = 0; i < NUM_OF_ALGORITHMS * 2; i += 2) {
		double rMin;
		double rMax;

		if (results[i]) {
			rMin = *results[i];
		} else {
			rMin = 1.0f;
		}

		if (results[i + 1]) {
			rMax = *results[i + 1];
		} else {
			rMax = 0.0f;
		}

		Score& score = scores[i / 2];
		score.minScore += rMin;
		if (rMin > score.minBound) {
			score.minBound = rMin;
		}
		score.maxScore += rMax;
		if (rMax < score.maxBound) {
			score.maxBound = rMax;
		}
	}
}

int Evaluator::getSizeFromFileName(std::string const& fileName) {
	std::string sizeStr;
	std::copy_if(fileName.begin(), fileName.end(), std::back_inserter(sizeStr), [](char c) { return std::isdigit(c); });
	return std::stoi(sizeStr);
}

void Evaluator::printHeader(FILE* file) {
	fprintf(file, "        ||");
	fprintf(file, "            Local Search Incremental           ||");
	fprintf(file, "            Local Search Convex Hull           ||");
	fprintf(file, "             Local Step Incremental            ||");
	fprintf(file, "             Local Step Convex Hull            ||");
	fprintf(file, "             Global Step Incremental           ||");
	fprintf(file, "             Global Step Convex Hull           ||");
	fprintf(file, "                   Subdivision                 ||\n");

	fprintf(file, "%8s||", "  Size  ");
	for (int i = 0; i < NUM_OF_ALGORITHMS; i++) {
		fprintf(file, " min score | max score | min bound | max bound ||");
	}
	fprintf(file, "\n");
}

void Evaluator::printScores(FILE* file, std::array<Score, NUM_OF_ALGORITHMS> const& scores) {
	for (auto const& score : scores) {
		fprintf(file, " %.7f | %.7f | %.7f | %.7f ||", score.minScore, score.maxScore, score.minBound, score.maxBound);
	}
	fprintf(file, "\n");
	fflush(file);
}

void Evaluator::alarmHandler(int) { _exit(1); }

} // namespace project
