#include "Drivers/Global/LocalSearch.hpp"
#include "Drivers/Local/ConvexHull.hpp"
#include "Drivers/Local/Incremental.hpp"

namespace project {
namespace drivers {
namespace global {

LocalSearchDriver::LocalSearchDriver(std::string pointSetInputFile, std::string pointSetOutputFile, unsigned long L,
									 Polygonization polygonization, double threshold)
	: AlgorithmDriver(std::move(pointSetInputFile), std::move(pointSetOutputFile), L, polygonization),
	  threshold_(threshold) {}

double LocalSearchDriver::run(Initialisation init) {
	// Start the timer
	auto start = std::chrono::steady_clock::now();

	local::EdgeSelection edgeSelection =
		polygonization_ == Polygonization::Max ? local::EdgeSelection::MaxArea : local::EdgeSelection::MinArea;

	Polygon2 polygon;
	switch (init) {
		case Initialisation::Incremental: {
			local::IncrementalDriver incrementalDriver(pointSetInputFile_, pointSetOutputFile_, edgeSelection,
													   local::IncrementalDriver::Ordering::DecreasingX);
			polygon = incrementalDriver.run();
			convexHullArea_ = incrementalDriver.convexHullArea;
			break;
		}
		case Initialisation::ConvexHull: {
			local::ConvexHullDriver convexHullDriver(pointSetInputFile_, pointSetOutputFile_, edgeSelection);
			polygon = convexHullDriver.run();
			convexHullArea_ = convexHullDriver.convexHullArea;
			break;
		}
	}

	initialPolygon_ = polygon;

	FT areaDelta = std::numeric_limits<FT>::max();

	do {
		// for every path V of length ≤ L
		std::vector<std::vector<Point2>> allPaths;
		unsigned long i = 1;
		for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it, i++) {
			auto ci = polygon.vertices_circulator();
			std::advance(ci, i - 1);
			std::vector<Point2> path;
			for (unsigned long j = 0; j < L_ + 2; j++) {
				path.push_back(*ci);
				ci++;
			}
			allPaths.push_back(path);
		}

		// For every edge of the polygon
		bool found = false;
		FT bestAreaDelta = 0;
		Polygon2 bestPolygon;
		for (const auto& e : polygon.edges()) {
			// Get all the paths that don't contain the edge
			std::vector<std::vector<Point2>> paths;
			std::copy_if(allPaths.begin(), allPaths.end(), std::back_inserter(paths),
						 [&](const std::vector<Point2>& path) {
							 return std::all_of(path.begin(), path.end(),
												[&](const Point2& p) { return p != e.start() && p != e.target(); });
						 });

			// For every path

			for (const auto& path : paths) {
				// Build the first polygon
				Polygon2 first(path.begin() + 1, path.end() - 1);
				first.push_back(e.source());
				first.push_back(e.target());

				// Build the second polygon
				Polygon2 second(path.begin(), path.end());

				// Check if the 2 polygons are simple
				if (!first.is_simple() || !second.is_simple()) {
					continue;
				}

				Polygon2 apply = withChanges(polygon, path, e);
				switch (polygonization_) {
					case Polygonization::Max:
						if (apply.is_simple() && abs(apply.area()) > abs(polygon.area())) {
							bestPolygon = apply;
							found = true;
							bestAreaDelta = abs(apply.area()) - abs(bestPolygon.area());
						}
						break;
					case Polygonization::Min:
						if (apply.is_simple() && abs(apply.area()) < abs(polygon.area())) {
							bestPolygon = apply;
							found = true;
							bestAreaDelta = abs(bestPolygon.area()) - abs(apply.area());
						}
						break;
				}
			}
		}

		if (!found) {
			break;
		}
		if (bestAreaDelta < areaDelta) {
			polygon = bestPolygon;
			areaDelta = bestAreaDelta;
		} else {
			break;
		}
	} while (static_cast<double>(areaDelta) >= threshold_);

	auto end = std::chrono::steady_clock::now();
	writeOutput(polygon, std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

	return (double)abs(polygon.area()) / (double)convexHullArea_;
}

Polygon2 LocalSearchDriver::withChanges(const Polygon2& polygon, const std::vector<Point2>& path,
										const Segment2& edge) {
	auto copy = polygon;

	auto it = copy.vertices_begin();
	while (*it != path.front()) {
		bump(copy, it);
	}
	bump(copy, it);
	while (*it != path.back()) {
		it = copy.erase(it);
		rewindIfAtEnd(copy, it);
	}
	while (*it != edge.source()) {
		bump(copy, it);
	}
	bump(copy, it);
	for (auto j = path.rbegin() + 1; j != path.rend() - 1; j++) {
		it = copy.insert(it, *j);
		bump(copy, it);
	}

	return copy;
}

void LocalSearchDriver::bump(const Polygon2& polygon, Polygon2::Vertex_iterator& it) {
	it++;
	rewindIfAtEnd(polygon, it);
}

void LocalSearchDriver::rewindIfAtEnd(const Polygon2& polygon, Polygon2::Vertex_iterator& it) {
	if (it == polygon.vertices_end()) {
		it = polygon.vertices_begin();
	}
}

void LocalSearchDriver::writeInfoToOutput(std::ostream& os, const Polygon2& polygon) const {

	os << "Algorithm: local_search_" << (polygonization_ == Polygonization::Max ? "max" : "min") << std::endl;
	os << "area_initial " << sprintf_int128(abs(initialPolygon_.area())) << std::endl;
	os << "area " << sprintf_int128(abs(polygon.area())) << std::endl;
	os << "ratio_initial: " << (long double)abs(initialPolygon_.area()) / (long double)convexHullArea_ << std::endl;
	os << "ratio: " << (long double)abs(polygon.area()) / (long double)convexHullArea_ << std::endl;
}

} // namespace global
} // namespace drivers
} // namespace project