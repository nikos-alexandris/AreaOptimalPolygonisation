#include "Drivers/Global/SimulatedAnnealing.hpp"
#include "Common.hpp"
#include "Drivers/Local/ConvexHull.hpp"
#include "Drivers/Local/Incremental.hpp"
#include <CGAL/convex_hull_2.h>

#include <CGAL/Fuzzy_iso_box.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits_2.h>

using Traits = CGAL::Search_traits_2<project::Kernel>;
using Tree = CGAL::Kd_tree<Traits>;
using Fuzzy_iso_box = CGAL::Fuzzy_iso_box<Traits>;

namespace project {
namespace drivers {
namespace global {

SimulatedAnnealingDriver::SimulatedAnnealingDriver(std::string pointSetInputFile, std::string pointSetOutputFile,
												   unsigned long L, Polygonization polygonization, Annealing annealing)
	: AlgorithmDriver(std::move(pointSetInputFile), std::move(pointSetOutputFile), L, polygonization),
	  annealing_(annealing) {}

double SimulatedAnnealingDriver::energyDelta(FT areaDelta, size_t n, FT chArea) {
	return (double)n * ((double)areaDelta / (double)chArea);
}

double SimulatedAnnealingDriver::run(Initialisation init) {
	srand(time(nullptr));

	// Start the timer
	auto start = std::chrono::steady_clock::now();

	Polygon2 result;
	switch (annealing_) {
		case Annealing::Local: {
			result = localStep(init);
			break;
		}
		case Annealing::Global: {
			local::EdgeSelection edgeSelection =
				polygonization_ == Polygonization::Max ? local::EdgeSelection::MaxArea : local::EdgeSelection::MinArea;

			Polygon2 polygon;
			switch (init) {
				case Initialisation::Incremental: {
					local::IncrementalDriver driver(pointSetInputFile_, pointSetOutputFile_, edgeSelection,
													local::IncrementalDriver::Ordering::DecreasingX);
					polygon = driver.run();
					convexHullArea_ = driver.convexHullArea;
					break;
				}
				case Initialisation::ConvexHull: {
					local::ConvexHullDriver driver(pointSetInputFile_, pointSetOutputFile_, edgeSelection);
					polygon = driver.run();
					convexHullArea_ = driver.convexHullArea;
					break;
				}
			}

			initialPolygon_ = polygon;
			result = globalStep(polygon, boost::optional<Point2>(), boost::optional<Point2>());
			break;
		}
		case Annealing::Subdivision: {
			result = subdivision();
			break;
		}
	}

	auto end = std::chrono::steady_clock::now();
	writeOutput(result, std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

	return (double)abs(result.area()) / (double)convexHullArea_;
}

Polygon2 SimulatedAnnealingDriver::localStep(Initialisation init) {
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

	Polygon2 convexHull;
	CGAL::convex_hull_2(polygon.begin(), polygon.end(), std::back_inserter(convexHull));
	FT convexHullArea = abs(convexHull.area());
	size_t numOfVertices = polygon.vertices().size();

	double T = 1.0f;

	// Local
	// Get 1 random point from the polygon
	// and try swapping it with its successor
	// If the energy is higher, keep the swap with a probability of e^(-deltaE/T)
	for (unsigned long i = 0; i < L_; i++) {
		// Get a random point
		auto qIt = polygon.vertices_begin();
		std::advance(qIt, rand() % polygon.vertices().size());

		// Get the rIt point
		auto rIt = qIt + 1;
		if (rIt == polygon.vertices_end()) {
			rIt = polygon.vertices_begin();
		}

		// Get the pIt point
		Polygon2::Vertex_iterator pIt;
		if (qIt == polygon.vertices_begin()) {
			pIt = polygon.vertices_end() - 1;
		} else {
			pIt = qIt - 1;
		}

		// Get the sIt point
		auto sIt = rIt + 1;
		if (sIt == polygon.vertices_end()) {
			sIt = polygon.vertices_begin();
		}

		// Swap the points
		Point2 p1 = *qIt;
		Point2 p2 = *rIt;
		*qIt = p2;
		*rIt = p1;

		// Check if the new edges intersect each other
		Segment2 pr(*pIt, *rIt);
		Segment2 qs(*qIt, *sIt);
		if (CGAL::do_intersect(pr, qs)) {
			// Swap back
			*qIt = p1;
			*rIt = p2;
			continue;
		}

		// Check if it is valid using kd-trees

		// Create a kd-tree
		Tree tree(polygon.begin(), polygon.end());

		// Create a fuzzy iso box
		FT minX = std::min({pIt->x(), qIt->x(), rIt->x(), sIt->x()});
		FT maxX = std::max({pIt->x(), qIt->x(), rIt->x(), sIt->x()});
		FT minY = std::min({pIt->y(), qIt->y(), rIt->y(), sIt->y()});
		FT maxY = std::max({pIt->y(), qIt->y(), rIt->y(), sIt->y()});

		Point2 first(minX, minY);
		Point2 second(maxX, maxY);

		Fuzzy_iso_box box(first, second);

		// Search if there are any points in the box that are not p, q, r, s
		std::vector<Point2> result;
		tree.search(std::back_inserter(result), box);
		result.erase(std::remove_if(result.begin(), result.end(),
									[&](const auto& p) { return p == *pIt || p == *qIt || p == *rIt || p == *sIt; }),
					 result.end());

		std::vector<Polygon2::Edge_const_iterator> edges;
		for (auto it = polygon.edges_begin(); it != polygon.edges_end(); it++) {
			for (const auto& p : result) {
				if (it->source() == p || it->target() == p) {
					edges.push_back(it);
					break;
				}
			}
		}

		// Check if any of the edges collide with pr, qs
		bool valid = true;
		for (auto e : edges) {
			if (CGAL::do_intersect(pr, *e) || CGAL::do_intersect(qs, *e)) {
				// Swap back
				*qIt = p1;
				*rIt = p2;
				valid = false;
			}
		}
		if (!valid) {
			continue;
		}

		// Calculate pqr and qrs areas
		FT pqrArea = abs(CGAL::area(*pIt, *qIt, *rIt));
		FT qrsArea = abs(CGAL::area(*qIt, *rIt, *sIt));

		double delta = energyDelta(polygonization_ == Polygonization::Max ? pqrArea - qrsArea : qrsArea - pqrArea,
								   numOfVertices, convexHullArea);

		// If the energy is lower, keep the swap
		if (delta >= 0) {
			double p = exp(-delta / T);
			double r = (double)rand() / (double)RAND_MAX;
			if (r >= p) {
				// Undo the swap
				*qIt = p1;
				*rIt = p2;
			}
		}
		T -= 1 / (double)L_;
	}

	return polygon;
}

Polygon2 SimulatedAnnealingDriver::globalStep(Polygon2 polygon, boost::optional<Point2> left,
											  boost::optional<Point2> right) {
	Polygon2 convexHull;
	CGAL::convex_hull_2(polygon.begin(), polygon.end(), std::back_inserter(convexHull));
	FT convexHullArea = abs(convexHull.area());
	size_t numOfVertices = polygon.vertices().size();

	double T = 1.0f;

	for (unsigned long i = 0; i < L_;) {
		// Get a random point
		auto qIt = polygon.vertices_begin();
		std::advance(qIt, rand() % polygon.vertices().size());

		// Get a second random point, that is at least 2 vertices away from q
		Polygon2::Vertex_iterator sIt;
		do {
			sIt = polygon.vertices_begin();
			std::advance(sIt, rand() % polygon.vertices().size());
		} while (abs(std::distance(qIt, sIt)) < 2);

		// If s is after q, swap them
		if (sIt > qIt) {
			std::swap(qIt, sIt);
		}

		// Get the point that precedes q
		auto rIt = prevVertex(polygon, qIt);
		long rIdx = std::distance(polygon.vertices_begin(), rIt);

		// Get the point that follows s
		auto tIt = nextVertex(polygon, sIt);

		auto pIt = nextVertex(polygon, qIt);

		if (left.has_value()) {
			if (*qIt == left.value() || *qIt == right.value() || *sIt == left.value() || *sIt == right.value() ||
				*rIt == left.value() || *rIt == right.value() || *tIt == left.value() || *tIt == right.value() ||
				*pIt == left.value() || *pIt == right.value()) {
				continue;
			}
		}
		i++;

		// Get the q point
		Point2 q = *qIt;

		// Delete the q point and get the p iterator
		pIt = polygon.erase(qIt);
		pIt = rewindIfAtEnd(polygon, pIt);
		long pIdx = std::distance(polygon.vertices_begin(), pIt);

		Point2 p = *pIt;

		// Insert q between s and t and update t, r and p iterators
		qIt = polygon.insert(tIt, q);
		tIt = qIt + 1;
		rIt = polygon.vertices_begin() + rIdx + 1;
		pIt = polygon.vertices_begin() + pIdx + 1;

		// Check that the new edge rp does not intersect the other
		// new edges sq and qt
		Segment2 rp(*rIt, *pIt);
		Segment2 sq(*sIt, *qIt);
		Segment2 qt(*qIt, *tIt);
		if (CGAL::do_intersect(rp, sq) || CGAL::do_intersect(rp, qt)) {
			// Undo the swap
			polygon.erase(qIt);
			polygon.insert(polygon.vertices_begin() + pIdx, q);
			continue;
		}

		// Check that none of the new edges intersect any other edge
		bool valid = true;
		for (auto e = polygon.edges_begin(); e != polygon.edges_end(); e++) {
			if (*e == rp || *e == sq || *e == qt) {
				continue;
			}
			if (e->target() == *rIt) {
				if (CGAL::do_intersect(sq, *e) || CGAL::do_intersect(qt, *e)) {
					valid = false;
					break;
				}
			} else if (e->source() == *pIt) {
				if (CGAL::do_intersect(sq, *e) || CGAL::do_intersect(qt, *e)) {
					valid = false;
					break;
				}
			} else if (e->target() == *sIt) {
				if (CGAL::do_intersect(rp, *e) || CGAL::do_intersect(qt, *e)) {
					valid = false;
					break;
				}
			} else if (e->source() == *tIt) {
				if (CGAL::do_intersect(rp, *e) || CGAL::do_intersect(sq, *e)) {
					valid = false;
					break;
				}
			} else {
				if (CGAL::do_intersect(rp, *e) || CGAL::do_intersect(sq, *e) || CGAL::do_intersect(qt, *e)) {
					valid = false;
					break;
				}
			}
		}
		if (!valid) {
			// Undo the swap
			polygon.erase(qIt);
			polygon.insert(polygon.vertices_begin() + pIdx, q);
			continue;
		}

		// Calculate rqp and sqt areas
		FT rqpArea = abs(CGAL::area(*rIt, *qIt, *pIt));
		FT sqtArea = abs(CGAL::area(*sIt, *qIt, *tIt));

		double delta = energyDelta(polygonization_ == Polygonization::Max ? rqpArea - sqtArea : sqtArea - rqpArea,
								   numOfVertices, convexHullArea);

		if (delta >= 0) {
			double p = exp(-delta / T);
			double r = (double)rand() / (double)RAND_MAX;
			if (r >= p) {
				// Undo the swap
				polygon.erase(qIt);
				polygon.insert(polygon.vertices_begin() + pIdx, q);
			}
		}
		T -= 1 / (double)L_;
	}
	return polygon;
}

Polygon2 SimulatedAnnealingDriver::subdivision() {
	// Read input
	std::vector<Point2> points;
	points = readInput();

	size_t n = points.size();
	size_t m = 0;

	// Sort the points
	std::vector<Point2> sorted;
	sorted.reserve(n);
	std::vector<size_t> indices;
	indices.reserve(n);
	for (size_t i = 0; i < n; i++) {
		indices.push_back(i);
	}
	std::sort(indices.begin(), indices.end(),
			  [&points](size_t i1, size_t i2) { return points[i1].x() < points[i2].x(); });
	for (size_t i = 0; i < n; i++) {
		sorted.push_back(points[indices[i]]);
	}

	size_t maxYPos = n - 1;
	FT maxY = sorted[maxYPos].y();
	for (auto it = sorted.rbegin() + 1; it != sorted.rend(); it++) {
		if (it->x() == sorted.back().x() && it->y() > maxY) {
			maxY = it->y();
			maxYPos = n - 1 - std::distance(sorted.rbegin(), it);
		} else {
			break;
		}
	}
	std::swap(sorted[maxYPos], sorted.back());

	maxYPos = 0;
	maxY = sorted[maxYPos].y();
	for (auto it = sorted.begin() + 1; it != sorted.end(); it++) {
		if (it->x() == sorted.front().x() && it->y() > maxY) {
			maxY = it->y();
			maxYPos = std::distance(sorted.begin(), it);
		} else {
			break;
		}
	}
	std::swap(sorted[maxYPos], sorted.front());

	while (true) {
		m += 10;
		size_t k = ceil(((double)n - 1.0f) / ((double)m - 1.0f));

		// Split the sorted points into k segments, where in each segment after the first,
		// the first point is the last point of the previous segment
		std::vector<std::vector<Point2>> segments;
		segments.reserve(k);
		for (size_t i = 0; i < k; i++) {
			std::vector<Point2> segment;
			segment.reserve(m);
			if (i == 0) {
				segment.push_back(sorted[0]);
			} else {
				segment.push_back(segments[i - 1][m - 1]);
			}
			for (size_t j = 1; j < m; j++) {
				size_t idx = i * (m - 1) + j;
				if (idx >= n) {
					break;
				}
				segment.push_back(sorted[idx]);
			}
			segments.push_back(segment);
		}

		// Run the Convex Hull algorithm on each segment and get
		// the resulting polygons
		bool mustContinue = false;
		std::vector<Polygon2> polygons;
		polygons.reserve(k);
		for (size_t i = 0; i < k; i++) {
			local::ConvexHullDriver convexHullDriver(segments[i], polygonization_ == Polygonization::Max
																	  ? local::EdgeSelection::MaxArea
																	  : local::EdgeSelection::MinArea);
			Polygon2 polygon;
			try {
				polygon = convexHullDriver.run();
			} catch (local::ConvexHullDriver::EdgePointIsLowestException& e) {
				mustContinue = true;
				break;
			} catch (local::ConvexHullDriver::NoVisibleEdgeException& e) {
				mustContinue = true;
				break;
			}

			polygons.push_back(polygon);
		}
		if (mustContinue) {
			continue;
		}

		// Run the Global Step algorithm on the resulting polygons and store them
		// in a vector
		std::vector<Polygon2> globalStepPolygons;
		globalStepPolygons.reserve(k);
		for (size_t i = 0; i < k; i++) {
			globalStepPolygons.push_back(globalStep(polygons[i], segments[i].front(), segments[i].back()));
		}

		// Merge the polygons into one polygon
		Polygon2 mergedPolygon = globalStepPolygons[0];
		for (size_t i = 0; i < segments.size() - 1; i++) {
			const Point2& q = segments[i].back();

			auto q1It = std::find(mergedPolygon.begin(), mergedPolygon.end(), q);
			auto q2It = std::find(globalStepPolygons[i + 1].begin(), globalStepPolygons[i + 1].end(), q);
			auto rIt = nextVertex(globalStepPolygons[i + 1], q2It);

			// Merge the 2 polygons
			// mergedPolygon.insert(q1It, rIt)
			while (rIt != q2It) {
				q1It = mergedPolygon.insert(q1It, *rIt) + 1;
				rIt = nextVertex(globalStepPolygons[i + 1], rIt);
			}
		}

		return mergedPolygon;
	}
}

void SimulatedAnnealingDriver::writeInfoToOutput(std::ostream& os, const Polygon2& polygon) const {

	os << "Algorithm: simulated_annealing_" << (polygonization_ == Polygonization::Max ? "max" : "min") << std::endl;
	if (annealing_ != Annealing::Subdivision) {
		os << "area_initial " << sprintf_int128(abs(initialPolygon_.area())) << std::endl;
	}
	os << "area " << sprintf_int128(abs(polygon.area())) << std::endl;
	if (annealing_ != Annealing::Subdivision) {
		os << "ratio_initial: " << (long double)abs(initialPolygon_.area()) / (long double)convexHullArea_ << std::endl;
	}
	os << "ratio: " << (long double)abs(polygon.area()) / (long double)convexHullArea_ << std::endl;
}

} // namespace global
} // namespace drivers
} // namespace project