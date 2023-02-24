#include "Drivers/Local/Incremental.hpp"
#include "Config/Error.hpp"

#include <iostream>

#include <CGAL/convex_hull_2.h>
#include <chrono>

namespace project {
namespace drivers {
namespace local {

IncrementalDriver::IncrementalDriver(std::string pointSetInputFile, std::string pointSetOutputFile,
									 EdgeSelection edgeSelection, Ordering ordering)
	: AlgorithmDriver(std::move(pointSetInputFile), std::move(pointSetOutputFile)), edgeSelection_(edgeSelection),
	  ordering_(ordering) {}

Polygon2 IncrementalDriver::run() {
	// Read input
	std::vector<Point2> points;
	points = readInput();

	// Sort the points according to the ordering
	sortPoints(points);

	// Build the initial Polygon (triangle) and convex hull.
	std::pair<Polygon2, bool> initial = buildInitialPolygon(points);
	Polygon2 polygon = initial.first;
	if (initial.second) { // if all the points are collinear
		return polygon;
	}

	if (polygon.is_clockwise_oriented()) {
		polygon.reverse_orientation();
	}

	Polygon2 convexHull = polygon;

	auto edgeSelector = getEdgeSelector();

	std::vector<Segment2> redEdges;
	std::vector<Segment2> visibleEdges;

	// Loop for all the remaining points
	while (!points.empty()) {
		// Get the next point
		Point2 point = extractNextPoint(points);

		// Build the convex hull of the polygon with the new point
		Polygon2 augmentedConvexHull = buildAugmentedConvexHull(convexHull, point);

		// Find the augmented convex hull's edges that contain the new point
		std::array<Point2, 2> limits = findLimitsOfConvexHullRedEdges(augmentedConvexHull, point);

		// Find red convex hull edges
		redEdges = findRedEdges(polygon, limits);

		// Find visible edges
		visibleEdges = findVisibleEdges(point, redEdges);

		// Choose a visible edge
		Segment2 visibleEdge = edgeSelector(visibleEdges, point);

		addToPolygon(polygon, visibleEdge, point);

		// Update the current convex hull
		convexHull = augmentedConvexHull;

		redEdges.clear();
		visibleEdges.clear();
	}

	return polygon;
}

void IncrementalDriver::sortPoints(std::vector<Point2>& points) const {
	std::sort(points.begin(), points.end(), getComparator());
	// Reverse because we pop the points from the back
	std::reverse(points.begin(), points.end());
}

std::function<bool(const Point2&, const Point2&)> IncrementalDriver::getComparator() const {
	switch (ordering_) {
		case Ordering::DecreasingX: {
			return
				[](const Point2& p1, const Point2& p2) { return p1.x() == p2.x() ? p1.y() > p2.y() : p1.x() > p2.x(); };
		}
		case Ordering::IncreasingX: {
			return
				[](const Point2& p1, const Point2& p2) { return p1.x() == p2.x() ? p1.y() < p2.y() : p1.x() < p2.x(); };
		}
		case Ordering::DecreasingY: {
			return
				[](const Point2& p1, const Point2& p2) { return p1.y() == p2.y() ? p1.x() > p2.x() : p1.y() > p2.y(); };
		}
		case Ordering::IncreasingY: {
			return
				[](const Point2& p1, const Point2& p2) { return p1.y() == p2.y() ? p1.x() < p2.x() : p1.y() < p2.y(); };
		}
		default: {
			unreachable();
		}
	}
}

std::function<Segment2(const std::vector<Segment2>&, const Point2&)> IncrementalDriver::getEdgeSelector() const {
	std::function<bool(FT, FT)> compareAreas;
	switch (edgeSelection_) {
		case EdgeSelection::Random: {
			srand(time(nullptr));
			return [](const std::vector<Segment2>& edges, const Point2&) { return edges[rand() % edges.size()]; };
		}
		case EdgeSelection::MinArea: {
			compareAreas = [](FT newArea, FT bestArea) { return newArea < bestArea; };
			break;
		}
		case EdgeSelection::MaxArea: {
			compareAreas = [](FT newArea, FT bestArea) { return newArea > bestArea; };
			break;
		}
	}
	return [compareAreas](const std::vector<Segment2>& visibleEdges, const Point2& point) {
		FT bestArea;
		size_t bestIndex = 0;

		for (size_t i = 0; i < visibleEdges.size(); i++) {
			const Segment2& edge = visibleEdges[i];

			// Create the possible new triangle
			Polygon2 poly;
			poly.push_back(point);
			poly.push_back(edge.source());
			poly.push_back(edge.target());

			// Calculate the area (absolute because we don't care about the orientation)
			FT area = abs(poly.area());
			if (i == 0 || compareAreas(area, bestArea)) {
				bestIndex = i;
				bestArea = area;
			}
		}
		return visibleEdges[bestIndex];
	};
}

std::pair<Polygon2, bool> IncrementalDriver::buildInitialPolygon(std::vector<Point2>& points) {
	Polygon2 polygon;

	Point2 p1 = points.back();
	points.pop_back();
	polygon.push_back(p1);

	Point2 p2 = points.back();
	points.pop_back();
	polygon.push_back(p2);

	bool isDegenerate = true;
	while (true) {
		Point2 p = points.back();
		points.pop_back();
		polygon.push_back(p);

		if (!CGAL::collinear(p1, p2, p)) {
			isDegenerate = false;
			break;
		}
		if (points.empty()) {
			break;
		}
	}

	return {polygon, isDegenerate};
}

Point2 IncrementalDriver::extractNextPoint(std::vector<Point2>& points) {
	Point2 point = points.back();
	points.pop_back();
	return point;
}

Polygon2 IncrementalDriver::buildAugmentedConvexHull(const Polygon2& convexHull, const Point2& point) {
	Polygon2 temp = convexHull;
	temp.push_back(point);
	Polygon2 augmentedConvexHull;
	CGAL::convex_hull_2(temp.begin(), temp.end(), std::back_inserter(augmentedConvexHull));
	return augmentedConvexHull;
}

std::array<Point2, 2> IncrementalDriver::findLimitsOfConvexHullRedEdges(const Polygon2& augmentedConvexHull,
																		const Point2& point) {
	std::array<Point2, 2> limits;
	for (auto it = augmentedConvexHull.edges_begin(); it != augmentedConvexHull.edges_end(); it++) {
		if (it->target() == point) {
			limits[0] = it->source();
			if (it + 1 == augmentedConvexHull.edges_end()) {
				limits[1] = augmentedConvexHull.edges_begin()->target();
			} else {
				limits[1] = (it + 1)->target();
			}
			break;
		} else if (it->source() != point && it->has_on(point)) {
			limits[0] = it->source();
			limits[1] = it->target();
			break;
		}
	}

	return limits;
}

std::vector<Segment2> IncrementalDriver::findRedEdges(const Polygon2& polygon, const std::array<Point2, 2>& limits) {
	std::vector<Segment2> redEdges;
	bool foundFirst = false;
	auto circulator = polygon.edges_circulator();
	while (true) {
		if (foundFirst) {
			redEdges.push_back(*circulator);
			if (circulator->target() == limits[1]) {
				break;
			}
		} else if (circulator->source() == limits[0]) {
			foundFirst = true;
			redEdges.push_back(*circulator);
			if (circulator->target() == limits[1]) {
				break;
			}
		}
		++circulator;
	}

	return redEdges;
}

bool IncrementalDriver::redEdgeIsVisible(const Point2& point, const Segment2& segment,
										 const std::vector<Segment2>& redEdges) {
	Segment2 firstSeg(point, segment.source());
	Segment2 secondSeg(point, segment.target());

	for (auto const& redEdge : redEdges) {
		if (redEdge == segment) {
			continue;
		}

		if (redEdge.target() != firstSeg.target() && CGAL::do_intersect(firstSeg, redEdge)) {
			return false;
		}

		if (redEdge.source() != secondSeg.target() && CGAL::do_intersect(secondSeg, redEdge)) {
			return false;
		}
	}

	return true;
}

std::vector<Segment2> IncrementalDriver::findVisibleEdges(const Point2& point, const std::vector<Segment2>& redEdges) {
	std::vector<Segment2> visibleEdges;
	for (auto const& edge : redEdges) {
		if (redEdgeIsVisible(point, edge, redEdges)) {
			visibleEdges.push_back(edge);
		}
	}
	return visibleEdges;
}

void IncrementalDriver::addToPolygon(Polygon2& polygon, const Segment2& visibleEdge, const Point2& point) {
	auto it = std::find(polygon.vertices_begin(), polygon.vertices_end(), visibleEdge.target());
	polygon.insert(it, point);
}

} // namespace local
} // namespace drivers
} // namespace project