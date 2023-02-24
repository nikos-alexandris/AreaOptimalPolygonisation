#include "Drivers/Local/ConvexHull.hpp"

#include <CGAL/convex_hull_2.h>
#include <iostream>

#include <CGAL/Arr_naive_point_location.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Simple_polygon_visibility_2.h>

namespace project {
namespace drivers {
namespace local {

ConvexHullDriver::ConvexHullDriver(std::string pointSetInputFile, std::string pointSetOutputFile,
								   EdgeSelection edgeSelection)
	: AlgorithmDriver(std::move(pointSetInputFile), std::move(pointSetOutputFile)), edgeSelection_(edgeSelection),
	  edgePoints_(boost::none) {}

ConvexHullDriver::ConvexHullDriver(std::vector<Point2> points, EdgeSelection edgeSelection)
	: AlgorithmDriver("", ""), edgeSelection_(edgeSelection), points_(std::move(points)) {
	edgePoints_ = std::make_pair(points_.front(), points_.back());
}

Polygon2 ConvexHullDriver::run() {
	// Randomize
	srand(time(nullptr));

	std::vector<Point2> points;
	if (edgePoints_.has_value()) {
		points = std::move(points_);
	} else {
		// Read input
		points = readInput();
	}

	// Calculate convex hull of points using CGAL
	Polygon2 polygon;
	CGAL::convex_hull_2(points.begin(), points.end(), std::back_inserter(polygon));

	boost::optional<std::pair<Segment2, Segment2>> edgeSegments = boost::none;
	if (edgePoints_.has_value()) {
		auto lIt = std::find(polygon.vertices_begin(), polygon.vertices_end(), edgePoints_.value().first);
		auto nIt = nextVertex(polygon, lIt);
		Segment2 leftEdge = Segment2(*lIt, *nIt);
		if (nIt->y() >= lIt->y()) {
			throw EdgePointIsLowestException();
		}

		auto rIt = std::find(polygon.vertices_begin(), polygon.vertices_end(), edgePoints_.value().second);
		auto pIt = prevVertex(polygon, rIt);
		Segment2 rightEdge = Segment2(*pIt, *rIt);
		if (pIt->y() >= rIt->y()) {
			throw EdgePointIsLowestException();
		}

		edgeSegments = std::make_pair(leftEdge, rightEdge);
	}

	// Remove all the points used to build the convex hull from the points vector
	for (const auto& point : polygon) {
		points.erase(std::remove(points.begin(), points.end(), point), points.end());
	}

	auto edgeSelector = getEdgeSelector();

	while (!points.empty()) {
		// Add every edge to the vector of the closest edges to the appropriate point
		std::unordered_map<Point2, std::vector<Segment2>> closestPointToEdgesMap;
		for (const auto& edge : polygon.edges()) {
			auto closestPoint =
				std::min_element(points.begin(), points.end(), [&edge](const Point2& a, const Point2& b) {
					return CGAL::squared_distance(edge, a) < CGAL::squared_distance(edge, b);
				});
			closestPointToEdgesMap[*closestPoint].push_back(edge);
		}

		// Remove the edges that are not visible from each point
		for (auto& pair : closestPointToEdgesMap) {
			std::vector<Segment2> visibleEdges = findVisibleEdgesFromPoint(polygon, pair.first, edgeSegments);
			pair.second.erase(std::remove_if(pair.second.begin(), pair.second.end(),
											 [&visibleEdges](const Segment2& edge) {
												 return std::find(visibleEdges.begin(), visibleEdges.end(), edge) ==
														visibleEdges.end();
											 }),
							  pair.second.end());
		}

		// Remove points whose closest edges are empty
		for (auto it = closestPointToEdgesMap.begin(); it != closestPointToEdgesMap.end();) {
			if (it->second.empty()) {
				it = closestPointToEdgesMap.erase(it);
			} else {
				++it;
			}
		}

		auto selectedPair = edgeSelector(closestPointToEdgesMap);
		Point2 selectedPoint = selectedPair.first;
		Segment2 selectedEdge = selectedPair.second;

		// Insert the point into the polygon in the correct position
		auto it = std::find(polygon.vertices_begin(), polygon.vertices_end(), selectedEdge.target());
		polygon.insert(it, selectedPoint);

		// remove the point from the points vector
		points.erase(std::remove(points.begin(), points.end(), selectedPoint), points.end());
	}

	return polygon;
}

bool ConvexHullDriver::edgeIsVisible(const Polygon2& polygon, const Point2& point, const Segment2& segment) {
	Segment2 firstSeg(point, segment.source());
	Segment2 secondSeg(point, segment.target());

	for (auto const& redEdge : polygon.edges()) {
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

std::vector<Segment2>
ConvexHullDriver::findVisibleEdgesFromPoint(const Polygon2& polygon, const Point2& point,
											const boost::optional<std::pair<Segment2, Segment2>>& edgeSegments) {
	std::vector<Segment2> visibleEdges;
	for (auto const& edge : polygon.edges()) {
		if (edgeSegments.has_value() && (edge == edgeSegments.value().first || edge == edgeSegments.value().second)) {
			continue;
		}
		if (edgeIsVisible(polygon, point, edge)) {
			visibleEdges.push_back(edge);
		}
	}

	if (visibleEdges.empty()) {
		throw NoVisibleEdgeException();
	}

	return visibleEdges;
}

std::function<std::pair<Point2, Segment2>(const std::unordered_map<Point2, std::vector<Segment2>>&)>
ConvexHullDriver::getEdgeSelector() const {
	std::function<bool(FT, FT)> compareAreas;
	switch (edgeSelection_) {
		case EdgeSelection::Random: {
			srand(time(nullptr));
			return [](const std::unordered_map<Point2, std::vector<Segment2>>& closestPointToEdgesMap) {
				Point2 point;
				Segment2 edge;
				int p = rand() % closestPointToEdgesMap.size();
				auto pIt = closestPointToEdgesMap.begin();
				std::advance(pIt, p);
				point = pIt->first;
				int e = rand() % pIt->second.size();
				auto eIt = pIt->second.begin();
				std::advance(eIt, e);
				edge = *eIt;
				return std::make_pair(point, edge);
			};
		}
		case EdgeSelection::MinArea: {
			compareAreas = [](FT newArea, FT bestArea) { return newArea > bestArea; };
			break;
		}
		case EdgeSelection::MaxArea: {
			compareAreas = [](FT newArea, FT bestArea) { return newArea < bestArea; };
			break;
		}
	}
	return [compareAreas](const std::unordered_map<Point2, std::vector<Segment2>>& closestPointToEdgesMap) {
		FT bestArea;
		Point2 bestPoint;
		Segment2 bestEdge;
		bool first = true;
		for (const auto& it : closestPointToEdgesMap) {
			for (const auto& edge : it.second) {
				// Find the area of the triangle
				FT area = abs(CGAL::area(it.first, edge.source(), edge.target()));

				// Check if the area is better than the current best
				if (first || compareAreas(area, bestArea)) {
					first = false;
					bestArea = area;
					bestPoint = it.first;
					bestEdge = edge;
				}
			}
		}

		return std::make_pair(bestPoint, bestEdge);
	};
}

} // namespace local
} // namespace drivers
} // namespace project