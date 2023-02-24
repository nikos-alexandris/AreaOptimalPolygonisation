#pragma once

#include "Drivers/Local/Algorithm.hpp"

namespace project {
namespace drivers {
namespace local {

class IncrementalDriver : public AlgorithmDriver {
public:
	enum class Ordering {
		DecreasingX, // 1a
		IncreasingX, // 1b
		DecreasingY, // 2a
		IncreasingY	 // 2b
	};

	IncrementalDriver(std::string pointSetInputFile, std::string pointSetOutputFile, EdgeSelection edgeSelection,
					  Ordering ordering);

	~IncrementalDriver() override = default;

	Polygon2 run() override;

private:
	const EdgeSelection edgeSelection_;
	const Ordering ordering_;

	void sortPoints(std::vector<Point2>& points) const;

	std::function<bool(const Point2&, const Point2&)> getComparator() const;

	std::function<Segment2(const std::vector<Segment2>&, const Point2&)> getEdgeSelector() const;

	static std::pair<Polygon2, bool> buildInitialPolygon(std::vector<Point2>& points);

	static Point2 extractNextPoint(std::vector<Point2>& points);

	static Polygon2 buildAugmentedConvexHull(const Polygon2& convexHull, const Point2& point);

	static std::array<Point2, 2> findLimitsOfConvexHullRedEdges(const Polygon2& augmentedConvexHull,
																const Point2& point);

	static std::vector<Segment2> findRedEdges(const Polygon2& polygon, const std::array<Point2, 2>& limits);

	static bool redEdgeIsVisible(const Point2& point, const Segment2& segment, const std::vector<Segment2>& redEdges);

	static std::vector<Segment2> findVisibleEdges(const Point2& point, const std::vector<Segment2>& redEdges);

	static void addToPolygon(Polygon2& polygon, const Segment2& visibleEdge, const Point2& point);
};

} // namespace local
} // namespace drivers
} // namespace project