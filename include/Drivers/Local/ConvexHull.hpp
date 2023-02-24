#pragma once

#include "Drivers/Local/Algorithm.hpp"

namespace project {
namespace drivers {
namespace local {

class ConvexHullDriver : public AlgorithmDriver {
public:
	ConvexHullDriver(std::string pointSetInputFile, std::string pointSetOutputFile, EdgeSelection edgeSelection);

	ConvexHullDriver(std::vector<Point2> points, EdgeSelection edgeSelection);

	~ConvexHullDriver() override = default;

	Polygon2 run() override;

	class NoVisibleEdgeException : public std::exception {
	public:
		NoVisibleEdgeException() = default;
		~NoVisibleEdgeException() override = default;

		const char* what() const noexcept override { return "No visible edge found"; }
	};

	class EdgePointIsLowestException : public std::exception {
	public:
		EdgePointIsLowestException() = default;
		~EdgePointIsLowestException() override = default;

		const char* what() const noexcept override { return "Edge point is lowest"; }
	};

private:
	const EdgeSelection edgeSelection_;
	std::vector<Point2> points_;
	boost::optional<std::pair<Point2, Point2>> edgePoints_;

	std::function<std::pair<Point2, Segment2>(const std::unordered_map<Point2, std::vector<Segment2>>&)>
	getEdgeSelector() const;

	static bool edgeIsVisible(const Polygon2& polygon, const Point2& point, const Segment2& segment);

	static std::vector<Segment2>
	findVisibleEdgesFromPoint(const Polygon2& polygon, const Point2& point,
							  const boost::optional<std::pair<Segment2, Segment2>>& edgeSegments);
};

} // namespace local
} // namespace drivers
} // namespace project