#pragma once

#include <stdexcept>

#include <CGAL/Cartesian.h>
#include <CGAL/Polygon_2.h>
#include <future>

namespace project {

using FT = __int128;
using Kernel = CGAL::Cartesian<FT>;
using Point2 = CGAL::Point_2<Kernel>;
using Polygon2 = CGAL::Polygon_2<Kernel>;
using Segment2 = CGAL::Segment_2<Kernel>;

class NotImplementedError : public std::logic_error {
public:
	NotImplementedError();
};

[[noreturn]] void unreachable();

inline Polygon2::Vertex_iterator nextVertex(const Polygon2& polygon, Polygon2::Vertex_iterator it) {
	auto next = it;
	++next;
	if (next == polygon.vertices_end()) {
		next = polygon.vertices_begin();
	}
	return next;
}

inline Polygon2::Vertex_iterator prevVertex(const Polygon2& polygon, Polygon2::Vertex_iterator it) {
	if (it == polygon.vertices_begin()) {
		return polygon.vertices_end() - 1;
	}
	return it - 1;
}

inline Polygon2::Vertex_iterator rewindIfAtEnd(const Polygon2& polygon, Polygon2::Vertex_iterator it) {
	if (it == polygon.vertices_end()) {
		return polygon.vertices_begin();
	}
	return it;
}

// https://stackoverflow.com/a/44910009
char* sprintf_int128(__int128 n);

} // namespace project
