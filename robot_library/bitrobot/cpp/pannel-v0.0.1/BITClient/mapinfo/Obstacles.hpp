#pragma once
#include <string>
#include <vector>
//#include "helper.hpp"

/*
#ifdef __GNUC__
#if __GNUC__ < 4 || __GNUC_MINOR__ < 6
#ifndef nullptr
#define nullptr 0   // GCC 4.4 doesn't support nullptr yet...
#endif
#endif
#endif  
*/
namespace impl {


class MapData {
public:
    size_t load(const std::string& name, bool append = false);
    size_t size() const {
        return obPoint.size() + obLine.size() + obCPoly.size();
    }
    void clear() {
        obPoint.clear(); obLine.clear(); obCPoly.clear();
    }

    struct Point  { float x, y;};
    struct Circle { Point q; float r; };
    struct Line   { Point p, q; };
    typedef std::vector<Point> CPoly;

    float obUnit = 1.0f;
    std::vector<Circle> obPoint;
    std::vector<Line>   obLine;
    std::vector<CPoly>  obCPoly;
};



} // namespace impl
