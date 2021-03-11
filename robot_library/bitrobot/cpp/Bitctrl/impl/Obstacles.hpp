#pragma once
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
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
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif


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

	double obUnit;
    std::vector<Circle> obPoint;
    std::vector<Line>   obLine;
    std::vector<CPoly>  obCPoly;
};

struct IndexTrans {
	IndexTrans() : nrow(0), ncol(0) {}
	IndexTrans(int r, int c) : nrow(r), ncol(c) {}

	// (i, j) --> index
	int operator() (int r, int c) const { return r * ncol + c; }
	int row(int index) const { return ncol > 0 ? index / ncol : 0; }
	int col(int index) const { return ncol > 0 ? index % ncol : 0; }
	int w(int index) const { return col(index); }
	int h(int index) const { return row(index); }

	double HamiltonDistance(int p, int q) const {
		return std::abs(row(p) - row(q)) + std::abs(col(p) - col(q));
	}
	double EucildDistance(int p, int q)  const {
		int dx = row(p) - row(q);
		int dy = col(p) - col(q);
		return std::sqrt(dx * dx + dy * dy);
	}

	bool valid(int r, int c) const {
		return r >= 0 && r < nrow && c >= 0 && c < ncol;
	}
	bool valid(int index) const {
		return valid(row(index), col(index));
	}


	int nrow, ncol;
};

struct ObGrid {
	ObGrid() {
		x0 = -20; x1 = 20;
		y0 = -20; y1 = 20;
		edge_len = 0.25;    // in meters
		dscaling = 10;
		dmax = 10000;
	}

	int y2RowMin(double y) const { return int((y - y0) / edge_len); }
	int x2ColMin(double x) const { return int((x - x0) / edge_len); }
	int y2RowMax(double y) const { return int((y - y0) / edge_len + 0.5); }
	int x2ColMax(double x) const { return int((x - x0) / edge_len + 0.5); }
	double rowCenter2Y(int row) const { return ((double)row + 0.5) * edge_len + y0; }
	double colCenter2X(int col) const { return ((double)col + 0.5) * edge_len + x0; }
	int nrow() const { return y2RowMax(y1) + 1; }
	int ncol() const { return x2ColMax(x1) + 1; }

	void init_obgrid(impl::MapData& mapinfo, const std::string& saveto = "") {
		obgrid.clear();
		obgrid.resize(nrow() * ncol(), dmax);
		coord = IndexTrans(nrow(), ncol());

		// Add Point Obstacles To Grid
		for (unsigned i = 0; i < mapinfo.obPoint.size(); ++i) {
			newPoint(mapinfo.obPoint[i]);
		}

		// Add Line Obstacles To Grid
		for (unsigned i = 0; i < mapinfo.obLine.size(); ++i) {
			newLine(mapinfo.obLine[i]);
		}

		// Add Poly Obstacles To Grid
		for (unsigned i = 0; i < mapinfo.obCPoly.size(); ++i) {
			newCPoly(mapinfo.obCPoly[i]);
		}

		// Compute Distance
		for (int i = 0; i < 20; ++i) {
			if (!distance_update()) break;
		}

		if (!saveto.empty()) {
			save_grid_image(saveto);
		}
	}
	void save_grid_path(const std::vector<int>& path, const std::string& fname);

	double x0, x1;
	double y0, y1;
	double edge_len;
	double dscaling;
	int dmax;
	IndexTrans coord;
	std::vector<int> obgrid;

private:
	double sqnorm(double dx, double dy) {
		return std::sqrt(dx * dx + dy * dy);
	}
	void newPoint(const MapData::Circle& circle) {
		// Compute Bounding Rect
		int r0, r1, c0, c1;
		r0 = y2RowMin((double)circle.q.y - circle.r);
		r1 = y2RowMax((double)circle.q.y + circle.r);
		c0 = x2ColMin((double)circle.q.x - circle.r);
		c1 = x2ColMax((double)circle.q.x + circle.r);

		// Iterate through the bouding rect and check if it is inside
		for (int r = r0; r <= r1; ++r) {
			for (int c = c0; c <= c1; ++c) {
				if (coord.valid(r, c)) {
					double xm = colCenter2X(c);
					double ym = rowCenter2Y(r);
					double d = sqnorm(xm - circle.q.x, ym - circle.q.y);
					if (d <= circle.r) {
						int index = coord(r, c);
						if (index < (int)obgrid.size()) {
							obgrid[index] = 0;
						}
					}
				}
			}
		}
	}
	void newLine(const MapData::Line& line) {
		double dx = (double)line.q.x - line.p.x;
		double dy = (double)line.q.y - line.p.y;
		double len = sqnorm(dx, dy);
		dx /= len;
		dy /= len;
		double step = edge_len / 5;
		int nstep = int(len / step + 0.5);

		for (int k = 0; k < nstep; ++k) {
			double x = line.p.x + dx * step * k;
			double y = line.p.y + dy * step * k;
			int r0 = y2RowMin(y);
			int c0 = x2ColMin(x);
			if (coord.valid(r0, c0)) {
				int index = coord(r0, c0);
				if (index < (int)obgrid.size()) {
					obgrid[index] = 0;
				}
			}
		}
	}
	void newCPoly(const MapData::CPoly& poly) {
		// Ensure the last point is the first point
		int npoint = (int)poly.size();
		if (npoint < 3) return;
		{
			double diff = sqnorm((double)poly[0].x - poly[npoint - 1].x, poly[0].y - poly[npoint - 1].y);
			if (diff > 0.1) {
				MapData::CPoly p2 = poly;
				p2.push_back(poly[0]);
				return newCPoly(p2);
			}
		}

		// Now there are npoint - 1 different points
		// Compute Bounding Rect
		int r0 = nrow(), r1 = 0, c0 = ncol(), c1 = 0;
		for (int i = 0; i < npoint - 1; ++i) {
			r0 = std::min(r0, y2RowMin(poly[i].y));
			r1 = std::max(r1, y2RowMax(poly[i].y));
			c0 = std::min(c0, x2ColMin(poly[i].x));
			c1 = std::max(c1, x2ColMax(poly[i].x));
		}

		// Iterate through the bouding rect and check if it is inside
		for (int r = r0; r <= r1; ++r) {
			for (int c = c0; c <= c1; ++c) {
				if (coord.valid(r, c)) {
					double xm = colCenter2X(c);
					double ym = rowCenter2Y(r);
					bool outside = false;
					for (int i = 0; i < npoint - 1; ++i) {
						double dx = xm - poly[i].x;
						double dy = ym - poly[i].y;
						double tx = (double)poly[i + 1].x - poly[i].x;
						double ty = (double)poly[i + 1].y - poly[i].y;

						// Left-hand side normal vector is (-ty, tx)
						// which points to the outside of this obstacle
						double dot = -dx * ty + dy * tx;
						if (dot > 0) {
							outside = true;
							break;
						}
					}
					if (!outside) {
						int index = coord(r, c);
						if (index < (int)obgrid.size()) {
							obgrid[index] = 0;
						}
					}
				}
			}
		}
	}
	
	bool distance_update(bool withDiagnoal = true) {
		bool changed = false;
		for (int r = 0; r < coord.nrow; ++r) {
			for (int c = 0; c < coord.ncol; ++c) {
				int& dmin = obgrid[coord(r, c)];
				for (int dr = -1; dr <= 1; ++dr) {
					for (int dc = -1; dc <= 1; ++dc) {
						int r2 = r + dr;
						int c2 = c + dc;
						if (coord.valid(r2, c2)) {
							int d2 = obgrid[coord(r2, c2)];
							double tmp = std::sqrt(((double)dr * dr + (double)dc * dc)) * dscaling;
							int d12 = (tmp <= (double)dmax) ? (int)tmp : dmax;
							if (dmin - d2 > d12) {
								dmin = d2 + d12;
								changed = true;
							}
						}
					}
				}
			}
		}
		return changed;
	}

	void save_grid_image(const std::string& name);
};

} // namespace impl
