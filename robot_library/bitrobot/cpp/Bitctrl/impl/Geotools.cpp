#include "Geotools.hpp"
#include "Base.hpp"
#include <algorithm>

namespace impl{

double angleInRange(double angle){
	while (angle >= pi) angle -= 2 * pi;
	while (angle < -pi) angle += 2 * pi;
	return angle;
}

std::vector<pt2D> FormationInfo::getSequenceDxy(const std::map<int, pt2D>& dxy, const DataSet& dat){
    // return only the portion in dxy with ID of dat's ID
    // ENSURE that sdxy.size <= dat.size,   09/21
    std::vector<pt2D> sdxy;
    for (size_t i = 0; i < dat.size(); ++i){
		int id = dat.ID.at(i);
		std::map<int, pt2D>::const_iterator id_dxy = dxy.find(id);
		if (id_dxy != dxy.end()) {
			sdxy.push_back(id_dxy->second);
		}
	}
    return sdxy;
}

double evaluate_formation(const pt2D& center, const std::vector<pt2D>& xy, const std::vector<pt2D>& sdxy) {
	size_t n = std::min(xy.size(), sdxy.size());
	double err = 0;
	for (size_t i = 0; i < n; ++i) {
		pt2D e = xy[i] - (center + sdxy[i]);
		err += norm2(e);
	}
	return n > 0 ? err / n : 0;
}

void FormationInfo::setData(const std::map<int, pt2D>&  sdxy, const DataSet& dat){
	const std::vector<pt2D>& xy = dat.xy;

	std::vector<int> valid_datIndex;
	for (size_t i = 0; i < dat.size(); ++i) {
		int oneID = dat.ID[i];
		if (sdxy.find(oneID) != sdxy.end()) {
			valid_datIndex.push_back(i);
		}
	}
	size_t sz = valid_datIndex.size();

	reserveAndResize(sz, pts, ID);
	center = pt2D(0, 0);
	centerPhy = pt2D(0, 0);
	for (size_t i = 0; i < sz; ++i){
		int datIndex = valid_datIndex[i];
		int robotID = dat.ID[datIndex];
		ID[i] = robotID;
		pts[i] = xy.at(datIndex) - sdxy.at(robotID);
		center += pts[i];
		centerPhy += xy.at(datIndex);
	}
	center /= sz;
	centerPhy /= sz;

	
	pt2D heading = pt2D();
	for (size_t i = 0; i < sz; ++i){
		int datIndex = valid_datIndex[i];
		double th = dat.th.at(datIndex);
		heading += pt2D(std::cos(th), std::sin(th));
	}
	heading /= sz;
	centerTh = angleInRange(atan2(heading.y, heading.x));

	reserveAndResize(sz, eth, epos);
	for (size_t i = 0; i < sz; ++i){
		int datIndex = valid_datIndex[i];
		int robotID = dat.ID[datIndex];
		eth[i]  = std::abs(anglediff(dat.th.at(datIndex), centerTh));
		epos[i] = norm2(center + sdxy.at(robotID) - xy.at(datIndex));
	}
		
	getStats(thError, eth);
	getStats(posError, epos);
}
/*
void FormationInfo::setData(const std::map<int, pt2D>& sdxy, const DataSet& dat) {
	const std::vector<pt2D>& xy = dat.xy;


	size_t sz = dat.size();
	reserveAndResize(sz, pts);
	for (size_t i = 0; i < sz; ++i) {
		pts[i] = xy.at(i) - sdxy.at(i);
	}

	center = mean(pts);
	centerPhy = mean(dat.xy);

	pt2D heading = pt2D();
	for (size_t i = 0; i < sz; ++i) {
		double th = dat.th.at(i);
		heading += pt2D(std::cos(th), std::sin(th));
	}
	heading /= sz;
	centerTh = angleInRange(atan2(heading.y, heading.x));

	reserveAndResize(sz, eth, epos);
	for (size_t i = 0; i < sz; ++i) {
		eth[i] = std::abs(anglediff(dat.th.at(i), centerTh));
		epos[i] = norm2(center + sdxy.at(i) - xy.at(i));
	}

	getStats(thError, eth);
	getStats(posError, epos);
}
*/

void FormationInfo::selectFormation(const std::map<int, pt2D>& dxy, const DataSet& dat, std::map<int, int>& idremap) {
	// Input: dxy size n_swarm
	//        dat size n_receive,
	// Require: dat.size <= dxy.size
	std::vector<pt2D> xy, sdxy;
	std::vector<int> id;
	for (unsigned i = 0; i < dat.size(); ++i) {
		std::map<int, pt2D>::const_iterator one = dxy.find(dat.ID[i]);
		if (one != dxy.end()) {
			xy.push_back(dat.xy[i]);
			sdxy.push_back(one->second);
			id.push_back(one->first);
		}
	}
	size_t n = sdxy.size();
	if (n < dxy.size()) {
		// Note, not enough data to perform shape selection
		loginfo(DEBUG_INFO_FUNCTION, "[SelectFormation] Not Enough Data");
		return;
	}
	// loginfo(DEBUG_INFO_FUNCTION, "[SelectFormation] dxy.size={}, sdxy.size={}", dxy.size(), sdxy.size());


	pt2D center = mean(xy); // Potential Defect,
							// for algorithm to converge, we must have the same formation center
	//std::vector<pt2D> best = sdxy;
	double errBest = evaluate_formation(center, xy, sdxy); // FormationInfo(sdxy, dat).posError.meansq2;

	// The job is to find a permutation to sdxy
	// So that the err is minimized
	std::vector<size_t> idlist(n), idbest(n);
	for (size_t i = 0; i < n; ++i) {
		idlist[i] = i;
	}

	std::vector<pt2D> sdxy2;
	reserveAndResize(n, sdxy2);
	while (std::next_permutation(idlist.begin(), idlist.end())) {
		for (size_t i = 0; i < n; ++i) {
			sdxy2[i] = sdxy[idlist[i]];
		}

		double err = evaluate_formation(center, xy, sdxy2);
		if (err < errBest) {
			errBest = err;
			idbest = idlist;
			//best = sdxy2;
		}
	}

	// Now we wirte the changes back to dxy
	idremap.clear();
	for (size_t i = 0; i < n; ++i) {
		idremap[id[i]] = id[idbest[i]];
	}
	//return FormationInfo(best, dat);
}

/*
void FormationInfo::selectFormation(std::map<int, pt2D>& dxy, const DataSet& dat){
    // Input: dxy size n_swarm
    //        dat size n_receive,
    // Require: dat.size <= dxy.size
    pt2D center = mean(dat.xy);     // Potential Defect,
                                    // for algorithm to converge, we must have the same formation center
    std::vector<pt2D> sdxy = getSequenceDxy(dxy, dat);  // Size of sdxy = dat.size
	std::vector<pt2D> best = sdxy;

	size_t n = sdxy.size();
    double errBest = evaluate_formation(dat, center, sdxy); // FormationInfo(sdxy, dat).posError.meansq2;

	// The job is to find a permutation to sdxy
	// So that the err is minimized
	std::vector<size_t> idlist;
	for (size_t i = 0; i < n; ++i){
		idlist.push_back(i);
	}
	while (std::next_permutation(idlist.begin(), idlist.end())){
		std::vector<pt2D> sdxy2;
		reserveAndResize(n, sdxy2);
		for (size_t i = 0; i < n; ++i){
			sdxy2[i] = sdxy[idlist[i]];
		}

        double err = evaluate_formation(dat, center, sdxy2);
		if (err < errBest){
			errBest = err;
			best = sdxy2;
		}
	}

    // Now we wirte the changes back to dxy
	for (size_t i = 0; i < dat.size(); ++i){
		int id = dat.ID[i];
        dxy[id] = best[i];  // Note, only dxy of id in dat.ID will be changed
	}
	//return FormationInfo(best, dat);
}
*/

ShapeTrace ShapeTrace::Line(double headingInDegree, double length, double vline){
	ShapeTrace shape;
	double dx = vline * std::cos(deg2rad(headingInDegree));
	double dy = vline * std::sin(deg2rad(headingInDegree));

	int npoints = 3;
	for (int i = 0; i < npoints; ++i){
		double t = length / vline / (npoints - 1) * i;
		shape.tlist.push_back(t);
		shape.xlist.push_back(dx*t);
		shape.ylist.push_back(dy*t);
	}
	return shape;
}

ShapeTrace ShapeTrace::Circle(int npoints, double R, double vline, double startAt){
	ShapeTrace shape;
	double dth = 2 * pi / npoints;
	double dt = dth * R / vline;

	double t = 0;
	double th = deg2rad(startAt);
    // The tail needs to be the head
    // so one extra point
	for (int i = 0; i <= npoints; ++i){
		shape.tlist.push_back(t);
		shape.xlist.push_back(R*std::cos(th));
		shape.ylist.push_back(R*std::sin(th));
		th += dth;
		t += dt;
	}
	return shape;
}

}
