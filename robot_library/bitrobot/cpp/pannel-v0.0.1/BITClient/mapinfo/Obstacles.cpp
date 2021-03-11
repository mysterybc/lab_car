#include "Obstacles.hpp"
#include "ssconfig.hpp"
#include <fstream>

namespace impl {


size_t MapData::load(const std::string& name, bool append) {
	std::ifstream file(name.c_str());
	sscfg::ConfigFile conf = sscfg::ConfigFile::load(file);

	if (!append) {
		clear();
	}

	size_t n_old = size();
	if (conf.n_items() > 0) {
		std::vector<std::string> prefix;
		conf.get("obList", prefix);
        obUnit = 1.0f;
		if (conf.exist("obUnit")) {
            conf.get("obUnit", obUnit);
		}

        float unit = obUnit;
		size_t n = prefix.size();
		for (size_t i = 0; i < n; ++i) {
			// Load Circle
			if (conf.exist(prefix[i] + "_CircleX")) {
				std::vector<float> x, y, r;
				conf.get(prefix[i] + "_CircleX", x);
				conf.get(prefix[i] + "_CircleY", y);
				conf.get(prefix[i] + "_CircleR", r);
				if (x.size() != y.size() || x.size() != r.size()) {
					printf("Warning: Get mismatched Circle Obstacle for %s\n", prefix[i].c_str());
				}
				else {
                    Circle one;
					for (size_t k = 0; k < x.size(); ++k) {
                        one.q = { x[k] / unit, y[k] / unit };
						one.r = r[k] / unit;
						obPoint.push_back(one);
					}
				}
			}

            // Load Lines
            if (conf.exist(prefix[i] + "_LineX")) {
                std::vector<float> x, y;
                std::vector<int> num;
                conf.get(prefix[i] + "_LineX", x);
                conf.get(prefix[i] + "_LineY", y);
                conf.get(prefix[i] + "_LineNum", num);

                if (x.size() != y.size() || num.size() == 0) {
                    printf("Warning: Get mismatched Line Obstacle for %s\n",  prefix[i].c_str());
                }
                else {
                    size_t nLine = num.size();
                    size_t base = 0;
                    for (size_t i = 0; i < nLine; ++i){
                        size_t nSeg = num[i];
                        for (size_t j = 0; j < nSeg - 1; ++j){
                            Point p = { x[base + j] / unit, y[base + j] / unit };
                            Point q = { x[base + j + 1] / unit, y[base + j + 1] / unit };
                            obLine.push_back(Line{p, q});
                        }
                        base += nSeg;
                    }
                }
            }

			// Load Poly
			if (conf.exist(prefix[i] + "_CPolyX")) {
				std::vector<float> px, py;
				std::vector<int>   num;
				conf.get(prefix[i] + "_CPolyX", px);
				conf.get(prefix[i] + "_CPolyY", py);
				conf.get(prefix[i] + "_CPolyNum", num);
				if (px.size() != py.size() || px.size() < num.size()) {
					printf("Warning: Get mismatching CPoly Obstacle for %s\n", prefix[i].c_str());
				}
				else {
					size_t npoly = num.size();
					size_t base = 0;
					for (size_t i = 0; i < npoly; ++i) {
						CPoly one(num[i]);
                        for (int j = 0; j < num[i]; ++j) {
                            one[j] = Point{ px[base + j] / unit, py[base + j] / unit };
						}
						
						// Check if it is closed
						// if not, close it
						Point& p0 = *one.begin();
						Point& p1 = *(one.end() - 1);
						if (p0.x != p1.x || p0.y != p1.y) {
							one.push_back(p0);
						}
						obCPoly.push_back(one);
						base += num[i];
					}
				}
			}
		}
	}
	return size() - n_old;
}





}
