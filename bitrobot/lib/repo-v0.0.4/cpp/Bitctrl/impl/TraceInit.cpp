#include "../BIT.h"
#include "Base.hpp"
#include "Geotools.hpp"
#include "Combined.hpp"


namespace impl{
/*
struct GeneralConfig{
	double vLine;
	double vCircle;
	double t0;
};




static void buildTrace(ShapeTrace& trace, double vLine, double dzone = 0.01){
	size_t n = trace.xlist.size();
	std::vector<double>& datX = trace.xlist;
	std::vector<double>& datY = trace.ylist;

	trace.tlist.resize(n);
	trace.tlist[0] = 0;
	for (size_t i = 1; i<n; ++i){
		double dx = datX[i] - datX[i - 1];
		double dy = datY[i] - datY[i - 1];
		double sz = std::sqrt(dx*dx + dy*dy);
		if (sz < dzone) sz = 0.01;
		trace.tlist[i] = trace.tlist[i - 1] + sz / vLine;
	}
}

static bool setUpTestEight(Controller* ctrl, sscfg::ConfigFile& config, GeneralConfig* overwirte = nullptr){
    double R = 2;
    double vLine = 0.5;
    int    n = 2;
	
	double anchor[2];
	if (!config.get("eight_latlon0", anchor, 2)){
		GPSGetAnchor(anchor, anchor + 1, nullptr);
		printf("[Ctrl] Warn: Missing eight_latlon0, using default %.8f, %.8f\n", anchor[0], anchor[1]);
	}
	float x0, y0;
	GPS2Local(anchor[0], anchor[1], 0, &x0, &y0, nullptr);
	x0 /= 100;	// Note: shapeTrace accpets m
	y0 /= 100;	//       GPS2Local gives cm

    if (!config.get("eight_Rm", R)){
        printf("[Ctrl] Warn: Missing eight_Rm, using default %.2f\n", R);
    }
    if (!config.get("eight_n", n)){
        printf("[Ctrl] Warn: Missing eight_n, using default %d\n", n);
    }

	if (overwirte && overwirte->vLine > 0){
		vLine = overwirte->vLine;
	}
	else if (!config.get("eight_Vm", vLine)){
        printf("[Ctrl] Warn: Missing eight_Vm, using default %.2f\n", vLine);
    }

    ShapeTrace one = ShapeTrace::Circle(90, R, vLine, 90).shift(0, -R).reverse()
                     + ShapeTrace::Circle(90, R, vLine, -90).shift(0, R);
    ShapeTrace trace;
    for (int i = 0; i<n; ++i){
		trace = trace + one;
    }
	trace = trace.shift(x0, y0);
	
	// Note: we have to set time
	buildTrace(trace, vLine);
	if (overwirte && overwirte->t0 > 0){
		trace = trace.shiftTime(overwirte->t0);
	}
	ctrl->setPath(trace);
    return true;
}

static bool setUpTestLine(Controller* ctrl, sscfg::ConfigFile& config, GeneralConfig* overwirte = nullptr){
    double L = 10;
    double vLine = 0.5;

	double anchor[2];
	if (!config.get("line_latlon0", anchor, 2)){
		GPSGetAnchor(anchor, anchor + 1, nullptr);
		printf("[Ctrl] Warn: Missing line_latlon0, using default %.8f, %.8f\n", anchor[0], anchor[1]);
	}
	float x0, y0;
	GPS2Local(anchor[0], anchor[1], 0, &x0, &y0, nullptr);
	x0 /= 100;		// Note: shapeTrace accpets m
	y0 /= 100;		//       GPS2Local gives cm

    if (!config.get("line_Lm", L)){
        printf("[Ctrl] Warn: Missing line_Lm, using default %.2f\n", L);
    }
	if (overwirte && overwirte->vLine > 0){
		vLine = overwirte->vLine;
	}
	else if (!config.get("line_Vm", vLine)){
        printf("[Ctrl] Warn: Missing line_Vm, using default %.2f\n", vLine);
    }
    ShapeTrace shape = ShapeTrace::Line(0, L, vLine);
	shape = shape.shift(x0, y0);
	
	buildTrace(shape, vLine);
	if (overwirte && overwirte->t0 > 0){
		shape = shape.shiftTime(overwirte->t0);
	}
    ctrl->setPath(shape);
    return true;
}

static bool setUpEightOne(Controller* ctrl, sscfg::ConfigFile& config, GeneralConfig* overwirte = nullptr, bool reverse = false){
	double R = 2, L = 10;
	double vCircle = 0.5;
	double vLine   = 0.5;
	int    n = 1;

	double anchor[2];
	if (!config.get("shp81_latlon0", anchor, 2)){
		GPSGetAnchor(anchor, anchor + 1, nullptr);
		printf("[Ctrl] Warn: Missing shp81_latlon0, using default %.8f, %.8f\n", anchor[0], anchor[1]);
	}
	float x0, y0;
	GPS2Local(anchor[0], anchor[1], 0, &x0, &y0, nullptr);
	x0 /= 100;		// Note: shapeTrace accpets m
	y0 /= 100;		//       GPS2Local gives cm

	//if (overwirte && overwirte->vLine > 0){
	//	vLine = overwirte->vLine;
	//}	else 
	if (!config.get("shp81_VmLine", vLine)){
		printf("[Ctrl] Warn: Missing shp81_VmLine, using default %f\n", vLine);
	}

	//if (overwirte && overwirte->vCircle > 0){
	//	vCircle = overwirte->vCircle;
	//}else 
	if (!config.get("shp81_VmCircle", vCircle)){
		printf("[Ctrl] Warn: Missing shp81_VmCircle, using default %f\n", vCircle);
	}

	if (!config.get("shp81_n", n)){
		printf("[Ctrl] Warn: Missing shp81_n, using default %d\n", n);
	}

	double t0 = 0;
	if (overwirte && overwirte->t0 > 0){
		t0 = overwirte->t0;
	}
	else if (config.get("shp81_dt", t0)){
		printf("[Ctrl] Time shift for shp81 is %.2f seconds\n", t0);
	}

	if (!config.get("shp81_Lm", L)){
		printf("[Ctrl] Warn: Missing shp81_Lm, using default %f\n", L);
	}
	if (!config.get("shp81_Rm", R)){
		printf("[Ctrl] Warn: Missing shp81_Rm, using default %f\n", R);
	}

	ShapeTrace part  = ShapeTrace::Circle(90, R, vCircle, 90).shift(0, -R).reverse()
					 + ShapeTrace::Circle(90, R, vCircle, -90).shift(0, R);
	ShapeTrace eight = part;
	for (int i = 1; i < n; ++i) 
		eight = eight + part;

	ShapeTrace line = ShapeTrace::Line(0, L, vLine);
	if (reverse) line = line.reverse();
	
	buildTrace(line, vLine);
	buildTrace(eight, vCircle);

	ShapeTrace shape;
	if (!reverse){
		line = line.shiftTime(1);
		shape = eight + line;
	}
	else{
		eight = eight.shiftTime(1);
		shape = line + eight;
	}
	if (t0 > 0){
		shape = shape.shiftTime(t0);
	}

	shape = shape.shift(x0, y0);
	ctrl->setPath(shape);
	return true;
}

static bool setUpOneEight(Controller* ctrl, sscfg::ConfigFile& config, GeneralConfig* overwirte = nullptr){
	return setUpEightOne(ctrl, config, overwirte, true);
}
*/

static ShapeTrace Concat(const ShapeTrace& a, const ShapeTrace& b){
	ShapeTrace c;
	size_t na = a.tlist.size();
	size_t nb = b.tlist.size();
	if (na == 0)
		return b;
	if (nb == 0)
		return a;

	size_t nc = na + nb - 1;
	c.tlist.resize(nc);
	c.xlist.resize(nc);
	c.ylist.resize(nc);

	for (size_t i = 0; i < na; ++i){
		c.tlist[i] = a.tlist[i];
		c.xlist[i] = a.xlist[i];
		c.ylist[i] = a.ylist[i];
	}
	
	double dx = b.xlist[0] - a.xlist[na - 1];
	double dy = b.ylist[0] - a.ylist[na - 1];
	double dt = b.tlist[0] - a.tlist[na - 1];
	for (size_t i = 0; i < nb - 1; ++i){
		c.tlist[na + i] = b.tlist[i + 1] - dt;
		c.xlist[na + i] = b.xlist[i + 1] - dx;
		c.ylist[na + i] = b.ylist[i + 1] - dy;
	}
	return c;
}



static ShapeTrace Repeat(int n, const ShapeTrace& trace){
	ShapeTrace res = trace;
	while (n > 1){
		res = Concat(res, trace);
		n--;
	}
	return res;
}

/*
 * setUpFormation, Utility function for setUpTrace
 * Function: 
 *   Read the trace definition with "prefix" from "config";
 *   The resulting trace is stored into "trace" of type ShapeTrace;
 *   Call ctrl->setPath(trace) to load the trace into the controller.
 * Trace definition:
 *   prefix_path: the trace name for this prefix
 *   prefix_dt/spdup/dxy: trace's t0, t_speed, translation
 *   prefix_repeat:   for circular trace, this would produce loops
 *   prefix_latlon0:  anchor point
 *   path_name_x/y/t: local coordinates of the trace 
 */
static bool setUpFormation(Controller* ctrl, const std::string& prefix, sscfg::ConfigFile& config){
	double anchor[2];   // lat, lon
	std::string path_name;
    ShapeTrace  trace;
    std::vector<double>& datX = trace.xlist;
    std::vector<double>& datY = trace.ylist;
	std::vector<double>& datT = trace.tlist;

	// Get trace name for experiment "prefix"
	if (!config.get(prefix + "_path", path_name)){
		printf("[Ctrl] Err: Failed to get %s_path\n", prefix.c_str());
		return false;
	}

	// Get trace shifts for experiment "prefix"
	int   repeat = 1;
	float ratioT = 1.0f;
	float t0 = 0;
	float dxy0[2] = { 0, 0 };
	if (config.get(prefix + "_dt", t0)){
		printf("[Ctrl] Time shift for %s is %.2f seconds\n", prefix.c_str(), t0);
	}
	if (config.get(prefix + "_spdup", ratioT)){
		printf("[Ctrl] Speed up for %s is %.2f\n", prefix.c_str(), ratioT);
	}
	if (config.get(prefix + "_dxy", dxy0, 2) == 2){
		printf("[Ctrl] Origin shift for %s is (%.2f, %.2f)\n", prefix.c_str(), dxy0[0], dxy0[1]);
	}
	else{
		dxy0[0] = dxy0[1] = 0;
	}
	if (config.get(prefix + "_repeat", repeat)){
		if (repeat < 1)
			repeat = 1;
		printf("[Ctrl] Repeat for %s is %d\n", prefix.c_str(), repeat);
	}

	// Get trace anchor for experiment "prefix"
	if (config.get(prefix + "_latlon0", anchor, 2) != 2){
		GPSGetAnchor(anchor, anchor + 1, nullptr);
		printf("[Ctrl] Missing %s_latlon0, using (%.8f, %.8f)\n", prefix.c_str(), anchor[0], anchor[1]);
	}

	/*GeneralConfig c;
	//c.t0 = t0;
	//if (!config.get(prefix+"_Vm", vLine)){
	//   vLine = 0.5;    // Default: m/s
	//    printf("[Ctrl] Warn: Missing %s_Vm, using default %.2f\n", prefix.c_str(), vLine);
	//}
	// Setup as Shapes
	if (path_name == "eight"){
		return setUpTestEight(ctrl, config, &c);
	}
	if (path_name == "line"){
		return setUpTestLine(ctrl, config, &c);
	}
	if (path_name == "eightLine"){
		return setUpEightOne(ctrl, config, &c);
	}
	if (path_name == "lineEight"){
		return setUpOneEight(ctrl, config, &c);
	}
	*/

	// Get corresponding path points
	bool getX = config.get(path_name + "_x", datX) != 0;
	bool getY = config.get(path_name + "_y", datY) != 0;
	bool getT = config.get(path_name + "_t", datT) != 0;

	if (!(getX && getY && getT)){
		printf("[Ctrl] Failed to get %s_x/y/t for exp %s\n", path_name.c_str(), prefix.c_str());
		return false;
	}
	if (datX.size() != datY.size() || datX.size() != datT.size()){
		printf("[Ctrl] Size x(%d), y(%d), t(%d) mismatch for path %s\n", (int)datX.size(), (int)datY.size(), (int)datT.size(), path_name.c_str());
		return false;
	}

    // Note, datX, datY is in cm, so is anchor
    float x0, y0;
    GPS2Local(anchor[0], anchor[1], 0, &x0, &y0, nullptr);
	x0 += dxy0[0];
	y0 += dxy0[1];
    for (size_t i = 0; i < datX.size(); ++i){
        datX[i] = (datX[i] + x0)/100;
        datY[i] = (datY[i] + y0)/100;
		datT[i] = datT[i]/ratioT + t0;
    }
	if (repeat > 1){
		trace = Repeat(repeat, trace);
	}

    ctrl->setPath(trace);
    return true;
}


bool setUpTrace(Controller* ctrl, int expID, int robotID, const std::string& config_dir){
    int expValid[] = {
        EXP_FORMATION, EXP_TRACKING, EXP_INTERVENTION, EXP_TEST_EIGHT, EXP_TEST_LINE
    };
    if (std::find(expValid, expValid + 5, expID) == expValid + 5){
        puts("[Ctrl] Bug in setUpTrace");
        return false;
    }

    std::string fname = config_dir + "shapeConfig.txt";
    std::ifstream config_file(fname.c_str());
    sscfg::ConfigFile config = sscfg::ConfigFile::load(config_file);
    if (!config_file.is_open() ||config.n_items() == 0){
        printf("[Ctrl] Warn: Failed to load shapeConfig.txt at %s\n", config_dir.c_str());
        return false;
    }
    if (expID == EXP_TEST_EIGHT){
        return false;
        //return setUpTestEight(ctrl, config);
    }
    if (expID == EXP_TEST_LINE){
        return false;
        //return setUpTestLine(ctrl, config);
    }
    if (expID == EXP_FORMATION || expID == EXP_INTERVENTION){
        // Group Mode
        std::string prefix;
        if (expID == EXP_FORMATION)    prefix = "group";
        if (expID == EXP_INTERVENTION) prefix = "share";
        if (!prefix.empty()){
            return setUpFormation(ctrl, prefix, config);
        }
    }
    if (expID == EXP_TRACKING){
        // Single Mode
        std::string prefix = "single0";
        prefix[6] += robotID;	// single1, single2, ..., single4
        return setUpFormation(ctrl, prefix, config);
    }

    return false;
}


} // namespace impl
