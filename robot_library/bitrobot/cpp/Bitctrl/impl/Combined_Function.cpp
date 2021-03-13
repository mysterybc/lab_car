#include "Combined.hpp"
#include <algorithm>
#include <iterator>

namespace impl{
	void append_eight(pt2D* p0, 
		std::vector<double>& tlist, std::vector<double>& xlist, std::vector<double>& ylist,
		double R, double vLine, int nRound, int nPoint, double rotTheta, double tStay) 
	{
		double dw = 2 * pi / nPoint;
		double dt = dw * R / vLine;
	
		if (tlist.empty()) {
			if (p0) {
				tlist.clear(); xlist.clear(); ylist.clear();
				tlist.push_back(0);
				xlist.push_back(p0->x);
				ylist.push_back(p0->y);
			}
			else {
				// This is not normal...
				return;
			}
		}
		
		if (tStay <= 0) {
			tStay = dt;
		}

		pt2D qM(xlist.back(), ylist.back());
		pt2D q0;
		double th;
		double t = tlist.back() + tStay;
		for (int nR = 0; nR < nRound; ++nR) {
			// First Circle
			q0 = pt2D(0, R);
			th = -pi / 2 + dw;
			for (int i = 0; i < nPoint; ++i) {
				pt2D dq(std::cos(th), std::sin(th));
				dq = qM + rot(q0 + R*dq, rotTheta);
				xlist.push_back(dq.x);
				ylist.push_back(dq.y);
				tlist.push_back(t);
				th += dw;
				t += dt;
			}

			// Second Circle
			q0 = pt2D(0, -R);
			th = pi / 2 - dw;
			for (int i = 0; i < nPoint; ++i){
				pt2D dq(std::cos(th), std::sin(th));
				dq = qM + rot(q0 + R*dq, rotTheta);
				xlist.push_back(dq.x);
				ylist.push_back(dq.y);
				tlist.push_back(t);
				th -= dw;
				t += dt;
			}
		}
	}

    //
    // About Shape Selection
    //
    void Combined::initShapeSelection(){
        shapeReceive.clear();
        selected_counter = 0;
		shape_missing_counter  = 0;
		shape_mismatch_counter = 0;
        shape_selected = !use_autoselect;
		if (DebugMode == 1 && use_autoselect){
			shape_selected = true;
			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Auto shape selection is disabled in DebugMode({})", DebugMode);
		}
    }

    void Combined::onFormationChange(){
		tracking.lineShift = dxy_desire[ID];
		consenTrack.dq = consenRend.dq = dxy_desire[ID];
		tracking_single.lineShift = pt2D();
        //tracer.human_shift = tracer_default_shift + dxy[ID] / 100;
	}
    
    char Combined::getMyShapeIndex(){
		std::map<int, int>::iterator it = dxyRemap.find(getMyID());
		if (it != dxyRemap.end()) {
			return it->second;
		}

        LOGWARN("Error: Failed to find my shape!");
        runtimeState = OTHER_STATE_ERROR;
        return -1;
    }
 
	void Combined::selectShape(std::map<int, pt2D>& dxy, const DataSet& dat){
		if (dxy.size() < 5){
			FormationInfo::selectFormation(dxy, dat, dxyRemap);
			LOGDEBUG("Formation Selected");
			for (size_t i = 0; i < dat.ID.size(); ++i){
				int id = dat.ID[i];
				LOGDEBUG("  dxy[{}] is {:5.0f} {:5.0f}", id, dxy[id].x, dxy[id].y);
			}
		}
	}

    // Return ture iff shape_selected change from false -> true
    bool Combined::onSelectFormation(const DataSet& dat){
        if (!shape_selected) {
            selectShape(dxy_base, dat);  // select shape based on everyone's current state

            bool good_match = false;
            size_t swarm_size = dxy_base.size();		// Number of robots in a formation
            if (shapeReceive.size() == swarm_size - 1) {
				shape_missing_counter = 0;

				// 1. Check repetition
				// 2. Check compatibility
                std::set<char> already;		// A shape selection is a uint8, so we store it as a char
                already.insert(getMyShapeIndex());
                std::map<int, char>::iterator iter;
				bool compatiable = true;
                for (iter = shapeReceive.begin(); iter != shapeReceive.end(); ++iter){
                    if (already.find(iter->second) == already.end()){
                        already.insert(iter->second);
                    }
                    else{
						compatiable = false;
                        break;
                    }
                }
				shape_mismatch_counter = compatiable ? 0: shape_mismatch_counter + 1;

                if (already.size() == swarm_size) {
                    // If everyone has desired a different position,
                    // then we're GOOD, shape Matched
                    good_match = true;
                    if (++selected_counter >= 10){
                        // But for the result to be robust
                        // in case anyone changed his mind
                        // we'll wait for 10 rounds of consensus
                        LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Shape Selected.");
                        std::string log_shape = fmt::format("({})-{}", (int)ID, (int)getMyShapeIndex());
                        for (iter = shapeReceive.begin(); iter != shapeReceive.end(); ++iter){
                            log_shape += fmt::format(", ({})-{}", (int)iter->first, (int)iter->second);
                        }
                        LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl]      {}", log_shape);
                        shape_selected = true;
						
						// Load dxy to dxy_desire
						dxy_desire.clear();
						for (std::map<int, int>::iterator p = dxyRemap.begin(); p != dxyRemap.end(); ++p) {
							int id = p->first;
							int id_mapto = p->second;
							dxy_desire[id] = dxy_base[id_mapto];
						}
                    }	
                }
            } // if (shapeReceive.size() == swarm_size - 1)
			else {
				shape_missing_counter++;
			}

			if (shape_mismatch_counter > 0 && shape_mismatch_counter % 10 == 0) {
				LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] WARN: Shape incompatiable for last {} steps.", shape_mismatch_counter);
			}
			if (shape_missing_counter > 0 && shape_missing_counter % 10 == 0) {
				LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] WARN: Missing shape info for last {} steps: {}/{}", 
					shape_missing_counter, shapeReceive.size() + 1, swarm_size);
				std::map<int, char>::iterator it = shapeReceive.begin();
				while (it != shapeReceive.end()) {
					LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] WARN: Missing shape -> ShapeRecv[{}]={}", it->first, (int)it->second);
					++it;
				}
			}

            // We have to get a continuous 10 good match
            // to confirm that shape is selected
            if (!good_match){
                selected_counter = 0;
            }
            return shape_selected;
        }
		return false;
	}

	//
	// About SetFunction, SetPath
	//

	std::string func_name(int funcID){
		switch (funcID)
		{
		case FUNC_UNSET:            return "Unset";
		case FUNC_RENDEZVOUS:		return "Rendezvous";
		case FUNC_TRACKING_SINGLE:  return "Tracking(Single_Mode)";
		case FUNC_TRACKING_GROUP:	return "Tracking(Group_Mode)";
		case FUNC_TRACE_HUMAN:	    return "Trace_Human";
		case FUNC_TURN_TO:		    return "Turn_To";
		case FUNC_TELE_BASIC:       return "Tele_Basic";
		case FUNC_TELE_DXY:         return "Tele_Dxy";
		case FUNC_FAULT_DETECT:		return "Fault_Detect";
		case FUNC_ACT_FAULTY:		return "Act_Faulty";
		case FUNC_HUMAN_TELE:		return "Human_Tele";
		case FUNC_LATENCY_TEST:     return "Latency_Test";
		default:
			return "Undefined";
		}
	}

	void Combined::setTargetPoint(PathPoint pt) {
		target_point = pt2D(pt.x, pt.y);
		target_point.setValid(true);
		target_angle.setValid(false);
		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Get TargetPoint at ({:.2f}, {:.2f}) in function {}", pt.x, pt.y, func_name(base_function()));
	}
	void Combined::setTargetHeading(real_t thDegree, bool isRelative) {
		if (isRelative) thDegree += (real_t)rad2deg(lastData.th[0]);
		target_angle = thDegree;
		target_angle.setValid(true);
		target_point.setValid(false);
		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Get TargetHeading at {:.2f} in function {}", thDegree, func_name(base_function()));
	}

	int  Combined::setFunction(int funcID, bool is_on){
		static const int low_ID[] = {
			FUNC_RENDEZVOUS,
			FUNC_TRACKING_SINGLE,
			FUNC_TRACKING_GROUP,
			FUNC_TRACE_HUMAN,
			FUNC_TURN_TO,
			FUNC_TELE_BASIC,
			FUNC_TELE_DXY,
			FUNC_TEST_EIGHT
		};
		static const int* low_ID_end = low_ID + 8;

		static const int all_high = FUNC_FAULT_DETECT | FUNC_HUMAN_TELE | FUNC_ACT_FAULTY | FUNC_LATENCY_TEST;
		static const int low_mask = 0xff;
		static const int high_mask = 0xff00;

		int high = funcID & all_high;
		int low = funcID & 0xff;

		// What function state should be set at the end of this function
		bool next_valid = false;
		std::pair<int, bool> next_action;

		bool low_good = false;
		bool high_good = false;
		
		int function_test_eight = FUNC_TRACE_HUMAN;
		if (std::find(low_ID, low_ID_end, low) != low_ID_end || low == 0xff){
			if (is_on){
				/*
				0921, do not auto reload
				if (Controller::loadconfig() != 0){
					runtimeState = OTHER_STATE_ERROR;
					return ERR_FUNC_INIT_ERROR;
				}
				*/

				// When Turning ON some basic functions
				if (low == FUNC_TURN_TO) {
					// TURN_TO is different
					int curr_base = base_function();
					if (curr_base != 0) {
						if (curr_base == FUNC_TURN_TO) {
							runtimeState = STATE_GOOD;
							LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Turn ON function {} (running)", func_name(low));
						}
					}
					else {
						functionState = (functionState & high_mask) | low;
						runtimeState = STATE_GOOD;
						LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Turn ON function {}", func_name(low));
					}
					/*
					if ((functionState & low_mask) == 0 || runtimeState == STATE_TARGET_REACHED) {
						// If there's no existing active low functions
						// or if the existing function has finished, 
						// then overwrite existing function
						functionState = (functionState & high_mask) | low;
						runtimeState = STATE_GOOD;
						LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Turn ON function {}", func_name(low));
					}
					else {
						LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Function {} is temporarily ignored", func_name(low));
					}
					*/
				}
				else if ((low == function_test_eight || low == FUNC_TEST_EIGHT) && testEight.enabled) {
					testEight.all_prepared = false;
					testEight.R = std::max(testEight.R, 1.0f);
					testEight.vel = testEight.vel > 0 ?
								    testEight.vel : cm2m(default_vpath);	// in m/s
					if (testEight.vel > 0.8f*cm2m(maxVel())) {
						testEight.vel = 0.8f*cm2m(maxVel());					// Limit vel by maxVel
					}
					if (testEight.vel > deg2rad(maxRotate()) * testEight.R * 0.8f) {
						testEight.vel = deg2rad(maxRotate()) * testEight.R * 0.8f;		// Limit wVel by maxRot
					}

					low = FUNC_TRACKING_GROUP;
					functionState = (functionState & high_mask) | low;
					runtimeState = STATE_GOOD;
					target_point.setValid(false);

					// This the same as reset tracking below
					tracking.reset(lastTime.timeElapsed / 1000.0f);
					reach_heading.reset();
					reach_position.reset();

					// Initialize shape selection
					initShapeSelection();
					LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Turn ON function {} (for test-8)", func_name(low));
				}
				else {
					functionState = (functionState & high_mask) | low;
					runtimeState = STATE_GOOD;
					target_point.setValid(false);

					if (needShapeSelection(low)) {
						initShapeSelection();
					}
					if (low == FUNC_TRACE_HUMAN) {
						//tracer.reset();
						human_pos.setValid(false);
					}
					else if (low == FUNC_TRACKING_SINGLE) {
						tracking_single.reset(lastTime.timeElapsed / 1000.0f);
						reach_heading.reset();
						reach_position.reset();
						//tracking_t = lastTime.timeElapsed / 1000.0f;
						//lineXY.setT0(lastTime.timeElapsed / 1000.0);
					}
					else if (low == FUNC_TRACKING_GROUP) {
						// This part is missing previously... Is it a bug?
						// Previously, setT0 is done when setPath, so... a minor bug it is
						tracking.reset(lastTime.timeElapsed / 1000.0f);
						reach_heading.reset();
						reach_position.reset();
					}
					else if (low == FUNC_TELE_BASIC) {
						teleDirect.setValid(false);
					}
					else if (low == FUNC_TELE_DXY) {
						teleDirect.setValid(false);
					}
					LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Turn ON function {}", func_name(low));
				}
			}
			else{
                // Turn off functions
				lineXY.clear();
				human_pos.setValid(false);
				target_angle.setValid(false);
				target_point.setValid(false);
				ivLast.setValid(false);
				teleDirect.setValid(false);
				teleDirectVxy.setValid(false);
				functionState = functionState & high_mask;
				runtimeState = STATE_GOOD;
				if (low == 0xff) {
					LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Turn OFF all LOW functions");
				}
				else {
					LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Turn OFF function {}.", func_name(low));
				}
			}
			
			// Every time the main function changed
			current_progress = 0.0f;
			low_good = true;
		}
		/*
		if (low == 0xff && !is_on){
            // Turn OFF all low-functions
			functionState = functionState & high_mask;
			runtimeState = STATE_GOOD;

			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Turn OFF all functions");
			low_good = true;
		}
		*/
		if (high != 0) {
			if (high & FUNC_FAULT_DETECT) {
				iamfaulty = false;
                if (is_on) {
                    // Reset Fault Dectection Algo
                    errList.clear();
                    fault_xy.reset();
                    fault_th.reset();
                    last_predict.setValid(false);

                    // If use autofault, the fault agent will act faulty
                    if (auto_faulty.find(ID) != auto_faulty.end() && use_autofault){
                        high |= FUNC_ACT_FAULTY;
						tFautlStart = lastTime;
                        LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Turn ON function ActFaulty (auto mode)");
                    }
					LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] ErrList is cleared");

                    // Turn off human field in the next action
                    // next_valid = true;
                    // next_action.first  = FUNC_HUMAN_TELE;
                    // next_action.second = false;
                } // if (is_on)
			} // if (high & FUNC_FAULT_DETECT)
			if (high & FUNC_HUMAN_TELE) {
                if (is_on){
                    human_field.reset();
                    hfNet.clear();
                    hfMe = HumanField();
                    hiLast.setValid(false);
                    ivLast.setValid(false);
                }
			}
			if ((high & FUNC_ACT_FAULTY) && is_on) {
				// Nothing needs to be reset
				tFautlStart = lastTime;
			}
			if ((high & FUNC_LATENCY_TEST) && is_on) {
				if (!latency_test.reset()){
					LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] LatencyTest config error, failed to Turn ON");
				}
			}

			if (is_on){
				functionState |= high;
				LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Turn ON function {}", func_name(high));
			}
			else {
				functionState &= ((~high) | low_mask);
				if (high != all_high)
					LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Turn OFF function {}", func_name(high));
				else {
					errList.clear();
					iamfaulty = false;
					LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Turn OFF all HIGH functions");
					LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] ErrList is cleared");
				}
			}
			high_good = true;
		} // if (high != 0)

		if (high_good || low_good) { 
			if (next_valid){
				LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] SetFunction continues on {}: {}", 
					func_name(next_action.first), (next_action.second ? "ON": "OFF"));
				return setFunction(next_action.first, next_action.second);
			}
            // Reload basic configs everytime setting ON functions
            //if (is_on && Controller::loadconfig() != 0){
			//	runtimeState = OTHER_STATE_ERROR;
			//	return ERR_FUNC_INIT_ERROR;
			//}
			
			return 0; 
		} // if (high_good || low_good)
		else {
			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Error: Invalid FuncID {} in ControllerSetFunction", funcID);
			return ERR_INVALID_FUNCID;
		}
	}

    // This internal interface should be m, and m/s
    void Combined::setPath(const ShapeTrace& trace){
        lineXY.setpoints(trace);
        lineXY.setAutoLoop(false);
        lineXY.setAutoStop(true);
        lineXY.setT0(lastTime.timeElapsed / 1000.0);
        runtimeState = 0;
    }

	

	void Combined::setPath(uint num, const PathPoint* path){
		PathPoint tmp_path[2];
		pt2D last(0, 0);
		uint istart = 0;
		if (num < 2) {
			istart = 0;
			if (!lastData.xy.empty()) {
				last = lastData.xy[0];
			}
			else { 
				return; 
			}
		}
		else {
			istart = 1;
			last.x = path[0].x;
			last.y = path[0].y;
		}
		
		// Logging
		if (num > 0){	
			if (num > 10){
				LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Received {} path points, from ({}, {}) to ({}, {}), current pos is ({}, {}).",
					num, path[0].x, path[0].y, path[num - 1].x, path[num - 1].y, last.x, last.y);
			}
			else{
				LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Received {} path points, current pos is ({}, {}):", num, last.x, last.y);
				for (uint i = 0; i < num; ++i){
					LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl]\t({:.2f}, {:.2f})\n", path[i].x, path[i].y);
				}
			}
		}
		else{
			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Received 0 path points.");
			tmp_path[0].v = 1;
			tmp_path[0].x = (real_t)lastData.xy[0].x;
			tmp_path[0].y = (real_t)lastData.xy[0].y;
			tmp_path[1] = tmp_path[0];
			path = tmp_path;
			num  = 2;

			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] --Using a fake path, from ({}, {}) to ({}, {}), current pos is ({}, {}).",
				path[0].x, path[0].y, path[num - 1].x, path[num - 1].y, last.x, last.y);
		}

		ShapeTrace trace;
		double tstop = 1.0;			// Time spent at stop points
		double step_size = 0.3;		// Automatically add inter points every "step_size" meters
		double step_max = 1;		// Maximamum inter points step_size (terminal condition)
		size_t num_stop = 0;		// Counter of stop points

		double path_length = 0;
		//double vel = default_vpath; // Force use the default vel
		//if (vel > 0.8f*maxVel()) {
		//	vel = 0.8f*maxVel();
		//}
		//vel = 20; // 0.2cm/s for testing, 2020/09/11, Chengsi Shang
		//vel /= 100;		// to m/s
		//LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] VPath = {:.2f} m/s", vel);

		trace.tlist.push_back(0);
		//trace.xlist.push_back(cm2m(path[0].x));
		//trace.ylist.push_back(cm2m(path[0].y));
		trace.xlist.push_back(cm2m(last.x));
		trace.ylist.push_back(cm2m(last.y));
		for (uint i = istart; i < num; ++i) {
			pt2D q0(trace.xlist.back(), trace.ylist.back());
			pt2D q1(cm2m(path[i].x), cm2m(path[i].y));
			pt2D dir = normalize(q1 - q0);

			double dis = norm2(q1 - q0);
			if (dis <= 0.01) {
				// This is a STOP Point
				double t0 = trace.tlist.back();
				trace.tlist.push_back(t0 + tstop);
				trace.xlist.push_back(trace.xlist.back());
				trace.ylist.push_back(trace.ylist.back());
				num_stop++;
			}
			else {
				double L = dis;
				double lastT = trace.tlist.back();
				double lastX = trace.xlist.back();
				double lastY = trace.ylist.back();
				double step;
				double vel = default_vpath;
				if (path[i].v > 0) {
					vel = cm2m(path[i].v);
				}
				vel = std::min(vel, 0.8 * maxVel());

				bool first_step = false;
				while (L > 0) {
					if (first_step) {
						step = std::min(L, step_max / 2);
						first_step = false;
					}
					else {
						if (L > step_max) step = step_size;
						else if (L >= step_max / 2) step = step_max / 2;
						else step = L;
					}
					

					lastT += step / vel;
					lastX += step * dir.x;
					lastY += step * dir.y;
					L -= step;
					trace.tlist.push_back(lastT);
					trace.xlist.push_back(lastX);
					trace.ylist.push_back(lastY);
					path_length += step;
				}
			}
		}

		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl]   generated {} path points, from ({:.2f}, {:.2f}) to ({:.2f}, {:.2f})",
			trace.tlist.size(), trace.xlist[0], trace.ylist[0], trace.xlist.back(), trace.ylist.back());
		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl]   with rangeX [{:.2f}, {:.2f}], rangeY [{:.2f}, {:.2f}], length = {:.2f}",
			impl::min(trace.xlist), impl::max(trace.xlist), impl::min(trace.ylist), impl::max(trace.ylist), path_length);
		if (num_stop > 0){
			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl]   with {} stop points, stop time set to {}s", num_stop, tstop);
		}
		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl]   time span is {}", trace.tlist.back());

		lineXY.setpoints(trace);
		lineXY.setAutoLoop(false);
		lineXY.setAutoStop(true);
		lineXY.setT0(lastTime.timeElapsed / 1000.0);
		runtimeState = 0;
	}


	void Combined::logtrace(const TimeInfo& t, const DataSet& dat, const ControlInfo& u) {
		//return;
		FILE* trace = getExtTraceFile();
		if (trace) {
			fprintf(trace, "%6d ", t.timeElapsed);
			fprintf(trace, "%7.2f %7.2f %6.2f ", dat.xy[0].x, dat.xy[0].y, rad2deg(dat.th[0]));
			fprintf(trace, "%6.2f %6.2f ", dat.vw[0].x, rad2deg(dat.vw[0].y));
			fprintf(trace, "%6.2f %6.2f ", u.v, u.w);
			fprintf(trace, "\n");
			if (t.loopCounter % 10 == 0) {
				fflush(trace);
			}
		}
	}


	bool Combined::computePath(std::vector<PathPoint>& path, pt2D q0, pt2D q1, real_t v, real_t safe) {
		if (obgrid.obgrid.empty()) {
			// No mapdata, return the trival path
			path.resize(2);
			path[0].x = (real_t)q0.x; path[0].y = (real_t)q0.y; path[0].v = v;
			path[1].x = (real_t)q1.x; path[1].y = (real_t)q1.y; path[1].v = v;
			return true;
		}

		IndexTrans& coord = obgrid.coord;
		int c0 = obgrid.x2ColMin(cm2m(q0.x));
		int r0 = obgrid.y2RowMin(cm2m(q0.y));
		int c1 = obgrid.x2ColMin(cm2m(q1.x));
		int r1 = obgrid.y2RowMin(cm2m(q1.y));
		int p0 = coord(r0, c0);
		int p1 = coord(r1, c1);
		int kmin = int(cm2m(safe) / obgrid.edge_len * obgrid.dscaling + 0.5);
		
		std::vector<int> p = planner.plan(obgrid.obgrid, p0, p1, coord, kmin, true);
		if (!p.empty()) {
			static int nplanned = 0;
			std::string fname = "path-00.bmp";
			if (nplanned < 10) fname[6] = '0' + nplanned;
			else if (nplanned < 99) {
				fname[5] = '0' + (nplanned / 10);
				fname[6] = '0' + (nplanned % 10);
			}
			nplanned = (nplanned + 1) % 100;
			obgrid.save_grid_path(p, fname);

			path.resize(p.size());
			for (unsigned i = 0; i < p.size(); ++i) {
				int index = p[i];
				int c = coord.col(index);
				int r = coord.row(index);
				path[i].x = (real_t)m2cm(obgrid.colCenter2X(c));
				path[i].y = (real_t)m2cm(obgrid.rowCenter2Y(r));
				path[i].v = v;
			}
			return true;
		}
		return false;
	}

	bool Combined::computePath(std::vector<PathPoint>& path, const std::vector<PathPoint>& target, real_t safe, bool startAtMe) {
		unsigned ntarget = target.size();
		if (startAtMe && lastData.empty()) {
			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] ComputePath Failed, lastData is Empty");
			return false;
		}
		if ((startAtMe && ntarget < 1) || (!startAtMe && ntarget < 2)) {
			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] ComputePath Failed, target size too small");
			return false;
		}

		PathPoint q0 = target[0];
		if (startAtMe) {
			q0.x = (real_t)lastData.xy[0].x;
			q0.y = (real_t)lastData.xy[0].y;
			q0.v = target[0].v;
		}

		if (obgrid.obgrid.empty()) {
			// No mapdata, return the trival path
			if (startAtMe) path = target;
			else {
				path.resize(ntarget + 1);
				path[0] = q0;
				for (unsigned i = 0; i < ntarget; ++i) {
					path[i + 1] = target[i];
				}
			}
			return true;
		}

		path.clear();
		path.push_back(q0);

		std::vector<PathPoint> segment;
		unsigned indexNext = startAtMe ? 0 : 1;
		for (; indexNext < ntarget; ++indexNext) {
			pt2D beg(q0.x, q0.y);
			pt2D end(target[indexNext].x, target[indexNext].y);
			segment.clear();
			if (computePath(segment, beg, end, target[indexNext].v, safe)) {
				// Will skip the first element, since it is the end of the last element
				std::copy(segment.begin() + 1, segment.end(), std::back_inserter(path));
			}
			else break;
			
			q0 = target[indexNext];
		}

		return !path.empty();
	}

}   // namespace impl
