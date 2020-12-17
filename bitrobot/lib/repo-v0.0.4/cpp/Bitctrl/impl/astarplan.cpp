#include "astarplan.hpp"
#include <cstdio>

namespace impl {
	
std::vector<int> PlanAstar::plan(std::vector<int>& obmap, int p0, int target, IndexTrans coord, int safe_distance, bool withDiagonal) {
	static std::vector<int> empty;

	int num = (int)obmap.size();
	if (p0 < 0 || p0 >= num) return empty;
	if (target < 0 || target >= num) return empty;

	distMap = std::vector<double>(num, 1e10);
	parent = std::vector<int>(num, -1);	// -1 means uninitialized
	closedset = std::vector<bool>(num, false);

	// Utility functions
	FreeChecker is_free(obmap, safe_distance);
	CostComputer getCost(distMap, coord, target);
	//auto getCost = [&](int p) { return distMap[p] + coord.EucildDistance(p, target); };

	// Check Validity
	if (!is_free(p0)) { puts("Start Point is obstacle."); return empty; }
	if (!is_free(target)) { puts("End Point is obstacle."); return empty; }

	// Initialization
	distMap[p0] = 0;
	push(openset, p0, getCost(p0));

	// Running
	int count = 0;
	bool found = false;
	NeighbourSet neighbours;
	while (!openset.empty() && !found) {
		++count;
		Node curr = pop(openset);
		closedset[curr.index] = true;
		if (curr.index == target) {
			found = true;
			break;
		}

		// Get neighbours
		int nNeb = neighbours.update(curr.index, coord, withDiagonal);

		// Add to openset ...
		double currDis = distMap[curr.index];
		for (int i = 0; i < nNeb; ++i) {
			int next = neighbours.neb[i];

			// Note: For a consistent heurisitc, a node in closedset needs not to be explored twice
			// --> p explored before q means that
			//     g[p] + h[p] <= g[q] + h[q] <= g[q] + h[p] + dist(p, q),  (h is consistent)
			// --> g[p] <= g[q] + dist(p, q)
			// Thus g[p] will not be smaller if it chooses to go from q to p
			if (!is_free(next)) {
				continue;
			}

			// Update distance metric
			double newDistance = currDis + neighbours.edge_length[i];
			if (distMap[next] > newDistance) {
				distMap[next] = newDistance;
				closedset[next] = false;
				distMap[next] = newDistance;

				// Update parent, open/closed set
				parent[next] = curr.index;
				push(openset, next, getCost(next));
			}
		}  // End neb loop
	} // End openset loop


	if (found) {
		printf("Found target. %d steps, mapsize = %dx%d\n", count, coord.ncol, coord.nrow);
		// Found target, make path
		std::vector<int> path;
		int curr = target;
		while (curr != -1) {
			path.push_back(curr);
			curr = parent[curr];
		}
		std::reverse(path.begin(), path.end());
		//return path;
		return rewiseplan(path, obmap, coord, safe_distance);
	}
	printf("Failed to find target. %d steps, mapsize = %dx%d\n", count, coord.ncol, coord.nrow);
	// Failed to found target
	return empty;
}

bool line_check_greater(const std::vector<int>& obmap, int p, int q, IndexTrans coord, int thresh) {
	if (p == q) return obmap.at(p) > thresh;
	if (obmap.at(p) <= thresh || obmap.at(q) <= thresh) return false;

	int r0 = coord.row(p), c0 = coord.col(p);
	int r1 = coord.row(q), c1 = coord.col(q);
	int dr = r1 - r0, dc = c1 - c0;
	double len = int(std::sqrt(dr * dr + dc * dc));
	
	double step_r = (double)dr / len;
	double step_c = (double)dc / len;
	int kmax = int(len);
	for (int i = 0; i < kmax; ++i) {
		int r = int(r0 + step_r * i);
		int c = int(c0 + step_c * i);
		if (coord.valid(r, c)) {
			int index = coord(r, c);
			if (obmap.at(index) <= thresh) {
				return false;
			}
		}
	}
	return true;
}

std::vector<int> PlanAstar::rewiseplan(const std::vector<int>& path, std::vector<int>& obmap, IndexTrans coord, int safe_distance) {
	if (path.size() <= 2) return path;

	const int step_max = -1;
	FreeChecker is_free(obmap, safe_distance);

	std::vector<int> pnew;
	pnew.push_back(path[0]);
	int last0 = path[0];
	int lastp = path[1];

	unsigned npoint = path.size();
	for (unsigned i = 2; i < npoint; ++i) {
		int next = path[i];
		if (line_check_greater(obmap, last0, next, coord, safe_distance)) {
			lastp = next;
		}
		else {
			pnew.push_back(lastp);
			last0 = lastp;
			lastp = next;
		}
		/*
		int c2 = coord.col(test);
		int r2 = coord.row(test);

		// Test if (r2, c2) can be connected to (r0, c0) directly
		bool good = true;
		double dr = (double)r2 - r0;
		double dc = (double)c2 - c0;
		double sz = std::sqrt(dr * dr + dc * dc);
		if (sz <= step_max || step_max < 0) {
			int smax = int(2 * sz);
			dr /= 2 * sz;  dc /= 2 * sz;
			for (int s = 0; s < smax; ++s) {
				int rk = int((double)r0 + s * dr);
				int ck = int((double)c0 + s * dc);
				if (coord.valid(rk, ck)) {
					if (!is_free(coord(rk, ck))) {
						good = false;
						break;
					}
				}
			}
		}
		else {
			good = false;
		}

		if (!good) {
			pnew.push_back(lastp);
			c0 = coord.col(lastp);
			r0 = coord.row(lastp);
		}
		lastp = test;
		*/
	}
	pnew.push_back(lastp);
	return pnew;
}



}
