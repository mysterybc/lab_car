#pragma once
#include "Obstacles.hpp"

namespace impl {
	struct NeighbourSet {
		int update(int index, const IndexTrans& coord, bool withDiagonal = true) {
			neb.clear();
			edge_length.clear();
			int r = coord.row(index), c = coord.col(index);
			static const int dr[] = { -1, 1, 0, 0, -1, -1, 1, 1 };
			static const int dc[] = { 0, 0, -1,1, -1, 1, -1, 1 };
			int end = withDiagonal ? 8 : 4;

			for (int i = 0; i < end; ++i) {
				if (check_add(r + dr[i], c + dc[i], coord)) {
					edge_length.push_back(i < 4 ? 1 : 1.4142);
				}
			}
			return (int)neb.size();
		}

		std::vector<int> neb;
		std::vector<double> edge_length;
	private:
		bool check_add(int i, int j, const IndexTrans& coord) {
			if (coord.valid(i, j)) {
				neb.push_back(coord(i, j));
				return true;
			}
			return false;
		}
	};

	struct PlanAstar {
		struct Node {
			Node(): index(0), cost(0) {}
			Node(int p, double c) : index(p), cost(c) {}
			int index; double cost;
		};
		struct FreeChecker {
			FreeChecker(const std::vector<int>& obmap, int safe_distance) : obmap(obmap), safe_distance(safe_distance) {}
			bool operator() (int index) const { return obmap.at(index) > safe_distance; }

			const std::vector<int>& obmap;
			int safe_distance;
		};
		struct CostComputer {
			CostComputer(const std::vector<double>& distmap, IndexTrans coord, int target)
				: distmap(distmap), coord(coord), target(target) {}
			double operator() (int p) const { return distmap.at(p) + coord.EucildDistance(p, target); }

			const std::vector<double>& distmap;
			IndexTrans coord;
			int target;
		};

		std::vector<int> plan(std::vector<int>& obmap, int p0, int target, IndexTrans coord, int safe_distance, bool withDiagonal = true);
		
		std::vector<int>  parent;
		std::vector<bool> closedset;
		std::vector<double> distMap;
		std::vector<Node> openset;

	private:
		std::vector<int> rewiseplan(const std::vector<int>& path, std::vector<int>& obmap, IndexTrans coord, int safe_distance);
		static bool comp(const Node& p, const Node& q) { return p.cost > q.cost; }
		static void push(std::vector<Node>& s, int p, double cost) {
			s.push_back(Node(p, cost));
			std::push_heap(s.begin(), s.end(), comp);
		}
		static Node pop(std::vector<Node>& s) {
			Node r = s[0];
			std::pop_heap(s.begin(), s.end(), comp);
			s.erase(--s.end());
			return r;
		}
	};
}
