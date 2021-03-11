#pragma once
#include "EigenQP.hpp"

namespace NSAvoid {


namespace impl {
    template<class Scalar>
    struct util{
        typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VecX;
        typedef typename Eigen::Matrix<Scalar, 2, 1, Eigen::DontAlign> Vec2X;
        typedef typename Eigen::Matrix<Scalar, 2, Eigen::Dynamic, Eigen::DontAlign> Mat2X;

        // Some common variables
        const static Scalar Pi, TwoPi, HalfPi;
        static Scalar inf() {
            return std::numeric_limits<Scalar>::infinity();
        }

		static Scalar delta_theta(Scalar th1, Scalar th0) {
			Scalar c1 = std::cos(th1 - th0);
			Scalar s1 = std::sin(th1 - th0);
			return std::atan2(s1, c1);
		}

        // Angle comparisons
        static bool in_between(Scalar theta, const Scalar& t_min, const Scalar& t_max) {
			Scalar th1 = delta_theta(theta, t_min);
			Scalar thM = delta_theta(t_max, t_min);
			if (th1 < 0) th1 += util::Pi * 2;
			if (thM < 0) thM += util::Pi * 2;
			return th1 > 0 && th1 < thM;
        }

        // Vector conversion
        static Vec2X to2X(const VecX& x) { return Vec2X(x(0), x(1)); }
        static VecX toX(const Vec2X& x) { VecX a(2); a << x(0), x(1); return a; }

        // return if theta is on the right-hand-side of ref_theta
        static bool is_on_right(const Scalar& theta, const Scalar& ref_theta) {
            return in_between(theta, ref_theta - Pi, ref_theta) || (theta == ref_theta);
        }

        // return theta + 2k Pi such that theta in [base_theta, base_theta + 2Pi)
        static Scalar round_angle(Scalar theta, const Scalar& base_theta = 0) {
            while (theta < base_theta) theta += TwoPi;          // Left closed
            while (theta >= base_theta + TwoPi) theta -= TwoPi; // Right open
            return theta;
        }

        // Check if it's a closed polygon in 2D, with pN = p0
        static bool is_closed(const Mat2X& points) {
            int n = points.cols() - 1;  // The last one should be the first, thus in total, n points
            if (n < 3) return false;    // Not even a triangle
            return (points.col(0) - points.col(n)).norm() == 0;
        }

        // Does the convexity check a series of points [p0, ..., pN]
        // requires that p0 = pN (closed in 2D)
        static bool is_convex(const Mat2X& points) {
            if (is_closed(points)) {
                int n = points.cols() - 1;
                VecX heading = heading_angles(points);
                for (int i = 0; i < n; ++i) {
					if (!util::is_on_right(heading(i + 1), heading(i))){
						printf("%.2f is not on the right of %.2f\n", heading(i + 1), heading(i));
						Scalar theta = heading(i + 1);
						Scalar th_max = heading(i);
						Scalar th_min = th_max - util::Pi;
						Scalar dth0 = delta_theta(theta, th_min);
						Scalar dth1 = delta_theta(th_max, th_min);
						bool check = util::in_between(theta, th_min, th_max);
						printf("dth_theta = %.2f, dth_theta_max = %.2f, check = %d\n", dth0, dth1, check);
						return false;
					}
                }
                return true;
            }
            return false;
        }

        // Return the heading angle of each edge
        static VecX heading_angles(const Mat2X& p){
            int n = p.cols() - 1;
            VecX heading(n + 1);
            for (int i = 0; i < n; ++i) {
                Scalar dx = p(0, i+1)-p(0, i);
                Scalar dy = p(1, i+1)-p(1, i);
                heading(i) = round_angle(std::atan2(dy, dx));
            }
            heading(n) = heading(0);
            return heading;
        }
    };

    template<class Scalar>
    const Scalar util<Scalar>::Pi = static_cast<Scalar>(3.141592653589793);

    template<class Scalar>
    const Scalar util<Scalar>::TwoPi = static_cast<Scalar>(3.141592653589793 * 2);

    template<class Scalar>
    const Scalar util<Scalar>::HalfPi = static_cast<Scalar>(3.141592653589793 / 2);
}



}
