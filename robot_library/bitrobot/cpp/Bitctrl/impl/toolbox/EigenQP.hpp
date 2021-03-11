#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include "LoggingUtil.hpp"

namespace EigenQuad {
    enum {
        QPErr_Good = 0,
        QPErr_EqconsLinear,
        QPErr_Infeasible,
        QPErr_Unknown
    };

    template<class Scalar>
    class QPResult{
    public:
        typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatX;
        typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VecX;
        typedef typename Eigen::LLT<MatX, Eigen::Lower> LowerChol;
        typedef typename Eigen::VectorXi VecXi;

        QPResult(){
            state    = QPErr_Unknown;
            n_active = 0;
        }
        const char* StateInfo() const{
            switch (state) {
            case QPErr_Good:         return "QPErr_Good";
            case QPErr_EqconsLinear: return "QPErr_EqconsLinear";
            case QPErr_Infeasible:   return "QPErr_Infeasible";
            case QPErr_Unknown:      return "QPErr_Unkown";
            default:                 return "QPErr_Unkown";
            }
        }

        int    state;       // Solver State
        VecX   x;           // Optimal x
        Scalar min_value;	// Optimal value
        int    n_active;	// Number of active constraints
        VecXi  active_set;	// Active set (index)
        VecX   lagrange;	// Lagrange multipliers of active constraints
        VecX   residue;     // Residues of constraints, s(i) < 0 if ith-cons is violated
        VecXi  redundant;   // Which of the CE constraints are linear dependent to others
    };

    template<class Scalar>
    class QPProblem{
    public:
        typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatX;
        typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VecX;
        typedef typename Eigen::VectorXi VecXi;

        QPProblem relax(const VecX& wCE, const VecX& wCI) const {
            QPProblem p;
            VecXi ke, ki;
            ke.setZero(CE.cols());
            ki.setZero(CI.cols());

            int i, j;
            int ne = 0, ni = 0;
            for (i = 0, j = 0; i < wCE.size(); ++i){
                if (wCE(i) > 0) { ke(j) = i; j++; ne++; }
            }
            for (i = 0, j = 0; i < wCI.size(); ++i){
                if (wCI(i) > 0) { ki(j) = i; j++; ni++; }
            }

            int n0 = G.cols();
            int n = n0 + ne + ni;
            p.G.setZero(n, n);
            p.G.topLeftCorner(n0, n0) = G;
            for (i = 0; i < ne; ++i) {
                p.G(n0+i, n0+i) = wCE(ke(i));
            }
            for (i = 0; i < ni; ++i) {
                p.G(n0+ne+i, n0+ne+i) = wCI(ki(i));
            }

            p.g0.setZero(n);
            p.g0.head(n0) = g0;

            p.CE.setZero(n, CE.cols());
            p.CE.topRows(n0) = CE;
            for (i = 0; i < ne; ++i) {
                p.CE(n0 + i, ke(i)) = 1;
            }
            p.ce0 = ce0;

            p.CI.setZero(n, CI.cols());
            p.CI.topRows(n0) = CI;
            for (i = 0; i < ni; ++i) {
                p.CI(n0 + ne + i, ki(i)) = 1;
            }
            p.ci0 = ci0;

            return p;
        }
        MatX G,  CE,  CI;
        VecX g0, ce0, ci0;
    };

    /*
     * Solver Interface
     */
    // Solve QP without a lower chol decomp
    //  min  0.5 * x G x + g0 x
    // s.t.  CE^T x + ce0 = 0
    //	     CI^T x + ci0 >= 0
    template<class Scalar>
    class QPSolver {
    public:
        typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatX;
        typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VecX;
        typedef typename Eigen::LLT<MatX, Eigen::Lower> LowerChol;
        typedef typename Eigen::VectorXi VecXi;
        typedef QPResult<Scalar> Solution;
        typedef QPProblem<Scalar> Problem;

        Solution operator ()(const Problem& p) const {
            const MatX& G = p.G;
            LowerChol chol(G.cols());
            Scalar c1;

            // compute the trace of the original matrix G
            c1 = G.trace();

            // decompose the matrix G in the form LL^T
            chol.compute(G);

            return (*this)(p, chol, c1);
        }

        // Solve QP with a chol
        // const LowerChol &chol, Scalar c1, const VecX & g0, const MatX & CE, const VecX & ce0, const MatX & CI, const VecX & ci0
        Solution operator () (const Problem& prob, const LowerChol& chol, Scalar c1) const {
            const Scalar inf = std::numeric_limits<Scalar>::infinity();
            const Scalar esp = std::numeric_limits<Scalar>::epsilon();
            const Scalar Zero = (Scalar)0;
            const Scalar One  = (Scalar)1;
            const Scalar Half = (Scalar)0.5;

            Solution ret;
            int    &state   = ret.state;
            Scalar &f_value = ret.min_value;
            int    &q = ret.n_active;
            VecXi  &A = ret.active_set;
            VecX   &u = ret.lagrange;
            VecX   &s = ret.residue;
            VecX   &x = ret.x;
            VecXi  &redun = ret.redundant;

            const MatX& CE = prob.CE;
            const MatX& CI = prob.CI;
            const VecX& ce0 = prob.ce0;
            const VecX& ci0 = prob.ci0;
            const VecX& g0 = prob.g0;

            // Initialize Solver State
            state = QPErr_Good;

            int i, k, l; /* indices */
            int ip, me, mi;
            int n = g0.size();
            int p = CE.cols();
            int m = CI.cols();
            MatX R(g0.size(), g0.size()), J(g0.size(), g0.size());

            s.resize(m + p);
            u.resize(m + p);
            redun.setZero(p);
            VecX z(n), r(m + p), d(n), np(n);
            VecX x_old(n), u_old(m + p);
            Scalar psi, c2, sum, ss, R_norm;
            Scalar t, t1, t2; /* t is the step length, which is the minimum of the partial step length t1
                              * and the full step length t2 */
            A.resize(m + p);
            VecXi A_old(m + p), iai(m + p), iaexcl(m + p);
            int iq, iter = 0;

            me = p; /* number of equality constraints */
            mi = m; /* number of inequality constraints */
            q = 0;  /* size of the active set A (containing the indices of the active constraints) */

            /*
            * Preprocessing phase
            */

            /* initialize the matrix R */
            d.setZero();
            R.setZero();
            R_norm = One; /* this variable will hold the norm of the matrix R */

            /* compute the inverse of the factorized matrix G^-1, this is the initial value for H */
            // J = L^-T
            J.setIdentity();
            J = chol.matrixU().solve(J);
            c2 = J.trace();
#ifdef TRACE_SOLVER
            print_matrix("J", J, n);
#endif

            /* c1 * c2 is an estimate for cond(G) */

            /*
            * Find the unconstrained minimizer of the quadratic form 0.5 * x G x + g0 x
            * this is a feasible point in the dual space
            * x = G^-1 * g0
            */
            x = chol.solve(g0);
            x = -x;
            /* and compute the current solution value */
            f_value = Half * g0.dot(x);
#ifdef TRACE_SOLVER
            std::cerr << "Unconstrained solution: " << f_value << std::endl;
            print_vector("x", x, n);
#endif

            /* Add equality constraints to the working set A */
            iq = 0;
            for (i = 0; i < me; i++) {
                np = CE.col(i);
                compute_d(d, J, np);
                update_z(z, J, d, iq);
                update_r(R, r, d, iq);
#ifdef TRACE_SOLVER
                print_matrix("R", R, iq);
                print_vector("z", z, n);
                print_vector("r", r, iq);
                print_vector("d", d, n);
#endif

                /* compute full step length t2: i.e., the minimum step in primal space s.t. the contraint
                becomes feasible */
                t2 = Zero;
                if (std::abs(z.dot(z)) > esp) // i.e. z != 0
                    t2 = (-np.dot(x) - ce0(i)) / z.dot(np);

                x += t2 * z;

                /* set u = u+ */
                u(iq) = t2;
                u.head(iq) -= t2 * r.head(iq);

                /* compute the new solution value */
                f_value += Half * (t2 * t2) * z.dot(np);
                A(i) = -i - 1;

                if (!add_constraint(R, J, d, iq, R_norm)) {
                    // FIXME: it should raise an error
                    // Equality constraints are linearly dependent
                    if (std::abs(t2) > 0){
                        printf("Weired...\n");
                    }
                    state = QPErr_EqconsLinear;
                    redun(i) = 1;
                    //return ret;
                }
            }
            if (state != QPErr_Good) {
                return ret;
            }

            /* set iai = K \ A */
            for (i = 0; i < mi; i++)
                iai(i) = i;

            int IterFuck = 0;
            int IterFuckLim = 1000;
        l1:	iter++;
            if (++IterFuck > IterFuckLim) {
                state = QPErr_Infeasible;
                printf("Reaching Limit\n");
                return ret;
            }
#ifdef TRACE_SOLVER
            print_vector("x", x, n);
#endif
            /* step 1: choose a violated constraint */
            for (i = me; i < iq; i++) {
                ip = A(i);
                iai(ip) = -1;
            }

            /* compute s(x) = ci^T * x + ci0 for all elements of K \ A */
            ss = Zero;
            psi = Zero; /* this value will contain the sum of all infeasibilities */
            ip = 0; /* ip will be the index of the chosen violated constraint */
            for (i = 0; i < mi; i++) {
                iaexcl(i) = 1;
                sum = CI.col(i).dot(x) + ci0(i);
                s(i) = sum;
                psi += std::min(Zero, sum);
            }
#ifdef TRACE_SOLVER
            print_vector("s", s, mi);
#endif

            if (std::abs(psi) <= mi * esp * c1 * c2* Scalar(100)) {
                /* numerically there are not infeasibilities anymore */
                q = iq;
                state = QPErr_Good;
                return ret;
            }

            /* save old values for u, x and A */
            u_old.head(iq) = u.head(iq);
            A_old.head(iq) = A.head(iq);
            x_old = x;

        l2: /* Step 2: check for feasibility and determine a new S-pair */
            if (++IterFuck > IterFuckLim) {
                state = QPErr_Infeasible;
                printf("Reaching Limit\n");
                return ret;
            }
            for (i = 0; i < mi; i++) {
                if (s(i) < ss && iai(i) != -1 && iaexcl(i)) {
                    ss = s(i);
                    ip = i;
                }
            }
            if (ss >= Zero) {
                // The most "violated" constraint is not violated
                // No more infeasibilities
                q = iq;
                state = QPErr_Good;
                return ret;
            }

            /* set np = n(ip) */
            np = CI.col(ip);
            /* set u = (u 0)^T */
            u(iq) = Zero;
            /* add ip to the active set A */
            A(iq) = ip;

#ifdef TRACE_SOLVER
            std::cerr << "Trying with constraint " << ip << std::endl;
            print_vector("np", np, n);
#endif

        l2a:/* Step 2a: determine step direction */
            /* compute z = H np: the step direction in the primal space (through J, see the paper) */
            if (++IterFuck > IterFuckLim) {
                state = QPErr_Infeasible;
                printf("Reaching Limit\n");
                return ret;
            }
            compute_d(d, J, np);
            update_z(z, J, d, iq);
            /* compute N* np (if q > 0): the negative of the step direction in the dual space */
            update_r(R, r, d, iq);
#ifdef TRACE_SOLVER
            std::cerr << "Step direction z" << std::endl;
            print_vector("z", z, n);
            print_vector("r", r, iq + 1);
            print_vector("u", u, iq + 1);
            print_vector("d", d, n);
            print_ivector("A", A, iq + 1);
#endif

            /* Step 2b: compute step length */
            l = 0;
            /* Compute t1: partial step length (maximum step in dual space without violating dual feasibility */
            t1 = inf; /* +inf */
                      /* find the index l s.t. it reaches the minimum of u+(x) / r */
            for (k = me; k < iq; k++) {
                Scalar tmp;
                if (r(k) > Zero && ((tmp = u(k) / r(k)) < t1)) {
                    t1 = tmp;
                    l = A(k);
                }
            }
            /* Compute t2: full step length (minimum step in primal space such that the constraint ip becomes feasible */
            if (std::abs(z.dot(z)) > esp) // i.e. z != 0
                t2 = -s(ip) / z.dot(np);
            else
                t2 = inf; /* +inf */

                          /* the step is chosen as the minimum of t1 and t2 */
            t = std::min(t1, t2);
#ifdef TRACE_SOLVER
            std::cerr << "Step sizes: " << t << " (t1 = " << t1 << ", t2 = " << t2 << ") ";
#endif

            /* Step 2c: determine new S-pair and take step: */

            /* case (i): no step in primal or dual space */
            if (t >= inf) {
                /* QPP is infeasible */
                // FIXME: unbounded to raise
                q = iq;
                state = QPErr_Infeasible;
                return ret;
            }
            /* case (ii): step in dual space */
            if (t2 >= inf) {
                /* set u = u +  t * [-r 1) and drop constraint l from the active set A */
                u.head(iq) -= t * r.head(iq);
                u(iq) += t;
                iai(l) = l;
                delete_constraint(R, J, A, u, p, iq, l);
#ifdef TRACE_SOLVER
                std::cerr << " in dual space: "
                    << f_value << std::endl;
                print_vector("x", x, n);
                print_vector("z", z, n);
                print_ivector("A", A, iq + 1);
#endif
                goto l2a;
            }

            /* case (iii): step in primal and dual space */

            x += t * z;
            /* update the solution value */
            f_value += t * z.dot(np) * (Half * t + u(iq));

            u.head(iq) -= t * r.head(iq);
            u(iq) += t;
#ifdef TRACE_SOLVER
            std::cerr << " in both spaces: "
                << f_value << std::endl;
            print_vector("x", x, n);
            print_vector("u", u, iq + 1);
            print_vector("r", r, iq + 1);
            print_ivector("A", A, iq + 1);
#endif

            if (t == t2) {
#ifdef TRACE_SOLVER
                std::cerr << "Full step has taken " << t << std::endl;
                print_vector("x", x, n);
#endif
                /* full step has taken */
                /* add constraint ip to the active set*/
                if (!add_constraint(R, J, d, iq, R_norm)) {
                    iaexcl(ip) = 0;
                    delete_constraint(R, J, A, u, p, iq, ip);
#ifdef TRACE_SOLVER
                    print_matrix("R", R, n);
                    print_ivector("A", A, iq);
#endif
                    for (i = 0; i < m; i++)
                        iai(i) = i;
                    for (i = 0; i < iq; i++) {
                        A(i) = A_old(i);
                        iai(A(i)) = -1;
                        u(i) = u_old(i);
                    }
                    x = x_old;
                    goto l2; /* go to step 2 */
                }
                else
                    iai(ip) = -1;
#ifdef TRACE_SOLVER
                print_matrix("R", R, n);
                print_ivector("A", A, iq);
#endif
                goto l1;
            }

            /* a patial step has taken */
#ifdef TRACE_SOLVER
            std::cerr << "Partial step has taken " << t << std::endl;
            print_vector("x", x, n);
#endif
            /* drop constraint l */
            iai(l) = l;
            delete_constraint(R, J, A, u, p, iq, l);
#ifdef TRACE_SOLVER
            print_matrix("R", R, n);
            print_ivector("A", A, iq);
#endif

            s(ip) = CI.col(ip).dot(x) + ci0(ip);

#ifdef TRACE_SOLVER
            print_vector("s", s, mi);
#endif
            goto l2a;
        }


    private:
        // Utility functions
        static Scalar distance(Scalar a, Scalar b) {
            const Scalar One = (Scalar)1;
            Scalar a1, b1, t;
            a1 = std::abs(a);
            b1 = std::abs(b);
            if (a1 > b1) {
                t = (b1 / a1);
                return a1 * std::sqrt(One + t * t);
            }
            else if (b1 > a1) {
                t = (a1 / b1);
                return b1 * std::sqrt(One + t * t);
            }
            return a1 * std::sqrt(Scalar(2.0));
        }

        void compute_d(VecX &d, const MatX& J, const VecX& np) const {
            d = J.adjoint() * np;
        }

        void update_z(VecX& z, const MatX& J, const VecX& d, int iq) const {
            z = J.rightCols(z.size() - iq) * d.tail(d.size() - iq);
        }

        void update_r(const MatX& R, VecX& r, const VecX& d, int iq) const {
            //MatX tl = R.topLeftCorner(iq, iq);
            //typename Eigen::TriangularView<MatX, Eigen::Upper> view = tl.template triangularView<Eigen::Upper > ( );
            //r.head(iq) = view.solve(d.head(iq));
            r.head(iq) = R.topLeftCorner(iq, iq).template triangularView< Eigen::Upper >().solve(d.head(iq));
        }

        bool add_constraint(MatX& R, MatX& J, VecX& d, int& iq, Scalar& R_norm) const {
            const Scalar Zero = (Scalar)0;
            const Scalar One = (Scalar)1;
            //const Scalar Half = (Scalar)0.5;

            int n = J.rows();
#ifdef TRACE_SOLVER
            std::cerr << "Add constraint " << iq << '/';
#endif
            //int i,
            int j, k;
            Scalar cc, ss, h, t1, t2, xny;

            /* we have to find the Givens rotation which will reduce the element
            d(j) to zero.
            if it is already zero we don't have to do anything, except of
            decreasing j */
            for (j = n - 1; j >= iq + 1; j--) {
                /* The Givens rotation is done with the matrix (cc cs, cs -cc).
                If cc is one, then element (j) of d is zero compared with element
                (j - 1). Hence we don't have to do anything.
                If cc is zero, then we just have to switch column (j) and column (j - 1)
                of J. Since we only switch columns in J, we have to be careful how we
                update d depending on the sign of gs.
                Otherwise we have to apply the Givens rotation to these columns.
                The i - 1 element of d has to be updated to h. */
                cc = d(j - 1);
                ss = d(j);
                h = distance(cc, ss);
                if (h == Zero)
                    continue;
                d(j) = Zero;
                ss = ss / h;
                cc = cc / h;
                if (cc < Zero) {
                    cc = -cc;
                    ss = -ss;
                    d(j - 1) = -h;
                }
                else
                    d(j - 1) = h;
                xny = ss / (One + cc);
                for (k = 0; k < n; k++) {
                    t1 = J(k, j - 1);
                    t2 = J(k, j);
                    J(k, j - 1) = t1 * cc + t2 * ss;
                    J(k, j) = xny * (t1 + J(k, j - 1)) - t2;
                }
            }
            /* update the number of constraints added*/
            iq++;
            /* To update R we have to put the iq components of the d vector
            into column iq - 1 of R
            */
            R.col(iq - 1).head(iq) = d.head(iq);
#ifdef TRACE_SOLVER
            std::cerr << iq << std::endl;
#endif
            //Scalar val = std::abs(d(iq - 1));
            //Scalar val_test = std::numeric_limits<Scalar>::epsilon() * R_norm;
            if (std::abs(d(iq - 1)) <= std::numeric_limits<Scalar>::epsilon() * R_norm)
                // problem degenerate
                return false;
            R_norm = std::max<Scalar>(R_norm, std::abs(d(iq - 1)));
            return true;
        }

        void delete_constraint(MatX& R, MatX& J, VecXi& A, VecX& u, int p, int& iq, int l) const {
            const Scalar Zero = (Scalar)0;
            const Scalar One = (Scalar)1;

            int n = R.rows();
#ifdef TRACE_SOLVER
            std::cerr << "Delete constraint " << l << ' ' << iq;
#endif
            int i, j, k, qq;
            Scalar cc, ss, h, xny, t1, t2;

            /* Find the index qq for active constraint l to be removed */
            for (i = p; i < iq; i++)
                if (A(i) == l) {
                    qq = i;
                    break;
                }

            /* remove the constraint from the active set and the duals */
            for (i = qq; i < iq - 1; i++) {
                A(i) = A(i + 1);
                u(i) = u(i + 1);
                R.col(i) = R.col(i + 1);
            }

            A(iq - 1) = A(iq);
            u(iq - 1) = u(iq);
            A(iq) = 0;
            u(iq) = 0.0;
            for (j = 0; j < iq; j++)
                R(j, iq - 1) = Zero;
            /* constraint has been fully removed */
            iq--;
#ifdef TRACE_SOLVER
            std::cerr << '/' << iq << std::endl;
#endif

            if (iq == 0)
                return;

            for (j = qq; j < iq; j++) {
                cc = R(j, j);
                ss = R(j + 1, j);
                h = distance(cc, ss);
                if (h == 0.0)
                    continue;
                cc = cc / h;
                ss = ss / h;
                R(j + 1, j) = Zero;
                if (cc < 0.0) {
                    R(j, j) = -h;
                    cc = -cc;
                    ss = -ss;
                }
                else
                    R(j, j) = h;

                xny = ss / (One + cc);
                for (k = j + 1; k < iq; k++) {
                    t1 = R(j, k);
                    t2 = R(j + 1, k);
                    R(j, k) = t1 * cc + t2 * ss;
                    R(j + 1, k) = xny * (t1 + R(j, k)) - t2;
                }
                for (k = 0; k < n; k++) {
                    t1 = J(k, j);
                    t2 = J(k, j + 1);
                    J(k, j) = t1 * cc + t2 * ss;
                    J(k, j + 1) = xny * (J(k, j) + t1) - t2;
                }
            }
        }
    };



}
