#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
typedef CPPAD_TESTVECTOR(double) Dvector;

using namespace std;

const size_t N = 13;
const double dt = 0.1;
const size_t n_vars = N*6+(N-1)*2;
const size_t n_constraints = N*6;
const double Lf = 2.67;
const double ref_v = 30 *0.44704; //m/s;


const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

class MPC {
 public:



Dvector vars; 
Dvector vars_lowerbound;
Dvector vars_upperbound;
Dvector constraints_lowerbound;
Dvector constraints_upperbound;

std::vector<double> mpc_x_vals;
std::vector<double> mpc_y_vals;

double steering;
double throttle;


  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
