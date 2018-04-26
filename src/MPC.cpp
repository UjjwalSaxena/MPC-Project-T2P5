#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration


// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.




size_t i;
size_t t;
class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    fg[0] = 0.0;

    double a=20;
    double b=20;
    double c=10;
    double d=400;
    double e=10;
    double f=350;
    double g=15;
    // The part of the cost based on the reference state.
    for (t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[cte_start + t], 2)*a;
      fg[0] += CppAD::pow(vars[epsi_start + t], 2)*b;
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2)*c;
    }

    // Minimize the use of actuators.
    for (t = 0; t < N - 1; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t], 2)*d;
      fg[0] += CppAD::pow(vars[a_start + t], 2)*e;
    }

    // Minimize the value gap between sequential actuations.
    for (t = 0; t < N - 2; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2)*f;
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2)*g;
    }

    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0+ coeffs[2]* x0*x0+ coeffs[3]*x0*x0*x0;
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0*x0  );


      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }




  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {

  this->vars.resize(n_vars);
  this->vars_lowerbound.resize(n_vars);
  this->vars_upperbound.resize(n_vars);
  this->constraints_lowerbound.resize(n_constraints);
  this->constraints_upperbound.resize(n_constraints);

  for (i = 0; i < delta_start; i++) {
    this->vars_lowerbound[i] = -1.0e19;
    this->vars_upperbound[i] = 1.0e19;
  }

  for (i = delta_start; i < a_start; i++) {
    this->vars_lowerbound[i] = -0.436332;
    this->vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (i = a_start; i < n_vars; i++) {
    this->vars_lowerbound[i] = -1.0;
    this->vars_upperbound[i] = 1.0;
  }

  for (i = 0; i < n_constraints; i++) {
    this->constraints_lowerbound[i] = 0.0;
    this->constraints_upperbound[i] = 0.0;
  }
}
MPC::~MPC() {}

void MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs) {
  
  typedef CPPAD_TESTVECTOR(double) Dvector;


  const double x = x0[0];
  const double y = x0[1];
  const double psi = x0[2];
  const double v = x0[3];
  const double cte = x0[4];
  const double epsi = x0[5];


  for(i=0; i<n_vars; i++)
  {
    this->vars[i] = 0.0;
  }
  this->vars[x_start] = x;
  this->vars[y_start] = y;
  this->vars[psi_start] = psi;
  this->vars[v_start] = v;
  this->vars[cte_start] = cte;
  this->vars[epsi_start] = epsi;


  for (i = 0; i < n_constraints; i++) {
    this->constraints_lowerbound[i] = 0.0;
    this->constraints_upperbound[i] = 0.0;
  }

  this->constraints_lowerbound[x_start] = x;
  this->constraints_lowerbound[y_start] = y;
  this->constraints_lowerbound[psi_start] = psi;
  this->constraints_lowerbound[v_start] = v;
  this->constraints_lowerbound[cte_start] = cte;
  this->constraints_lowerbound[epsi_start] = epsi;

  this->constraints_upperbound[x_start] = x;
  this->constraints_upperbound[y_start] = y;
  this->constraints_upperbound[psi_start] = psi;
  this->constraints_upperbound[v_start] = v;
  this->constraints_upperbound[cte_start] = cte;
  this->constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  this->mpc_x_vals = {};
  this->mpc_y_vals = {};

  this->steering= solution.x[delta_start];
  this->throttle= solution.x[a_start];

  for(i=0; i<N; i++)
  {
    this->mpc_x_vals.push_back(solution.x[x_start+i]);
    this->mpc_y_vals.push_back(solution.x[y_start+i]);
  }


}


