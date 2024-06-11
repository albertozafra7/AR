#include <iostream>
#include <casadi/casadi.hpp>

// dx/dt = f(x,u)
casadi::MX f(const casadi::MX& x, const casadi::MX& u) {
  return vertcat(x(1), u(0), x(3), u(1), x(5), u(2));
}

int main() {
  int N = 100; // number of control intervals

  casadi::Opti opti = casadi::Opti(); // Optimization problem

  casadi::Slice all;
  // ---- decision variables ---------
  casadi::MX X = opti.variable(6, N + 1); // state trajectory [pos_x, vel_x, pos_y, vel_y, pos_z, vel_z]
  auto pos_x = X(0, all);
  auto vel_x = X(1, all);
  auto pos_y = X(2, all);
  auto vel_y = X(3, all);
  auto pos_z = X(4, all);
  auto vel_z = X(5, all);

  casadi::MX U = opti.variable(3, N); // control trajectory (accelerations)
  casadi::MX T = opti.variable(); // final time

  // ---- objective ---------
  // Minimize final time and smooth control inputs
  casadi::MX control_diff = 0;
  for (int k = 0; k < N-1; ++k) {
    control_diff += sumsqr(U(all, k+1) - U(all, k));
  }
  opti.minimize(T + 0.1 * control_diff); // adjust weight as needed

  // ---- dynamic constraints --------
  casadi::MX dt = T / N;
  for (int k = 0; k < N; ++k) {
    casadi::MX k1 = f(X(all,k),         U(all,k));
    casadi::MX k2 = f(X(all,k)+dt/2*k1, U(all,k));
    casadi::MX k3 = f(X(all,k)+dt/2*k2, U(all,k));
    casadi::MX k4 = f(X(all,k)+dt*k3,   U(all,k));
    casadi::MX x_next = X(all,k) + dt/6*(k1+2*k2+2*k3+k4);
    opti.subject_to(X(all,k+1)==x_next); // close the gaps 
  }

  // ---- path constraints -----------
  opti.subject_to(-1 <= U <= 1); // control limits (accelerations)

  // ---- boundary conditions --------
  opti.subject_to(pos_x(0) == 0); // start position
  opti.subject_to(vel_x(0) == 0); // start with zero velocity
  opti.subject_to(pos_y(0) == 0);
  opti.subject_to(vel_y(0) == 0);
  opti.subject_to(pos_z(0) == 0);
  opti.subject_to(vel_z(0) == 0);

  opti.subject_to(pos_x(N) == 0); // end position
  opti.subject_to(pos_y(N) == 2);
  opti.subject_to(pos_z(N) == 2);

  // ---- misc. constraints ----------
  opti.subject_to(T >= 0); // Time must be positive

  // ---- initial values for solver ---
  opti.set_initial(T, 1);
  opti.set_initial(X, 0); // initial guess for state trajectory

  // ---- solve NLP ------
  opti.solver("ipopt"); // set numerical backend
  casadi::OptiSol sol = opti.solve(); // actual solve

  // Retrieve and print the results
  std::vector<double> x_sol = std::vector<double>(sol.value(pos_x));
  std::vector<double> y_sol = std::vector<double>(sol.value(pos_y));
  std::vector<double> z_sol = std::vector<double>(sol.value(pos_z));

  std::cout << "Optimal state trajectory:" << std::endl;
  for (size_t i = 0; i < x_sol.size(); ++i) {
    std::cout << "Step " << i << ": x = " << x_sol[i] << ", y = " << y_sol[i] << ", z = " << z_sol[i] << std::endl;
  }

  double T_sol = static_cast<double>(sol.value(T));
  std::cout << "Optimal final time: " << T_sol << std::endl;

  return 0;
}
