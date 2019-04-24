#ifndef MPC_H
#define MPC_H

#include <vector>
#include <queue>
#include "Eigen-3.3/Eigen/Core"

using namespace std;
const double V = 12;
const double Lf = 2.85;
class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, Eigen::MatrixXd RLS_phi, Eigen::MatrixXd omega_coeffs, double speed);
};

#endif /* MPC_H */
