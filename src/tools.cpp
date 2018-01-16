#include <math.h>
#include <iostream>

#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    return rmse;
  }
  VectorXd sum(4);
  sum << 0, 0, 0, 0;
  // accumulate squared residuals
  for (int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    sum += residual;
  }

  // calculate the mean
  VectorXd mean = sum.array() / estimations.size();

  // calculate the squared root
  rmse = mean.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  MatrixXd Hj(3, 4);
  Hj << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  // TODO CHECK DIVIDE BY ZERO
  // recover state parameters
  double px = x_state(0);
  double px_2 = px * px;
  double py = x_state(1);
  double py_2 = py * py;
  double sqrt_px2_py2 = sqrt(px_2 + py_2);
  if (sqrt_px2_py2 == 0) {
    // Potential divide by zero. We want to ignore this state update so?
  }

  double vx = x_state(2);
  double vy = x_state(3);

  double px_over_sqrt = px / sqrt(px_2 + py_2);
  double py_over_sqrt = py / sqrt(px_2 + py_2);
  double denom_1 = pow((px_2 + py_2), 1.5);
  double num_1 = py * ((vx * py) - (vy * px));

  Hj << px_over_sqrt, py_over_sqrt, 0, 0, -1 * py / (px_2 + py_2),
      px / px_2 + py_2, 0, 0, num_1 / denom_1,
      (py * (vx * py - vy * px)) / denom_1, px_over_sqrt, py_over_sqrt;

  return Hj;
}

double Tools::NormalizeAngle(double radians_in) {
  double normalized_angle = radians_in;
  double two_pi = 2 * M_PI;
  while (normalized_angle < -M_PI || normalized_angle > M_PI) {
    if (normalized_angle > M_PI) normalized_angle -= two_pi;
    if (normalized_angle < -M_PI) normalized_angle -= two_pi;
  }
  return normalized_angle;
}
