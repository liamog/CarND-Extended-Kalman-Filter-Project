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

bool Tools::CalculateJacobian(const VectorXd &x_state, MatrixXd *Hj) {
  assert(Hj != nullptr);
  assert(Hj->rows() == 3);
  assert(Hj->cols() == 4);

  *Hj << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = (c1 * c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return false;
  }

  // compute the Jacobian matrix
  *Hj << (px / c2), (py / c2), 0, 0, -(py / c1), (px / c1), 0, 0,
      py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2,
      py / c2;

  return true;
}

double Tools::NormalizeAngle(double radians_in) {
  double normalized_angle = radians_in;
  double two_pi = 2 * M_PI;
  while (normalized_angle < -M_PI || normalized_angle > M_PI) {
    if (normalized_angle > M_PI) {
      normalized_angle -= two_pi;
    }
    if (normalized_angle < -M_PI) {
      normalized_angle += two_pi;
    }
  }
  return normalized_angle;
}

VectorXd Tools::PositionSpaceToRadarMeasurementSpace(const VectorXd &x) {
  VectorXd rm_space(3);
  // recover state parameters
  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);

  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = (c1 * c2);

  double ro = c2;
  double theta = atan2(py, px);
  double ro_dot = (px * vx + py * vy) / c2;
  rm_space << ro, theta, ro_dot;
  return rm_space;
}