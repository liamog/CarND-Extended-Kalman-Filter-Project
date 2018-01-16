#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  Ht_ = H_.transpose();
  I_ = MatrixXd::Identity(4, 4);
}

void KalmanFilter::Predict() {
  VectorXd x_prime = F_ * x_;
  MatrixXd P_prime = F_ * P_ * F_.transpose();

  x_ = x_prime;
  P_ = P_prime;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  VectorXd x_prime = x_ + (K * y);
  MatrixXd P_prime = (I_ - K * H_) * P_;

  // new state
  x_ = x_prime;
  P_ = P_prime;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}

void KalmanFilter::UpdateDt(double dt, double noise_ax, double noise_ay) {
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  // set the process covariance matrix Q
  Q_ = MatrixXd(4, 4);
  Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0, 0, dt_4 / 4 * noise_ay,
      0, dt_3 / 2 * noise_ay, dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0, 0,
      dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;
}
