#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  lidar_enabled_ = true;
  radar_enabled_ = true;

  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);

  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0, 0, 1, 0, 0;

  Hj_ = MatrixXd(3, 4);

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian
    coordinates.
    */
    // cout << "Kalman Filter Initialization " << endl;

    // set the state with the initial location and zero velocity
    Eigen::VectorXd x_in(4);

    Eigen::MatrixXd P_in(4, 4);
    // clang-format off
    P_in << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    Eigen::MatrixXd F_in(4, 4);
    F_in << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Eigen::MatrixXd Q_in(4, 4);
    Q_in << 0, 0, 0, 0, 
            0, 0, 0, 0, 
            0, 0, 0, 0, 
            0, 0, 0, 0;
    // clang-format on

    // first measurement
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR &&
        radar_enabled_) {
      Eigen::VectorXd x_in(4);
      cout << "EKF: First Measurement RADAR" << endl;

      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double ro = measurement_pack.raw_measurements_[0];
      double theta =
          Tools::NormalizeAngle(measurement_pack.raw_measurements_[1]);
      double px = cos(theta) * ro;
      double py = sin(theta) * ro;
      x_in << px, py, 0, 0;
      ekf_.Init(x_in, P_in, F_in, Hj_, R_laser_, R_radar_, Q_in);
      is_initialized_ = true;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER &&
               lidar_enabled_) {
      cout << "EKF: First Measurement LIDAR" << endl;
      /**
      Initialize state.
      */
      x_in << measurement_pack.raw_measurements_[0],
          measurement_pack.raw_measurements_[1], 0, 0;
      ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, R_radar_, Q_in);
      is_initialized_ = true;
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  constexpr double kNoiseAx = 9.0;
  constexpr double kNoiseAy = 9.0;
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;

  ekf_.UpdateDt(dt, kNoiseAx, kNoiseAy);

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR &&
      radar_enabled_) {
    // Radar updates
    double ro = measurement_pack.raw_measurements_[0];
    double theta =
        Tools::NormalizeAngle(measurement_pack.raw_measurements_[1]);
    double ro_dot = measurement_pack.raw_measurements_[2];

    Eigen::VectorXd z(3);
    z << ro, theta, ro_dot;
    ekf_.UpdateEKF(z);
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER &&
             lidar_enabled_) {
    // Laser updates
    Eigen::VectorXd z(2);
    z << measurement_pack.raw_measurements_;
    ekf_.Update(z);
  }
  previous_timestamp_ = measurement_pack.timestamp_;
}
