#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_ << 1.0, 0, 0, 0,
              0.0, 1, 0, 0;

  Hj_ << 1.0, 1.0, 0, 0,
           0, 1.0, 0, 0,
           0, 0, 0, 1;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  float dt = float(measurement_pack.timestamp_ - previous_timestamp_);
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      double zho = measurement_pack.raw_measurements_(0);
      double phi = measurement_pack.raw_measurements_(1);
      double dzho = measurement_pack.raw_measurements_(2);
      VectorXd x(4);
      x << zho * cos(phi), zho * sin(phi), dzho * cos(phi), dzho * sin(dzho);
      MatrixXd P_in;


      ekf_.Init(x, P_in, F_in, Hj_, R_radar_, Q_in);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      VectorXd x(4);
      x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      MatrixXd F_in(4,4);
      F_in << 1.0, 0, dt, 0,
              0,  1.0, 0, dt,
              0, 0, 1.0, 0,
              0, 0, 0, 1.0;
      
      MatrixXd P_in(4,4), Q_in(4,4);
      Q_in << 0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0;

      ekf_.Init(x, P_in, F_in, H_laser_, R_laser_, Q_in);
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float noise_ax_2 = 9.f * 9.f;
  float noise_ay_2 = 9.f * 9.f;
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;

  ekf_.Q_ << dt4 * noise_ax_2/4.f, 0, dt3 * noise_ax_2/2.f, 0,
        0, dt4 * noise_ay_2/4.f, 0, dt3 * noise_ay_2/2.f,
        dt3 * noise_ax_2/2.f, 0, dt2 * noise_ax_2, 0,
        0, dt3 * noise_ay_2/2.f, 0, dt2 * noise_ay_2;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    VectorXd z(2);
    z << measurement_pack.raw_measurements_[0],
         measurement_pack.raw_measurements_[1];
    ekf_.UpdateEKF(z);
  } else {
    // TODO: Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_; 
    VectorXd z(3);
    float px = measurement_pack.raw_measurements_[0];
    float py = measurement_pack.raw_measurements_[1];
    z << sqrt(px*px + py*py), atan2f(py, px), 0;
    ekf_.Update(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;

  previous_timestamp_ = measurement_pack.timestamp_;
}
