#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  cout << "Init called" << endl;
  cout << "Q_"  << Q_ << endl;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() +  Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */  
  VectorXd y = z - H_ * x_;
  _UpdateWithError(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // compute the error by diff between two different space
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  double px2py2_sqrt = sqrt(px*px + py*py);

  // guard against divided by zero, also this value should not be zero as it is 
  // the sensor's location
  if (px2py2_sqrt == 0)
    return;

  VectorXd z_(3);
  z_ << px2py2_sqrt, atan2(py, px), (px * vx + py * vy) / px2py2_sqrt;
  VectorXd y = z_ - z;
  
  // normalize angle error in a circular space ~ [-pi, pi]
  y(1) = atan2(sin(y(1)), cos(y(1)));
  _UpdateWithError(y);
}

void KalmanFilter::_UpdateWithError(const VectorXd &y) {
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y; 
  P_ = (MatrixXd::Identity(4, 4) - K * H_) * P_;
}
