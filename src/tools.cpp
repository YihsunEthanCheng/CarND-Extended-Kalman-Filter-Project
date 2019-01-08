#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
   VectorXd rmse(ground_truth[0].size());
   rmse = VectorXd::Zero(ground_truth[0].size());

   if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
      cout << "Invalid estimation or ground_truth data" << endl;
      return rmse;
   }

   for (int i=0;i< ground_truth.size(); ++i) {
      VectorXd d = estimations[i] - ground_truth[i];
      rmse += VectorXd(d.array()*d.array());
   }

   rmse = rmse.array().sqrt() / ground_truth.size();

   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
   
   MatrixXd Jacobian(3,4);

  // recover state parameters
  float px2y2 = x_state(0) * x_state(0) + x_state(1) * x_state(1);
  float sqrt_px2y2 = sqrt(px2y2);
  float sqrt_px2y2_3 = sqrt(px2y2* px2y2* px2y2);

  // check division by zero
  if (fabs(px2y2) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
  } else{
    // compute the Jacobian matrix
    Jacobian << 
       x_state(0)/sqrt_px2y2, x_state(1)/sqrt_px2y2, 0, 0,
      -x_state(1)/px2y2, x_state(0)/px2y2, 0, 0,
       x_state(1)*(x_state(2)*x_state(1) - x_state(3)*x_state(0))/sqrt_px2y2_3,
       x_state(0)*(x_state(3)*x_state(0) - x_state(2)*x_state(1))/sqrt_px2y2_3,
       x_state(0)/sqrt_px2y2, x_state(1)/sqrt_px2y2;
   }

  return Jacobian;
}

void cout_VectorXd(const VectorXd& v)
{
  for (int i =0; i< v.size() ; ++i)
    cout << v(i) << "\t";
  cout << endl;
}
