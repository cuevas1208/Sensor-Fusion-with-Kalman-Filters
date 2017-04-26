// File: tools.cpp
// Desc: function to calculate RMSE, the Jacobian matrix,
// fromPolarToCartesian, and fromCartesianToPolar
// ######################################################################################
//importing useful packages
#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

/**
 * CalculateRMSE measure how well the Kalman filter performs
 * It calculates the root mean squared error comparing the
 * Kalman filter results with the provided ground truth.
 * Input: estimations - Kalman filter's location and velocity
 *        ground_truth - the true value of the object
 * Ouput: root mean squared error
*/
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  //if(estimationssize() != ground)
  rmse << 0,0,0,0;
  // check the validity of the following inputs:
  // * the estimation vector size should not be zero
  // * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
      || estimations.size() == 0){
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  //accumulate squared residuals                      */
  for(unsigned int i=0; i < estimations.size(); ++i){
    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  //calculate the mean
  rmse = rmse/estimations.size();
  //calculate the squared root
  rmse = rmse.array().sqrt();
  //return the result
  return rmse;
}

/**
 * Calculates Jacobian measure. In order to get a gaussian distribution
 * for Radar we compute the Jacobian transformation for x’ in the
 * measurement function.
 * Input: x_state - prediction state x’
 * Ouput: Hj - Jacobian matrix
*/
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  //check division by zero
  if(fabs(c1) < 0.0001){
      cout << "CalculateJacobian () - Error - Division by Zero" << endl;
      return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
     -(py/c1), (px/c1), 0, 0,
     py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}

/**
* A helper method to calculate convert from polar to Cartesian.
*/
MatrixXd Tools::fromPolarToCartesian(const Eigen::VectorXd& x){

  VectorXd X(4);

  //recover state parameters
  X << x(0)*cos(x(1)),  x(0)*sin(x(1)), x(2), 0;

  return X;
}

/**
* A helper method to calculate convert from Cartesian to Polar
*/
MatrixXd Tools::fromCartesianToPolar(const Eigen::VectorXd& x){

  VectorXd X(4);

  //recover state parameters
  float r = x(0);
  float angle = x(1);

  r =  sqrt(pow(x(0),2) + pow(x(1),2));
  angle =  atan(x(1)/x(0));
  if (x(3))
    X(2) = ((x(0)*x(2))+(x(1)*x(3)))/r;

  X << r, angle, X(2), 0;

  return X;
}
