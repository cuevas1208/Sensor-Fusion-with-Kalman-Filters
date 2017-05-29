// File: kalman_filter.cpp
// Desc: defines the predict function, the update function for lidar, and the
// update function for radar
// The KalmanFilter class is defined in kalman_filter.cpp and kalman_filter.h.
// this class contains functions for the prediction and update steps.
// ######################################################################################
//importing useful packages
#include "kalman_filter.h"
#include <iostream>
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

//Initialize Kalman filter variables
KalmanFilter::KalmanFilter() {
  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  Hj = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
}

KalmanFilter::~KalmanFilter() {}

//predict the state
//predict where our object is going to be after a time step Δt
void KalmanFilter::Predict() {
  x_ = F_*x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

//MEASUREMENT UPDATE
//update where our object is based on Lidar sensor measurements
//by using Kalman Filter equations
void KalmanFilter::Update(const VectorXd &z) {
  //measurement matrix
  VectorXd y = z - H_ * x_;
  MatrixXd S  = H_*P_* H_.transpose() +R_laser_;
  MatrixXd K  = P_ *  H_.transpose() * S.inverse();

  //new estimate
  x_ = x_ +(K*y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

//MEASUREMENT UPDATE
//update where our object is based on Lidar sensor measurements
//by using Extended Kalman Filter equations
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  Hj = tools.CalculateJacobian(x_);
  MatrixXd S  = Hj * P_ * Hj.transpose() + R_radar_;
  MatrixXd K  = P_ * Hj.transpose() * S.inverse();

  //new estimate
  //equation y=z−H_*x the ekf does not become y=z−Hj*x. Instead,
  //for extended Kalman filters, we'll use the x_ in polar form
  VectorXd X = tools.fromCartesianToPolar(x_);
  VectorXd y = z.head(3) - X.head(3);
  cout << endl << "+X      ---  "<< endl << X << endl;
  cout << endl << "+y      ---  "<< endl << y << endl;
  x_ = x_ +(K*y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;
}
