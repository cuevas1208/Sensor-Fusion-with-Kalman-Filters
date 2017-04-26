// File: kalman_filter.h
// Desc: defines the predict function, the update function for lidar, and the
// update function for radar
// The KalmanFilter class is defined in kalman_filter.cpp and kalman_filter.h.
// this class contains functions for the prediction and update steps.
// ######################################################################################
//importing useful packages
#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"

// This class contains functions for the prediction and update steps.
class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // Jacobian measurement matrix
  Eigen::MatrixXd Hj;

  // measurement covariance matrix
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;


  /**
   * Constructor
   * Initializes Hj_, R_laser_, and R_radar_
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();


  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

private:
  Tools tools;
};

#endif /* KALMAN_FILTER_H_ */
