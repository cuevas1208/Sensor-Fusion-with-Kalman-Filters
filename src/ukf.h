// File: ukf.h
// Overview: initializes the filter, calls the predict function, calls the update function
// Desc: This file takes the sensor data and initializes variables and updates variables.
// Unscented Kalman filter equations are used in this file. 
// Tools, and knowledge used in this project were taught by Andrei Vatavu, Sensor Fusion
// Engineer at Mercedes-Benz as part of the Udacity Self Driving Car Program
// ######################################################################################
#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_ = false;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 1;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 1;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  double std_radr = 0.3;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.3;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  // measurement matrix
  MatrixXd H_;

  // measurement covariance matrix
  MatrixXd R_laser_;
  MatrixXd R_radar_;

  //time delta in seconds
  double previous_timestamp_ = 0;

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = n_x + 2;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  //create example vector for predicted state mean
  VectorXd x = VectorXd(n_x);

  //create example matrix for predicted state covariance
  MatrixXd P = MatrixXd(n_x,n_x);

  //set vector for weights
  VectorXd weights;

  //create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(2 * n_aug + 1, n_aug); 

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void lidarUpdate(const VectorXd &z);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /*AugmentedSigmaPoints
  * craete sigma poinst and augmentation
  * @param Xsig_out - augmented Sigma Points matrix 
  */
  void AugmentedSigmaPoints(MatrixXd* Xsig_out);

  /*SigmaPointsAndPrediction
  * prediction of sigma points using the process model
  * @param Xsig_out - augmented Sigma Points matrix 
  */
  void SigmaPointPrediction(MatrixXd* Xsig_out, double delta_t);

  /*PredictionMeandCovariance 
  * prediction of sigma points
  * @param x_out - prediction
  *        P_out - covariance
  */
  void PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out);

  /*PredictUpdateRadarMeasurement 
  * Updates Prediction and Covariance from radar measurement 
  * @param z - raw measurment  
  */
  void PredictUpdateRadarMeasurement(const VectorXd& z);

  // tool object used to compute Jacobian and RMSE
  Tools tools;

};


#endif /* UKF_H */
