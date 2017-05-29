// File: ukf.cpp
// Overview: initializes the filter, calls the predict function, calls the update function
// Desc: This file takes the sensor data and initializes variables and updates variables.
// Unscented Kalman filter equations are used in this file. 
// Tools, and knowledge used in this project were taught by Andrei Vatavu, Sensor Fusion
// Engineer at Mercedes-Benz as part of the Udacity Self Driving Car Program
// ######################################################################################
#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define ESP 0.0000001

//Initializes Unscented Kalman filter
UKF::UKF() {

  //set vector for weights
  weights = VectorXd(2*n_aug+1);
  weights(0) = lambda/(lambda+n_aug);
  for (int i=1; i<2*n_aug+1; i++){
      weights(i) = 1/(2*(n_aug+lambda));
  }

  P <<
  1,0,0,0,0,
  0,1,0,0,0,
  0,0,1,0,0,
  0,0,0,1,0,
  0,0,0,0,1; 

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx_ * std_laspx_, 0,
            0, std_laspy_ * std_laspy_;

  //add measurement noise covariance matrix
  R_radar_ = MatrixXd(3,3);
  R_radar_ <<    std_radr*std_radr, 0, 0,
                 0, std_radphi*std_radphi, 0,
                 0, 0,std_radrd*std_radrd;

  //measurement matrix
  H_ = MatrixXd(2,5);
  H_ << 1.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0, 0.0;

}

UKF::~UKF() {}

/**Process measurment data using UKF
 * @param meas_package - The latest measurement data
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {

  double meas_1 =  measurement_pack.raw_measurements_[0];
  double meas_2 =  measurement_pack.raw_measurements_[1];
  //  Initialization
  if (!is_initialized_) {
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      //Convert radar from polar to cartesian coordinates and initialize state.
      x << tools.fromPolarToCartesian(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0, 0; 
    }
    previous_timestamp_ = measurement_pack.timestamp_;

    // do not initialize if data is NULL
    if ((x(0) > 0) && (x(1) > 0))
        is_initialized_ = true;
    return;
  } 

  //compute the time elapsed between the current and previous measurements
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  //cout << "dt   " << dt << endl;

  // So skipping prediction steps in UKF is a very very bad idea (as opposed to EKF)
  //if ( dt > 0.001 )
  {
  previous_timestamp_ = measurement_pack.timestamp_;

  Prediction(dt); 
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
  // Radar updates
    if (fabs(meas_1) > 0.001){
      PredictUpdateRadarMeasurement(measurement_pack.raw_measurements_);
    }
  } else {
  // Laser updates
    if (sqrt(pow(meas_1,2) + pow(meas_2,2)) > 0.001){
      lidarUpdate(measurement_pack.raw_measurements_); 
    }
  }
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
   
    AugmentedSigmaPoints(&Xsig_aug);

    SigmaPointPrediction(&Xsig_aug, delta_t);

    PredictMeanAndCovariance(&x, &P);
  
}


/*lidarUpdate
 *update where our object is based on Lidar sensor measurements
* by using Kalman Filter
* @param z - measurmemts reading 
*/
void UKF::lidarUpdate(const VectorXd &z) {
  
  //measurement matrix
  VectorXd y  = z - H_ * x;
  MatrixXd S  = H_*P* H_.transpose() +R_laser_;
  MatrixXd K  = P *  H_.transpose() * S.inverse();

  //new estimate
  x = x +(K*y);
  long x_size = x.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P = (I - K * H_) * P;

  //Calculate NIS
  NIS_laser_ = (y).transpose()* S * (y);
}

/*AugmentedSigmaPoints
* craete sigma poinst and augmentation
* @param Xsig_out - augmented Sigma Points matrix 
*/
void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug, n_aug); 

  //create sigma point matrix
  MatrixXd Xsig_aug_ = MatrixXd(n_aug, 2 * n_aug + 1);

  //create augmented mean state
  x_aug.head(5) = x;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P;
  P_aug(5,5) = std_a*std_a;
  P_aug(6,6) = std_yawdd*std_yawdd;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.col(0)  = x_aug;
  for (int i = 0; i< n_aug; i++)
  {
    Xsig_aug_.col(i+1)       = x_aug + sqrt(lambda+n_aug) * L.col(i);
    Xsig_aug_.col(i+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L.col(i);
  }

  //write result
  *Xsig_out = Xsig_aug_;

}

/*SigmaPointsAndPrediction
* prediction of sigma points using the process model
* @param Xsig_out - augmented Sigma Points matrix 
*/
void UKF::SigmaPointPrediction(MatrixXd* Xsig_out, double delta_t) {
  
   //predict sigma points
  for (int i = 0; i< 2*n_aug+1; i++)
  {
    //tools.normalization(&Xsig_aug(3,i));
    
    //extract values for better readability
    const double p_x = Xsig_aug(0,i);
    const double p_y = Xsig_aug(1,i);
    const double v = Xsig_aug(2,i);
    const double yaw = Xsig_aug(3,i);
    const double yawd = Xsig_aug(4,i);
    const double nu_a = Xsig_aug(5,i);
    const double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise  polar to cartesian
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }

  //std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;
  *Xsig_out = Xsig_pred;

}

/*PredictionMeandCovariance 
* prediction of sigma points
* @param x_out - prediction
*        P_out - covariance
*/
void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) {
  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points
    x = x+ weights(i) * Xsig_pred.col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    tools.normalization(&(x_diff(3)));

    P = P + weights(i) * x_diff * x_diff.transpose() ;
  }

  //write result
  *x_out = x;
  *P_out = P;
}



/*PredictUpdateRadarMeasurement 
* Updates Prediction and Covariance from radar measurement 
* @param z - raw measurment  
*/
void UKF::PredictUpdateRadarMeasurement(const VectorXd& z) {
  //create matrix with sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
  VectorXd z_pred = VectorXd(n_z);

  //create matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z,n_z);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);
    tools.normalization(&yaw);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    //fromCartesianToPolar( 
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / std::max(ESP, sqrt(p_x*p_x + p_y*p_y));
  }

  //mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug+1; i++) {
      z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  //calculate cross correlation matrix
  Tc.fill(0.0);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    tools.normalization(&(z_diff(1)));
    S = S + weights(i) * z_diff * z_diff.transpose();

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    tools.normalization(&(x_diff(3)));

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

   S = S + R_radar_;
  //print result
  //cout << "z_pred: " << std::endl << z_pred << std::endl;
  //cout << "z: " << std::endl << z << std::endl;

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd y = z - z_pred;

  //angle normalization
  tools.normalization(&(y(1)));

  //update state mean and covariance matrix
  x = x + K * y;
  P = P - K*S*K.transpose();

  //print result
  //cout << "Updated state x: " << std::endl << x << std::endl;
  //cout << "Updated state covariance P: " << std::endl << P << std::endl;

  //Calculate NIS
  NIS_radar_ = y.transpose()* S * y;
}

