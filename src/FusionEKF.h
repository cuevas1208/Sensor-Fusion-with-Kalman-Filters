// File: FusionEKF.h
// Overview: initializes the filter, calls the predict function, calls the update function
// Desc: This file takes the sensor data and initializes variables and updates variables.
// ######################################################################################
//importing useful packages
#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

/**
* This class has a variable called ekf_, which is an instance of a KalmanFilter class.
* The ekf_ will hold the matrix and vector values. ekf_ instance is used to call the
* predict and update equations.
*/
class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  ProcessMeasurement() is responsible for the initialization the Kalman filter
  as well as calling the prediction and update steps of the Kalman filter.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous time stamp
  long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
};

#endif /* FusionEKF_H_ */
