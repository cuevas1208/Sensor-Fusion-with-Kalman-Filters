// File: measurement_package.h
// Desc: List of the Kalman filter's location and velocity of the object
// (each line represents a measurement at a time stamp
// ######################################################################################
//importing useful packages
#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:
  long long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;
};

#endif /* MEASUREMENT_PACKAGE_H_ */
