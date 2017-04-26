// File: ground_truth_package.h
// Desc: List of the true position and velocity of the object
// (each line represents a measurement at a time stamp
// ######################################################################################
//importing useful packages
#ifndef GROUND_TRUTH_PACKAGE_H_
#define GROUND_TRUTH_PACKAGE_H_

#include "Eigen/Dense"

class GroundTruthPackage {
public:
  long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd gt_values_;

};

#endif /* GROUND_TRUTH_PACKAGE_H_ */
