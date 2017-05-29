// File: tools.h
// Desc: This project uses lidar measurements and radar measurements to track
// an object's position and velocity that travels around the vehicle. Sensor
// data is processed by an extended Kalman filter in C++.
// ######################################################################################
//importing useful packages
#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

 /**
   * CalculateRMSE measure how well the Kalman filter performs
   * It calculates the root mean squared error comparing the
   * Kalman filter results with the provided ground truth.
   * Input: estimations - Kalman filter's location and velocity
   *        ground_truth - the true value of the object
   * Ouput: root mean squared error
 */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
    * A helper method to calculate Jacobians.. In order to get a gaussian distribution
    * for Radar we compute the Jacobian transformation for x’ in the
    * measurement function.
    * Input: x_state - prediction state x’
    * Ouput: Hj - Jacobian matrix
  */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  /**
  * A helper method to calculate convert from polar to cartesian.
  */
  Eigen::MatrixXd fromPolarToCartesian(const Eigen::VectorXd& x);
 
  /**
  * A helper method to calculate convert from Cartesian to Polar
  */
  Eigen::MatrixXd fromCartesianToPolar(const Eigen::VectorXd& x);

  /**
  * A helper method to normalize angle
  */
  void normalization(double *z_diff);

};

#endif /* TOOLS_H_ */
