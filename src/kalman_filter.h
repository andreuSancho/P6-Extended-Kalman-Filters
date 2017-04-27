#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include <iostream>
#include <cmath>
#include "Eigen/Dense"
#include "tools.h"

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transistion matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;
  
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

	/**
	* It instantiates the covariance matrix.
	*/
	void Update_covariance_matrix();
	
	/**
	*	It updates the F_ matrix with the time difference.
	*	@param value is the value of the time difference.
	*/
	void Update_transition_matrix(float value);
	
	/**
	*	It updates the Q_ matrix with the given data.
	*	@param noise_ax is the level of noise for the x component.
	*	@param noise_ay is the level of noise for the y component.
	*	@param time_difference is the ellapsed time in seconds.
	*/
	void update_covariance_matrix(float noise_ax, float noise_ay, float time_difference);
	  
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
	/**
	*	It computes the common updates for standard and extended Kalman
	*   filter.
	*   @param y is the error vector.
	*/
	void CommonUpdate(const Eigen::VectorXd &y);
	
	// Useful constants/methods.
	Tools tools;

};

#endif /* KALMAN_FILTER_H_ */
