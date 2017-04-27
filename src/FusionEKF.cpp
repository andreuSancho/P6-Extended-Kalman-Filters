#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225f, 0.0f,
        0.0f, 0.0225f;

  //measurement covariance matrix - radar
  R_radar_ << 0.09f, 0.0f, 0.0f,
        0.0f, 0.0009f, 0.0f,
        0.0f, 0.0f, 0.09f;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  //[Andreu]: I hope it refers to the following:
	H_laser_ << 1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f;
	
	Hj_ << 1.0f, 1.0f, 0.0f, 0.0f,
			1.0f, 1.0f, 0.0f, 0.0f,
			1.0f, 1.0f, 1.0f, 1.0f;
	
	ekf_.Update_covariance_matrix();
	ekf_.Update_transition_matrix(1.0f);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) { // [Andreu]: All this should be in the class constructor...
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    //cout << "EKF: " << endl;
    ekf_.x_ = Eigen::VectorXd(4); // [Andreu]: that's not an elegant solution... OOP should avoid this via a constructor/set method.
    ekf_.x_ << 1.0f, 1.0f, 1.0f, 1.0f; // [Andreu]: also notice I'm a C/C++ floating point-correctness maniac.
    
	float range = (measurement_pack.raw_measurements_(0) < Tools::Epsilon)? 
		Tools::Epsilon : measurement_pack.raw_measurements_(0); 
	float bearing = (measurement_pack.raw_measurements_(1) < Tools::Epsilon)? 
		Tools::Epsilon : measurement_pack.raw_measurements_(1); 
		
	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
		float range_rate = (measurement_pack.raw_measurements_(2) < Tools::Epsilon)? 
			Tools::Epsilon : measurement_pack.raw_measurements_(2);
		float x = range * cos(bearing); 
		float y = range * sin(bearing);
		float vx = range_rate * cos(bearing);
		float vy = range_rate * sin(bearing);
		ekf_.x_ << x, y, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
		ekf_.x_ << range, bearing, 0.0f, 0.0f;
    }

	// Also grab the initital timestamp.
	previous_timestamp_ = measurement_pack.timestamp_;
	// done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
	long long time_difference = (measurement_pack.timestamp_ - previous_timestamp_);
	previous_timestamp_ = measurement_pack.timestamp_; // Update time stamp.
	float float_time_diff = (float)(time_difference / 1000000.0);
	// Update the state trainsition matrix.
	ekf_.Update_transition_matrix(float_time_diff);
	// Update the covariance matrix with the provided noise_ax and noise_ay values.
	ekf_.update_covariance_matrix(9.0f, 9.0f, float_time_diff);
	
	ekf_.Predict();
  
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
   
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
	ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
