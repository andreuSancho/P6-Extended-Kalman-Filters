#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in; // State (position and velocity).
  P_ = P_in; // Covariance matrix.
  F_ = F_in; // Transition matrix.
  H_ = H_in; // Measurement matrix.
  R_ = R_in; // Measurement covariance matrix.
  Q_ = Q_in; // Process covariance matrix.
}


void KalmanFilter::Update_covariance_matrix() {
	P_ = Eigen::MatrixXd(4, 4);
	P_ << 	 1.0f, 0.0f, 0.0f, 0.0f,
             0.0f, 1.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 1000.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 1000.0f;
}

void KalmanFilter::Update_transition_matrix(float value) {
	F_ = Eigen::MatrixXd(4, 4);
	F_ << 	 1.0f, 0.0f, value, 0.0f,
             0.0f, 1.0f, 0.0f, value,
             0.0f, 0.0f, 1.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 1.0f;
}

void KalmanFilter::update_covariance_matrix(float noise_ax, float noise_ay, float time_difference) {
	float At_pow2_ = time_difference * time_difference;
	float At_pow3_ = At_pow2_ * time_difference;
	float At_pow3_2 = 0.5f * At_pow3_;
	float At_pow4_ = At_pow3_ * time_difference;
	float At_pow4_4 = 0.25f * At_pow4_;
	Q_ = Eigen::MatrixXd(4, 4);
	Q_ <<  At_pow4_4 * noise_ax, 0.0f, At_pow3_2 * noise_ax, 0.0f,
	           0.0f, At_pow4_4 * noise_ay, 0.0f, At_pow3_2 * noise_ay,
	           At_pow3_2 * noise_ax, 0.0f, At_pow2_ * noise_ax, 0.0f, 
	           0.0f, At_pow3_2 * noise_ay, 0.0f, At_pow2_ * noise_ay;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	Eigen::VectorXd y = z - (H_ * x_);
	CommonUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	float range = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
	float bearing = atan2(x_(1), x_(0));
	float range_rate = 0.0f;
	// Check division by zero.
	if(range < Tools::Epsilon) {
	    std::cout << "Error [EKF]: division by zero!" << std::endl;
	} else { 
		range_rate = (x_(0) * x_(2) + x_(1) * x_(3)) / range;	
	}
	
	Eigen::VectorXd h = VectorXd(3); 
	h << range, bearing, range_rate;
	Eigen::VectorXd y = z - h;
	CommonUpdate(y);
}

void KalmanFilter::CommonUpdate(const Eigen::VectorXd &y) {
	Eigen::MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
	Eigen::MatrixXd H_t = H_.transpose();
	Eigen::MatrixXd S = H_ * P_ * H_t + R_;
	Eigen::MatrixXd K = P_ * H_t * S.inverse();
	
	x_ = x_ + (K * y);
	P_ = (I - (K * H_)) * P_;
}
