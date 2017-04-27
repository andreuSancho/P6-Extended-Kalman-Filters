#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

float const Tools::Epsilon = 0.000001f;

Tools::Tools() {}

Tools::~Tools() {}

Eigen::VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	Eigen::VectorXd rmse(4);
	rmse << 0.0f, 0.0f, 0.0f, 0.0f;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
    if(estimations.size() <= 0) {
        std::cout << "Error [RMSE]: Estimation has size zero!" << std::endl;
        return(rmse);
    }
    if(estimations.size() != ground_truth.size()) {
        std::cout << "Error [RMSE]: Estimation and ground truth missmatch sizes!" << std::endl;
        return(rmse);
    }
	// Accumulate squared residuals
	for(int i = 0; i < estimations.size(); ++i){
		Eigen::VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array() * residual.array();
		rmse += residual;
  }

  // Calculate the mean
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

Eigen::MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	Eigen::MatrixXd Hj(3, 4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float sumSquaresP = px * px + py * py;
	//check division by zero
	if(sumSquaresP <= Tools::Epsilon) {
	    std::cout << "Error [Jacobian]: division by zero when computing the Jacobian!" << std::endl;
	    sumSquaresP = Epsilon;
	}
	
	//compute the Jacobian matrix.
    float sqrtSumSquaresP = sqrtf(sumSquaresP);
    float weirdSumSquaresP = sumSquaresP * sqrtSumSquaresP;
    
	Hj << px / sqrtSumSquaresP, py / sqrtSumSquaresP, 0.0f, 0.0f,
        - py / sumSquaresP, px / sumSquaresP, 0.0f, 0.0f,
        (py * (vx * py - vy * px)) / weirdSumSquaresP, 
		(px * (vy * px - vx * py)) / weirdSumSquaresP, 
		px / sqrtSumSquaresP, py / sqrtSumSquaresP;
    
	return Hj;
}
