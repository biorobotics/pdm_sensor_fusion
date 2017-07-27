#ifndef FLITER_CLASSES_H_
#define FLITER_CLASSES_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Core> 
#include <Eigen/LU>
#include <Eigen/Dense>

#include <iostream>


using namespace std; 

/// Base class 
class KalmanFilter_1D { 				// state & userinput n = m = 2, measuring l = 1 
public:
	// Identification & Headers
	int id_; 							// object id
	int model_id_; 

	std_msgs::Header z_info_previous_; 
	std_msgs::Header z_info_current_; 

	std_msgs::Header timestamp_in_ ; 
	std_msgs::Header timestamp_out_; 

	// State, control, measurment
	Eigen::Vector2f x;					// State vector 
	Eigen::Matrix<float, 2, 2> P; 		// State covariance
	Eigen::Vector2f u; 					// User input
	float z; 							// Measurement

	// Process Matrices
	// x_prior[k] = F[k]*x_posterior[k-1] + B[k]*u[k]
	Eigen::Matrix<float, 2, 2> F; 		
	Eigen::Matrix<float, 2, 2> B; 
	Eigen::Matrix<float, 2, 2> Q; 		// Proccess noise covariance

	// Measurement Matrices
	// z_estimated[k] = H[k]*x_prior[k]
	Eigen::Matrix<float, 1, 2> H; 
	float R; 							// Measurment noise covariance

	// Kalman
	float y_pre; 						// innovation (pre-fit)
	float y_post; 						// innovation (post-fit)
	float S; 							// innovation covariance
	Eigen::Matrix<float, 2, 1> K; 		// Kalman Gain


	// Member functions
	void myCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

	// Constructor & Destructor
	KalmanFilter_1D(int model_id, int object_id);
	~KalmanFilter_1D();
}; 

/*
class callbackclass {
public: 
	
	int model_id_; 						// id_ associated with type of model 

	std_msgs::Header header_vo_; 
	std_msgs::Header header_in_; 
	std_msgs::Header header_out_; 

	KalmanFilter_1D kf_1D_;  

	// Callback function 
	void myCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

	// Constructor & Destructor
	callbackclass();
	~callbackclass();
};
*/

#endif // FILTER_CLASSES_H_
