#ifndef FLITER_CLASSES_H_
#define FLITER_CLASSES_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
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
	ros::NodeHandle n_; 

	ros::Publisher pub_pose_;
	ros::Publisher pub_kfmsg_;

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
	Eigen::Vector2f K; 					// Kalman Gain

	// Model parameters; 
	float dt_; 
	float g_; 
	float w_; 
	float leq_;


	// Member functions
	void myCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void initialize_noise(float sigma_process1, float sigma_process2, float sigma_measurement);
	void update_sigma_z(float sigma_z); // same as sigma_measurement, but may not be constant (can come from vision)
	void update_process_model(void);
	void set_model_parameters(float dt, float g, float w, float leq);

	// Constructor & Destructor
	KalmanFilter_1D(int model_id, int object_id, ros::NodeHandle n);
	~KalmanFilter_1D();
}; 

#endif // FILTER_CLASSES_H_