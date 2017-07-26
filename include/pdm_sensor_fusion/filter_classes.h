#ifndef FLITER_CLASSES_H_
#define FLITER_CLASSES_H_


#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Core> 
#include <Eigen/LU>
#include <Eigen/Dense>

#include <iostream>


using namespace std; 

/// Class 1
class callbackclass {
public: 
	// sequence ID (for information)
	std_msgs::String name; 
	int seqID_previous;
	int seqID_current; 

	// Kalman Filter Matrices and vectors
	Eigen::Matrix2f F; 
	Eigen::Matrix2f B; 
	Eigen::Matrix2f H; 
	Eigen::Matrix2f Q; 
	Eigen::Matrix2f R; 
	Eigen::Vector2f u; 

	// State vector and State Covariance
	Eigen::Vector2f x_previous;
	Eigen::Vector2f x_current; 
	Eigen::Matrix2f P_previous; 
	Eigen::Matrix2f P_current;

	// Callback function 
	void myCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

	// Constructor & Destructor
	callbackclass();
	~callbackclass();
};


#endif // FILTER_CLASSES_H_
