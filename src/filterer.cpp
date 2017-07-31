#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Core> 
#include <Eigen/LU>
#include <Eigen/Dense>

#include <iostream>

#include <pdm_sensor_fusion/filter_classes.h>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "filterer");

  ros::NodeHandle nh;

  // Kalman Filtering 1D jumping: 

  // paramters 
  //(TODO: Some model parameters might need to vary over time...)
  float sigma_proc = 1.0; 
  float sigma_meas = 1.0; 

  float dt = 0.01;
 
  float m = 1; 
  float k = 25; 
  float l0 = 1; 

  float leq = l0 - m*9.81/k; 
  float w = sqrt(k/m);


  // Motion-Model 1 : 1D Constant velocity 
  KalmanFilter_1D model1D_1(1,1, nh);  
  model1D_1.set_model_parameters(dt, 9.81, -1, 0); 
  model1D_1.update_process_model(); 
  model1D_1.initialize_noise(sigma_proc, sigma_proc, sigma_meas); 


  // Motion-Model 2 : 1D Ballistic 
  KalmanFilter_1D model1D_2(2,1, nh); 
  model1D_2.set_model_parameters(dt, 9.81, -1, 0);
  model1D_2.update_process_model(); 
  model1D_2.initialize_noise(sigma_proc, sigma_proc, sigma_meas); 

  // Motion-Model 3: 1D Spring-Mass
  KalmanFilter_1D model1D_3(3,1, nh); 
  model1D_3.set_model_parameters(dt, 9.81, w, leq);
  model1D_3.update_process_model(); 
  model1D_3.initialize_noise(sigma_proc, sigma_proc, sigma_meas); 

  // Callback function(s): ---------------------------------------------------------------------------------
  ros::Subscriber sub1 = nh.subscribe("pose_chatter2", 100, &KalmanFilter_1D::myCb, &model1D_1);
  ros::Subscriber sub2 = nh.subscribe("pose_chatter2", 100, &KalmanFilter_1D::myCb, &model1D_2);
  ros::Subscriber sub3 = nh.subscribe("pose_chatter2", 100, &KalmanFilter_1D::myCb, &model1D_3);

  while (ros::ok())
  {
	  	ros::spinOnce();
  }
  return 0;
}