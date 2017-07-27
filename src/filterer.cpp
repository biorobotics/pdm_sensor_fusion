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
  ros::NodeHandle n;

  // Kalman Filtering 1D jumping: 

  // paramters 
  //(TODO: Some model parameters might need to vary over time...)
  float dt = 0.01;
  float sigma_proc = 1.0; 
  float sigma_meas = 1.0; 

  float g = 9.81; 

  float m = 1; 
  float k = 25; 
  float l0 = 1; 

  float leq = l0 - m*g/k; 
  float w = sqrt(k/m);


  // Motion-Model 1 : 1D Constant velocity 
  KalmanFilter_1D model1D_1(1,1);  

  model1D_1.F << 1.0, dt, 0.0, 1.0;
  model1D_1.Q << sigma_proc, 0, 0, sigma_proc; 
  model1D_1.R = sigma_meas; 

/*
  // Motion-Model 2 : 1D Ballistic 
  KalmanFilter_1D model1D_2(2,1); 

  model1D_2.F << 1.0, dt, 0.0, 1.0;
  model1D_2.u << -g*0.5*dt*dt, -g*dt; 
  model1D_2.Q << sigma_proc, 0, 0, sigma_proc; 
  model1D_2.R = sigma_meas;

  // Motion-Model 3: 1D Spring-Mass
  KalmanFilter_1D model1D_3(3,1); 

  model1D_3.F << cos(w*dt), sin(w*dt)/w, -w*sin(w*dt), cos(w*dt); 
  model1D_3.u << leq*(1 - cos(w*dt) - sin(w*dt)/w), leq*(1 + w*sin(w*dt) - cos(w*dt));
  model1D_1.Q << sigma_proc, 0, 0, sigma_proc; 
  model1D_1.R = sigma_meas;   

*/
  // Callback function(s): ---------------------------------------------------------------------------------
  ros::Subscriber sub1 = n.subscribe("pose_chatter1", 100, &KalmanFilter_1D::myCb, &model1D_1);
 /* 
  ros::Subscriber sub2 = n.subscribe("pose_chatter2", 100, &KalmanFilter_1D::myCb, &model1D_2);
  ros::Subscriber sub3 = n.subscribe("pose_chatter2", 100, &KalmanFilter_1D::myCb, &model1D_3);
*/
  while (ros::ok())
  {
	  	ros::spinOnce();
  }
  return 0;
}