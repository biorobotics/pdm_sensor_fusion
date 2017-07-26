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

  ros::init(argc, argv, "listener");
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
  callbackclass momod1D_1; 
  momod1D_1.name.data = "1D Model1 - constant velocity";

  momod1D_1.F << 1.0, dt, 0.0, 1.0;

  momod1D_1.Q << sigma_proc, 0, 0, sigma_proc; 
  momod1D_1.R << sigma_meas, 0, 0, sigma_meas; 

  // Motion-Model 2 : 1D Ballistic 
  callbackclass momod1D_2; 
  momod1D_2.name.data = "1D Model2 - ballistic";

  momod1D_2.F << 1.0, dt, 0.0, 1.0;
  momod1D_2.u << -g*0.5*dt*dt, -g*dt; 

  momod1D_2.Q << sigma_proc, 0, 0, sigma_proc; 
  momod1D_2.R << sigma_meas, 0, 0, sigma_meas;

  // Motion-Model 3: 1D Spring-Mass
  callbackclass momod1D_3; 
  momod1D_3.name.data = "1D Model3 - spring & mass";

  momod1D_3.F << cos(w*dt), sin(w*dt)/w, -w*sin(w*dt), cos(w*dt); 
  momod1D_3.u << leq*(1 - cos(w*dt) - sin(w*dt)/w), leq*(1 + w*sin(w*dt) - cos(w*dt));

  momod1D_1.Q << sigma_proc, 0, 0, sigma_proc; 
  momod1D_1.R << sigma_meas, 0, 0, sigma_meas;   


  // Callback function(s): ---------------------------------------------------------------------------------
  ros::Subscriber sub1 = n.subscribe("pose_chatter2", 100, &callbackclass::myCb, &momod1D_1);
  ros::Subscriber sub2 = n.subscribe("pose_chatter2", 100, &callbackclass::myCb, &momod1D_2);
  ros::Subscriber sub3 = n.subscribe("pose_chatter2", 100, &callbackclass::myCb, &momod1D_3);

  while (ros::ok())
  {
	  	ros::spinOnce();
  }
  return 0;
}