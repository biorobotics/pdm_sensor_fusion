#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Core> 
#include <Eigen/LU>
#include <Eigen/Dense>

#include <iostream>



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

// Constructor
callbackclass::callbackclass(void) 
{
	std::cout<< "Info: Object callbackclass is being created." << std::endl;
	
	// Initialisation

	name.data = "_"; 

	seqID_previous = 0; 
	seqID_current = 0;

	x_previous << Eigen::Vector2f::Zero(2); 
	x_current << Eigen::Vector2f::Zero(2); 

	P_previous << 1000* Eigen::Matrix2f::Identity(2,2); 
	P_current << 1000* Eigen::Matrix2f::Identity(2,2); 

	F << Eigen::Matrix2f::Identity(2,2); 
	B << Eigen::Matrix2f::Identity(2,2); 
	H << Eigen::Matrix2f::Identity(2,2);  
	Q << Eigen::Matrix2f::Identity(2,2); 
	R << Eigen::Matrix2f::Identity(2,2); 
	u << Eigen::Vector2f::Zero(2);  

}

// Destructor 
callbackclass::~callbackclass(void)
{
	std::cout << "Info: Object callbackclass is being deleted." << std::endl; 
}

// Member function(s)
void callbackclass::myCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	// Storing current measurement in vector
	Eigen::Vector2f z_current; 
	z_current << msg->pose.pose.position.z, 0; 

	// Local variables: 

	Eigen::Vector2f x_temp;
	Eigen::Matrix2f P_temp;  

	Eigen::Vector2f innov; 
	Eigen::Matrix2f S; 
	Eigen::Matrix2f K; 
	
	// Prediction Step
	x_temp = F*x_previous + B*u; 
	P_temp = F*P_previous*F.transpose() + Q; 

	
	// Update Step 
	innov = z_current - H*x_temp; 
	S = R + H*P_temp*H.transpose();
	K = P_temp*H.transpose()*S.inverse();
	 
	x_current = x_temp + K*innov; 	
	P_current = (Eigen::Matrix2f::Identity(2,2) - K*H)*P_temp;

	// Print information
	std::cout << "Name: \t" << name.data.c_str() << std::endl; 
	std::cout << "Sequence ID \t previous/current:\t" << seqID_previous << "/" << seqID_current << std::endl; 
	std::cout << "z_current:\n" << z_current << std::endl; 
  	std::cout << "x_current:\n" << x_current << "\n" << std::endl; 
  	 
  	// Shifting the state vector and covariance as well as IDs
  	x_previous = x_current; 
  	P_previous = P_current; 

  	seqID_previous = seqID_current; 
	seqID_current = msg->header.seq;
}



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