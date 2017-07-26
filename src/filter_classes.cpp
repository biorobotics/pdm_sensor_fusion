#include <pdm_sensor_fusion/filter_classes.h>

using namespace std; 



/// Constructor
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

/// Destructor 
callbackclass::~callbackclass(void)
{
	std::cout << "Info: Object callbackclass is being deleted." << std::endl; 
}

/// Member function(s)
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


