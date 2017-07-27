#include <pdm_sensor_fusion/filter_classes.h>

using namespace std; 

/// Constructor
KalmanFilter_1D::KalmanFilter_1D(int model_id, int object_id) 
{
	id_ = object_id; 
	model_id_ = model_id;

	// Header initialisations
	z_info_previous_.seq 	= -1;
	z_info_current_.seq 	= -1; 
	timestamp_in_.seq 		= -1; 
	timestamp_out_.seq 		= -1;

	z_info_previous_.stamp 	= ros::Time::now();
	z_info_current_.stamp 	= z_info_previous_.stamp;
	timestamp_in_.stamp 	= z_info_previous_.stamp; 
	timestamp_out_.stamp 	= z_info_previous_.stamp; 


	// KF structure initialisation 
	x << Eigen::Vector2f::Zero(2); 
	P << 1000*Eigen::Matrix2f::Identity(2,2); 
	u << Eigen::Vector2f::Zero(2);  
	z = 0.0; 
	
	F << Eigen::Matrix2f::Identity(2,2); 
	B << Eigen::Matrix2f::Identity(2,2); 
	Q << Eigen::Matrix2f::Identity(2,2); 

	H << 1.0, 0.0;  
	R = 1.0; 

	cout<< "{Info} \tKalmanFilter_1D initialized (model_id = " << model_id_<< ", object_id = "<< id_ << ")." << endl;
}

/// Destructor 
KalmanFilter_1D::~KalmanFilter_1D(void)
{
	cout << "{Info} \tObject KalmanFilter_1D is being deleted." << endl; 
}

/// Member function(s)
void KalmanFilter_1D::myCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	timestamp_in_.stamp = ros::Time::now(); 
	timestamp_in_.seq 	= msg->header.seq; 
	timestamp_out_.seq 	= msg->header.seq; 

	z_info_current_ = msg->header;
	z = msg->pose.pose.position.z; 

	ros::Duration z_Dt = z_info_current_.stamp - z_info_previous_.stamp;
	double z_dt = z_Dt.toSec(); 

	// Local variables (to avoid aliasing):
	Eigen::Vector2f x_temp;
	Eigen::Matrix2f P_temp;  

	// Prediction Step
	x_temp = F*x + B*u; 
	P_temp = F*P*F.transpose() + Q; 

	// Update Step 
	y_pre = z - H*x_temp; 
	S = R + H*P_temp*H.transpose();
	K = P_temp*H.transpose()/S; 						//K = P_temp*H.transpose()*S.inverse();

	x = x_temp + K*y_pre; 	
	P = (Eigen::Matrix2f::Identity(2,2) - K*H)*P_temp;
	y_post = z - H*x; 
	

	// Print information
	/*
	cout << "\nModel_id:" << model_id_ << " Object_id: "<< id_ << endl;
	cout << z_info_current_ << endl; 
	cout << "z \n" << z << endl; 
	cout << "x \n" << x << endl;
	cout << "det(P) \n" << P.determinant() << endl; 
	cout << "y_post \n" << y_post << endl; 
	*/

	// Shifting information
  	z_info_previous_ = z_info_current_;    


	timestamp_out_.stamp = ros::Time::now(); 
	ros::Duration processing_Dt = timestamp_out_.stamp - timestamp_in_.stamp;
	double processing_dt = processing_Dt.toSec(); 


	cout << "-------" << z_info_current_.seq << endl; 

	cout << z_Dt << endl;
	cout << z_dt << endl;

	cout << processing_Dt << endl;
	cout << processing_dt << endl;
	 
}


