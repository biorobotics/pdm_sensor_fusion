#include <pdm_sensor_fusion/filter_classes.h>

using namespace std; 

/// Constructor
KalmanFilter_1D::KalmanFilter_1D(int model_id, int object_id) 
{
	id_ = object_id; 
	model_id_ = model_id;

	// Creation node & publisher
	string s1 = string("kfpose_m") + to_string(model_id_) + string("_") + to_string(id_); 
	string s2 = string("kfinfo_m") + to_string(model_id_) + string("_") + to_string(id_);
	
	ros::NodeHandle n1_; 
	ros::NodeHandle n2_; 
	pub_pose_ 	= n1_.advertise<geometry_msgs::PoseWithCovarianceStamped>(s1, 100);
	pub_kfmsg_ 	= n2_.advertise<std_msgs::Float32MultiArray>(s2, 100);

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
 
 	// Model parameters
 	dt_ = 100; 
	g_ = 9.81; 
	w_ = -1; 
	leq_ = 0;

	cout<< "{Info} \tKalmanFilter_1D initialized (model_id = " << model_id_<< ", object_id = "<< id_ << ")." << endl;
}

/// Destructor 
KalmanFilter_1D::~KalmanFilter_1D(void)
{
	cout << "{Info} \tKalmanFilter_1D deleted (model_id = " << model_id_<< ", object_id = "<< id_ << ")." << endl; 
}

/// Member function(s)
void KalmanFilter_1D::myCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	timestamp_in_.stamp = ros::Time::now(); 
	timestamp_in_.seq 	= msg->header.seq; 
	timestamp_out_.seq 	= msg->header.seq; 

	int index = 3; // third axis
	z_info_current_ = msg->header;
	z = msg->pose.pose.position.z; 

	ros::Duration z_Dt = z_info_current_.stamp - z_info_previous_.stamp;
	ros::Duration lag_Dt = timestamp_in_.stamp - z_info_current_.stamp; 
	double z_dt = z_Dt.toSec();
	double lag_dt = lag_Dt.toSec();  

	// Adjust dt and update process matrices
	dt_ = z_dt; 
	update_process_model(); 


	cout << "Sigma_measurement " << R << endl; 
	cout << "z_estimatet VO" << msg->pose.covariance[14] << endl;  



	// Local variables (to avoid aliasing):
	Eigen::Vector2f x_temp;
	Eigen::Matrix2f P_temp;  

	// Prediction Step
	x_temp = F*x + B*u; 
	P_temp = F*P*F.transpose() + Q; 

	// Update Step 
	y_pre = z - H*x_temp; 
	S = R + H*P_temp*H.transpose();
	K = P_temp*H.transpose()/S; 						

	x = x_temp + K*y_pre; 	
	P = (Eigen::Matrix2f::Identity(2,2) - K*H)*P_temp;
	y_post = z - H*x; 
	
	// Shifting information
  	z_info_previous_ = z_info_current_;    

	timestamp_out_.stamp = ros::Time::now(); 
	ros::Duration processing_Dt = timestamp_out_.stamp - timestamp_in_.stamp;
	double processing_dt = processing_Dt.toSec(); 

	// Publishing data

	geometry_msgs::PoseWithCovarianceStamped filtered_pose; 

	filtered_pose.header = msg->header;
	filtered_pose.pose = msg->pose; 
	filtered_pose.pose.covariance[(index - 1)*7] = P(1,1); 
	filtered_pose.pose.pose.position.z = x(0);

	pub_pose_.publish(filtered_pose);

	std_msgs::Float32MultiArray kf_custom_msg;
	kf_custom_msg.data.clear(); 

	kf_custom_msg.data.push_back(msg->header.seq); 
	kf_custom_msg.data.push_back(msg->header.stamp.sec);
	kf_custom_msg.data.push_back(msg->header.stamp.nsec);

	kf_custom_msg.data.push_back(x(0)); 
	kf_custom_msg.data.push_back(x(1)); 
	kf_custom_msg.data.push_back(P(0,0)); 
	kf_custom_msg.data.push_back(P(1,0)); 
	kf_custom_msg.data.push_back(P(0,1)); 
	kf_custom_msg.data.push_back(P(1,1));
	kf_custom_msg.data.push_back(K(0));
	kf_custom_msg.data.push_back(K(1));
	kf_custom_msg.data.push_back(y_pre); 
	kf_custom_msg.data.push_back(y_post); 

	kf_custom_msg.data.push_back(z_dt); 
	kf_custom_msg.data.push_back(lag_dt); 
	kf_custom_msg.data.push_back(processing_dt);

	pub_kfmsg_.publish(kf_custom_msg);
}


void KalmanFilter_1D::initialize_noise(float sigma_process1, float sigma_process2, float sigma_measurement)
{
	Q << sigma_process1, 0, 0, sigma_process2; 
	R = sigma_measurement; 
}

void KalmanFilter_1D::update_sigma_z(float sigma_z)
{
	R = sigma_z; 
}

void KalmanFilter_1D::update_process_model(void)
{
	if (model_id_ == 1)
		F << 1.0, dt_, 0.0, 1.0;
	else if (model_id_ == 2)
	{
		F << 1.0, dt_, 0.0, 1.0;
  		u << -g_*0.5*dt_*dt_, -g_*dt_;
	}	
	else if (model_id_ == 3)
	{
		F << cos(w_*dt_), sin(w_*dt_)/w_, -w_*sin(w_*dt_), cos(w_*dt_); 
  		u << leq_*(1 - cos(w_*dt_)), leq_*w_*sin(w_*dt_);
	}
	else 
		cout << "Warning, model_id " << model_id_ << " not known (object id " << id_ << "). Process not updated." << endl; 
}

void KalmanFilter_1D::set_model_parameters(float dt, float g, float w, float leq)
{
 	dt_ = dt; 
 	g = g_; 
 	w_ = w; 
 	leq_ = leq; 
}