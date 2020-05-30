/*ros*/
#include <ros/ros.h>
#include <tf/tf.h>
/*msg*/
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
/*C*/
#include <fstream>
/*opencv*/
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class SaveImageWithIMU{
	private:
		/*node hangle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscriber*/
		ros::Subscriber sub_imu;
		ros::Subscriber sub_image;
		ros::Subscriber sub_odom;
		/*publisher*/
		ros::Publisher pub_pose;
		/*objects*/
		sensor_msgs::Imu imu;
		nav_msgs::Odometry odom_now;
		nav_msgs::Odometry odom_last;
		geometry_msgs::PoseStamped pose;
		std::ofstream file;
		/*counter*/
		int counter = 0;
		/*flag*/
		bool got_first_cb_odom = false;
		/*param*/
		std::string save_dir_path;
		std::string save_csv_path;
		int save_data_limit;
		double th_diff_position_m;
		double th_diff_angle_deg;
	public:
		SaveImageWithIMU();
		void InitializePose(geometry_msgs::PoseStamped& pose);
		void CallbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void CallbackImage(const sensor_msgs::ImageConstPtr& msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		bool HasOdomDiff(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2);
		void Publication(void);
		void Record(cv_bridge::CvImagePtr cv_ptr);
};

SaveImageWithIMU::SaveImageWithIMU()
	:nhPrivate("~")
{
	/*param*/
	nhPrivate.param("save_dir_path", save_dir_path, std::string("dataset"));
	std::cout << "save_dir_path = " << save_dir_path << std::endl;
	nhPrivate.param("save_csv_path", save_csv_path, std::string(save_dir_path + "/save_image_with_imu.csv"));
	std::cout << "save_csv_path = " << save_csv_path << std::endl;
	nhPrivate.param("save_data_limit", save_data_limit, 10);
	std::cout << "save_data_limit = " << save_data_limit << std::endl;
	nhPrivate.param("th_diff_position_m", th_diff_position_m, 10.0);
	std::cout << "th_diff_position_m = " << th_diff_position_m << std::endl;
	nhPrivate.param("th_diff_angle_deg", th_diff_angle_deg, 30.0);
	std::cout << "th_diff_angle_deg = " << th_diff_angle_deg << std::endl;

	/*subscriber*/
	sub_imu = nh.subscribe("/imu/data", 1, &SaveImageWithIMU::CallbackIMU, this);
	sub_image = nh.subscribe("/image_raw", 1, &SaveImageWithIMU::CallbackImage, this);
	sub_odom = nh.subscribe("/odom", 1, &SaveImageWithIMU::CallbackOdom, this);
	/*publisher*/
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose", 1);
	/*initialize*/
	InitializePose(pose);
	file.open(save_csv_path);
	if(!file){
		std::cout << "Cannot open " << save_csv_path << std::endl;
		exit(1);
	}
}

void SaveImageWithIMU::InitializePose(geometry_msgs::PoseStamped& pose)
{
	pose.pose.position.x = 0.0;
	pose.pose.position.y = 0.0;
	pose.pose.position.z = 0.0;
	pose.pose.orientation.x = 0.0;
	pose.pose.orientation.y = 0.0;
	pose.pose.orientation.z = 0.0;
	pose.pose.orientation.w = 1.0;
}

void SaveImageWithIMU::CallbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	imu = *msg;

	pose.header.stamp = msg->header.stamp;
	pose.pose.orientation = msg->orientation;

	Publication();
}

void SaveImageWithIMU::CallbackImage(const sensor_msgs::ImageConstPtr& msg)
{
	/* std::cout << "msg->encoding = " << msg->encoding << std::endl; */
	if(counter < save_data_limit){
		if(got_first_cb_odom && HasOdomDiff(odom_now, odom_last)){
			try{
				cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				Record(cv_ptr);
			}
			catch(cv_bridge::Exception& e){
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
		}
	}
	else if(file.is_open()){
		file.close();
		std::cout << save_csv_path << " was closed" << std::endl;
	}
}

void SaveImageWithIMU::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	/* std::cout << "msg->twist.twist.linear.x = " << msg->twist.twist.linear.x << std::endl; */
	pose.header.stamp = msg->header.stamp;
	pose.header.frame_id = msg->header.frame_id;

	if(!got_first_cb_odom){
		odom_last = *msg;
		got_first_cb_odom = true;
	}
	odom_now = *msg;
}

void SaveImageWithIMU::Publication(void)
{
	/*publish*/
	pub_pose.publish(pose);
}

bool SaveImageWithIMU::HasOdomDiff(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2)
{
	/*position*/
	double dx = odom2.pose.pose.position.x - odom1.pose.pose.position.x;
	double dy = odom2.pose.pose.position.y - odom1.pose.pose.position.y;
	double dz = odom2.pose.pose.position.z - odom1.pose.pose.position.z;
	double diff_position_m = sqrt(dx*dx + dy*dy + dz*dz);
	/*rotation*/
	tf::Quaternion q1, q2, q_rel_rot;
	quaternionMsgToTF(odom1.pose.pose.orientation, q1);
	quaternionMsgToTF(odom2.pose.pose.orientation, q2);
	q_rel_rot = q2.inverse()*q1;
	double droll, dpitch, dyaw;
	tf::Matrix3x3(q_rel_rot).getRPY(droll, dpitch, dyaw);
	double diff_angle_deg = q_rel_rot.getAngle()/M_PI*180.0;
	/*print*/
	/* std::cout << "d_xyz : "  */
	/* 	<< dx << ", " */
	/* 	<< dy << ", " */
	/* 	<< dz << std::endl; */
	/* std::cout << "d_rpy : "  */
	/* 	<< droll/M_PI*180.0 << ", " */
	/* 	<< dpitch/M_PI*180.0 << ", " */
	/* 	<< dyaw/M_PI*180.0 << std::endl; */
	/*judge*/
	if(diff_position_m > th_diff_position_m)	return true;
	if(diff_angle_deg > th_diff_angle_deg)	return true;
	return false;
}

void SaveImageWithIMU::Record(cv_bridge::CvImagePtr cv_ptr)
{
	/*image*/
	std::string save_img_name = save_dir_path + "/img" + std::to_string(counter) + ".jpg";
	cv::imwrite(save_img_name, cv_ptr->image);
	/*imu*/
	file 
		<< imu.linear_acceleration.x << "," 
		<< imu.linear_acceleration.y << "," 
		<< imu.linear_acceleration.z << ","
		<< save_img_name << std::endl;
	/*print*/
	double r, p, y;
	tf::Quaternion q;
	quaternionMsgToTF(odom_now.pose.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(r, p, y);
	std::cout << counter << " save point: " 
		<< odom_now.pose.pose.position.x << ", "
		<< odom_now.pose.pose.position.y << ", "
		<< odom_now.pose.pose.position.z << ", "
		<< r/M_PI*180.0 << ", " 
		<< p/M_PI*180.0 << ", " 
		<< y/M_PI*180.0 << std::endl;
	std::cout << "imu acc: " 
		<< imu.linear_acceleration.x << "," 
		<< imu.linear_acceleration.y << "," 
		<< imu.linear_acceleration.z << std::endl;
	/*count*/
	++counter;
	odom_last = odom_now;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_image_with_imu");

	SaveImageWithIMU save_image_with_imu;

	ros::spin();
}
