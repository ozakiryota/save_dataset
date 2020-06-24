/*ros*/
#include <ros/ros.h>
#include <tf/tf.h>
/*msg*/
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
/*C*/
#include <fstream>
/*opencv*/
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class SaveImageWithIMU{
	private:
		/*node hangle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_imu;
		ros::Subscriber _sub_image;
		ros::Subscriber _sub_odom;
		/*objects*/
		sensor_msgs::Imu _imu;
		nav_msgs::Odometry _odom_now;
		nav_msgs::Odometry _odom_last;
		std::ofstream _csvfile;
		/*counter*/
		int _counter = 0;
		/*flag*/
		bool _got_first_cb_odom = false;
		/*param*/
		std::string _save_dir_path;
		std::string _save_csv_path;
		std::string _save_img_name;
		int _save_data_limit;
		double _th_diff_position_m;
		double _th_diff_angle_deg;
	public:
		SaveImageWithIMU();
		void callbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void callbackImage(const sensor_msgs::ImageConstPtr& msg);
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		bool hasOdomDiff(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2);
		void publication(void);
		void record(cv_bridge::CvImagePtr cv_ptr);
};

SaveImageWithIMU::SaveImageWithIMU()
	:_nhPrivate("~")
{
	/*param*/
	_nhPrivate.param("save_dir_path", _save_dir_path, std::string("dataset"));
	std::cout << "_save_dir_path = " << _save_dir_path << std::endl;
	_nhPrivate.param("save_csv_path", _save_csv_path, std::string(_save_dir_path + "/save_image_with_imu.csv"));
	std::cout << "_save_csv_path = " << _save_csv_path << std::endl;
	_nhPrivate.param("save_img_name", _save_img_name, std::string("img"));
	std::cout << "_save_img_name = " << _save_img_name << std::endl;
	_nhPrivate.param("save_data_limit", _save_data_limit, 10);
	std::cout << "_save_data_limit = " << _save_data_limit << std::endl;
	_nhPrivate.param("th_diff_position_m", _th_diff_position_m, 10.0);
	std::cout << "_th_diff_position_m = " << _th_diff_position_m << std::endl;
	_nhPrivate.param("th_diff_angle_deg", _th_diff_angle_deg, 30.0);
	std::cout << "_th_diff_angle_deg = " << _th_diff_angle_deg << std::endl;

	/*subscriber*/
	_sub_imu = _nh.subscribe("/imu/data", 1, &SaveImageWithIMU::callbackIMU, this);
	_sub_image = _nh.subscribe("/image_raw", 1, &SaveImageWithIMU::callbackImage, this);
	_sub_odom = _nh.subscribe("/odom", 1, &SaveImageWithIMU::callbackOdom, this);
	/*initialize*/
	_csvfile.open(_save_csv_path, std::ios::app);
	if(!_csvfile){
		std::cout << "Cannot open " << _save_csv_path << std::endl;
		exit(1);
	}
}

void SaveImageWithIMU::callbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	_imu = *msg;
}

void SaveImageWithIMU::callbackImage(const sensor_msgs::ImageConstPtr& msg)
{
	if(_counter < _save_data_limit){
		if(_got_first_cb_odom && hasOdomDiff(_odom_now, _odom_last)){
			try{
				cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				record(cv_ptr);
			}
			catch(cv_bridge::Exception& e){
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
		}
	}
	else if(_csvfile.is_open()){
		_csvfile.close();
		std::cout << _save_csv_path << " was closed" << std::endl;
	}

	/* std::cout << "msg->encoding = " << msg->encoding << std::endl; */
}

void SaveImageWithIMU::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	_odom_now = *msg;
	if(!_got_first_cb_odom){
		_odom_last = *msg;
		_got_first_cb_odom = true;
	}
}

bool SaveImageWithIMU::hasOdomDiff(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2)
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
	if(diff_position_m > _th_diff_position_m)	return true;
	if(diff_angle_deg > _th_diff_angle_deg)	return true;
	return false;
}

void SaveImageWithIMU::record(cv_bridge::CvImagePtr cv_ptr)
{
	/*check*/
	std::string save_img_path = _save_dir_path + "/" + _save_img_name + std::to_string(_counter) + ".jpg";
	std::ifstream ifs(save_img_path);
	if(ifs.is_open()){
		std::cout << save_img_path << " already exists" << std::endl;
		exit(1);
	}
	/*save image*/
	cv::imwrite(save_img_path, cv_ptr->image);
	/*record imu*/
	_csvfile 
		<< _imu.linear_acceleration.x << "," 
		<< _imu.linear_acceleration.y << "," 
		<< _imu.linear_acceleration.z << ","
		<< save_img_path << std::endl;
	/*print*/
	double r, p, y;
	tf::Quaternion q;
	quaternionMsgToTF(_odom_now.pose.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(r, p, y);
	std::cout << _counter << " save point: " 
		<< _odom_now.pose.pose.position.x << ", "
		<< _odom_now.pose.pose.position.y << ", "
		<< _odom_now.pose.pose.position.z << ", "
		<< r/M_PI*180.0 << ", " 
		<< p/M_PI*180.0 << ", " 
		<< y/M_PI*180.0 << std::endl;
	std::cout << "imu acc: " 
		<< _imu.linear_acceleration.x << "," 
		<< _imu.linear_acceleration.y << "," 
		<< _imu.linear_acceleration.z << std::endl;
	/*count*/
	++_counter;
	_odom_last = _odom_now;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_image_with_imu");

	SaveImageWithIMU save_image_with_imu;

	ros::spin();
}
