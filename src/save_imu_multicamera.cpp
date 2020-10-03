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
		ros::Subscriber _sub_odom;
		std::vector<ros::Subscriber> _list_sub_images;
		/*struct*/
		struct CAMERA{
			std::string topic_name;
			std::string camera_name;
			bool got_new_image;
			cv_bridge::CvImagePtr cv_ptr;
			std::string save_path;
		};
		std::vector<CAMERA> _list_cameras;
		/*msg*/
		sensor_msgs::Imu _imu;
		nav_msgs::Odometry _odom_now;
		nav_msgs::Odometry _odom_last;
		/*file*/
		std::ofstream _csvfile;
		/*counter*/
		int _record_counter = 0;
		int _still_counter = 0;
		/*flag*/
		bool _got_first_imu = false;
		bool _got_first_odom = false;
		bool _is_still = false;
		/*param*/
		std::string _save_dir_path;
		std::string _save_csv_path;
		std::string _save_img_name;
		int _save_data_limit;
		double _th_diff_position_m;
		double _th_diff_angle_deg;
		double _th_still_position_m;
		double _th_still_angle_deg;
		int _th_still_counter;
		int _num_cameras;
	public:
		SaveImageWithIMU();
		void callbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void callbackImage(const ros::MessageEvent<sensor_msgs::Image const>& event);
		bool hasOdomDiff(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2);
		int getCameraIndex(std::string topic_name);
		bool gotAllNewImages(void);
		void record(void);
		std::vector<std::string> splitSentence(std::string sentence, std::string delimiter);
};

SaveImageWithIMU::SaveImageWithIMU()
	:_nhPrivate("~")
{
	/*param*/
	_nhPrivate.param("save_dir_path", _save_dir_path, std::string("dataset"));
	std::cout << "_save_dir_path = " << _save_dir_path << std::endl;
	_nhPrivate.param("save_csv_path", _save_csv_path, std::string(_save_dir_path + "/save_multiimage_with_imu.csv"));
	std::cout << "_save_csv_path = " << _save_csv_path << std::endl;
	_nhPrivate.param("save_img_name", _save_img_name, std::string("img"));
	std::cout << "_save_img_name = " << _save_img_name << std::endl;
	_nhPrivate.param("save_data_limit", _save_data_limit, 10);
	std::cout << "_save_data_limit = " << _save_data_limit << std::endl;
	_nhPrivate.param("th_diff_position_m", _th_diff_position_m, 10.0);
	std::cout << "_th_diff_position_m = " << _th_diff_position_m << std::endl;
	_nhPrivate.param("th_diff_angle_deg", _th_diff_angle_deg, 30.0);
	std::cout << "_th_diff_angle_deg = " << _th_diff_angle_deg << std::endl;
	_nhPrivate.param("th_still_position_m", _th_still_position_m, 0.001);
	std::cout << "_th_still_position_m = " << _th_still_position_m << std::endl;
	_nhPrivate.param("th_still_angle_deg", _th_still_angle_deg, 0.1);
	std::cout << "_th_still_angle_deg = " << _th_still_angle_deg << std::endl;
	_nhPrivate.param("th_still_counter", _th_still_counter, 10);
	std::cout << "_th_still_counter = " << _th_still_counter << std::endl;
	_nhPrivate.param("num_cameras", _num_cameras, 1);
	std::cout << "_num_cameras = " << _num_cameras << std::endl;
	for(int i=0; i<_num_cameras; ++i){
		CAMERA camera;
		_nhPrivate.param("image" + std::to_string(i), camera.topic_name, "/image" + std::to_string(i));
		camera.camera_name = splitSentence(camera.topic_name, "/")[1];
		camera.got_new_image = false;
		_list_cameras.push_back(camera);
		std::cout << "_list_cameras[" << i << "].topic_name = " << _list_cameras[i].topic_name << std::endl;
	}

	/*subscriber*/
	_sub_imu = _nh.subscribe("/imu/data", 1, &SaveImageWithIMU::callbackIMU, this);
	_sub_odom = _nh.subscribe("/odom", 1, &SaveImageWithIMU::callbackOdom, this);
	for(int i=0; i<_num_cameras; ++i){
		ros::Subscriber sub_image = _nh.subscribe(_list_cameras[i].topic_name, 1, &SaveImageWithIMU::callbackImage, this);
		_list_sub_images.push_back(sub_image);
	}
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
	if(!_got_first_imu)	_got_first_imu = true;
}

void SaveImageWithIMU::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	if(_got_first_odom)	_is_still = isStill(*msg, _odom_now);
	_odom_now = *msg;
	if(!_got_first_odom){
		_odom_last = *msg;
		_got_first_odom = true;
	}
}

void SaveImageWithIMU::callbackImage(const ros::MessageEvent<sensor_msgs::Image const>& event)
{
	const ros::M_string& header = event.getConnectionHeader();
	std::string topic_name = header.at("topic");
	const sensor_msgs::ImageConstPtr& msg = event.getMessage();
	// std::cout << "msg->encoding = " << msg->encoding << std::endl;

	if(_record_counter < _save_data_limit){
		if(_got_first_odom && _got_first_imu && hasOdomDiff(_odom_now, _odom_last)){
			try{
				int camera_index = getCameraIndex(topic_name);
				_list_cameras[camera_index].cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				_list_cameras[camera_index].got_new_image = true;
				if(gotAllNewImages() && _is_still)	record();
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
}

bool SaveImageWithIMU::isStill(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2)
{
	double diff_position_m, diff_angle_deg;
	getOdomDiff(odom1, odom2, diff_position_m, diff_angle_deg);
	if(diff_position_m < _th_still_position_m && diff_angle_deg < _th_still_angle_deg)	_is_still = true;
	else	_is_still = false;
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
	//std::cout << "d_xyz : " 
	//	<< dx << ", "
	//	<< dy << ", "
	//	<< dz << std::endl;
	//std::cout << "d_rpy : " 
	//	<< droll/M_PI*180.0 << ", "
	//	<< dpitch/M_PI*180.0 << ", "
	//	<< dyaw/M_PI*180.0 << std::endl;
	/*judge*/
	if(diff_position_m > _th_diff_position_m)	return true;
	if(diff_angle_deg > _th_diff_angle_deg)	return true;
	return false;
}

void SaveImageWithIMU::getOdomDiff(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2, double& diff_position_m, double& diff_angle_deg)
{
	/*position*/
	double dx = odom2.pose.pose.position.x - odom1.pose.pose.position.x;
	double dy = odom2.pose.pose.position.y - odom1.pose.pose.position.y;
	double dz = odom2.pose.pose.position.z - odom1.pose.pose.position.z;
	diff_position_m = sqrt(dx*dx + dy*dy + dz*dz);
	/*rotation*/
	tf::Quaternion q1, q2, q_rel_rot;
	quaternionMsgToTF(odom1.pose.pose.orientation, q1);
	quaternionMsgToTF(odom2.pose.pose.orientation, q2);
	q_rel_rot = q2.inverse()*q1;
	double droll, dpitch, dyaw;
	tf::Matrix3x3(q_rel_rot).getRPY(droll, dpitch, dyaw);
	diff_angle_deg = q_rel_rot.getAngle()/M_PI*180.0;
	/*print*/
	//std::cout << "d_xyz : " 
	//	<< dx << ", "
	//	<< dy << ", "
	//	<< dz << std::endl;
	//std::cout << "d_rpy : " 
	//	<< droll/M_PI*180.0 << ", "
	//	<< dpitch/M_PI*180.0 << ", "
	//	<< dyaw/M_PI*180.0 << std::endl;
}

int SaveImageWithIMU::getCameraIndex(std::string topic_name)
{
	for(size_t index=0; index<_list_cameras.size(); ++index){
		if(topic_name == _list_cameras[index].topic_name)	return index;
	}
	return -1;
}

bool SaveImageWithIMU::gotAllNewImages(void)
{
	for(size_t index=0; index<_list_cameras.size(); ++index){
		if(!_list_cameras[index].got_new_image)	return false;
	}
	return true;
}

void SaveImageWithIMU::record(void)
{
	for(size_t i=0; i<_list_cameras.size(); ++i){
		/*check*/
		// _list_cameras[i].save_path = _save_dir_path + "/" + _save_img_name + std::to_string(_record_counter) + "_" + _list_cameras[i].camera_name + ".jpg";
		_list_cameras[i].save_path = _save_dir_path + "/" + _save_img_name + _list_cameras[i].camera_name + "_" + std::to_string(_record_counter) + ".jpg";
		std::ifstream ifs(_list_cameras[i].save_path);
		if(ifs.is_open()){
			std::cout << _list_cameras[i].save_path << " already exists" << std::endl;
			exit(1);
		}
		/*save image*/
		cv::imwrite(_list_cameras[i].save_path, _list_cameras[i].cv_ptr->image);
	}
	/*record imu*/
	_csvfile 
		<< _imu.linear_acceleration.x << "," 
		<< _imu.linear_acceleration.y << "," 
		<< _imu.linear_acceleration.z;
	for(size_t i=0; i<_list_cameras.size(); ++i)	_csvfile << "," << _list_cameras[i].save_path;
	_csvfile << std::endl;
	/*print*/
	double r, p, y;
	tf::Quaternion q;
	quaternionMsgToTF(_odom_now.pose.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(r, p, y);
	std::cout << _record_counter << " save point: " 
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
	++_record_counter;
	/*reset*/
	for(size_t i=0; i<_list_cameras.size(); ++i)	_list_cameras[i].got_new_image = false;
	_odom_last = _odom_now;
}

std::vector<std::string> SaveImageWithIMU::splitSentence(std::string sentence, std::string delimiter)
{
	std::vector<std::string> words;
	size_t position = 0;

	while(sentence.find(delimiter.c_str(), position) != std::string::npos){
		size_t next_position = sentence.find(delimiter.c_str(), position);
		std::string word = sentence.substr(position, next_position-position);
		position = next_position + delimiter.length();
		words.push_back(word);
	}
	std::string last_word = sentence.substr(position, sentence.length()-position);
	words.push_back(last_word);

	return words;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_image_with_imu");

	SaveImageWithIMU save_image_with_imu;

	ros::spin();
}
