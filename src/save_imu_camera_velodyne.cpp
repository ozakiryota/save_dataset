/*ros*/
#include <ros/ros.h>
#include <tf/tf.h>
/*msg*/
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Odometry.h>
/*C*/
#include <fstream>
/*opencv*/
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
/*ocl*/
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class SaveIMUCameraVelodyne{
	private:
		/*node hangle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_imu;
		ros::Subscriber _sub_image;
		ros::Subscriber _sub_pc;
		ros::Subscriber _sub_odom;
		/*saved*/
		std::ofstream _csvfile;
		sensor_msgs::Imu _imu;
		cv_bridge::CvImagePtr _imgptr_color;
		cv::Mat _img_depth;
		/*point cloud*/
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _rings;
		/*odom*/
		nav_msgs::Odometry _odom_now;
		nav_msgs::Odometry _odom_last;
		/*counter*/
		int _counter = 0;
		/*flag*/
		bool _got_first_imu = false;
		bool _got_first_camera = false;
		bool _got_first_odom = false;
		/*param*/
		std::string _save_dir_path;
		std::string _save_csv_path;
		std::string _save_imgcolor_name;
		std::string _save_imgdepth_name;
		int _save_data_limit;
		double _th_diff_position_m;
		double _th_diff_angle_deg;
		int _num_ring;
		int _points_per_ring;
	public:
		SaveIMUCameraVelodyne();
		void callbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void callbackImage(const sensor_msgs::ImageConstPtr& msg);
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void pcToRings(const sensor_msgs::PointCloud2& pc_msg);
		void ringsToImage(void);
		bool hasOdomDiff(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2);
		void record(void);
};

SaveIMUCameraVelodyne::SaveIMUCameraVelodyne()
	:_nhPrivate("~")
{
	/*parameter*/
	_nhPrivate.param("save_dir_path", _save_dir_path, std::string("dataset"));
	std::cout << "_save_dir_path = " << _save_dir_path << std::endl;
	_nhPrivate.param("save_csv_path", _save_csv_path, std::string(_save_dir_path + "/save_imu_camera_velodyne.csv"));
	std::cout << "_save_csv_path = " << _save_csv_path << std::endl;
	_nhPrivate.param("save_imgcolor_name", _save_imgcolor_name, std::string("color_"));
	std::cout << "_save_imgcolor_name = " << _save_imgcolor_name << std::endl;
	_nhPrivate.param("save_imgdepth_name", _save_imgdepth_name, std::string("depth_"));
	std::cout << "_save_imgdepth_name = " << _save_imgdepth_name << std::endl;
	_nhPrivate.param("save_data_limit", _save_data_limit, 10);
	std::cout << "_save_data_limit = " << _save_data_limit << std::endl;
	_nhPrivate.param("th_diff_position_m", _th_diff_position_m, 10.0);
	std::cout << "_th_diff_position_m = " << _th_diff_position_m << std::endl;
	_nhPrivate.param("th_diff_angle_deg", _th_diff_angle_deg, 30.0);
	std::cout << "_th_diff_angle_deg = " << _th_diff_angle_deg << std::endl;
	_nhPrivate.param("num_ring", _num_ring, 32);
	std::cout << "_num_ring = " << _num_ring << std::endl;
	_nhPrivate.param("points_per_ring", _points_per_ring, 1092);
	std::cout << "_points_per_ring = " << _points_per_ring << std::endl;
	/*subscriber*/
	_sub_imu = _nh.subscribe("/imu/data", 1, &SaveIMUCameraVelodyne::callbackIMU, this);
	_sub_image = _nh.subscribe("/image_raw", 1, &SaveIMUCameraVelodyne::callbackImage, this);
	_sub_pc = _nh.subscribe("/velodyne_points", 1, &SaveIMUCameraVelodyne::callbackPC, this);
	_sub_odom = _nh.subscribe("/odom", 1, &SaveIMUCameraVelodyne::callbackOdom, this);
	/*initialize*/
	_csvfile.open(_save_csv_path, std::ios::app);
	if(!_csvfile){
		std::cout << "Cannot open " << _save_csv_path << std::endl;
		exit(1);
	}
	_rings.resize(_num_ring);
	for(size_t i=0 ; i<_rings.size() ; ++i){
		pcl::PointCloud<pcl::PointXYZI>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZI>);
		_rings[i] = tmp;
	}
}

void SaveIMUCameraVelodyne::callbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	_imu = *msg;
	if(!_got_first_imu)	_got_first_imu = true;
}

void SaveIMUCameraVelodyne::callbackImage(const sensor_msgs::ImageConstPtr& msg)
{
	try{
		_imgptr_color = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		if(!_got_first_camera)	_got_first_camera = true;
	}
	catch(cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	/* std::cout << "msg->encoding = " << msg->encoding << std::endl; */
}

void SaveIMUCameraVelodyne::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	if(!_got_first_imu || !_got_first_camera || !_got_first_odom)	return;
	if(_counter < _save_data_limit){
		if(hasOdomDiff(_odom_now, _odom_last)){
			for(size_t i=0 ; i<_rings.size() ; ++i)	_rings[i]->points.clear();
			pcToRings(*msg);
			ringsToImage();
			record();
		}
	}
	else if(_csvfile.is_open()){
		_csvfile.close();
		std::cout << _save_csv_path << " was closed" << std::endl;
	}
}

void SaveIMUCameraVelodyne::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	_odom_now = *msg;
	if(!_got_first_odom){
		_odom_last = *msg;
		_got_first_odom = true;
	}
}

void SaveIMUCameraVelodyne::pcToRings(const sensor_msgs::PointCloud2& pc_msg)
{
	sensor_msgs::PointCloud2ConstIterator<int> iter_ring(pc_msg,"ring");
	sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc_msg,"x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc_msg,"y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc_msg,"z");
	sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(pc_msg,"intensity");

	for( ; iter_ring!=iter_ring.end() ; ++iter_ring, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity){
		pcl::PointXYZI tmp;
		tmp.x = *iter_x;
		tmp.y = *iter_y;
		tmp.z = *iter_z;
		tmp.intensity = *iter_intensity;
		_rings[*iter_ring]->points.push_back(tmp);
	}
}

void SaveIMUCameraVelodyne::ringsToImage(void)
{
	_img_depth = cv::Mat::zeros(_num_ring, _points_per_ring, CV_64FC1);

	double angle_resolution = 2*M_PI/(double)_points_per_ring;
	for(size_t i=0 ; i<_rings.size() ; ++i){
		int row = _rings.size() - i - 1;
		for(size_t j=0 ; j<_rings[i]->points.size() ; ++j){
			double angle = atan2(_rings[i]->points[j].y, _rings[i]->points[j].x);
			int col = (int)((angle + M_PI)/angle_resolution);
			_img_depth.at<double>(row, col) = sqrt(_rings[i]->points[j].x*_rings[i]->points[j].x + _rings[i]->points[j].y*_rings[i]->points[j].y);
		}
	}
}

bool SaveIMUCameraVelodyne::hasOdomDiff(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2)
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

void SaveIMUCameraVelodyne::record(void)
{
	/*path*/
	std::string save_imgcolor_path = _save_dir_path + "/" + _save_imgcolor_name + std::to_string(_counter) + ".jpg";
	std::string save_imgdepth_path = _save_dir_path + "/" + _save_imgdepth_name + std::to_string(_counter) + ".jpg";
	/*check*/
	std::ifstream ifs(save_imgcolor_path);
	if(ifs.is_open()){
		std::cout << save_imgcolor_path << " already exists" << std::endl;
		exit(1);
	}
	/*record imu*/
	_csvfile 
		<< _imu.linear_acceleration.x << "," 
		<< _imu.linear_acceleration.y << "," 
		<< _imu.linear_acceleration.z << ","
		<< save_imgcolor_path << ","
		<< save_imgdepth_path << std::endl;
	/*save color image*/
	cv::imwrite(save_imgcolor_path, _imgptr_color->image);
	/*save depth image*/
	cv::imwrite(save_imgdepth_path, _img_depth);
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
	ros::init(argc, argv, "save_imu_camera_velodyne");

	SaveIMUCameraVelodyne save_imu_camera_velodyne;

	ros::spin();
}
