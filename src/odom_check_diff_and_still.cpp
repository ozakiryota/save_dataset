/*ros*/
#include <ros/ros.h>
#include <tf/tf.h>
/*msg*/
#include <nav_msgs/Odometry.h>

class OdomCheckDiffAndStill{
	private:
		/*node hangle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_odom;
		/*struct*/
		/*msg*/
		nav_msgs::Odometry _odom_now;
		nav_msgs::Odometry _odom_last;
		/*counter*/
		int _check_counter = 0;
		int _still_counter = 0;
		/*flag*/
		bool _got_first_odom = false;
		bool _is_still = false;
		/*param*/
		double _th_diff_position_m;
		double _th_diff_angle_deg;
		double _th_still_position_m;
		double _th_still_angle_deg;
		int _th_still_counter;
	public:
		OdomCheckDiffAndStill();
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		bool isStill(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2);
		bool hasOdomDiff(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2);
		void getOdomDiff(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2, double& diff_position_m, double& diff_angle_deg);
		void printState(void);
};

OdomCheckDiffAndStill::OdomCheckDiffAndStill()
	:_nhPrivate("~")
{
	/*param*/
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

	/*subscriber*/
	_sub_odom = _nh.subscribe("/odom", 1, &OdomCheckDiffAndStill::callbackOdom, this);
}

void OdomCheckDiffAndStill::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	if(_got_first_odom)	_is_still = isStill(*msg, _odom_now);
	_odom_now = *msg;
	if(!_got_first_odom){
		_odom_last = *msg;
		_got_first_odom = true;
		return;
	}
	if(_is_still && hasOdomDiff(_odom_now, _odom_last)){
		printState();
	}
}

bool OdomCheckDiffAndStill::isStill(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2)
{
	double diff_position_m, diff_angle_deg;
	getOdomDiff(odom1, odom2, diff_position_m, diff_angle_deg);
	if(diff_position_m < _th_still_position_m && diff_angle_deg < _th_still_angle_deg)	++_still_counter;
	else	_still_counter = 0;
	/*print*/
	// std::cout << "diff_position_m = " << diff_position_m << std::endl;
	// std::cout << "diff_angle_deg = " << diff_angle_deg << std::endl;
	/*judge*/
	if(_still_counter > _th_still_counter)	return true;
	else	return false;
}

bool OdomCheckDiffAndStill::hasOdomDiff(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2)
{
	double diff_position_m, diff_angle_deg;
	getOdomDiff(odom1, odom2, diff_position_m, diff_angle_deg);
	/*judge*/
	if(diff_position_m > _th_diff_position_m)	return true;
	if(diff_angle_deg > _th_diff_angle_deg)	return true;
	return false;
}

void OdomCheckDiffAndStill::getOdomDiff(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2, double& diff_position_m, double& diff_angle_deg)
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

void OdomCheckDiffAndStill::printState(void)
{
	/*print*/
	double r, p, y;
	tf::Quaternion q;
	quaternionMsgToTF(_odom_now.pose.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(r, p, y);
	std::cout << _check_counter << "ckeck point: " 
		<< _odom_now.pose.pose.position.x << ", "
		<< _odom_now.pose.pose.position.y << ", "
		<< _odom_now.pose.pose.position.z << ", "
		<< r/M_PI*180.0 << ", " 
		<< p/M_PI*180.0 << ", " 
		<< y/M_PI*180.0 << std::endl;
	/*count*/
	++_check_counter;
	/*reset*/
	_still_counter = 0;
	_odom_last = _odom_now;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_check_diff_and_still");

	OdomCheckDiffAndStill odom_check_diff_and_still;

	ros::spin();
}
