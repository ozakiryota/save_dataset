/*ros*/
#include <ros/ros.h>
/*msg*/
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

class ImuToPosestamped{
	private:
		/*node hangle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscriber*/
		ros::Subscriber sub_imu;
		/*publisher*/
		ros::Publisher pub_pose;
		/*objects*/
		geometry_msgs::PoseStamped pose;
		/*param*/
		std::string frame_id;
	public:
		ImuToPosestamped();
		void InitializePose(geometry_msgs::PoseStamped& pose);
		void CallbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void Publication(void);
};

ImuToPosestamped::ImuToPosestamped()
	:nhPrivate("~")
{
	/*param*/
	nhPrivate.param("frame_id", frame_id, std::string("/odom"));
	std::cout << "frame_id = " << frame_id << std::endl;

	/*subscriber*/
	sub_imu = nh.subscribe("/imu/data", 1, &ImuToPosestamped::CallbackIMU, this);
	/*publisher*/
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose", 1);
	/*initialize*/
	InitializePose(pose);
}

void ImuToPosestamped::InitializePose(geometry_msgs::PoseStamped& pose)
{
	pose.pose.position.x = 0.0;
	pose.pose.position.y = 0.0;
	pose.pose.position.z = 0.0;
	pose.pose.orientation.x = 0.0;
	pose.pose.orientation.y = 0.0;
	pose.pose.orientation.z = 0.0;
	pose.pose.orientation.w = 1.0;
}

void ImuToPosestamped::CallbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	pose.header.stamp = msg->header.stamp;
	pose.pose.orientation = msg->orientation;

	Publication();
}

void ImuToPosestamped::Publication(void)
{
	/*publish*/
	pose.header.frame_id = frame_id;
	pub_pose.publish(pose);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_imu_to_posestamped_for_visualization");

	ImuToPosestamped test_imu_to_posestamped_for_visualization;

	ros::spin();
}
