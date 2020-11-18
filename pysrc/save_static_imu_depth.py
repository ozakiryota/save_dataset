#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image as ImageMsg
from nav_msgs.msg import Odometry
import tf

from cv_bridge import CvBridge, CvBridgeError

from PIL import Image

import numpy as np
import math
import os
import csv

class SaveStaticImuDepth:
    def __init__(self):
        ## subscriber
        self.sub_imu = rospy.Subscriber("/imu/data", Imu, self.callbackIMU)
        self.sub_imgdepth = rospy.Subscriber("/depth_image", ImageMsg, self.callbackImageDepth)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callbackOdom)
        ## msg
        self.imu = Imu()
        self.odom_now = Odometry()
        self.odom_last = Odometry()
        ## cv_bridge
        self.bridge = CvBridge()
        ## img
        self.imgdepth_cv = np.empty(0)
        ## flag
        self.got_first_imu = False
        self.got_first_imgdepth = False
        self.got_first_odom = False
        self.is_still = False
        ## path
        current_dir_path = os.path.dirname(os.path.abspath(__file__))
        self.save_dir_path = rospy.get_param("save_dir_path", os.path.join(current_dir_path, "../save/tmp"))
        self.save_csv_path = rospy.get_param("save_csv_path", os.path.join(self.save_dir_path, "imu_lidar.csv"))
        self.save_data_name = rospy.get_param("save_data_name", "data_")
        ## counter
        self.record_counter = 0
        self.still_counter = 0
        ## threshold
        self.th_diff_position_m = rospy.get_param("th_diff_position_m", 1.0)
        self.th_diff_angle_deg = rospy.get_param("th_diff_angle_deg", 5.0)
        self.th_still_position_m = rospy.get_param("th_still_position_m", 0.001)
        self.th_still_angle_deg = rospy.get_param("th_still_angle_deg", 0.1)
        self.th_still_counter = rospy.get_param("th_still_counter", 10)
        ## print parameter
        print("self.save_dir_path = ", self.save_dir_path)
        print("self.save_csv_path = ", self.save_csv_path)
        print("self.save_data_name = ", self.save_data_name)
        print("self.th_diff_position_m = ", self.th_diff_position_m)
        print("self.th_diff_angle_deg = ", self.th_diff_angle_deg)
        print("self.th_still_position_m = ", self.th_still_position_m)
        print("self.th_still_angle_deg = ", self.th_still_angle_deg)
        print("self.th_still_counter = ", self.th_still_counter)

    def callbackIMU(self, msg):
        self.imu = msg
        if not self.got_first_imu:
            self.got_first_imu = True

    def callbackImageDepth(self, msg):
        ## CV64FC1
        # print(msg.encoding)
        try:
            self.imgdepth_cv = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            if not self.got_first_imgdepth:
                self.got_first_imgdepth = True
            if self.isReadyToSave():
                self.saveData()
        except CvBridgeError as e:
            print(e)

    def callbackOdom(self, msg):
        if self.got_first_odom:
            self.is_still = self.isStill(msg, self.odom_now)
        self.odom_now = msg
        if not self.got_first_odom:
            self.odom_last = msg
            self.got_first_odom = True

    def isReadyToSave(self):
        ## first message
        if not self.got_first_imu:
            return False
        if not self.got_first_odom:
            return False
        if not self.got_first_imgdepth:
            return False
        ## still
        if not self.is_still:
            return False
        ## odom diff
        if not self.hasOdomDiff(self.odom_now, self.odom_last):
            return False
        return True

    def isStill(self, o1, o2):
        ## get diff
	diff_position_m, diff_angle_deg = self.getOdomDiff(o1, o2)
        ## count
        if (diff_position_m < self.th_still_position_m) and (diff_angle_deg < self.th_still_angle_deg):
            self.still_counter += 1
        else:
            self.still_counter = 0
        ## judge
        if self.still_counter > self.th_still_counter:
            return True
        else:
            return False

    def hasOdomDiff(self, o1, o2):
        ## get diff
	diff_position_m, diff_angle_deg = self.getOdomDiff(o1, o2)
        ## judge
        if diff_position_m > self.th_diff_position_m:
            return True
        if diff_angle_deg > self.th_diff_angle_deg:
            return True
        return False

    def getOdomDiff(self, o1, o2):
        ## position
	dx = o2.pose.pose.position.x - o1.pose.pose.position.x
	dy = o2.pose.pose.position.y - o1.pose.pose.position.y
	dz = o2.pose.pose.position.z - o1.pose.pose.position.z
	diff_position_m = math.sqrt(dx*dx + dy*dy + dz*dz)
        ## orientation
        q1 = [
            o1.pose.pose.orientation.x,
            o1.pose.pose.orientation.y,
            o1.pose.pose.orientation.z,
            o1.pose.pose.orientation.w
        ]
        q2_inv = [
            -o2.pose.pose.orientation.x,
            -o2.pose.pose.orientation.y,
            -o2.pose.pose.orientation.z,
            o2.pose.pose.orientation.w
        ]
	q_rel_rot = tf.transformations.quaternion_multiply(q2_inv, q1)
        diff_angle_deg = self.getQuatAngle(q_rel_rot)/math.pi*180.0
        ## return
        return diff_position_m, diff_angle_deg

    def saveData(self):
        ## depth
        save_imgdepth_name = self.save_data_name + "depth_" + str(self.record_counter) + ".npy"
        self.saveImageDepth(self.imgdepth_cv, save_imgdepth_name)
        ## csv
        self.saveCSV(save_imgdepth_name)
        ## print
        print("SAVED!")
        print("self.record_counter = ", self.record_counter)
        print("self.still_counter = ", self.still_counter)
        print("self.imgdepth_cv.shape = ", self.imgdepth_cv.shape)
        ## reset
        self.record_counter += 1
        self.odom_last = self.odom_now

    def saveImageDepth(self, img_cv, save_name):
        save_path = os.path.join(self.save_dir_path, save_name)
        if os.path.isfile(save_path):
            print(save_path , " already exists")
            os._exit(1)
        np.save(save_path, img_cv)
        print("depth: ", save_path)

    def saveCSV(self, save_imgdepth_name):
        with open(self.save_csv_path, "a") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                self.imu.linear_acceleration.x,
                self.imu.linear_acceleration.y,
                self.imu.linear_acceleration.z,
                save_imgdepth_name
            ])

    def getQuatAngle(self, q):
        w = q[3].copy()
        w = max(-1.0, min(1.0, w))  #clip
        angle = 2*math.acos(w)
        # angle = math.atan2(math.sin(angle), math.cos(angle))
        # angle = abs(angle)
        return angle


def main():
    rospy.init_node('save_static_imu_depth', anonymous=True)
    
    save_static_imu_depth = SaveStaticImuDepth()

    rospy.spin()

if __name__ == '__main__':
    main()
