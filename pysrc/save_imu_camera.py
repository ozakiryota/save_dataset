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

class SaveImuCameraDepth:
    def __init__(self):
        ## subscriber
        self.sub_imu = rospy.Subscriber("/imu/data", Imu, self.callbackIMU)
        self.sub_imgcolor = rospy.Subscriber("/image_raw", ImageMsg, self.callbackImageColor)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callbackOdom)
        ## msg
        self.imu = Imu()
        self.odom_now = Odometry()
        self.odom_last = Odometry()
        ## cv_bridge
        self.bridge = CvBridge()
        ## img
        self.imgcolor_cv = np.empty(0)
        ## flag
        self.got_first_imu = False
        self.got_first_imgcolor = False
        self.got_first_odom = False
        ## path
        current_dir = os.path.dirname(os.path.abspath(__file__))
        default_rootpath = os.path.join(current_dir, "../dataset/imu_camera/tmp")
        self.rootpath = rospy.get_param("rootpath", default_rootpath)
        self.filename = rospy.get_param("filename", "data_")
        ## counter
        self.counter = 0
        ## threshold
        self.th_diff_position_m = rospy.get_param("th_diff_position_m", 3.0)
        self.th_diff_angle_deg = rospy.get_param("th_diff_angle_deg", 5.0)
        ## print parameter
        print("self.rootpath = ", self.rootpath)
        print("self.filename = ", self.filename)
        print("self.th_diff_position_m = ", self.th_diff_position_m)
        print("self.th_diff_angle_deg = ", self.th_diff_angle_deg)

    def callbackIMU(self, msg):
        self.imu = msg
        if not self.got_first_imu:
            self.got_first_imu = True

    def callbackImageColor(self, msg):
        ## rgb8
        # print(msg.encoding)
        try:
            self.imgcolor_cv = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            if not self.got_first_imgcolor:
                self.got_first_imgcolor = True
            if self.isReadyToSave():
                self.saveData()
        except CvBridgeError as e:
            print(e)

    def callbackOdom(self, msg):
        self.odom_now = msg
        if not self.got_first_odom:
            self.odom_last = msg
            self.got_first_odom = True

    def isReadyToSave(self):
        if not self.got_first_imu:
            return False
        if not self.got_first_odom:
            return False
        if not self.got_first_imgcolor:
            return False

        if self.hasOdomDiff(self.odom_now, self.odom_last):
            return True
        else:
            return False

    def hasOdomDiff(self, o1, o2):
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
        ## judge
        if diff_position_m > self.th_diff_position_m:
            return True
        if diff_angle_deg > self.th_diff_angle_deg:
            return True
        return False

    def saveData(self):
        print("SAVED!")
        print("self.counter = ", self.counter)
        print("self.imgcolor_cv.shape = ", self.imgcolor_cv.shape)
        ## color
        save_imgcolor_name = self.filename + "color_" + str(self.counter) + ".jpg"
        self.saveImageColor(self.imgcolor_cv, save_imgcolor_name)
        ## IMU with images
        self.saveCSV(save_imgcolor_name)
        ## count
        self.counter += 1
        self.odom_last = self.odom_now

    def saveImageColor(self, img_cv, save_name):
        save_path = os.path.join(self.rootpath, save_name)
        if os.path.isfile(save_path):
            print(save_path , " already exists")
            os._exit(1)
        img_pil = Image.fromarray(img_cv)
        img_pil.save(save_path)

    def saveCSV(self, save_imgcolor_name):
        save_csv_path = os.path.join(self.rootpath, "imu_color.csv")
        with open(save_csv_path, "a") as csvfile:
            acc = self.QuatToAcc([
                self.imu.orientation.x,
                self.imu.orientation.y,
                self.imu.orientation.z,
                self.imu.orientation.w
            ])
            writer = csv.writer(csvfile)
            writer.writerow([
                # self.imu.linear_acceleration.x,
                # self.imu.linear_acceleration.y,
                # self.imu.linear_acceleration.z,
                acc[0],
                acc[1],
                acc[2],
                save_imgcolor_name
            ])

    # def cvToPIL(self, img_cv):
    #     img_cv_copy = img_cv.copy()
    #     ## mono
    #     if img_cv_copy.ndim == 2:
    #         pass
    #     ## color
    #     elif img_cv_copy.shape[2] == 3:
    #         img_cv_copy = img_cv_copy[:, :, ::-1]
    #     ## color + alpha
    #     elif img_cv_copy.shape[2] == 4:
    #         img_cv_copy = img_cv_copy[:, :, [2, 1, 0, 3]]
    #     img_pil = Image.fromarray(img_cv_copy)
    #     return img_pil

    def QuatToAcc(self, q):
        g = 1.0
        rpy = tf.transformations.euler_from_quaternion(q)
        acc = (
            -g*math.sin(rpy[1]),
            g*math.sin(rpy[0])*math.cos(rpy[1]),
            g*math.cos(rpy[0])*math.cos(rpy[1])
        )
        print("self.imu.linear_acceleration: ",
            self.imu.linear_acceleration.x, ", ",
            self.imu.linear_acceleration.y, ", ",
            self.imu.linear_acceleration.z
        )
        print("acc = ", acc)
        return acc

    def getQuatAngle(self, q):
        w = q[3].copy()
        w = max(-1.0, min(1.0, w))  #clip
        angle = 2*math.acos(w)
        # angle = math.atan2(math.sin(angle), math.cos(angle))
        # angle = abs(angle)
        return angle


def main():
    rospy.init_node('save_imu_camera', anonymous=True)
    
    save_imu_camera = SaveImuCameraDepth()

    rospy.spin()

if __name__ == '__main__':
    main()
