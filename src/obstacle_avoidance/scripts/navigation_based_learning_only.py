#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('obstacle_avoidance')
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from deep_learning import *
from skimage.transform import resize
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32, Int8, Bool
from std_srvs.srv import SetBool, SetBoolResponse
import csv
import os
import time
import copy

class cource_following_learning_node:
	def __init__(self):
		rospy.init_node('cource_following_learning_node', anonymous=True)
		self.action_num = rospy.get_param("/LiDAR_based_learning_node/action_num", 1)
		print("action_num: " + str(self.action_num))
		self.dl = deep_learning(n_action = self.action_num)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera_center/image_raw", Image, self.callback)
		self.vel_sub = rospy.Subscriber("/nav_vel", Twist, self.callback_vel)
		self.start_learning_sub = rospy.Subscriber("/start_learn", Bool,self.start_learning_callback)
		self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_pose)
		self.action_pub = rospy.Publisher("action", Int8, queue_size=1)
		self.nav_pub = rospy.Publisher('/icart_mini/cmd_vel', Twist, queue_size=10)
#		self.nav_pub = rospy.Publisher('/learning_vel', Twist, queue_size=10)
		self.srv = rospy.Service('/visual_run', SetBool, self.callback_dl)
		self.step = 0
		self.action = 0.0
		self.angle_error = 0.0
		self.cov = 0.0
        	self.cov_x = 0.0
		self.cov_y = 0.0
		self.pose_x = 0.0
		self.pose_y = 0.0
		self.vel = Twist()
		self.cv_image = np.zeros((480,640,3), np.uint8)
		self.visual_run = False
		self.visual_run_th = False
		self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
		self.path = '/home/orne_box/data/result/'
		self.start_time_s = rospy.get_time()
		os.makedirs(self.path + self.start_time)
		self.dl.load()

		with open(self.path + self.start_time + '/' +  'result.csv', 'w') as f:
			writer = csv.writer(f, lineterminator='\n')
			writer.writerow(['step', 'mode', 'covariance', 'angle_error(rad)', 'x', 'y'])

	def callback(self, data):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

	def callback_pose(self, data):
		pos = data.pose.pose.position
		self.pose_x = pos.x
		self.pose_y = pos.y
		self.cov = data.pose.covariance[0] + data.pose.covariance[7]
		self.cov_x = data.pose.covariance[0]
		self.cov_y = data.pose.covariance[7]

	def callback_vel(self, data):
		self.vel = data
		
	def start_learning_callback(self, data):
		self.start = data.data
		if self.start ==True:
			self.visual_run_th = True
		else:
			self.visual_run_th = False

	def callback_dl(self, data):
		resp = SetBoolResponse()
		self.visual_run = data.data
		resp.message = "VISUAL_RUN: " + str(self.visual_run)
		resp.success = True
		return resp

	def loop(self):
		if self.cv_image.size != 640 * 480 * 3:
			return
		if self.cov > 100:
		     self.visual_run = True
		else:
	             self.visual_run = False
		ros_time = str(rospy.Time.now())
		

		if self.visual_run:
			img = resize(self.cv_image, (48, 64), mode='constant')
			r, g, b = cv2.split(img)
			imgobj = np.asanyarray([r,g,b])

			self.action = self.dl.act(imgobj)
			self.angle_error = abs(self.vel.angular.z - self.action)

			print("Visual RUN: " + " angle_error:" + str(self.angle_error) + " covariance:" + str(self.cov), str(self.pose_x), str(self.pose_y))
			line = [str(self.step), "test", str(self.cov), str(self.angle_error), str(self.cov_x), str(self.cov_y)]
			with open(self.path + self.start_time + '/' + 'result.csv', 'a') as f:
				writer = csv.writer(f, lineterminator='\n')
				writer.writerow(line)
			self.vel.linear.x = 0.4
			self.vel.angular.z = self.action
			self.nav_pub.publish(self.vel)
			self.step += 1
			temp = copy.deepcopy(img)
			cv2.imshow("Resized Image", temp)
			cv2.waitKey(1)
		else:
			print("NAVIGATION: " + " angle_error:" + str(self.angle_error) + " covariance:" + str(self.cov),str(self.cov_x),str(self.cov_y))
			line = [str(self.step), "navigation", str(self.cov), str(self.angle_error), str(self.pose_x), str(self.pose_y)]
			with open(self.path + self.start_time + '/' + 'result.csv', 'a') as f:
				writer = csv.writer(f, lineterminator='\n')
				writer.writerow(line)
			self.nav_pub.publish(self.vel)


if __name__ == '__main__':
	rg = cource_following_learning_node()
	DURATION = 0.2
	r = rospy.Rate(1 / DURATION)
	while not rospy.is_shutdown():
		rg.loop()
		r.sleep()
