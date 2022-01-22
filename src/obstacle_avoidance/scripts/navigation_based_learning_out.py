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
from std_msgs.msg import Float32, Int8
from std_srvs.srv import SetBool, SetBoolResponse, Trigger
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

		self.image_left_sub = rospy.Subscriber("/camera_left/image_raw", Image, self.callback_left_camera)
		self.image_right_sub = rospy.Subscriber("/camera_right/image_raw", Image, self.callback_right_camera)

		self.vel_sub = rospy.Subscriber("/nav_vel", Twist, self.callback_vel)
		self.joy_sub = rospy.Subscriber("/joy_vel", Twist, self.callback_joy)
		self.action_pub = rospy.Publisher("action", Int8, queue_size=1)
		self.nav_pub = rospy.Publisher('/icart_mini/cmd_vel', Twist, queue_size=10)
		self.srv = rospy.Service('/learn_out', SetBool, self.callback_dl)
		self.action = 0.0
		self.joy = Twist() 
		self.joy_linear=0.0
		self.joy_angular=0.0
		self.vel = Twist()
		self.cv_left_image = np.zeros((480,640,3), np.uint8)
		self.cv_right_image = np.zeros((480,640,3), np.uint8)
		self.cv_image = np.zeros((480,640,3), np.uint8)
		self.episode = 0
		self.cv_image = np.zeros((480,640,3), np.uint8)
		self.learning = True
		self.switch_mode = True
		self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
		self.path = '/home/orne_box/data/result/'
		self.start_time_s = rospy.get_time()
		os.makedirs(self.path + self.start_time)
		self.save_model = rospy.Service('/model_save', Trigger, self.model_save)

		with open(self.path + self.start_time + '/' +  'reward.csv', 'w') as f:
			writer = csv.writer(f, lineterminator='\n')
			writer.writerow(['step', 'mode', 'loss', 'angle_error(rad)'])

	def callback(self, data):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

	def callback_left_camera(self, data):
		try:
			self.cv_left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

	def callback_right_camera(self, data):
		try:
			self.cv_right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)					

	def callback_joy(self, data):
		self.joy = data
		self.joy_linear = self.joy.linear.x
		self.joy_angular = self.joy.angular.z

	def callback_vel(self, data):
		self.vel = data
		self.action = self.vel.angular.z

	def callback_dl(self, data):
		resp = SetBoolResponse()
		self.switch_mode = data.data
		resp.message = "Learn_out: " + str(self.switch_mode)
		resp.success = True
		return resp

	def model_save(self,data):
		rospy.wait_for_service('/model_save')
		service = rospy.ServiceProxy('/model_save', Trigger)
		self.dl.save()

	def loop(self):
		if self.cv_image.size != 640 * 480 * 3:
			return
		if self.cv_left_image.size != 640 * 480 * 3:
			return
		if self.cv_right_image.size != 640 * 480 * 3:
			return	
		
		img = resize(self.cv_image, (48, 64), mode='constant')
		r, g, b = cv2.split(img)
		imgobj = np.asanyarray([r,g,b])

		img_left = resize(self.cv_left_image, (48, 64), mode='constant')
		r, g, b = cv2.split(img_left)
		imgobj_left = np.asanyarray([r,g,b])

		img_right = resize(self.cv_right_image, (48, 64), mode='constant')
		r, g, b = cv2.split(img_right)
		imgobj_right = np.asanyarray([r,g,b])

		ros_time = str(rospy.Time.now())

		self.dl.load()

		if self.joy_linear > 0.0:
			self.learning= True
		else:
			self.learning= False

		if self.learning:
			target_action = self.joy_angular
			action, loss = self.dl.act_and_trains(imgobj, target_action)
			angle_error = abs(action - target_action)
			print(" episode: " + str(self.episode) + ", loss: " + str(loss) + ", angle_error: " + str(angle_error))
			self.episode += 1
			line = [str(self.episode), "training", str(loss), str(angle_error)]
			with open(self.path + self.start_time + '/' + 'reward.csv', 'a') as f:
				writer = csv.writer(f, lineterminator='\n')
				writer.writerow(line)
			self.vel.linear.x = 0.4
			self.vel.angular.z = target_action
			self.nav_pub.publish(self.vel)

		else:
			if self.switch_mode:
				target_action = self.dl.act(imgobj)
				print("TEST MODE: " + " angular:" + str(target_action))
				angle_error = abs(self.action - target_action)
				line = [str(self.episode), "test", "0", str(angle_error)]
				with open(self.path + self.start_time + '/' + 'reward.csv', 'a') as f:
					writer = csv.writer(f, lineterminator='\n')
					writer.writerow(line)
				self.vel.linear.x = 0.4
				self.vel.angular.z = target_action
				self.nav_pub.publish(self.vel)
			else:
				self.vel.linear.x = 0.4
				self.vel.angular.z = self.action
				self.nav_pub.publish(self.vel)

		temp = copy.deepcopy(img)
		cv2.imshow("Resized Image", temp)
		temp = copy.deepcopy(img_left)
		cv2.imshow("Resized Left Image", temp)
		temp = copy.deepcopy(img_right)
		cv2.imshow("Resized Right Image", temp)
		cv2.waitKey(1)

if __name__ == '__main__':
	rg = cource_following_learning_node()
	DURATION = 0.2
	r = rospy.Rate(1 / DURATION)
	while not rospy.is_shutdown():
		rg.loop()
		r.sleep()
