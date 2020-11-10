#!/usr/bin/env python3
import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist, Vector3
import math
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from candy_classifier import ImageClassifier

class FiniteStateController(object):
	"This class encompasses multiple behaviors for the simulated neato"

	def __init__(self, image_topic):
		rospy.init_node('finite_state')
		self.state = "teleop"
		self.vel_msg = Twist()

		# Image capturing parameters
		self.cv_image = None # the latest image from the camera
		self.bridge = CvBridge() # used to convert ROS messages to OpenCV
		cv2.namedWindow('video_window')
		cv2.namedWindow("binary_image", cv2.WINDOW_AUTOSIZE)

		# Pubs and subscribers
		self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
		# rospy.Subscriber('scan', LaserScan, self.process_scan)
		# rospy.Subscriber('odom', Odometry, self.process_odom)
		rospy.Subscriber(image_topic, Image, self.process_image)

		# Initializing user input for teleop
		self.settings = termios.tcgetattr(sys.stdin)
		self.key = None

		# Keypoint classification parameters
		self.classifier = ImageClassifier()

		# Parameters for the robot to react to the candy it finds
		self.likes = ['Smarties','Haribo','Nerds', 'Milk Duds']
		self.dislikes = ['Starburst', 'Swedish_Fish', 'Twizzlers']
		self.allergies = ['Reeses', 'Snickers']
		self.favorite = ['Skittles']
		self.candy_name = ''

	# def process_scan(self):
	# 	pass


	def process_image(self, msg):
		""" Process image messages from ROS and stash them in an attribute
			called cv_image for subsequent processing """
		self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
		self.binary_image = cv2.inRange(self.cv_image, (0, 0, 100), (20,20, 255))


	def run(self):
		""" The run loop repeatedly executes the current state function.  Each state function will return a function
		corresponding to the next state to run. """
		rospy.sleep(1)
		while not rospy.is_shutdown():
			if self.state == "teleop":
				self.teleop()
			if self.state == "square":
				self.square()
			if self.state == "candy":
				self.candy_classify()
			if self.state == "react":
				self.react_candy()

	def getKey(self):
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		return key

	def key_map(self,key, vel_msg):
		# Checking user input
		if key != '\x03':
			key = self.getKey()
			if key == 'w':
				vel_msg.angular.z = 0
				vel_msg.linear.x = 1
			elif key == 'a':
				vel_msg.linear.x = 0
				vel_msg.angular.z = 1
			elif key == 's':
				vel_msg.angular.z = 0
				vel_msg.linear.x = -1
			elif key == 'd':
				vel_msg.linear.x = 0
				vel_msg.angular.z = -1
			elif key == ' ':
				vel_msg.angular.z = 0
				vel_msg.linear.x = 0
			elif key == '1':
				vel_msg.angular.z = 0
				vel_msg.linear.x = 0
				self.state = "candy"
		return key

	def teleop(self):
		# Checking user input
		self.key = self.key_map(self.key, self.vel_msg)
		# send instructions to the neato
		self.vel_pub.publish(self.vel_msg)
		rospy.Rate(10).sleep


	def square(self):
		for i in range(4):
			self.vel_pub.publish(Twist(linear=Vector3(x=1)))
			rospy.sleep(2)
			self.vel_pub.publish(Twist(angular=Vector3(z=-math.pi/4)))
			rospy.sleep(2)
		#make sure it's not moving when it goes back to teleop
		self.vel_msg.angular.z = 0
		self.vel_msg.linear.x = 0
		self.vel_pub.publish(self.vel_msg)
		self.state = "teleop"


	def candy_classify(self):
	# Display the image in the different windows
		if not self.cv_image is None:
		   print("image shape: ", self.cv_image.shape)
		   # cv2.imshow('binary_image', self.binary_image)
		   cv2.imshow('video_window', self.cv_image)
		   cv2.waitKey(5)

		   # Classify the image
		   self.classifier.query = self.cv_image

		   # Uses a classification system to figure out what's in front of it
		   self.candy_name = self.classifier.run()
		self.state = "react"


	def react_candy(self):
		if self.candy_name in self.favorite:
			self.vel_pub.publish(Twist(angular=Vector3(z=-math.pi)))
			rospy.sleep(4)
			self.state = "teleop"
		if self.candy_name in self.likes:
			self.vel_pub.publish(Twist(linear=Vector3(x=1)))
			rospy.sleep(2)
		if self.candy_name in self.dislikes:
			self.vel_pub.publish(Twist(angular=Vector3(z=-math.pi/2)))
			rospy.sleep(0.5)
			self.vel_pub.publish(Twist(angular=Vector3(z=math.pi/2)))
			rospy.sleep(0.5)
		if self.candy_name in self.allergies:
			self.vel_pub.publish(Twist(linear=Vector3(x=-1)))
			rospy.sleep(1)

if __name__ == '__main__':
	node = FiniteStateController("/camera/image_raw")
	node.run()
