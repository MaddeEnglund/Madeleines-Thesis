import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

class CameraSwitcher:

	def __init__(self):
		"""
		Creates a new node called camera_switcher.
		The node subscribes to the robots' cameras, and to the topic called 'select_camera'.
		The node publishes the selected robot's camera feed.
		"""              
		#Subscribe to the camera topics of the robots
		self.sub_camera1 = rospy.Subscriber('/h1/realsense/color/image_raw/compressed', CompressedImage, self.camera1_sub_)
		self.sub_camera2 = rospy.Subscriber('/h2/realsense/color/image_raw/compressed',CompressedImage, self.camera2_sub_)
		
		#Subscribe to the topic that selects which robot's camera feed to output
		self.sub_select_feed = rospy.Subscriber('/select_camera',Float32, self.select_camera_sub_)
		self.selected_camera = 1.0 # Add this line to define the selected_camera attribute	

		#Publish the selected robot's camera feed
		self.img_pub = rospy.Publisher('/output_camera', Image, queue_size=10)
		
		#Create a timer that will call the timer_callback() function every 0.1 seconds
		self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
		
		#Define an instance of CvBridge, which is a ROS package that provides a bridge between OpenCV and ROS image formats
		self.bridge = CvBridge()

		self.camera1_data = None
		self.camera2_data = None
        
	def camera1_sub_(self, msg):
		"""
		Stores the camera data for camera1.
		:param msg: camera data
		"""
		self.camera1_data = msg.data

		#If the selected camera is camera1, convert the camera data to an image message and publish it
		if self.selected_camera == 1.0:
			image_np = cv2.imdecode(np.frombuffer(self.camera1_data, np.uint8), cv2.IMREAD_COLOR)
			image_msg = self.bridge.cv2_to_imgmsg(image_np, encoding="bgr8")
			self.img_pub.publish(image_msg)
    
	def camera2_sub_(self, msg):
		"""
		Stores the camera data for camera2.
		:param msg: camera data
		"""
		self.camera2_data = msg.data

		#If the selected camera is camera2, convert the camera data to an image message and publish it
		if self.selected_camera == 2.0:
			image_np = cv2.imdecode(np.frombuffer(self.camera2_data, np.uint8), cv2.IMREAD_COLOR)
			image_msg = self.bridge.cv2_to_imgmsg(image_np, encoding="bgr8")
			self.img_pub.publish(image_msg)

	def select_camera_sub_(self, msg):
		"""
		Selects which camera to output.
		:param msg: robot selection from '/select_camera'
		"""
		self.selected_camera = msg.data

	def timer_callback(self, event):
		"""
		Callback function called every 0.1 seconds.
		"""
		pass  #This function does nothing, but you can put any periodic tasks here.

rospy.init_node('cameraSelection')
cameraSelector = CameraSwitcher()
rospy.spin()
