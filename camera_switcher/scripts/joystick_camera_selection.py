#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, UInt16
from joy_feedback_ros.msg import Rumble

class CameraSwitcherJoystick:

	def __init__(self):
		"""
		Creates a new node called camera_switcher_joystick.
		The node subscribes to '/joy' to receive input from a joystick.
		The node publishes which robot is selected and vibration to the joystick.
		"""
		#Initialize the node and create subscribers and publishers.
		self.sub_joystick = rospy.Subscriber('/joy', Joy, self.joystick_selection)
		self.output_camera_selection = rospy.Publisher('/select_camera', Float32, queue_size=10)
		self.pub_rumble = rospy.Publisher('/rumble', Rumble, queue_size=10)
		self.pub_vibration = rospy.Publisher('/play', UInt16, queue_size=10)
		
		#Initialize messages to be published.
		self.output_message = Float32()
		self.output_message.data = 0.0
		self.vibration_message = UInt16()
		self.vibration_message.data = 0
		
		#Initialize rumble message to be published.
		self.rumble_message = Rumble() 
		self.rumble_message.strong_magnitude = 14000
		self.rumble_message.weak_magnitude = 14000
		

	def joystick_selection(self, msg):
		"""
		Publishes which robot is selected and vibration to the joystick.
		:param msg: joystick buttons
		"""
		self.pub_rumble.publish(self.rumble_message)
				
		self.joystick_data = msg.buttons

		#Check if the LB is pressed on the Xbox 360 joystick, and that 'output_message' is not 1.0. 
		#This ensures vibration is not published if camera is not switched.
		if self.joystick_data[4] == 1 and not self.output_message.data == 1.0:
			#If so, set the output message to indicate that the first robot is selected and publish vibration to the joystick.
			self.output_message.data = 1.0
			self.pub_vibration.publish(self.vibration_message)

		#Check if the RB is pressed on the Xbox 360 joystick, and that 'output_message' is not 1.0. 
		#This ensures vibration is not published if camera is not switched.
		elif (self.joystick_data[5] == 1) and not self.output_message.data == 2.0: 
			#If so, set the output message to indicate that the first robot is selected and publish vibration to the joystick.
			self.output_message.data = 2.0
			self.pub_vibration.publish(self.vibration_message)
		
		#Publish the output message to select the robot.
		self.output_camera_selection.publish(self.output_message)
		
# Initialize the node and start the main loop.
rospy.init_node('select_camera')
camera_selector = CameraSwitcherJoystick()
rospy.spin()
