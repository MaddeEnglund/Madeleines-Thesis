  #!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class CameraSwitcherJoystick:

	def __init__(self):
		"""
		Creates a new node called camera_switcher_joystick.
		The node subscribes to '/joy' to receive input from a joystick.
		The node publishes which robot is selected.
		"""
		self.sub_joystick = rospy.Subscriber('/joy', Joy, self.joystick_selection)
		self.output_camera_selection = rospy.Publisher('/select_camera', Float32, queue_size=10)
		self.outputMessage = Float32()
		self.outputMessage.data = 0.0
		
	def joystick_selection(self, msg):
		"""
		Stores the button data from the joystick.
		:param msg: joystick buttons
		"""
		self.joystick_data = msg.buttons
		if self.joystick_data[4] == 1: #LB on XBOS 360 joystick
			self.outputMessage.data = 1.0
		elif (self.joystick_data[5] == 1): #RB on XBOS 360 joystick
			self.outputMessage.data = 2.0
		self.output_camera_selection.publish(self.outputMessage)
		

rospy.init_node('select_camera')
cameraSelector = CameraSwitcherJoystick()
rospy.spin()
