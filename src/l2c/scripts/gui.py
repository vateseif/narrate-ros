#!/usr/bin/env python3
import rospy
from time import sleep
from std_msgs.msg import String
from std_msgs.msg import Empty
from random import sample

class GUI:
	def __init__(self) -> None:
		rospy.init_node('my_streamlit_ros_node')   
		self.plan_sub = rospy.Subscriber('/gui_plan', String, self.message_callback)
		self.optimization_sub = rospy.Subscriber('/gui_optimization', String, self.message_callback)
		self.user_message_pub = rospy.Publisher('/user_message', String, queue_size=1)
		self.optimization_trigger_pub = rospy.Publisher('/manual_optimization_trigger', Empty, queue_size=1)

		self.waiting_response = False

	def message_callback(self, message: String):
		print(f"\33[92m AI:{message.data} \033[0m \n")
		self.waiting_response = False

	def pub_optimization_trigger(self):
		self.optimization_trigger_pub.publish(Empty())

	def run(self):
		while not rospy.is_shutdown():
			user_msg = String()
			command = input("Q:")
			if command == "":
				self.pub_optimization_trigger()
			else:
				user_msg.data = command
			self.user_message_pub.publish(user_msg)
			self.waiting_response = True
			print("...")
			while self.waiting_response:
				sleep(0.2)


if __name__ == '__main__':
	gui = GUI()
	gui.run()