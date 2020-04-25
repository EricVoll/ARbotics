#!/usr/bin/env python3
import rospy 

#from rospy_message_converter import message_converter
#ROS 1 specific approach
#This can be used to map between costum ROS MSG and dict !
#string seralization https://github.com/uos/rospy_message_converter

from std_msgs.msg import String
import std_msgs.msg
import rosparam
import json


def shutdown_hook():
  print ("Close Server State Publisher")

class ARServerStatePublisher():

	def __init__(self):
		print("INIT")
		rospy.init_node('ar_state_publisher', anonymous=False)
		
		self.pub = rospy.Publisher('ar_state', String, queue_size=10)
	
		rospy.on_shutdown(shutdown_hook)

	def publish(self, data):

		if not isinstance(data, dict):
			raise TypeError("data is not a <dict>")
		if not ( 'data' in data.keys() ):
			raise ValueError("dict must contain key <data> entry")
		
		#pass by string #serialization could also be done by protobuff
		json_mylist = json.dumps(data, separators=(',', ':'))	
		ros_msg = String(data=json_mylist)

		self.pub.publish( ros_msg )



if __name__ == '__main__':

	#parameter server examples
	# rosparam.set_param('global_example',"example")
	# rosparam.set_param('list_of_floats', "[1., 2., 3., 4.]") #use yaml
	# rosparam.set_param('gains', "{'p': 1, 'i': 2, 'd': 3}")
	# rospy.get_param('gains/p') #should return 1
	# param_name = rospy.search_param('global_example')
	# v = rospy.get_param(param_name)
	# if rospy.has_param('gains/i'):
	# 	rospy.delete_param('gains/i')


	AsSSP = ARServerStatePublisher()
	dictionary = { 'data': {'key': [1, 2, 3], 'robot': "value"}}
	AsSSP.publish(data=dictionary)