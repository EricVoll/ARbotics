#!/usr/bin/env python3
import rospy 
from rospy_message_converter import message_converter
from std_msgs.msg import String
msg = String()
import std_msgs.msg
msg = std_msgs.msg.String()
import rosparam

#string seralization https://github.com/uos/rospy_message_converter

def shutdown_hook():
  print ("Close Server State Publisher")

class ARServerStatePublisher():

	def __init__(self):
		print("INIT")
		rospy.init_node('ar_state_publisher', anonymous=False)
		
		self.pub = rospy.Publisher('ar_state', String, queue_size=10)
	
		rospy.on_shutdown(shutdown_hook)

	def publish(self, data):




		print("not called anymore")
		
		if not isinstance(data, dict):
			print("ERROR2")
			raise TypeError("data is not a <dict>")
		if not ( 'data' in data.keys() ):
			print("ERROR1")
			raise ValueError("dict must contain key <data> entry")

		
		print("HERE WE G")
		print(data)
		data = {'data': "string"}
		msg = message_converter.convert_dictionary_to_ros_message('std_msgs/String', data)
		val = self.pub.publish( msg )
		print("MSG MSG:", msg.data)
		# except rospy.ROSInterruptException:
		# 	print("ERROR3")
		# 	print("ROS interrupt exeption during pub. data")	
	
		#self.rate.sleep()
	



if __name__ == '__main__':

	#parameter server is working easily


	# rosparam.set_param('global_example',"example")
	# rosparam.set_param('list_of_floats', "[1., 2., 3., 4.]") #use yaml
	# rosparam.set_param('gains', "{'p': 1, 'i': 2, 'd': 3}")
	# rospy.get_param('gains/p') #should return 1
	# param_name = rospy.search_param('global_example')
	# v = rospy.get_param(param_name)
		# while not rospy.is_shutdown():
		# #rospy.loginfo(hello_str)
		
		# dictionary = { 'data': {'key': [1, 2, 3], 'robot': "value"}}
		
		# print(message.data)

		
		# rate.sleep()


	# if rospy.has_param('gains/i'):
	# 	rospy.delete_param('gains/i')
	AsSSP = ARServerStatePublisher()
	dictionary = { 'data': {'key': [1, 2, 3], 'robot': "value"}}
	AsSSP.publish(data=dictionary)