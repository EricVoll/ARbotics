#!/usr/bin/env python3
import rospy 
from std_msgs.msg import String
import json

def shutdown_hook():
  print ("Closes ARServerStatePublisher Node")

class ARServerStatePublisher():
	"""ARServerStatePublisher publishes initalizes ar_state_publisher ros node.
	Publishes the state of the server serialized as a string. Same format as available via REST-requests
	This is done so mutiple clients can start and stop components via REST requests and Unity gets always notified about changes via the rostopic.
	"""

	def __init__(self):

		rospy.init_node('ar_state_publisher', anonymous=False)
		self.pub = rospy.Publisher('ar_state', String, queue_size=10)
	
		rospy.on_shutdown(shutdown_hook)

	def publish(self, data):
		"""serializes the given data and publishes on ar_state topic.

		Parameters
		----------
		data : dict
				containing data key (data can be a nested dict or list)

		Raises
		------
		TypeError
				data is not a dict
		ValueError
				dict must contain <data> key entry"
		"""
		if not isinstance(data, dict):
			raise TypeError("data is not a dict")
		if not ( 'data' in data.keys() ):
			raise ValueError("dict must contain <data> key entry")
		
		#pass by string #serialization could also be done by protobuff
		json_mylist = json.dumps(data, separators=(',', ':'))	
		ros_msg = String(data=json_mylist)

		self.pub.publish( ros_msg )



if __name__ == '__main__':

	AsSSP = ARServerStatePublisher()
	dictionary = { 'data': {'key': [1, 2, 3], 'robot': "value"}}
	AsSSP.publish(data=dictionary)