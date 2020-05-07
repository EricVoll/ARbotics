#!/usr/bin/env python3
import rospy 
from std_msgs.msg import String
import std_msgs.msg
import rosparam
import json

from file_server.srv import *

def call_service(name):
	print("start to wait for service")
	rospy.wait_for_service('/file_server/get_file')
	print("service available")
	try:
			file = rospy.ServiceProxy('/file_server/get_file', GetBinaryFile)
			ans = file(name)
			return ans.value
	except rospy.ServiceException as e:
			print ("Service call failed: %s"%e)


if __name__ == '__main__':
	print("Asked for file with name <%s> : answer fileserver <%s>"%(name,call_service(name)  ) )