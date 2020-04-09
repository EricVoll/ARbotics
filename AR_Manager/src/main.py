import os 

from client import CommandLineClient, RosClient
from server import Server

import logging

if __name__ == "__main__":
	client = CommandLineClient(id=10)
	client2 = RosClient(id=20)
	server = Server(client = client) 
	server.add_client(client2)
	ids = server.client_ids
	server.my_clients_do_something()

	print(server)
	id1 = server.start('UR3')
	id2 = server.start('UR5')
	id3 = server.start('UnityCollision')
	id4 = server.start('UnityCollision')
	print(server)

# PEP8 

# Variables:
# name_of_variable = 'Name'

# CONSTANTS:
# LIST_SETTING = [
# 	'setting1',
# 	'setting2'
# ]

# functions:
# #empty
# #empty
# def function_name():
# 	#empty
# 	return "something"
# #empty
# #empty

# Classes:
# #empty
# #empty
# class FunctionClass:
# 	#empty
# 	def __init__(self,name):
# 		#empty
# 		self.name = name
# 	#empty
# 	def fuc_number():
# 		#empty		
# 		return 0
# #empty
# #empty


# FactoryFunctions (functions return instances of classes)
# #in this case name of function upperletter

# def FunctionClassImp():
	
# 	return FunctionClass("Name") 


# class SomeClass():

# 	def __init__(self,name):

# 		self._name = name #implies no one should change the name
# 		return "Hello"
	
# 	@property	#getter method for name
# 	def name(self):
		
# 		return self.name

"""
sprint list: 
- server functions std 
- server config (best way)
- server functionality (starting ROS Nodes stopping ros nodes)
- how does this work in detail ?

MVP(all containers are created and up and running!)
Everything is configured on the ROS side !!! 
Just roslaunch possible no furter interactions

	CONFIG the capabilities of the server:

		ROS_Components:
			ROS_SHARP_Bridge(default launched):
				-
				-
				-
				-

			UR5:
				- name:
				- container:
				- launche:
				- topics_published:
				- services_offered:
			Camera:
				- name:
				- container:
				- launche: #container and launch file
				- topics_published:
				- services_offered:

	
		Unity_Components: 
			LidarSim: 
				- name:
				- container:
				- launch: #ROS sharp massage
				- topics_published: 
				- services_offered:		
			CollisionDetecter:
				- name:
				- container:
				- launch: #ROS sharp massage
				- topics_published: 
				- services_offered:

- config files must be updatable via the clients



When first implementation is running write unit test for this




-client functions (what must be able to be done)
- terminal client application
- ros client application
"""