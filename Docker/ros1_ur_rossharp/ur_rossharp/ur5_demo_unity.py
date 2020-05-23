#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi 
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import MoveGroupActionGoal
from geometry_msgs.msg import Pose
import math
from std_msgs.msg import Float32MultiArray 

import base64
from io import BytesIO
import os
import Queue
#import matplotlib.pyplot as plt
import numpy as np
import time


import vispy.plot as vp
import vispy.io as io
"""
What this shows:
- demonstrates planning in AR and interfaceing with MoveIt.
- 1. Stage: Follow published points. 
- 2. Visualize position in matlab
- 3. 
"""

class PosePublisher():

	def __init__(self):
		rospy.init_node('ur5_demo_pose_publisher', anonymous=False)
		self.pub = rospy.Publisher('euclidean_goal_pose', Pose, queue_size=1)
		self.sin_t = 0

	def publish(self, t, q):
		"""publish scipy convention quat

		Parameters
		----------
		t : [type]
				[description]
		q : [type]
				[description]
		"""
		ros_pose = Pose()
		ros_pose.position.x = t[0]
		ros_pose.position.y = t[1]
		ros_pose.position.z = t[2]
		ros_pose.orientation.x = q[0]
		ros_pose.orientation.y = q[1]
		ros_pose.orientation.z = q[2]
		ros_pose.orientation.w = q[3] 
		self.pub.publish(ros_pose)


	def move_sin(self):
		 
		amp = 0.2
		inc = 0.1
		self.sin_t += inc
		
		offset = math.sin(self.sin_t)*amp

		t = {'x':0.3, 'y':0.3, 'z':0.3+offset}
		q={}
		t['x'] = 0.3
		t['y'] = 0.3
		t['z'] = 0.3+offset
		q['x'] = 0.0
		q['y'] = 0.0
		q['z'] = 0.0
		q['w'] = 1.2
		
		self.publish(t,q)
		print(t['z'])

class JointPublisher():

	def __init__(self):
		rospy.init_node('ur5_demo_joint_publisher', anonymous=False)
		self.pub = rospy.Publisher('ar_joint_state', Float32MultiArray, queue_size=1)
		self.sin_t = 0
		self.last_pose = 0
		self.poses = [[+0.8, -1.2, 2.0, 0.0, 0.0],[-0.8, -1.2, 2.0, 0.0, 0.0]]

	def publish(self, joints):
		ros_array = Float32MultiArray()
		ros_array.data = joints
		self.pub.publish(ros_array)
	
	def move_sin(self):
		amp = pi/4
		inc = 2*pi
		self.sin_t += inc
		offset =  math.cos(self.sin_t)*amp

		joints = [0,-0.75,offset,0,0]
		self.publish(joints)
		print(joints[4])
	
	def box_avoidance(self):
		if self.last_pose == 1:
			self.last_pose = 0
		else:
			self.last_pose = 1
		
		self.publish(self.poses[self.last_pose])


class ImagePublisher():
	
	def __init__(self, topic='joint_speed'):
		self.topic = 'img/'+topic
		self.pub = rospy.Publisher(self.topic, String, queue_size=1)
	
	def publish(self, img):
		"""
		Converts Image into base64 and published it on self.topic as a String
		"""
		img_b64 = base64.b64encode(img.getvalue()).decode()
		self.pub.publish( String(data=img_b64) )


# <tooltip name="base_tooltip" topic="/test/topic">
# 	<parent link="base_footprint"/>
# </tooltip>


# if __name__ == '__main__':
# 	posePub = PosePublisher()
# 	rate = rospy.Rate(1)
# 	while not rospy.is_shutdown():
# 		posePub.move_sin()
# 		rate.sleep()

# if __name__ == '__main__':
# 	posePub = PosePublisher()
# 	rate = rospy.Rate(1)
# 	while not rospy.is_shutdown():
# 		t ={}
# 		q ={}
# 		t['x'] = 0.5
# 		t['y'] = 0.25
# 		t['z'] = 0.25
# 		q['x'] = 0.0
# 		q['y'] = 0.0
# 		q['z'] = 0.0
# 		q['w'] = 1
# 		posePub.publish(t,q)
# 		rate.sleep()


def track_joint_states(data,qs):
	q_joint, q_goal, goal = qs
	
	value = data.position[2]
	if q_joint.full():
		q_joint.get()
	q_joint.put(value)

	if q_goal.full():
		q_goal.get()
	q_goal.put(copy.deepcopy(goal))

def track_move_goal(data,goal):
	value =  data.goal.request.goal_constraints[0].joint_constraints[0].position
	goal[0] = value

class Plot ():
	def __init__(self, line_cfgs):
		"""
		provide a list of line_names
		"""
		self.fig_joint = plt.figure()

		self.ax_joint = self.fig_joint.add_subplot(111)
		self.ax_joint.set_ylim(-pi,pi)
		self.ax_joint.set_xlim(0,101)
		self.ax_joint.set_ylabel("Radians")
		self.lines = []
		for line in line_cfgs:
			tmp, = self.ax_joint.plot([],line['color'],label=line['name'])
			self.lines.append(tmp)

		self.ax_joint.legend()
		self.fig_joint.canvas.draw()
		self.axbackground = self.fig_joint.canvas.copy_from_bbox(self.ax_joint.bbox)
		
		plt.show(block=False)

	def update(self,x, ys):
		"""
		Input x is array of values for x-Axis
		ys is an list of arrays containing y values with same length as x
		"""
		if not (len(x) == len(ys[0])):
			raise TypeError()

		for i, line in enumerate(self.lines):
			line.set_data(x,ys[i])

		self.fig_joint.canvas.restore_region(self.axbackground)
		for i, line in enumerate(self.lines):
			self.ax_joint.draw_artist(line)
		self.fig_joint.canvas.blit(self.ax_joint.bbox)
		self.fig_joint.canvas.flush_events()

	def image_png(self):
		img = BytesIO()
		plt.savefig(img, format='png')
		img.seek(0)
		return img


import numpy as np
import sys


def menu():
	l = 6
	l2 = 74
	print(''.ljust(l,'-') + '+'.ljust(l2,'-')+'+')
	print('Key:'.ljust(l, ' ') + '| Action:'.ljust(l2, ' ') + '|' )
	print(''.ljust(l,'-') + '+'.ljust(l2,'-')+'+')
	print('s'.ljust(l, ' ') + '| Adds Tooltip to UR5-Instance Dyn-URDF via AR-Manager REST-Request'.ljust(l2, ' ') + '|' )
	print('d'.ljust(l, ' ') +'| Resets UR5-Instance Dyn-URDF via AR-Manager REST-Request'.ljust(l2, ' ') + '|' )

from rest_keyboard_interface import RestKeyboardListener

from vispy.plot import Fig

if __name__ == '__main__':

	#	publish joint goals
	posePub = PosePublisher()
	i = 0
	amp = 0.4

	rate = rospy.Rate(100)
	
	#timer = app.Timer('auto', connect=update, start=True)

	while not rospy.is_shutdown():
		xyz = [math.sin(i)*amp,math.sin(i)*amp,math.sin(i)*amp]
		i += 0.1
		posePub.publish(t=xyz ,q=[0,0,0,1] )
		rate.sleep()
		print(xyz)


