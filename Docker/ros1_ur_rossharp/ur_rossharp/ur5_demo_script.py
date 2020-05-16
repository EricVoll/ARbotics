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
"""
What this shows:
- demonstrates planning in AR and interfaceing with MoveIt.
- 1. Stage: Follow published points. 
- 2. Visualize position in matlab
- 3. 
"""

class PosePublisher():

	def __init__(self):
		rospy.init_node('ur5_demo', anonymous=False)
		self.pub = rospy.Publisher('euclidean_goal_pose', Pose, queue_size=1)
		self.sin_t = 0

	def publish(self, t, q):
		ros_pose = Pose()
		ros_pose.position.x = t['x']
		ros_pose.position.y = t['y']
		ros_pose.position.z = t['z']
		ros_pose.orientation.x = q['x']
		ros_pose.orientation.y = q['y']
		ros_pose.orientation.z = q['z']
		ros_pose.orientation.w = q['w'] 
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

from std_msgs.msg import Float32MultiArray  # noqa: F401
class JointPublisher():

	def __init__(self):
		rospy.init_node('ur5_demo', anonymous=False)
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
import Queue
import matplotlib.pyplot as plt
import numpy as np
import time

def track_joint_states(data,q):
	value = data.position[2]
	if q.full():
		q.get()
	q.put(value)

def track_move_goal(data,goal):
	goal[0] =  data.goal.request.goal_constraints[0].joint_constraints[0].position
	print(goal)

if __name__ == '__main__':
	
	# Values to update
	q_size=100
	q = Queue.Queue(maxsize=q_size)
	goal = [0]

	#Ros Topic Listener
	rospy.Subscriber("joint_states", JointState, track_joint_states,q)
	rospy.Subscriber("move_group/goal", MoveGroupActionGoal, track_move_goal,goal)

	jointPub = JointPublisher()
	rate = rospy.Rate(100)


	#Plotting
	blit = True
	i = 0
	fig_joint = plt.figure()

	ax_joint = fig_joint.add_subplot(111)
	ax_joint.set_ylim(-pi,pi)
	ax_joint.set_xlim(0,101)
	ax_joint.set_ylabel("Radians")

	line1_joint, = ax_joint.plot([],"r",label='Joint 0')
	line2_goal, = ax_joint.plot([], "--",label='Goal 0')
	ax_joint.legend()
	fig_joint.canvas.draw()

	if blit:
		axbackground = fig_joint.canvas.copy_from_bbox(ax_joint.bbox)
	
	plt.show(block=False)
	
	# x-axis
	X = list(range(0,100))

	while not rospy.is_shutdown():
		i = i + 1
		print(i)
		
		j_pan = []
		j_pan = list(q.queue)

		if len(j_pan) == 100:
			line1_joint.set_data(X,j_pan)
		line2_goal.set_data(X,goal*q_size)

		if blit:
			fig_joint.canvas.restore_region(axbackground)
			ax_joint.draw_artist(line1_joint)
			ax_joint.draw_artist(line2_goal)
			fig_joint.canvas.blit(ax_joint.bbox)
		

		fig_joint.canvas.flush_events()

		if i > 100:
			print("send command", goal)
			jointPub.box_avoidance()
			i = 0

		rate.sleep()

# if __name__ == '__main__':
# 	posePub = PosePublisher()
# 	rate = rospy.Rate(1)
# 	while not rospy.is_shutdown():
# 		posePub.move_sin()
# 		rate.sleep()