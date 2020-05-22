#!/usr/bin/env python
# Use python 2.7 important 

import rospy
from std_msgs.msg import String
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from std_msgs.msg import Float32MultiArray  # noqa: F401
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

import rospy
import tf
import geometry_msgs
import tf2_ros
from tf2_msgs.msg import TFMessage



def all_close(goal, actual, tolerance):
	"""
	Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
	@param: goal       A list of floats, a Pose or a PoseStamped
	@param: actual     A list of floats, a Pose or a PoseStamped
	@param: tolerance  A float
	@returns: bool
	"""
	all_equal = True
	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False

	elif type(goal) is geometry_msgs.msg.PoseStamped:
		return all_close(goal.pose, actual.pose, tolerance)

	elif type(goal) is geometry_msgs.msg.Pose:
		return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

	return True

rospy.init_node('moveit_ar_interface',
										anonymous=True)

class MoveGroupPythonIntefaceTutorial(object):

	def __init__(self):
		super(MoveGroupPythonIntefaceTutorial, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
		
		robot = moveit_commander.RobotCommander()

		scene = moveit_commander.PlanningSceneInterface()
		group_name = "manipulator"
		group = moveit_commander.MoveGroupCommander(group_name)

		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
																									moveit_msgs.msg.DisplayTrajectory,
																									queue_size=20)

		planning_frame = group.get_planning_frame()
		eef_link = group.get_end_effector_link()
		group_names = robot.get_group_names()

		self.box_name = ''
		self.robot = robot
		self.scene = scene
		self.group = group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_names
		#self.add_box()
		

	def go_to_joint_state(self,data):
		group = self.group
		joint_goal = group.get_current_joint_values()
		print(type(data.data))
		print(data)
		joint_goal[0:5] = data.data[0:5]
		group.go(joint_goal, wait=False)

		current_joints = self.group.get_current_joint_values()
		return all_close(joint_goal, current_joints, 0.01)

	def go_to_pose_goal(self, pose_goal):
	
		group = self.group
		
		group.set_pose_target(pose_goal)

		plan = group.go(wait=True)

		group.clear_pose_targets()
		current_pose = self.group.get_current_pose().pose
		return all_close(pose_goal, current_pose, 0.01)

	def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
		box_name = self.box_name
		scene = self.scene
		start = rospy.get_time()
		seconds = rospy.get_time()
		while (seconds - start < timeout) and not rospy.is_shutdown():
			# Test if the box is in attached objects
			attached_objects = scene.get_attached_objects([box_name])
			is_attached = len(attached_objects.keys()) > 0

			# Test if the box is in the scene.
			# Note that attaching the box will remove it from known_objects
			is_known = box_name in scene.get_known_object_names()

			# Test if we are in the expected state
			if (box_is_attached == is_attached) and (box_is_known == is_known):
				return True

			# Sleep so that we give other threads time on the processor
			rospy.sleep(0.1)
			seconds = rospy.get_time()
		return False


	def add_box(self, timeout=4):
		box_name = self.box_name
		scene = self.scene
		box_pose = geometry_msgs.msg.PoseStamped()

		box_pose.header.frame_id = "world"
		box_pose.pose.orientation.w = 1.0
		box_pose.pose.position.x = 0
		box_pose.pose.position.z = -0.005
		box_pose.pose.position.y = 0
		box_name = "ground_plane"
		#scene.add_box(box_name, box_pose, size=(5, 5, 0.01))

		return self.wait_for_state_update(box_is_known=True, timeout=timeout)

import numpy as np
from scipy.spatial.transform import Rotation as R


h_unity_to_world = np.zeros((4,4))
h_unity_to_world[0,:]=[0,0,1,0]
h_unity_to_world[1,:]=[-1,0,0,0]
h_unity_to_world[2,:]=[0,1,0,0]
h_unity_to_world[3,:]=[0,0,0,1]

t = geometry_msgs.msg.TransformStamped()
t.header.frame_id = "world"
t.header.stamp = rospy.Time.now()
t.child_frame_id = "unity"
t.transform.translation.x = 0.0
t.transform.translation.y = 0.0
t.transform.translation.z = 0.0

rot = R.from_dcm( h_unity_to_world[:3,:3] )
q_rot = rot.as_quat()
t.transform.rotation.x = q_rot[0]
t.transform.rotation.y = q_rot[1]
t.transform.rotation.z = q_rot[2]
t.transform.rotation.w = q_rot[3]

def t_to_homo(t):
	homo = np.eye(4)
	if isinstance(t, geometry_msgs.msg.Pose):
		homo[:3,:3] = R.from_quat( [t.orientation.x,t.orientation.y,t.orientation.z,t.orientation.w] ).as_dcm()
		homo[:3,3] = [t.position.x,t.position.y,t.position.z]
	if isinstance(t, geometry_msgs.msg.Transform):
		homo[:3,:3] = R.from_quat( [t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w] ).as_dcm()
		homo[:3,3] = [t.translation.x,t.translation.y,t.translation.z]
	return homo

def homo_to_t(homo):
	q = R.from_dcm( homo[:3,:3] ).as_quat() 
	p = Pose()
	p.orientation.x = q[0]
	p.orientation.y = q[1]
	p.orientation.z = q[2]
	p.orientation.w = q[3]
	p.position.x = homo[0,3]
	p.position.y = homo[1,3]
	p.position.z = homo[2,3]
	return p

def move_manipulator(data, robot):
	h_in = t_to_homo(data)
	h_out = np.dot( h_unity_to_world,h_in)
	robot.go_to_pose_goal(homo_to_t(h_out))

def joint_goal_manipulator(data, robot):
	robot.go_to_joint_state(data)

def move_it_ar_interface():

	robot = MoveGroupPythonIntefaceTutorial()
	rospy.Subscriber("euclidean_goal_pose", geometry_msgs.msg.Pose , move_manipulator, robot)
	rospy.Subscriber("ar_joint_state", Float32MultiArray, joint_goal_manipulator, robot)
	tf_pub = rospy.Publisher('tf_static', TFMessage , queue_size=1)
	tfm = TFMessage([t])
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		tf_pub.publish(tfm)
		rate.sleep()

if __name__ == '__main__':
	try:
		move_it_ar_interface()
	except rospy.ROSInterruptException:
		pass
	
