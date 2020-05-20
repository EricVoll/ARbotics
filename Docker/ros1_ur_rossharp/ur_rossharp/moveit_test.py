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

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

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

class MoveGroupPythonIntefaceTutorial(object):

	def __init__(self):
		super(MoveGroupPythonIntefaceTutorial, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move_group_python_interface_tutorial',
										anonymous=True)
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
		self.add_box()
		

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
		# box_pose.header.frame_id = "world"
		# box_pose.pose.orientation.w = 1.0
		# box_pose.pose.position.x = 0.9
		# box_pose.pose.position.z = 0.125
		# box_pose.pose.position.y = 0
		# box_name = "box"
		# scene.add_box(box_name, box_pose, size=(1, 0.01, 0.5))



		box_pose.header.frame_id = "world"
		box_pose.pose.orientation.w = 1.0
		box_pose.pose.position.x = 0
		box_pose.pose.position.z = -0.005
		box_pose.pose.position.y = 0
		box_name = "ground_plane"
		scene.add_box(box_name, box_pose, size=(5, 5, 0.01))

		return self.wait_for_state_update(box_is_known=True, timeout=timeout)

def move_manipulator(data, robot):
	print("Move Manipulator to: ", data)
	robot.go_to_pose_goal(data)

def joint_goal_manipulator(data, robot):

	robot.go_to_joint_state(data)

def move_it_ar_interface():
	robot = MoveGroupPythonIntefaceTutorial()
	rospy.Subscriber("euclidean_goal_pose", geometry_msgs.msg.Pose , move_manipulator, robot)
	rospy.Subscriber("ar_joint_state", Float32MultiArray, joint_goal_manipulator, robot)

	rate = rospy.Rate(100)


	while not rospy.is_shutdown():
		
		rate.sleep()

if __name__ == '__main__':
	try:
		move_it_ar_interface()
	except rospy.ROSInterruptException:
		pass
	
