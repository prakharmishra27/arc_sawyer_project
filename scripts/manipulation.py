import sys
import copy
from tabnanny import check
import numpy as np
import perception
from matplotlib import transforms
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from math import degrees, radians
from tf.transformations import quaternion_from_euler
import intera_interface
from intera_interface import CHECK_VERSION
from rospy import Time

from geometry_msgs.msg import (
	
	PoseStamped
)


def all_close(goal, actual, tolerance):

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

class SawyerManipulation():

	def __init__(self):

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('sawyermanipulation', anonymous=True)

		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()

		group_name = "right_arm"
		self.group = moveit_commander.MoveGroupCommander(group_name)

		planning_frame = self.group.get_planning_frame()
		print("============ Reference frame: %s" % planning_frame)

		eef_link = self.group.get_end_effector_link()
		print("============ End effector: %s" % eef_link)

		group_names = self.robot.get_group_names()
		print("============ Robot Groups:", self.robot.get_group_names())


	def gripper_open(limb= True):
		rs = intera_interface.RobotEnable(CHECK_VERSION)
		gripper = intera_interface.Gripper('right_gripper')
		# gripper_le = intera_interface.Gripper('left_gripper')
		# gripper.reboot()
		rospy.sleep(1)
		gripper.calibrate()
		rospy.sleep(1)
		gripper.open()
		# rospy.sleep(3)
		# gripper.close()

	def gripper_close(limb= True):
		rs = intera_interface.RobotEnable(CHECK_VERSION)
		gripper = intera_interface.Gripper('right_gripper')
		# gripper_le = intera_interface.Gripper('left_gripper')
		# gripper.reboot()
		rospy.sleep(1)
		gripper.calibrate()
		
		rospy.sleep(3)
		gripper.close()

	

	def go_to_pose_goal(self, target_pose):

		group = self.group

		group.set_pose_target(target_pose)
		group.set_pose_reference_frame('world')
		group.set_planning_time(2)
		# plan = group.go(wait=True)		
		
		group.allow_replanning(value=True)
		group.set_num_planning_attempts(4)
		group.set_max_acceleration_scaling_factor(value= 0.7)
		group.set_max_velocity_scaling_factor(value= 0.7)
		# group.plan()
		group.go()
		group.stop()
		group.clear_pose_targets()		
		current_pose = group.get_current_pose().pose
		
		return all_close(target_pose, current_pose, 0.01)