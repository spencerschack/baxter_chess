#! /usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from baxter_interface import *
import moveit_commander
import tf
from move import Move
from math import pi

RIGHT_ARM_DEFAULT_POSE = PoseStamped()
RIGHT_ARM_DEFAULT_POSE.pose.position = Point(0.665, -0.339, 0.068)
RIGHT_ARM_DEFAULT_POSE.pose.orientation = Quaternion(0.678, -0.671, -0.139, -0.264)

LEFT_ARM_DEFAULT_POSE = PoseStamped()
LEFT_ARM_DEFAULT_POSE.pose.position = Point(0.731, 0.256, 0.055)
LEFT_ARM_DEFAULT_POSE.pose.orientation = Quaternion(0.696, 0.677, -0.107, 0.214)

# This class acts as the top-level controller, coordinating everything
# that is not in its own node.
class Control:

	def __init__(self):
		rospy.init_node('baxter_chess_control')
		self.state_pub = rospy.Publisher('baxter_chess/control/state', String, latch=True)
		self.state = 'initializing'
		while not rospy.is_shutdown():
			self.state_pub.publish(self.state)
			# Calls the method 'state_<state>'
			getattr(self, 'state_' + self.state)()

	def state_initializing(self):
		RobotEnable().enable()
		rospy.on_shutdown(lambda: RobotEnable().disable())
		# GRIPPERS
		self.left_gripper = Gripper('left')
		self.right_gripper = Gripper('right')
		self.left_gripper.calibrate()
		self.right_gripper.calibrate()
		# CAMERAS
		self.left_hand_camera = CameraController('left_hand_camera')
		self.right_hand_camera = CameraController('right_hand_camera')
		resolution = CameraController.MODES[0]
		self.left_hand_camera.resolution = resolution
		self.right_hand_camera.resolution = resolution
		self.left_hand_camera.open()
		self.right_hand_camera.open()
		# COMMANDERS
		moveit_commander.roscpp_initialize(sys.argv)
		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		self.left_arm = moveit_commander.MoveGroupCommander('left_arm')
		self.right_arm = moveit_commander.MoveGroupCommander('right_arm')
		self.left_arm.set_planner_id('RRTConnectkConfigDefault')
		self.left_arm.set_planning_time(10)
		self.right_arm.set_planner_id('RRTConnectkConfigDefault')
		self.right_arm.set_planning_time(10)
		# TF
		self.tf = tf.TransformListener()
		# Move arms to somewhere they can see the board
		self.move('left_arm', LEFT_ARM_DEFAULT_POSE)
		self.move('right_arm', RIGHT_ARM_DEFAULT_POSE)

		self.state = 'developing'

	def state_developing(self):
		name = 'ar_marker_1'
		time = self.tf.getLatestCommonTime('base', name)
		position, rotation = self.tf.lookupTransform('base', name, time)
		pose = PoseStamped()
		pose.pose.position = Point(position[0], position[1], position[2] + 0.1)
		pose.pose.orientation = Quaternion(0, -1, 0, 0)
		self.move('right_arm', pose)
		pose.pose.position.z -= 0.05
		self.move('right_arm', pose)
		self.right_gripper.close(True)
		pose.pose.position.z += 0.1
		self.move('right_arm', pose)

	def move(self, name, pose):
		pose.header.frame_id = 'base'
		limb = self.left_arm if name == 'left_arm' else self.right_arm
		limb.set_pose_target(pose)
		limb.set_start_state_to_current_state()
		plan = limb.plan()
		limb.execute(plan)

if __name__ == '__main__':
	Control()
