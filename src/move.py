#! /usr/bin/env python

import sys
import rospy
import actionlib
import moveit_commander
from std_msgs.msg import String
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import FollowJointTrajectoryAction

class Move:

	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('baxter_chess_move')

		rospy.Subscriber('baxter_chess/move/left_arm', PoseStamped, self.received_command, 'left_arm')
		rospy.Subscriber('baxter_chess/move/right_arm', PoseStamped, self.received_command, 'right_arm')

		self.left_arm_state_pub = rospy.Publisher('baxter_chess/move/left_arm_state', String, latch=True)
		self.right_arm_state_pub = rospy.Publisher('baxter_chess/move/right_arm_state', String, latch=True)
		self.set_state('left_arm', 'initializing')
		self.set_state('right_arm', 'initializing')

		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()

		self.left_arm = moveit_commander.MoveGroupCommander('left_arm')
		self.right_arm = moveit_commander.MoveGroupCommander('right_arm')

		# Defaults from lab7
		self.left_arm.set_planner_id('RRTConnectkConfigDefault')
		self.left_arm.set_planning_time(10)
		self.right_arm.set_planner_id('RRTConnectkConfigDefault')
		self.right_arm.set_planning_time(10)

		self.set_state('left_arm', 'waiting')
		self.set_state('right_arm', 'waiting')

		rospy.spin()

	def set_state(self, name, state):
		if name == 'left_arm':
			self.left_arm_state = state
			self.left_arm_state_pub.publish(state)
		elif name == 'right_arm':
			self.right_arm_state = state
			self.right_arm_state_pub.publish(state)

	def get_state(self, name):
		if name == 'left_arm':
			return self.left_arm_state
		elif name == 'right_arm':
			return self.right_arm_state

	def get_limb(self, name):
		if name == 'left_arm':
			return self.left_arm
		elif name == 'right_arm':
			return self.right_arm

	def received_command(self, pose, name):
		# We don't need to lock/synchronize this state read because there is only
		# one thread per topic. So one thread per arm.
		self.set_state(name, 'receiving')
		pose.header.frame_id = "base"
		limb = self.get_limb(name)
		limb.set_pose_target(pose)
		limb.set_start_state_to_current_state()

		self.set_state(name, 'planning')
		plan = limb.plan()

		self.set_state(name, 'executing')
		limb.execute(plan)

		self.set_state(name, 'waiting')

if __name__ == '__main__':
	Move()