#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from baxter_interface import *
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

		self.left_arm_pub = rospy.Publisher('baxter_chess/move/left_arm', PoseStamped)
		self.right_arm_pub = rospy.Publisher('baxter_chess/move/right_arm', PoseStamped)
		self.piece_pub = rospy.Publisher('baxter_chess/piece/move', String)

		self.left_arm_state = None
		self.right_arm_state = None
		rospy.Subscriber('baxter_chess/move/left_arm_state', String, self.received_arm_state, 'left_arm')
		rospy.Subscriber('baxter_chess/move/right_arm_state', String, self.received_arm_state, 'right_arm')

		# Latch stores the last message published and sends it to new subscribers.
		self.state_pub = rospy.Publisher('baxter_chess/control/state', String, latch=True)

		self.state = 'initializing_robot'
		while not rospy.is_shutdown():
			self.state_pub.publish(self.state)
			# Calls the method 'state_<state>'
			getattr(self, 'state_' + self.state)()

	def received_arm_state(self, state, name):
		if name == 'left_arm':
			self.left_arm_state = state.data
		elif name == 'right_arm':
			self.right_arm_state = state.data

	def state_initializing_robot(self):
		RobotEnable().enable()
		rospy.on_shutdown(lambda: RobotEnable().disable())

		# GRIPPERS
		self.left_gripper = Gripper('left')
		self.right_gripper = Gripper('right')
		# These calibrate calls are blocking.
		self.left_gripper.calibrate()
		self.right_gripper.calibrate()

		# CAMERAS
		# These are closed by default. You can only have two open at a time.
		self.left_hand_camera = CameraController('left_hand_camera')
		self.right_hand_camera = CameraController('right_hand_camera')
		resolution = CameraController.MODES[0]
		self.left_hand_camera.resolution = resolution
		self.right_hand_camera.resolution = resolution
		self.left_hand_camera.open()
		self.right_hand_camera.open()

		# EXTREMITIES
		self.left_limb = Limb('left')
		self.right_limb = Limb('right')
		self.head = Head()

		self.state = 'initializing_locating_board'

	def state_initializing_locating_board(self):
		if self.left_arm_state == 'waiting' and self.right_arm_state == 'waiting':
			self.left_arm_pub.publish(LEFT_ARM_DEFAULT_POSE)
			self.right_arm_pub.publish(RIGHT_ARM_DEFAULT_POSE)
			rospy.sleep(10)
			self.piece_pub.publish('ar_marker_1')
			self.state = 'locating_board'

	def state_locating_board(self):
		rospy.spin()
		self.state = 'initializing_board'

	def state_initializing_board(self):
		# TODO: Place the pieces
		if not self.is_moving:
			if self.piece_is_out_of_place:
				move_piece
			else:
				# Determine if it is our turn if the white side of the board is
				# facing the robot, we can tell this by the color of the corners.
				is_turn = self.board_orientation > pi
				if is_turn:
					self.state = 'pondering'
				else:
					self.state = 'waiting'

	def state_pondering(self):
		# Pass piece positions to chess engine and wait for response
		if self.game_over:
			self.state = 'terminating'
		else:
			self.state = 'moving'

	def state_waiting(self):
		# Wait until player moves.
		self.state = 'moving'

	def state_moving(self):
		# Get coordinates of piece to move, pass to moveit, and command limb.
		self.state = 'waiting'

	def state_terminating():
		# Do whatever ROS needs to do to shutdown
		pass

if __name__ == '__main__':
	Control()
