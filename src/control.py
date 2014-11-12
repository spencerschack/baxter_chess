#! /usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from moveit_msgs.msg import OrientationConstraint, PositionConstraint, Constraints, BoundingVolume, MoveGroupAction
from shape_msgs.msg import SolidPrimitive
from baxter_interface import *
from actionlib import SimpleActionClient
from tuck import Tuck
import moveit_commander
import tf
import chess
import stockfish
from math import pi

RIGHT_ARM_DEFAULT_POSE = PoseStamped()
RIGHT_ARM_DEFAULT_POSE.pose.position = Point(0.595, -0.159, 0.233)
RIGHT_ARM_DEFAULT_POSE.pose.orientation = Quaternion(-0.142, 0.985, 0.031, 0.090)

LEFT_ARM_DEFAULT_POSE = PoseStamped()
LEFT_ARM_DEFAULT_POSE.pose.position = Point(0.598, 0.161, 0.222)
LEFT_ARM_DEFAULT_POSE.pose.orientation = Quaternion(0.140, 0.986, -0.031, 0.081)

DEFAULT_MARKER_TYPES = {
	0:  chess.PAWN,
	1:  chess.PAWN,
	2:  chess.PAWN,
	3:  chess.PAWN,
	4:  chess.PAWN,
	5:  chess.PAWN,
	6:  chess.PAWN,
	7:  chess.PAWN,
	8:  chess.ROOK,
	9:  chess.KNIGHT,
	10: chess.BISHOP,
	11: chess.QUEEN,
	12: chess.KING,
	13: chess.BISHOP,
	14: chess.KNIGHT,
	15: chess.ROOK,
	16: chess.PAWN,
	17: chess.PAWN,
	18: chess.PAWN,
	19: chess.PAWN,
	20: chess.PAWN,
	21: chess.PAWN,
	22: chess.PAWN,
	23: chess.PAWN,
	24: chess.ROOK,
	25: chess.KNIGHT,
	26: chess.BISHOP,
	27: chess.QUEEN,
	28: chess.KING,
	29: chess.BISHOP,
	30: chess.KNIGHT,
	31: chess.ROOK,
}

DEFAULT_MARKER_BOARD = [
	8, 9, 10, 11, 12, 13, 14, 15,
	0, 1,  2,  3,  4,  5,  6,  7
] + ([None] * 32) + [
	16, 17, 18, 19, 20, 21, 22, 23,
	24, 25, 26, 27, 28, 29, 30, 31
]

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
		self.game = chess.Bitboard()
		self.engine = stockfish.Engine()
		self.engine.newgame()
		self.marker_types = DEFAULT_MARKER_TYPES.copy()
		self.marker_board = DEFAULT_MARKER_BOARD.copy()
		RobotEnable().enable()
		# TF
		self.tf = tf.TransformListener()
		# COMMANDERS
		moveit_commander.roscpp_initialize(sys.argv)
		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		# The only reason this is necessary is because the python interface for
		# MoveGroupCommander calls the MoveGroup initializer with a timeout of
		# 5 seconds arbitrarily.
		SimpleActionClient('move_group', MoveGroupAction).wait_for_server()
		self.left_arm = moveit_commander.MoveGroupCommander('left_arm')
		self.right_arm = moveit_commander.MoveGroupCommander('right_arm')
		self.left_arm.set_planner_id('RRTConnectkConfigDefault')
		self.left_arm.set_planning_time(10)
		self.right_arm.set_planner_id('RRTConnectkConfigDefault')
		self.right_arm.set_planning_time(10)
		self.move_arm('left')
		self.move_arm('right')
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

		self.state = 'developing'

	def state_developing(self):
		rospy.sleep(10)
		pose = self.pose_for_marker('ar_marker_1')
		pose.pose.position.z += 0.1
		self.move_arm('right', pose)
		pose = self.pose_for_marker('ar_marker_1')
		self.move_arm('right', pose)
		self.right_gripper.close(True)
		pose.pose.position.z += 0.1
		self.move_arm('right', pose)
		pose = self.pose_for_marker('ar_marker_4')
		self.move_arm('right', pose)
		self.right_gripper.open(True)
		pose.pose.position.z += 0.1
		self.move_arm('right', pose)
		sys.exit(0)

	def state_waiting(self):
		for square in chess.SQUARES:
			piece = self.piece_at(square)
			if piece != None and piece != self.game.piece_at(square):
				self.state = 'resolving'
				break

	def state_resolving():
		for move in self.game.generate_legal_moves():
			# if move satisfies current game state:
			if self.update(move):
				state = 'playing'
			else:
				self.state = 'game_over'

	def state_playing(self):
		move_uci = self.engine.bestmove()['move']
		move = chess.Move.from_uci(move_uci)
		to_piece = self.piece_at(move.to_square)
		from_piece = self.piece_at(move.from_square)
		if to_piece:
			if to_piece
			self.move_piece(to_piece)
		self.move_piece(from_piece, move.to_square)
		self.move_arm('left')
		self.move_arm('right')
		if self.update(move):
			self.state = 'waiting'
		else:
			self.state = 'game_over'

	def state_game_over(self):
		self.move_arm('left')
		self.move_arm('right')
		rospy.signal_shutdown('Game Over')

	def update(self, move):
		if move.promotion != chess.NONE:
			marker_id = self.marker_at(move.to_square)
			self.marker_types[marker_id] = move.promotion
		self.game.push(move)
		self.engine.setposition([move.uci()])
		return not self.game.is_game_over()

	def piece_at(self, square):
		marker = self.marker_at(square)
		if marker:
			return self.piece_for(marker)

	def marker_at(self, square):
		return self.marker_board[square]

	def piece_for(self, marker):
		color = chess.WHITE if marker < 14 else chess.BLACK
		ptype = self.marker_types[marker]
		return Piece(ptype, color)

	def pose_for_marker(self, name, frame='base'):
		time = self.tf.getLatestCommonTime(frame, name)
		position, rotation = self.tf.lookupTransform('base', name, time)
		pose = PoseStamped()
		pose.pose.position = Point(*position)
		pose.pose.orientation = Quaternion(0, -1, 0, 0)
		return pose

	def move_arm(self, name, pose=None, constrain=False):
		if pose == None:
			pose = LEFT_ARM_DEFAULT_POSE if name == 'left' else RIGHT_ARM_DEFAULT_POSE
		pose.header.frame_id = 'base'
		limb = self.left_arm if name == 'left' else self.right_arm
		limb.set_pose_target(pose)
		limb.set_start_state_to_current_state()
		if constrain:
			constraints = Constraints()
			constraints.position_constraints = [self.position_constraint(name)]
			#limb.set_path_constraints(constraints)
		plan = limb.plan()
		limb.execute(plan)

	def position_constraint(self, name):
		constraint = PositionConstraint()
		link_name = 'left_gripper' if name == 'left' else 'right_gripper'
		constraint.link_name = link_name
		constraint.header.frame_id = 'base'
		constraint.weight = 1.0
		bounding_volume = BoundingVolume()
		solid = SolidPrimitive()
		solid.type = SolidPrimitive.BOX
		solid.dimensions = [0.5, 0.76, 0.76]
		solid_pose = Pose()
		solid_pose.position = Point(0, 0, 0)
		solid_pose.orientation = Quaternion(0, 0, 0, 1)
		bounding_volume.primitives = [solid]
		bounding_volume.primitive_poses = [solid_pose]
		constraint.constraint_region = bounding_volume
		p = self.pose_for_marker('ar_marker_0', frame=link_name).pose.position
		constraint.target_point_offset = Vector3(p.x, p.y, p.z + 0.38)
		return constraint

if __name__ == '__main__':
	Control()
