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
from math import pi, floor

PIECE_WIDTH = PIECE_HEIGHT = 0.045
PIECE_DEPTH = 0.0254
SQUARE_WIDTH = SQUARE_HEIGHT = 0.07

VERTICAL_CLEARANCE = 0.07
GRIP_DEPTH = 0.01

RIGHT_ARM_DEFAULT_POSE = PoseStamped()
RIGHT_ARM_DEFAULT_POSE.pose.position = Point(0.595, -0.159, 0.233)
RIGHT_ARM_DEFAULT_POSE.pose.orientation = Quaternion(-0.142, 0.985, 0.031, 0.090)

LEFT_ARM_DEFAULT_POSE = PoseStamped()
LEFT_ARM_DEFAULT_POSE.pose.position = Point(0.598, 0.161, 0.222)
LEFT_ARM_DEFAULT_POSE.pose.orientation = Quaternion(0.140, 0.986, -0.031, 0.081)

DEFAULT_MARKER_TYPES = {
	 0: chess.PAWN,  1: chess.PAWN,    2: chess.PAWN,    3: chess.PAWN,
	 4: chess.PAWN,  5: chess.PAWN,    6: chess.PAWN,    7: chess.PAWN,
	 8: chess.ROOK,  9: chess.KNIGHT, 10: chess.BISHOP, 11: chess.QUEEN,
	12: chess.KING, 13: chess.BISHOP, 14: chess.KNIGHT, 15: chess.ROOK,
	16: chess.PAWN, 17: chess.PAWN,   18: chess.PAWN,   19: chess.PAWN,
	20: chess.PAWN, 21: chess.PAWN,   22: chess.PAWN,   23: chess.PAWN,
	24: chess.ROOK, 25: chess.KNIGHT, 26: chess.BISHOP, 27: chess.QUEEN,
	28: chess.KING, 29: chess.BISHOP, 30: chess.KNIGHT, 31: chess.ROOK
}

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
		# Board and Piece poses
		rospy.Subscriber('ar_pose_marker/left', AlvarMarkers, self.received_ar_pose_markers, 'left')
		rospy.Subscriber('ar_pose_marker/right', AlvarMarkers, self.received_ar_pose_markers, 'right')
		self.board_pose = None
		self.marker_types = DEFAULT_MARKER_TYPES.copy()
		self.marker_board = [None] * 64
		self.marker_poses = {}
		# Chess
		self.game = chess.Bitboard()
		self.engine = stockfish.Engine()
		self.engine.newgame()
		# Enable
		RobotEnable().enable()
		# Commanders
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
		# Grippers
		self.left_gripper = Gripper('left')
		self.right_gripper = Gripper('right')
		self.left_gripper.calibrate()
		self.right_gripper.calibrate()
		# Cameras
		self.left_hand_camera = CameraController('left_hand_camera')
		self.right_hand_camera = CameraController('right_hand_camera')
		resolution = CameraController.MODES[0]
		self.left_hand_camera.resolution = resolution
		self.right_hand_camera.resolution = resolution
		self.left_hand_camera.open()
		self.right_hand_camera.open()

		self.state = 'game_over'

	def state_waiting(self):
		for square in chess.SQUARES:
			piece = self.piece_at(square)
			# TODO: better observation
			if piece != None and piece != self.game.piece_at(square):
				self.state = 'resolving'
				break

	def state_resolving():
		for move in self.game.generate_legal_moves():
			self.game.push(move)
			match = all(self.game.piece_at(square) == self.piece_at(square)
				for square in chess.SQUARES)
			self.game.pop(move)
			if match:
				if self.update(move):
						state = 'playing'
				else:
					self.state = 'game_over'
				break

	def state_playing(self):
		move_uci = self.engine.bestmove()['move']
		move = chess.Move.from_uci(move_uci)
		to_marker = self.marker_at(move.to_square)
		from_marker = self.marker_at(move.from_square)
		# Must check against `None` otherwise the 0 marker would be falsy
		if to_marker != None:
			# Castling
			to_color = self.piece_at(move.to_square).color
			from_color = self.piece_at(move.from_square).color
			if to_color == from_color:
				self.pickup_piece('left', to_marker)
				self.move_arm('left')
				self.pickup_piece('right', from_marker)
				self.move_arm('right')
				self.place_piece('left', move.from_square)
				self.move_arm('left')
				self.place_piece('right', move.to_square)
				self.move_arm('right')
			# Attack
			else:
				# TODO: en-passant case
				self.pickup_piece('left', to_marker)
				self.place_piece('left')
				self.move_arm('left')
				self.pickup_piece('right', from_marker)
				self.place_piece('right', move.to_square)
				self.move_arm('right')
		# Normal
		else:
			self.pickup_piece('right', from_marker)
			self.place_piece('right', move.to_square)
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

	def pose_for(self, marker):
		return this.marker_poses[marker]

	def received_ar_pose_markers(self, markers, side):
		for marker in markers:
			# Special case for AR tags that represent the board.
			if marker.id == 32:
				marker_position = marker.pose.pose.position
				# Move the position from the AR tag to the corner of the board.
				marker_position.x += SQUARE_WIDTH / 2
				marker_position.y += SQUARE_HEIGHT / 2
				self.board_pose = marker.pose
			elif self.board_pose:
				marker_position = marker.pose.pose.position
				board_position = self.board_pose.pose.position
				dx = marker_position.x - board_position.x
				dy = marker_position.y - board_position.y
				ix = floor(dx / SQUARE_WIDTH)
				iy = floor(dy / SQUARE_HEIGHT)
				self.marker_board[ix + iy * 8] = marker.id
				self.marker_poses[marker.id] = marker.pose

	# Move the arm to right above the marker, then move down to move the gripper
	# into place, close the gripper, move back up to original height.
	def pickup_piece(self, side, marker):
		pose = self.pose_for(marker)
		pose.pose.position.z += self.VERTICAL_CLEARANCE
		self.move_arm(side, pose)
		pose.pose.position.z -= VERTICAL_CLEARANCE - GRIP_DEPTH
		self.move_arm(side, pose)
		self.move_gripper(side, 'close')
		pose.pose.position.z += VERTICAL_CLEARANCE + PIECE_DEPTH
		self.move_arm(side, pose)

	# Assumes piece is already in gripper. Moves arm to right above the square,
	# moves down into place, opens gripper, moves back up to original height. If
	# no square is passed in, then the piece will be removed from the board.
	def place_piece(self, side, square=None):
		if square == None:
			pose = # TODO: find next available spot to place attacked piece
		else:
			pose = self.board_pose
			pose.pose.position.x += SQUARE_WIDTH * (square % 8)
			pose.pose.position.y += SQUARE_HEIGHT * (square / 8)
		pose.pose.position.z += VERTICAL_CLEARANCE + PIECE_DEPTH - GRIP_DEPTH
		self.move_arm(side, pose)
		pose.pose.position.z -= VERTICAL_CLEARANCE - PIECE_DEPTH
		self.move_arm(side, pose)
		self.move_gripper(side, 'open')
		pose.pose.position.z += VERTICAL_CLEARANCE + GRIP_DEPTH
		self.move_arm(side, pose)

	def move_gripper(self, side, action):
		gripper = self.left_gripper if side == 'left' else self.right_gripper
		# `True` makes the command blocking.
		gripper.open(True) if action == 'open' else gripper.close(True)

	def move_arm(self, name, pose=None):
		if pose == None:
			pose = LEFT_ARM_DEFAULT_POSE if name == 'left' else RIGHT_ARM_DEFAULT_POSE
		pose.header.frame_id = 'base'
		limb = self.left_arm if name == 'left' else self.right_arm
		limb.set_pose_target(pose)
		limb.set_start_state_to_current_state()
		plan = limb.plan()
		limb.execute(plan)

if __name__ == '__main__':
	Control()
