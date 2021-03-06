#! /usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from moveit_msgs.msg import OrientationConstraint, PositionConstraint, Constraints, BoundingVolume, MoveGroupAction
from shape_msgs.msg import SolidPrimitive
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker
from baxter_interface import *
from actionlib import SimpleActionClient
from tuck import Tuck
import moveit_commander
import tf
import chess
import stockfish
from math import pi, floor
from copy import deepcopy

PIECE_DEPTH = 0.028
SQUARE_SIZE = 0.07

VERTICAL_CLEARANCE = 0.05
DROP_HEIGHT = 0.02

GRIP_DEPTH = 0.025
# Measured from the tf frame 'right_gripper' to the end of the suction cup.
GRIPPER_DEPTH = 0.03

# The x coordinate of the suction cup is always a little off.
PICKUP_X_ADJUSTMENT = -0.015
PICKUP_Y_ADJUSTMENT = 0

RIGHT_ARM_DEFAULT_POSE = PoseStamped()
RIGHT_ARM_DEFAULT_POSE.pose.position = Point(0.595, -0.159, 0.233)
RIGHT_ARM_DEFAULT_POSE.pose.orientation = Quaternion(-0.142, 0.985, 0.031, 0.090)

LEFT_ARM_DEFAULT_POSE = PoseStamped()
LEFT_ARM_DEFAULT_POSE.pose.position = Point(0.598, 0.161, 0.222)
LEFT_ARM_DEFAULT_POSE.pose.orientation = Quaternion(0.140, 0.986, -0.031, 0.081)

LEFT_ARM_DEMO1_POSE = PoseStamped()
LEFT_ARM_DEMO1_POSE.pose.position = Point(0.376, 0.323, -0.124)
LEFT_ARM_DEMO1_POSE.pose.orientation = Quaternion(-0.142, 0.973, -0.088, -0.161)

RIGHT_ARM_DEMO1_POSE = PoseStamped()
RIGHT_ARM_DEMO1_POSE.pose.position = Point(0.613, -0.170, -0.013)
RIGHT_ARM_DEMO1_POSE.pose.orientation = Quaternion(0.482, 0.853, 0.114, -0.164)

LEFT_ARM_DEMO2_POSE = PoseStamped()
LEFT_ARM_DEMO2_POSE.pose.position = Point(0.376, 0.323, -0.124)
LEFT_ARM_DEMO2_POSE.pose.orientation = Quaternion(-0.142, 0.973, -0.088, -0.161)

RIGHT_ARM_DEMO2_POSE = PoseStamped()
RIGHT_ARM_DEMO2_POSE.pose.position = Point(0.501, 0.107, -0.054)
RIGHT_ARM_DEMO2_POSE.pose.orientation = Quaternion(0.097, 0.958, 0.079, -0.258)

LEFT_ARM_DEMO3_POSE = PoseStamped()
LEFT_ARM_DEMO3_POSE.pose.position = Point(0.376, 0.323, -0.124)
LEFT_ARM_DEMO3_POSE.pose.orientation = Quaternion(-0.142, 0.973, -0.088, -0.161)

RIGHT_ARM_DEMO3_POSE = PoseStamped()
RIGHT_ARM_DEMO3_POSE.pose.position = Point(0.654, -0.033, -0.047)
RIGHT_ARM_DEMO3_POSE.pose.orientation = Quaternion(0.146, 0.955, 0.093, -0.239)

LEFT_ARM_DEMO4_POSE = PoseStamped()
LEFT_ARM_DEMO4_POSE.pose.position = Point(0.376, 0.323, -0.124)
LEFT_ARM_DEMO4_POSE.pose.orientation = Quaternion(-0.142, 0.973, -0.088, -0.161)

RIGHT_ARM_DEMO4_POSE = PoseStamped()
RIGHT_ARM_DEMO4_POSE.pose.position = Point(0.609, -0.017, -0.010)
RIGHT_ARM_DEMO4_POSE.pose.orientation = Quaternion(0.720, -0.690, -0.069, -0.005)

DOWN = Quaternion(0, -1, 0, 0)

DEFAULT_MARKER_TYPES = {
	 0: chess.PAWN,   1: chess.PAWN,    2: chess.PAWN,    3: chess.PAWN,
	 4: chess.PAWN,   5: chess.PAWN,    6: chess.PAWN,    7: chess.PAWN,
	 8: chess.ROOK,   9: chess.KNIGHT, 10: chess.BISHOP, 11: chess.KING,
	12: chess.QUEEN, 13: chess.BISHOP, 14: chess.KNIGHT, 15: chess.ROOK,
	16: chess.PAWN,  17: chess.PAWN,   18: chess.PAWN,   19: chess.PAWN,
	20: chess.PAWN,  21: chess.PAWN,   22: chess.PAWN,   23: chess.PAWN,
	24: chess.ROOK,  25: chess.KNIGHT, 26: chess.BISHOP, 27: chess.KING,
	28: chess.QUEEN, 29: chess.BISHOP, 30: chess.KNIGHT, 31: chess.ROOK
}

DEMO_BOARD_STATES = {
	1: [
		(chess.E2, chess.WHITE, chess.KNIGHT),
		(chess.D4, chess.BLACK, chess.QUEEN),
		(chess.F4, chess.BLACK, chess.KING)
	],
	2: [
		(chess.D4, chess.WHITE, chess.KING),
		(chess.A4, chess.WHITE, chess.ROOK)
	],
	3: [
		(chess.C4, chess.BLACK, chess.PAWN),
		(chess.D4, chess.BLACK, chess.PAWN),
		(chess.E4, chess.BLACK, chess.PAWN),
		(chess.D3, chess.BLACK, chess.KNIGHT)
	],
	4: [
		(chess.E2, chess.BLACK, chess.KING),
		(chess.C3, chess.BLACK, chess.QUEEN),
		(chess.E3, chess.BLACK, chess.PAWN),
		(chess.C4, chess.BLACK, chess.ROOK),
		(chess.F4, chess.WHITE, chess.PAWN),
		(chess.E5, chess.WHITE, chess.ROOK),
		(chess.E6, chess.WHITE, chess.KING),
		(chess.F6, chess.WHITE, chess.KNIGHT)
	]
}

DEMO_MOVES = {
	1: 'e2d4',
	2: 'd4a4',
	3: 'd3c5',
	4: None
}

DEMO_LEFT_ARM_POSES = {
	1: LEFT_ARM_DEMO1_POSE,
	2: LEFT_ARM_DEMO2_POSE,
	3: LEFT_ARM_DEMO3_POSE,
	4: LEFT_ARM_DEMO4_POSE
}

DEMO_RIGHT_ARM_POSES = {
	1: RIGHT_ARM_DEMO1_POSE,
	2: RIGHT_ARM_DEMO2_POSE,
	3: RIGHT_ARM_DEMO3_POSE,
	4: RIGHT_ARM_DEMO4_POSE
}

# 1: Fork
# 2: Castle
# 3: Jump
# 4: Stockfish
DEMO = 4

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
		self.marker_squares = [None] * 32
		self.marker_poses = [None] * 32
		# Chess
		self.game = chess.Bitboard()
		self.engine = stockfish.Engine()
		self.engine.newgame()
		if DEMO and False:
			self.game.clear()
			for square, color, piece_type in DEMO_BOARD_STATES[DEMO]:
				self.game.set_piece_at(square, chess.Piece(piece_type, color))
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
		# Cameras
		# Just in case the head camera was open before. It must be closed because
		# you can only have 2 cameras open at a time.
		CameraController('head_camera').close()
		self.left_hand_camera = CameraController('left_hand_camera')
		self.right_hand_camera = CameraController('right_hand_camera')
		resolution = CameraController.MODES[0]
		self.left_hand_camera.resolution = resolution
		self.right_hand_camera.resolution = resolution
		self.left_hand_camera.open()
		self.right_hand_camera.open()

		self.state = 'checking'

	def state_checking(self):
		self.print_board()
		if raw_input() == 'p':
			self.state = 'placing'

	def state_waiting(self):
		raw_input()
		self.state = 'resolving'
		return
		for square in chess.SQUARES:
			piece = self.piece_at(square)
			# TODO: better observation
			# basically wait until the board has changed and all pieces are visible
			# maybe add a timeout depending on how reliable that is
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

	def state_placing(self):
		for square in chess.SQUARES:
			piece = self.piece_at(square)
			if piece:
				self.game.set_piece_at(square, piece)
			else:
				self.game.remove_piece_at(square)
		self.engine.setboard(self.game.fen())
		self.print_board()
		self.state = 'playing'

	def state_playing(self):
		move_uci = self.next_move()
		move = chess.Move.from_uci(move_uci)
		to_square = move.to_square
		from_square = move.from_square
		if self.game.piece_at(to_square):
			to_color = self.game.piece_at(to_square).color
			from_color = self.game.piece_at(from_square).color
			# Castling
			if to_color == from_color:
				self.pickup_piece('left', to_square)
				self.move_arm('left')
				self.pickup_piece('right', from_square)
				self.move_arm('right')
				self.place_piece('left', from_square)
				self.move_arm('left')
				self.place_piece('right', to_square)
				self.move_arm('right')
			# Attack
			else:
				self.pickup_piece('left', to_square)
				self.place_piece('left')
				self.move_arm('left')
				rospy.sleep(5)
				print 'picking up piece from ' + str(from_square)
				self.pickup_piece('right', from_square)
				self.place_piece('right', to_square)
				self.move_arm('right')
		# Normal
		else:
			piece = self.game.piece_at(from_square)
			is_pawn = piece.piece_type == chess.PAWN
			en_passant = self.col(from_square) != self.col(to_square)
			if is_pawn and en_passant:
				ep_square = to_square - 8 if piece.color == chess.WHITE else to_square + 8
				self.pickup_piece('left', ep_ssquare)
				self.place_piece('left')
				self.move_arm('left')
			self.pickup_piece('right', from_square)
			self.place_piece('right', to_square)
			self.move_arm('right')
		if self.update(move):
			self.state = 'waiting'
		else:
			self.state = 'game_over'

	def state_game_over(self):
		self.move_arm('left')
		self.move_arm('right')
		rospy.spin()
	
	def next_move(self):
		if DEMO and DEMO_MOVES[DEMO]:
			return DEMO_MOVES[DEMO]
		else:
			move = self.engine.bestmove()['move']
			return move

	def update(self, move):
		if move.promotion != chess.NONE:
			marker_id = self.marker_at(move.to_square)
			self.marker_types[marker_id] = move.promotion
		self.game.push(move)
		self.engine.setposition([move.uci()])
		return not self.game.is_game_over()

	def piece_at(self, square):
		marker = self.marker_at(square)
		if marker != None:
			return self.piece_for(marker)

	def marker_at(self, square):
		return self.marker_board[square]

	def piece_for(self, marker):
		color = chess.WHITE if marker < 16 else chess.BLACK
		piece_type = self.marker_types[marker]
		return chess.Piece(piece_type, color)

	def pose_for(self, marker):
		return self.marker_poses[marker]

	def row(self, square):
		return square / 8

	def col(self, square):
		return square % 8

	def print_board(self):
		print '-' * 18
		for i in range(8):
			print '|',
			for j in range(8):
				piece = self.piece_at(7 - j + i * 8)
				symbol = piece.symbol() if piece else ' '
				print symbol,
			print '|'
		print '-' * 18

	def received_ar_pose_markers(self, markers, side):
		for marker in markers.markers:
			# Special case for AR tags that represent the board.
			if marker.id == 32:
				marker.pose.pose.position.x += SQUARE_SIZE
				marker.pose.pose.position.y -= SQUARE_SIZE
				self.board_pose = marker.pose
			elif marker.id < 32:
				if self.board_pose:
					marker_position = marker.pose.pose.position
					board_position = self.board_pose.pose.position
					dx = marker_position.x - board_position.x
					# y is negated because the y-axis goes in the opposite
					# direction as the letters on chess rows.
					dy = board_position.y - marker_position.y
					# Plus one half because the board pose marker is half a square unit
					# away from the actual corner of the board.
					ix = int(floor(dx / SQUARE_SIZE + 0.5))
					iy = int(floor(dy / SQUARE_SIZE + 0.5))
					if 0 <= ix <= 7 and 0 <= iy <= 7:
						square = ix * 8 + iy
						last_square = self.marker_squares[marker.id]
						if last_square != None:
							self.marker_board[last_square] = None
						self.marker_squares[marker.id] = square
						self.marker_board[square] = marker.id
					self.marker_poses[marker.id] = marker.pose

	# Move the arm to right above the marker, then move down to move the gripper
	# into place, close the gripper, move back up to original height.
	def pickup_piece(self, side, square):
		pose = self.pose_at(self.row(square), self.col(square))
		right_fix = 0.02 if side == 'right' else 0
		pose.pose.position.z += VERTICAL_CLEARANCE + GRIPPER_DEPTH + right_fix
		pose.pose.position.x += 0.07
		self.move_arm(side, pose)
		# Sleep to get new ar tag positions from the camera
		rospy.sleep(4)
		pose = PoseStamped()
		marker = self.marker_at(square)
		pose.pose.position = self.pose_for(marker).pose.position
		pose.pose.orientation = DOWN
		pose.pose.position.x += PICKUP_X_ADJUSTMENT
		pose.pose.position.y += PICKUP_Y_ADJUSTMENT
		pose.pose.position.z += GRIPPER_DEPTH - GRIP_DEPTH + right_fix
		self.move_arm(side, pose, True)
		self.move_gripper(side, 'close')
		pose.pose.position.z += VERTICAL_CLEARANCE + PIECE_DEPTH
		self.move_arm(side, pose, True)

	# Assumes piece is already in gripper. Moves arm to right above the square,
	# moves down into place, opens gripper, moves back up to original height. If
	# no square is passed in, then the piece will be removed from the board.
	def place_piece(self, side, square=None):
		col = -1
		row = 2
		if square != None:
			col = self.col(square)
			row = self.row(square)
		pose = self.pose_at(row, col)
		pose.pose.position.z += VERTICAL_CLEARANCE + 2 * PIECE_DEPTH + GRIPPER_DEPTH - GRIP_DEPTH
		self.move_arm(side, pose)
		pose.pose.position.z += -VERTICAL_CLEARANCE - PIECE_DEPTH + DROP_HEIGHT
		self.move_arm(side, pose, True)
		self.move_gripper(side, 'open')
		pose.pose.position.z += VERTICAL_CLEARANCE + GRIP_DEPTH - DROP_HEIGHT
		self.move_arm(side, pose, True)

	def pose_at(self, row, col):
		pose = PoseStamped()
		pose.pose.position = deepcopy(self.board_pose.pose.position)
		pose.pose.orientation = DOWN
		print 'calculating pose at %d, %d'%(row, col)
		pose.pose.position.x += SQUARE_SIZE * row + PICKUP_X_ADJUSTMENT
		pose.pose.position.y -= SQUARE_SIZE * col + PICKUP_Y_ADJUSTMENT
		return pose

	def move_gripper(self, side, action):
		gripper = self.left_gripper if side == 'left' else self.right_gripper
		# `True` makes the command blocking.
		if action == 'open':
			gripper.open(True)
		elif action == 'close':
			gripper.close(True)

	def move_arm(self, name, pose=None, slow=False):
		if pose == None:
			if DEMO:
				if name == 'left':
					pose = DEMO_LEFT_ARM_POSES[DEMO]
				else:
					pose = DEMO_RIGHT_ARM_POSES[DEMO]
			else:
				if name == 'left':
					pose = LEFT_ARM_DEFAULT_POSE
				else:
					pose = RIGHT_ARM_DEFAULT_POSE
		pose.header.frame_id = 'base'
		limb = self.left_arm if name == 'left' else self.right_arm
		limb.set_pose_target(pose)
		limb.set_start_state_to_current_state()
		plan = limb.plan()
		if not plan:
			raise Exception('Could not plan')
		if slow:
			ratio = 3
			for point in plan.joint_trajectory.points:
				point.time_from_start *= ratio
				point.velocities = [v / ratio for v in point.velocities]
		if not limb.execute(plan):
			raise Exception('Could not execute')

if __name__ == '__main__':
	Control()
