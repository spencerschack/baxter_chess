import rospy
from baxter_interface import *
from math import *

class Controller:

	def __init__(self):
		self.state = 'initializing_robot'
		while True:
			# Calls the method 'state_<state>'
			getattr(self, 'state_' + self.state)()

	def state_initializing_robot(self):
		RobotEnable().enable()

		# GRIPPERS
		self.left_gripper = Gripper('left')
		self.right_gripper = Gripper('right')
		# These calibrate calls are blocking.
		self.left_gripper.calibrate()
		self.right_gripper.calibrate()

		# CAMERAS
		# These are closed by default. You can only have two open at a time.
		self.head_camera = CameraController('head_camera')
		self.left_hand_camera = CameraController('left_hand_camera')
		self.right_hand_camera = CameraController('right_hand_camera')
		resolution = CameraController.MODES[0]
		self.head_camera.resolution = resolution
		self.left_hand_camera.resolution = resolution
		self.right_hand_camera.resolution = resolution

		# EXTREMITIES
		self.left_limb = Limb('left')
		self.right_limb = Limb('right')
		self.head = Head()

		self.state = 'initializing_locating_board'

	def state_initializing_locating_board(self):
		self.head_camera.open()
		# Head pan range is [-PI2, PI2]
		self.head.set_pan(-2 * pi)
		self.state = 'locating_board'

	def state_locating_board(self):
		board_angle = 0 # TODO: find board and angle relative to camera
		if -10 < board_angle < 10:
			self.board_position = None # TODO: calculate position
			self.board_orientation = None # TODO: calculate orientation
			self.state = 'initializing_board'
		elif self.head.pan() < 2 * pi  + HEAD_PAN_ANGLE_TOLERANCE:
			raise Error('Could not find chessboard in field of view.')
		else:
			new_angle = self.head.pan() + HEAD_PAN_ANGLE_TOLERANCE
			self.head.set_pan(new_angle)

	def state_initializing_board(self):
		# TODO: Place the pieces
		if !is_moving:
			if piece_is_out_of_place:
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
	Controller()
