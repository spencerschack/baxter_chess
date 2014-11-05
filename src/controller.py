import rospy
import baxter_interface

class Controller:

	def __init__(self):
		self.state = 'initializingRobot'
		while True:
			# Calls the method 'state<State>'
			getattr(self, 'state' + self.state.capitalize())()

	def stateInitializingRobot(self):
		baxter_interface.RobotEnable().enable()
		self.initGrippers()
		self.initCameras()
		self.initLimbs()
		self.state = 'locatingBoard'

	def stateLocatingBoard(self):
		# Pan head and find the chessboard
		self.headCamera.open()
		self.state = 'initializingBoard'

	def stateInitializingBoard(self):
		# Place the pieces
		if self.isTurn:
			self.state = 'pondering'
		else:
			self.state = 'waiting'

	def statePondering(self):
		# Pass piece positions to chess engine and wait for response
		if self.gameOver:
			self.state = 'terminating'
		else:
			self.state = 'moving'

	def stateWaiting(self):
		# Wait until player moves.
		self.state = 'moving'

	def stateMoving(self):
		# Get coordinates of piece to move, pass to moveit, and command limb.
		self.state = 'waiting'

	def stateTerminating():
		# Do whatever ROS needs to do to shutdown
		pass

	def initGrippers(self):
		self.leftGripper = baxter_interface.Gripper('left')
		self.rightGripper = baxter_interface.Gripper('right')
		# These calibrate calls are blocking.
		self.leftGripper.calibrate()
		self.rightGripper.calibrate()

	def initCameras(self):
		# These are closed by default. You can only have two open at a time.
		self.headCamera = baxter_interface.CameraController('head_camera')
		self.leftHandCamera = baxter_interface.CameraController('left_hand_camera')
		self.rightHandCamera = baxter_interface.CameraController('right_hand_camera')
		resolution = baxter_interface.CameraController.MODES[0]
		self.headCamera.resolution = resolution
		self.leftHandCamera.resolution = resolution
		self.rightHandCamera.resolution = resolution

	def initLimbs(self):
		self.leftLimb = baxter_interface.Limb('left')
		self.rightLimb = baxter_interface.Limb('right')

if __name__ == '__main__':
	Controller()
