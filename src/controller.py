import rospy
import baxter_interface

class Controller:

	def __init__(self):
		baxter_interface.RobotEnable().enable()
		self.initGrippers()
		self.initCameras()

	def initGrippers():
		self.leftGripper = baxter_interface.Gripper('left')
		self.rightGripper = baxter_interface.Gripper('right')
		# These calibrate calls are blocking.
		self.leftGripper.calibrate()
		self.rightGripper.calibrate()

	def initCameras():
		# These are closed by default. You can only have two open at a time.
		self.headCamera = baxter_interface.CameraController('head_camera')
		self.leftHandCamera = baxter_interface.CameraController('left_hand_camera')
		self.rightHandCamera = baxter_interface.CameraController('right_hand_camera')
		resolution = baxter_interface.CameraController.MODES[0]
		self.headCamera.resolution = resolution
		self.leftHandCamera.resolution = resolution
		self.rightHandCamera.resolution = resolution

if __name__ == '__main__':
	Controller()
