#! /usr/bin/env python

import rospy
from baxter_interface import Gripper
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import tf

class Piece:

	def __init__(self):
		rospy.init_node('baxter_chess_piece')
		self.right_gripper = Gripper('right')
		self.state_pub = rospy.Publisher('baxter_chess/piece/state', String, latch=True)
		self.move_pub = rospy.Publisher('baxter_chess/move/right_arm', PoseStamped)
		self.set_state('waiting')
		self.tfListener = tf.TransformListener()
		rospy.Subscriber('baxter_chess/piece/move', String, self.move_received)
		rospy.spin()

	def set_state(self, state):
		self.state = state
		self.state_pub.publish(self.state)

	def move_received(self, msg):
		name = msg.data
		time = self.tfListener.getLatestCommonTime('base', name)
		position, rotation = self.tfListener.lookupTransform('base', name, time)
		pose = PoseStamped()
		pose.pose.position = Point(position[0], position[1], position[2] + 0.1)
		pose.pose.orientation = Quaternion(0, -1, 0, 0)
		self.move_pub.publish(pose)
		pose.pose.position.z -= 0.05
		self.move_pub.publish(pose)
		rospy.sleep(20)
		self.right_gripper.close(True)
		pose.pose.position.z += 0.1
		self.move_pub.publish(pose)

if __name__ == '__main__':
	Piece()