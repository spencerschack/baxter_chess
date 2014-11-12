#! /usr/bin/env python

import rospy
import tf
from ar_track_alvar.msg import AlvarMessages, AlvarMessage

listener = None

def callback(markers):
    for marker in markers:
        piece = "ar_marker_" + str(marker.id)
        try:
            (trans,rot) = listener.lookupTransform(piece, 'base', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

if __name__ == '__main__':
    rospy.init_node('chess_piece_locator')
    listener = tf.TransformListener()
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
    rospy.spin()
