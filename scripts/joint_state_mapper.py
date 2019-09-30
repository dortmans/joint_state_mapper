#!/usr/bin/env python

"""This node maps joint states"""

# For Python2/3 compatibility
from __future__ import division

import rospy
import sys
from sensor_msgs.msg import JointState

__author__ = "Eric Dortmans"

class JointStateMapper:
    """Map joint states
    
    """

    def __init__(self):
        self.pose_subscriber = rospy.Subscriber("joint_states", 
                          JointState, self.joint_states_callback, queue_size=1)
        self.states_pub = rospy.Publisher("joint_states_mapped", 
                          JointState, queue_size=1)      

    def joint_states_callback(self, joint_state_msg):
        """Handle JointState message
        """
        pass


def main(args):
    rospy.init_node('joint_state_mapper', anonymous=True)
    mapper = JointStateMapper()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
