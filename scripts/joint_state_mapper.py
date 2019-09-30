#!/usr/bin/env python

"""This node maps joint states"""

# For Python2/3 compatibility
from __future__ import print_function
from __future__ import division

import rospy
import sys
from sensor_msgs.msg import JointState

__author__ = "Eric Dortmans"

class JointStateMapper:
    """Map joint states
    
    """

    def __init__(self):
        self.joint_states_sub = rospy.Subscriber("joint_states", 
                          JointState, self.joint_states_callback, queue_size=1)
        self.joint_states_pub = rospy.Publisher("joint_states_mapped", 
                          JointState, queue_size=1)
        
        self.mapping = rospy.get_param("~mapping", None)
        #for m in self.mapping:
        #  print( "{f} --> {t}".format(f=m['from'], t=m['to']) )
       

    def joint_states_callback(self, joint_state):
        """Handle JointState message
        """
        mapped_joint_state = JointState()
        #mapped_joint_state.header = Header()
        #mapped_joint_state.header.stamp = rospy.Time.now()
        mapped_joint_state.header = joint_state.header
        mapped_joint_state.name = []
        mapped_joint_state.position = []
        for m in self.mapping:
          if m['from'] in joint_state.name:
            index = joint_state.name.index(m['from'])
            mapped_joint_state.name.append(m['to'])
            mapped_joint_state.position.append(joint_state.position[index])
        
        self.joint_states_pub.publish(mapped_joint_state)
        #print(mapped_joint_state)

def main(args):
    rospy.init_node('joint_state_mapper', anonymous=True)
    mapper = JointStateMapper()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
