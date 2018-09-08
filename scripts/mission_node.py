#!/usr/bin/env python
"""
Mission node
============
Provides a framwork to build basic behaviors used to make up a mission.
"""

import roslib
roslib.load_manifest('zoidberg_ros')
import rospy
import time
from mavros_msgs.srv import StreamRate, CommandBool, SetMode

class Mission:
    """Build up a mission with common task framework"""
    def __init__(self):
        """Setup possible tasks"""
        self.rate = rospy.Rate(10)
        s1 = rospy.ServiceProxy('/apm/set_stream_rate', StreamRate)
        s1(stream_id=0, message_rate=10, on_off=True)
        #self.mode_setter = rospy.ServiceProxy('/apm/set_mode', SetMode)
        self.armer = rospy.ServiceProxy('/apm/cmd/arming', CommandBool)
        self.curr_vision = None

    def do(self, isterm_cb, timeout, guidance_cb=None):
        """callbacks provide termination condition and can modify guidance"""
        tstart = time.time()

        while not isterm_cb(self):
            # timeout termination behvior
            if time.time() - tstart > timout:
                rospy.loginfo('Timeout preempted')
                break

            if guidance_cb is not None:
                guidance_cb(self)

            # send command feedback
            self.rate.sleep()

    def arm(self, is_armed):
        """Change the arm state of vehicle, set to Boolean value"""
        self.armer(value=is_armed)

    def _set_curr_vision(self, object_coords):
        """Set object x and y coordinates from vision"""
        self.curr_vision = object_coords

    def _set_curr_pose(self, dvl_output):
        """Set the currunt position, velocity and altitude from navigation"""
        pass


if __name__ == '__main__':
    rospy.init_node('mission')
    mission = Mission()
    rospy.spin()
