#!/usr/bin/env python

# Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('optoforce_component')

import sys
import rospy
import math

import std_msgs.msg
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import Image
import PyKDL
import tf_conversions.posemath as pm

class OptoforceMarkers:

    # ****************** markers **********************

    def convertToRGB(self, value):
        r = 0
        g = 0
        b = 0
        if value<256:
            b = 255
            g = value
        elif value<512:
            b = 255-(value-256)
            g = 255
        elif value<768:
            g = 255
            r = (value-512)
        elif value<1024:
            r = 255
            g = 255-(value-768)
        else:
            r = 255
            g = 0
            b = 0
        result = []
        result.append(r)
        result.append(g)
        result.append(b)
        return result

    def spin(self):
        number_str = ['One', 'Two', 'Three'];

        while not rospy.is_shutdown():

            for i in range(3):
                if self.force[i] != None:
                    w = WrenchStamped()
                    w.header.stamp = self.force[i].header.stamp
                    w.header.frame_id = self.prefix + '_HandFinger' + number_str[i] + 'KnuckleThreeOptoforceBase'
                    w.wrench.force = self.force[i].vector
                    self.pub[i].publish(w)

            rospy.sleep(0.05)

    def force0_callback(self, data):
        self.force[0] = data;

    def force1_callback(self, data):
        self.force[1] = data;

    def force2_callback(self, data):
        self.force[2] = data;

    def __init__(self, prefix):
        self.prefix = prefix
        self.force = [None, None, None]
        self.pub = []
        for i in range(3):
            self.pub.append( rospy.Publisher('/' + self.prefix + '_hand/optoforce_marker'+str(i), WrenchStamped, queue_size=100) )

        rospy.Subscriber('/optoforce/force0_scaled', Vector3Stamped, self.force0_callback)
        rospy.Subscriber('/optoforce/force1_scaled', Vector3Stamped, self.force1_callback)
        rospy.Subscriber('/optoforce/force2_scaled', Vector3Stamped, self.force2_callback)
        rospy.sleep(1)

if __name__ == '__main__':
    a = []
    for arg in sys.argv:
        a.append(arg)

    if len(a) > 1:
        prefix = a[1]
    else:
        print "Usage: %s prefix"%a[0]
        exit(0)

    rospy.init_node(prefix+'_hand_optoforce_vis', anonymous=True)
    bhm = OptoforceMarkers(prefix)
    bhm.spin()

