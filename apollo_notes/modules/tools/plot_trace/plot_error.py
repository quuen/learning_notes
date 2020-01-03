#!/usr/bin/env python

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

import csv
import sys
import math
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import rospy
from std_msgs.msg import String

from modules.canbus.proto import chassis_pb2
from modules.localization.proto import localization_pb2

GPS_X = list()
GPS_Y = list()
DRIVING_MODE_TEXT = ""
CHASSIS_TOPIC = "/apollo/canbus/chassis"
LOCALIZATION_TOPIC = "/apollo/localization/pose"
IS_AUTO_MODE = False
position_error = list()
s = list()

def chassis_callback(chassis_data):
    global IS_AUTO_MODE
    if chassis_data.driving_mode == chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE:
        IS_AUTO_MODE = True
    else:
        IS_AUTO_MODE = False

    DRIVING_MODE_TEXT = str(chassis_data.driving_mode)


def localization_callback(localization_data):
    global GPS_X
    global GPS_Y
    global IS_AUTO_MODE
    if IS_AUTO_MODE:
        GPS_X.append(localization_data.pose.position.x)
        GPS_Y.append(localization_data.pose.position.y)


def setup_listener():
    rospy.init_node('plot_error', anonymous=True)
    rospy.Subscriber(CHASSIS_TOPIC, chassis_pb2.Chassis, chassis_callback)
    rospy.Subscriber(LOCALIZATION_TOPIC, localization_pb2.LocalizationEstimate,
                     localization_callback)


def update(frame_number):
    global GPS_X
    global GPS_Y
    global position_error
    if IS_AUTO_MODE and len(GPS_X) > 1:
       error = 100000000000
       P_X = GPS_X[-1]
       P_Y = GPS_Y[-1]
       for i in range(0,trace_data.shape[0]):
                    #print trace_data[i][0]
                    lateral_error = P_X - trace_data[i][0]
                    longitunal_error = P_Y - trace_data[i][1]
                    if error > math.sqrt(lateral_error**2+longitunal_error**2):
                       index = i
                       error = math.sqrt(lateral_error**2+longitunal_error**2)
       s.append(trace_data[index][2])
       position_error.append(error)                
       print len(position_error),position_error[-1]
       #min_len = min(len(GPS_X), len(GPS_Y)) - 1
       print len(s),len(position_error)
       ERROR_LINE.set_data(s[:-1],position_error[:-1])



if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(
        description=
        """A visualization tool that can plot a manual driving trace produced by the rtk_player tool,
        and plot the autonomous driving trace in real time.
        The manual driving trace is the blue lines, and the autonomous driving trace is the red lines.
        It is visualization a way to verify the precision of the autonomous driving trace.
        If you have a rosbag, you can play the rosbag and the tool will plot the received localization
        message in realtime. To do that, start this tool first with a manual driving trace, and then
        play rosbag use another terminal with the following command [replace your_bag_file.bag to your
        own rosbag file]: rosbag play your_bag_file.bag
        """)
    parser.add_argument(
        "trace",
        action='store',
        type=str,
        help='the manual driving trace produced by rtk_player')

    args = parser.parse_args()
  
    xmajorLocator = MultipleLocator(5)
    xminorLocator = MultipleLocator(1)

    #ymajorLocator = MultipleLocator(5)
    yminorLocator = MultipleLocator(0.1)

    fig, ax = plt.subplots()
    handle = file(args.trace, 'r')
    trace_data = np.genfromtxt(handle, delimiter=',', names=True)#生成一个二维数组
    handle.close()

    x_min = min(trace_data['s'])
    x_max = max(trace_data['s'])

    setup_listener()
    #print type(trace_data['s'].tolist()),type(position_error)
    ERROR_LINE, = ax.plot([],[], 'r', linewidth=3, label="position error")
    plt.xlim((x_min,x_max))
    plt.ylim((0,1.5))

    ani = animation.FuncAnimation(fig, update, interval=100)
    
    ax.xaxis.set_major_locator(xmajorLocator)
    #ax.yaxis.set_major_locator(ymajorLocator)
    ax.xaxis.set_minor_locator(xminorLocator)
    ax.yaxis.set_minor_locator(yminorLocator)
    plt.xlabel('s(m)')
    plt.ylabel('error(m)')
    plt.grid(linestyle='-.')
    plt.legend()
    plt.show()
