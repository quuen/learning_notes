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

import rospy
import sys
import json
import matplotlib.pyplot sa plt
from modules.map.relative_map.proto import navigation_pb2


if __name__ == '__main__':
    #if len(sys.argv) != 4:
       # print "Usage:",sys.argv 
    navi_files = sys.argv[1]
    destination = sys.argv[2:4]
    rospy.init_node("navigator_offline", anonymous=True)
    navigation_pub = rospy.Publisher(
        "/apollo/navigation", navigation_pb2.NavigationInfo, queue_size=1)

    #search the nearest point to the designated point
    #x_origin = []
    #y_origin = []
    with open(navi_files,'r') as f:
        cnt = 0
        error = float('inf')
        for line in f:
            cnt += 1
            js_point = json.loads(line)
            lateral_error = destination[0] - js_point['x']
            longitunal_error = destination[1] - js_point['y']
            #x_origin.append(js_point['x'])
            #y_origin.append(js_point['y'])
            if error > math.sqrt(lateral_error**2+longitunal_error**2):
                index = cnt
                error = math.sqrt(lateral_error**2+longitunal_error**2)
    print "index = ",index
    #plt.plot(x_origin,y_origin,'k')



    # generate navigation info
    navigation_info = navigation_pb2.NavigationInfo()
    priority = 0
    for fdata in navi_files:
        print "processing " + fdata
        navigation_path = navigation_info.navigation_path.add()
        navigation_path.path_priority = priority
        priority += 1
        navigation_path.path.name = "navigation"

        f = open(fdata, 'r')
        #w_f = open("new_path.txt.smoothed",'w')
        # x_new = []
        # y_new = []
        cnt = 0
        for line in f:
            ### generate a new navigation line 
            #w_f.write(line)
            cnt += 1
            if cnt < 3:
                continue
            if cnt > inex:
                break
            json_point = json.loads(line)
            point = navigation_path.path.path_point.add()
            point.x = json_point['x']
            point.y = json_point['y']
            point.s = json_point['s']
            point.theta = json_point['theta']
            point.kappa = json_point['kappa']
            point.dkappa = json_point['dkappa']
            # x_new.append(point.x)
            # y_new.append(point.y)
        f.close()
        #w_f.close()
    #plt.plot(x_new,y_new,'r')
    #plt.show()


    # send navigation info to /apollo/navigation
    r = rospy.Rate(0.5)  # 0.5hz
    while not rospy.is_shutdown():
        r.sleep()
        navigation_pub.publish(navigation_info)
        r.sleep()
        break
