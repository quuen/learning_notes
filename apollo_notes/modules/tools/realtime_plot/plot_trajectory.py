import math
import sys
import threading

import gflags
import matplotlib.pyplot as plt
import numpy as np
import rospy

import common.proto_utils as proto_utils
from item import Item
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.planning.proto.planning_pb2 import ADCTrajectory
from stitem import Stitem
from xyitem import Xyitem
from modules.perception.proto import perception_obstacle_pb2

VehicleLength = 2.85
HistLine2display = 2  #The number of lines to display
MaxSteerAngle = 470  #Maximum Steering Angle
SteerRatio = 16
WindowSize = 80

FLAGS = gflags.FLAGS
gflags.DEFINE_boolean('show_heading', True,
                      'Show heading instead of acceleration')
gflags.DEFINE_boolean('show_st_graph', False, 'Show st graph')

rospy.init_node('realtime_plot', anonymous=True)

new_planning = False
new_perception = False
planning_xy = []
perception_polygon = []

plt.ion()
def callback_planning(data):

            if len(data.trajectory_point) == 0:
                print data
                return

            x, y, speed, theta, kappa, acc, relative_time = np.array(
                proto_utils.flatten(data.trajectory_point,
                                    ['path_point.x',
                                     'path_point.y',
                                     'v',
                                     'path_point.theta',
                                     'path_point.kappa',
                                     'a',
                                     'relative_time']))
            theta = theta*180/3.1415926
            relative_time += data.header.timestamp_sec
            #plt.clf()
            #plt.plot(-y,x)
            #plt.pause(0.01)
            global new_planning
            new_planning = True
            global planning_xy
            planning_xy = []
            planning_xy.append(-y)
            planning_xy.append(x)


def callback_perception(data):
  global perception_polygon
  perception_polygon = []
  for ob in data.perception_obstacle:
    #print(ob.polygon_point)
    perception_polygon.append(ob.polygon_point)
    pass
  global new_perception
  new_perception = True
  
  
rospy.Subscriber(
        '/apollo/planning',
        ADCTrajectory,
        callback_planning,
        queue_size=3)

pubPerception = rospy.Subscriber('/apollo/perception/obstacles', 
                        perception_obstacle_pb2.PerceptionObstacles, 
                        callback_perception,
                        queue_size=10)
while not rospy.is_shutdown():
  if new_perception and new_planning:
    obs_poly = []
    for pol in perception_polygon:
      x = []
      y = []
      for point in pol:
        x.append(point.x)
        y.append(-point.y)
      obs_poly.append([x, y])
    plt.clf()
    print(planning_xy[0].shape)
    plt.scatter(planning_xy[0],planning_xy[1])
    plt.plot(planning_xy[0]+0.6,planning_xy[1])
    plt.plot(planning_xy[0]-0.6,planning_xy[1])

    #x05 = np.arange(0,20,1)
    y05 = np.arange(0,20,1)
    x05 = np.zeros(20)+5
    plt.plot(x05, y05,color='r')
    plt.plot(-x05, y05,color='r')
    plt.axis("equal")
    for ob in obs_poly:
      plt.plot(ob[1], ob[0])
    plt.pause(0.01)
    new_perception = False
    new_planning = False
    print('ok')




