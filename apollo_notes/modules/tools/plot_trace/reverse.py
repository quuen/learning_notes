import rospy
import sys
import numpy as np
import math
from modules.map.relative_map.proto import navigation_pb2
from modules.localization.proto import localization_pb2
from modules.control.proto import control_cmd_pb2

"""
Usage:python reverse.py xxxx.csv
"""

GPS_X = list()
GPS_Y = list()
GPS_H = list()
map_msg = None
index = 0
gear = 2
previous_error = 0
integral = 0
flag = 1
Ok = 1
boundary_distance = 100000
H_error = 100000
STOP = 0

def AnglePid(error):
    global previous_error
    global integral
    kp = 32
    ki = 0
    kd = -0.6
    output_saturation_high = 100
    output_saturation_low = -100
    #diff = 0;
    #output = 0;
    diff = error - previous_error
    integral += error

    previous_error = error
    output = error * kp + integral * ki + diff * kd

    if output > output_saturation_high:
      output = output_saturation_high
    elif output < output_saturation_low:
      output = output_saturation_low
    else:
      return output

    return output

def localization_callback(localization_data):
    global GPS_X
    global GPS_Y
    global GPS_H
    global index
    global flag
    global Ok
    global boundary_distance
    global H_error
    global gear
    global STOP
    GPS_X.append(localization_data.pose.position.x)
    GPS_Y.append(localization_data.pose.position.y)
    GPS_H.append(localization_data.pose.heading)
    if Ok == 1:
       print "Target point is found!!!"
       FindNearstPoint(path_data)
       Ok = 0    
    #error = float("inf") 
    Cur_X = GPS_X[-1]
    Cur_Y = GPS_Y[-1]
    Cur_Heading = GPS_H[-1]
    X_error = Cur_X - path_data[index][0]
    Y_error = Cur_Y - path_data[index][1]
    H_error = flag * (Cur_Heading - path_data[index][5])
    
    if STOP == 0:
            print "heading_error=",H_error,"   index=",index,"   theta=",path_data[index][5],"  cur=",Cur_Heading
	    if map_msg is not None:
		for lane in map_msg.hdmap.lane:#only one lane
		    if 0 <= path_data[index][5] < 3.1415926:
		       if -3.1415926 <= H_error < 0:
		          print "Turn Right,Caution Right Boundary"
		          target = AnglePid(H_error)
			  #boundary_distance = MinDisToRight(lane)
			  boundary_distance = MinDisToLeft(lane)
		       else:
		          print "Turn Left,Caution Left Boundary"
		          H_error < -3.1415926 and H_error + 2*3.1415926 or H_error
		          target = AnglePid(H_error)
			  #boundary_distance = MinDisToLeft(lane)
			  boundary_distance = MinDisToRight(lane)
		    else:
		       if 0 <= H_error < 3.1415926:
		          target = AnglePid(H_error)
			  boundary_distance = MinDisToRight(lane)
		       else:
		          H_error > 3.1415926 and H_error - 2*3.1415926 or H_error
		          target = AnglePid(H_error) 
			  boundary_distance = MinDisToLeft(lane)
		#publish_control(110,0,flag*target,gear)     
		print "boundary_distance=",boundary_distance,"  H_error=",H_error
	        #if math.fabs(boundary_distance) < 0.8 or math.fabs(H_error) < 0.5:
		if math.fabs(boundary_distance) < 0.8 or math.fabs(H_error) < 0.5:
		   print "WARNING distance is so small!" 
		   #publish_control(0,0,0,gear)
		   if math.fabs(H_error) < 0.5:
		      print "MISSION SUCCESS!"
                      STOP = 1
		      publish_control(0,0,0,gear)
		   else:
		      gear = (gear == 2 and 1 or 2)
		      print "GEAR=",gear
		      flag = -flag
                      publish_control(110,0,flag*target,gear)
		      while (math.fabs(boundary_distance) < 0.8):
			    if STOP == 0:
				    if map_msg is not None:
					for lane in map_msg.hdmap.lane:#only one lane
					    if 0 <= path_data[index][5] < 3.1415926:
					       if -3.1415926 <= H_error < 0:
						  print "Turn Right,Caution Right Boundary"
						  target = AnglePid(H_error)
						  #boundary_distance = MinDisToRight(lane)
						  boundary_distance = MinDisToLeft(lane)
					       else:
						  print "Turn Left,Caution Left Boundary"
						  H_error < -3.1415926 and H_error + 2*3.1415926 or H_error
						  target = AnglePid(H_error)
						  #boundary_distance = MinDisToLeft(lane)
						  boundary_distance = MinDisToRight(lane)
					    else:
					       if 0 <= H_error < 3.1415926:
						  target = AnglePid(H_error)
						  boundary_distance = MinDisToRight(lane)
					       else:
						  H_error > 3.1415926 and H_error - 2*3.1415926 or H_error
						  target = AnglePid(H_error) 
						  boundary_distance = MinDisToLeft(lane) 
                else:
                   publish_control(110,0,flag*target,gear) 


def MinDisToLeft(lane):
    for curve in lane.left_boundary.curve.segment:
        if curve.HasField('line_segment'):
            left_distance = []
            for p in curve.line_segment.point:
                left_distance.append(math.sqrt((p.x)**2+(p.y)**2))
 		if len(left_distance)>100:
		   break
    return min(left_distance)

def MinDisToRight(lane):
    for curve in lane.right_boundary.curve.segment:
        if curve.HasField('line_segment'):
            right_distance = []
            for p in curve.line_segment.point:
                right_distance.append(math.sqrt((p.x)**2+(p.y)**2))
 		if len(right_distance)>100:
		   break
    return min(right_distance)



def map_callback(map_msg_pb):
    global map_msg
    map_msg = map_msg_pb


def FindNearstPoint(path_data):
       error = float("inf") 
       P_X = GPS_X[0]
       P_Y = GPS_Y[0]
       print path_data.shape[0]
       for i in range(0,path_data.shape[0]):
                    #print trace_data[i][0]
                    X_error = P_X - path_data[i][0]
                    Y_error = P_Y - path_data[i][1]
                    if error > math.sqrt(X_error**2+Y_error**2):
                       global index
                       index = i
                       error = math.sqrt(X_error**2+Y_error**2)
       #s.append(trace_data[index][2])
       #position_error.append(error)                

def publish_control(throttle,brake,steering,gear):#left(+) right(-)
	#controlcmd.header.sequence_num = sequence_num
	#sequence_num = sequence_num + 1
	controlcmd.header.timestamp_sec = rospy.get_time()
	controlcmd.throttle = throttle
	controlcmd.brake = brake
	controlcmd.steering_target = steering
	controlcmd.gear_location = gear
        print "gear=",controlcmd.gear_location
        #print "publish control!"
	control_pub.publish(controlcmd)


if __name__ == '__main__':
    handle = file(sys.argv[1], 'r')
    path_data = np.genfromtxt(handle, delimiter=',', names=True)
    handle.close()
    rospy.init_node("reverse", anonymous=True)
    controlcmd = control_cmd_pb2.ControlCommand()
    control_pub = rospy.Publisher(
            	     "/apollo/control", control_cmd_pb2.ControlCommand, queue_size=1)
    rospy.Subscriber('/apollo/relative_map',
                     navigation_pb2.MapMsg,
                     map_callback)
    rospy.Subscriber("/apollo/localization/pose", localization_pb2.LocalizationEstimate,
                     localization_callback)

    rospy.spin()
