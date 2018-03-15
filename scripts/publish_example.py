#!/usr/bin/env python

import rospy
import roslib
from   std_msgs.msg import Float32MultiArray

#get variables from your imu code
#just initializing some values
quat_w    = 1.0
quat_x    = 0.0
quat_y    = 0.0 
quat_z    = 0.0
acc_x     = 0.2
acc_y     = 0.0
acc_z     = 0.0
roll_rate = 0.0
pitch_rate= 0.0
yaw_rate  = 0.0
yaw       = 77
c         = 0.0

def talker():
	global quat_w,quat_x,quat_y,quat_z,yaw
	global acc_x,acc_y,acc_z
	global roll_rate,pitch_rate,yaw_rate
      
	pub=rospy.Publisher('imu_data',Float32MultiArray,queue_size = 10)
	rospy.init_node('publish_example')
	r=rospy.Rate(5.0)
	mat=Float32MultiArray()
	mat.data=[0]*11

	while not rospy.is_shutdown():
		global c,i
		if c==2:
			acc_x=0
		
		mat.data = [quat_w,quat_x,quat_y,quat_z,acc_x,acc_y,acc_z,roll_rate,pitch_rate,yaw_rate,yaw]
		pub.publish(mat)
		yaw = yaw + 1
		c=c+1
		r.sleep()
if __name__ == '__main__':
	talker()		
