#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Float32MultiArray


#get variables from your imu code
#just initializing some values
quat_w=1
quat_x=quat_y=quat_z=0.0
acc_x=0.2
acc_y=acc_z=0
roll_rate=pitch_rate=0
yaw_rate=0.2

def talker():
        global quat_w,quat_x,quat_y,quat_z
        global acc_x,acc_y,acc_z
        global roll_rate,pitch_rate,yaw_rate

	pub=rospy.Publisher('imu_data',Float32MultiArray,queue_size = 10)
	rospy.init_node('publish_example')
	r=rospy.Rate(1)
	mat=Float32MultiArray()
	mat.data=[0]*10

	while not rospy.is_shutdown():
		mat.data = [quat_w,quat_x,quat_y,quat_z,acc_x,acc_y,acc_z,roll_rate,pitch_rate,yaw_rate]
		pub.publish(mat)
		r.sleep()
if __name__ == '__main__':
	talker()		
