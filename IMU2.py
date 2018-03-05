#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray  
from sensor_msgs.msg import Imu



rospy.init_node('imu_publisher')
imu_pub = rospy.Publisher("Imu4",Imu, queue_size=50)

def callback(data):
    global acc_x , acc_y , yaw_rate
    quat_w=data.data[0]
    quat_x=data.data[1]
    quat_y=data.data[2]
    quat_z=data.data[3]
    acc_x =data.data[4]
    acc_y =data.data[5]
    acc_z =data.data[6]
    roll_rate =data.data[7]
    pitch_rate=data.data[8]
    yaw_rate  =data.data[9]
    
def listener():
    rospy.Subscriber('IMU',Float32MultiArray,callback)
 
imu_data = Imu()
r = rospy.Rate(1.0)

while not rospy.is_shutdown():
	    listener()
            imu_data.header.stamp = rospy.Time.now()
            imu_data.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            imu_data.orientation.w = quat_w
            imu_data.orientation.x = quat_x
            imu_data.orientation.y = quat_y
            imu_data.orientation.z = quat_z
            imu_data.linear_acceleration.x = acc_x
            imu_data.linear_acceleration.y = acc_y
            imu_data.linear_acceleration.z = acc_z
            imu_data.linear_acceleration_covariance[0] = -1
            imu_data.angular_velocity.x = roll_rate
            imu_data.angular_velocity.y = pitch_rate
            imu_data.angular_velocity.z = yaw_rate
            imu_data.angular_velocity_covariance[0] = -1
	    imu_pub.publish(imu_data)
	    r.sleep()

 
