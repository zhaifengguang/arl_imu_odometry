#!/usr/bin/env python

import tf
import rospy
import numpy             as     np
import matplotlib.pyplot as     plt
from   sensor_msgs.msg   import Imu
from   nav_msgs.msg      import Odometry
from   math              import sin, cos, pi , radians , degrees
from   std_msgs.msg      import Float32MultiArray
from   geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

rospy.init_node('arl_imu_odometry')
odom_pub         = rospy.Publisher("odom_imu", Odometry, queue_size=50)
imu_pub          = rospy.Publisher("Imu",Imu, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

s          = 0.0
x          = 0.0
y          = 0.0
th         = 0.0
vx         = 0.0
vy         = 0.0
vth        = 0.0
quat_w     = 0.0
quat_x     = 0.0
quat_y     = 0.0
quat_z     = 0.0
yaw        = 0.0
init_yaw   = 0.0
acc_x_raw  = 0.0
acc_y_raw  = 0.0
acc_z_raw  = 0.0
roll_rate  = 0.0
pitch_rate = 0.0
yaw_rate   = 0.0


def listener():
    rospy.Subscriber('imu_data', Float32MultiArray , callback)

def callback(data):
    global quat_w,quat_x,quat_y,quat_z
    global acc_x_raw,acc_y_raw,acc_z_raw
    global roll_rate,pitch_rate,yaw_rate
    global init_yaw , yaw , s

    quat_w    = data.data[0]
    quat_x    = data.data[1]
    quat_y    = data.data[2]
    quat_z    = data.data[3]
    acc_x_raw = data.data[4]
    acc_y_raw = data.data[5]
    acc_z_raw = data.data[6]
    roll_rate = data.data[7]
    pitch_rate= data.data[8]
    yaw_rate  = data.data[9]
    yaw       = data.data[10]

    if s == 0 :
        init_yaw = yaw
        s = 1
        print "initialized yaw as : ",init_yaw

r = rospy.Rate(5.0) 

while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    listener()
                           #####################
                           # nav_msgs.msg/odom #
    ###################################################################  
    dt      = 0.2 
    th      = init_yaw-yaw
    th      = radians(th)
    rot_mat = np.matrix(([cos(th),-1*sin(th)],[sin(th),cos(th)]))
    acc_mat = np.matrix(([acc_x_raw],[acc_y_raw]))
    acc_mat = rot_mat*acc_mat	 
    acc_x   = float(acc_mat[0][0])
    acc_y   = float(acc_mat[1][0])
    vx      = vx + (acc_x*dt) 
    vy      = vy + (acc_y*dt)
    vth     = yaw_rate    
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    x      += delta_x
    y      += delta_y
    
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
        )

    odom = Odometry()
    odom.header.stamp    = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose       = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    odom.child_frame_id  = "base_link"
    odom.twist.twist     = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    odom_pub.publish(odom)
    ###################################################################
    
        
                           ###################
                           # sensor_msgs/Imu #
    ###################################################################
    imu = Imu()
    imu.header.stamp    = current_time
    imu.header.frame_id = "odom"    
    imu.orientation.w = quat_w
    imu.orientation.x = quat_x
    imu.orientation.y = quat_y
    imu.orientation.z = quat_z
    imu.linear_acceleration.x = acc_x_raw
    imu.linear_acceleration.y = acc_y_raw
    imu.linear_acceleration.z = acc_z_raw
    imu.linear_acceleration_covariance[0] = -1
    imu.angular_velocity.x = roll_rate
    imu.angular_velocity.y = pitch_rate
    imu.angular_velocity.z = yaw_rate
    imu.angular_velocity_covariance[0] = -1
    imu_pub.publish(imu)    
    ###################################################################
    r.sleep()
