#!/usr/bin/env python

import tf
import math
import rospy
from math import sin, cos, pi
from std_msgs.msg import Float32MultiArray  
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


rospy.init_node('imu_publisher')

odom_pub = rospy.Publisher("odom_imu", Odometry, queue_size=50)
imu_pub = rospy.Publisher("Imu",Imu, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

def callback(data):
    global quat_w,quat_x,quat_y,quat_z
    global acc_x,acc_y,acc_z
    global roll_rate,pitch_rate,yaw_rate

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
    rospy.Subscriber('imu_data', Float32MultiArray , callback) 

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
vth = 0.0

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(1.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    listener()
    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    vx = vx + (acc_x*dt) # acc_x from imu
    vy = vy + (acc_y*dt) # acc_y from imu
    vth = yaw_rate         # yaw_rate from imu
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    odom_pub.publish(odom)
    
    #sensor_msgs/Imu
    imu_data = Imu()
    imu_data.header.stamp = current_time
    imu_data.header.frame_id = "odom"
    
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
    
    last_time = current_time
    r.sleep()
