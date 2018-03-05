#!/usr/bin/env python

import math
from math import sin, cos, pi
import rospy
from std_msgs.msg import Float32MultiArray  
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom4", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

def callback(data):
    global acc_x , acc_y , yaw_rate
    acc_x=data.data[0]
    acc_y=data.data[1]
    yaw_rate=data.data[2]

def listener():
    rospy.Subscriber('IMU', Float32MultiArray , callback) 

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
    global acc_x ,acc_y ,yaw_rate
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

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
