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
dt		   = 0.2
th         = 0.0
vx_raw     = 0.0
vy_raw     = 0.0
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
P_new	   = np.matrix(([0,0],[0,0]))
P_old      = np.matrix(([0,0],[0,0]))
Pv_new     = np.matrix(([0,0],[0,0]))
Pv_old	   = np.matrix(([0,0],[0,0]))
Fx         = np.matrix(([1,0],[0,1]))
Fv 		   = np.matrix(([1,0],[0,1]))
Fu 		   = np.matrix(([dt,0],[0,dt]))
Q          = np.matrix(([(0.019*0.019),0],[0,(0.019*0.019)]))
Pose_cov   = np.array([0.0]*36)
Twist_cov  = np.array([0.0]*36)

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
    th      = init_yaw-yaw
    th      = radians(th)
    
    rot_mat = np.matrix(([cos(th),-1*sin(th)],[sin(th),cos(th)]))
    #acc_mat = np.matrix(([acc_x_raw],[acc_y_raw]))
    	 
    vx_raw  = vx_raw + (acc_x_raw*dt) 
    vy_raw  = vy_raw + (acc_y_raw*dt)
    
    v_mat   = np.matrix(([vx_raw],[vy_raw]))
    v_mat   = rot_mat*v_mat
    
    vx      = v_mat[0,0]
    vy      = v_mat[1,0]
    vth     = yaw_rate    
    
    x      += (vx*dt)
    y      += (vy*dt)
    
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
        )
    
    if acc_x_raw != 0.0 :
	    Pv_new = (Fv*Pv_old*Fv.T) + (Fu*Q*Fu.T)
    
    P_new  = (Fx*P_old*Fx.T)  + (Fv*Pv_new*Fu.T)
    
    Pose_cov[0]  = P_new[0,0]
    Pose_cov[1]  = P_new[0,1]
    Pose_cov[6]  = P_new[1,0]
    Pose_cov[7]  = P_new[1,1]
    
    Twist_cov[0] = Pv_new[0,0]
    Twist_cov[1] = Pv_new[0,1]
    Twist_cov[6] = Pv_new[1,0]
    Twist_cov[7] = Pv_new[1,1]

    odom = Odometry()
    odom.header.stamp    = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose       = Pose(Point(x, y, 0.), Quaternion(*odom_quat)) 
    odom.pose.covariance = Pose_cov
    odom.child_frame_id  = "base_link"
    odom.twist.twist     = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    odom.twist.covariance= Twist_cov
    odom_pub.publish(odom)
    
    Pv_old = Pv_new
    P_old  = P_new 
    ###################################################################
    r.sleep()
