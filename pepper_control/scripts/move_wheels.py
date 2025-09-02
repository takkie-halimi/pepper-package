#!/usr/bin/env python
# ============================================================
# File Name: <move_wheels.py>
# Developer: TAKKIE HALIMI
# Email: takkie8halimi@gmail.com
#
# Description: 
#     <This code is a ROS (Robot Operating System) node 
#      written in Python for simulating and publishing 
#      odometry data for the virtual Pepper robot Its 
#      primary purpose is to simulate Pepper's circular motion 
#      and generate odometry data, which is useful for 
#      navigation and localization tasks>
#
# Version History:
#     v1.0 - Initial version
#
# ============================================================
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import tf
import math

# Initialize the ROS node
rospy.init_node('pepper_odometry_publisher')

# Publishers
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
wheel_fl_pub = rospy.Publisher('/pepper/WheelFL_position_controller/command', Float64, queue_size=10)
wheel_fr_pub = rospy.Publisher('/pepper/WheelFR_position_controller/command', Float64, queue_size=10)
wheel_b_pub = rospy.Publisher('/pepper/WheelB_position_controller/command', Float64, queue_size=10)

# Parameters
wheel_radius = 0.1  # Radius of each wheel in meters
wheel_base = 0.5    # Distance between front wheels in meters
speed = 0.5         # Base speed for forward motion
radius = 1.0        # Radius of the circular path (in meters)
angular_speed = speed / radius  # Angular speed for circular motion

# Time tracking
last_time = rospy.Time.now()

# Initial pose
x = 0.0
y = 0.0
theta = 0.0  # Orientation (yaw)

# Set loop rate
rate = rospy.Rate(20)  # 20 Hz

# Main loop
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()

    # Calculate the different wheel velocities to induce circular motion
    v_fl = speed  # Front-left wheel
    v_fr = speed + (angular_speed * wheel_base / 2.0)  # Front-right wheel (faster)
    v_b = speed  # Back wheel (same speed as front-left)

    # Compute linear and angular velocities
    vx = (v_fl + v_fr + v_b) / 3.0  # Average linear velocity
    vth = (v_fr - v_fl) / wheel_base  # Angular velocity (yaw rate)

    # Update robot pose
    delta_x = vx * math.cos(theta) * dt
    delta_y = vx * math.sin(theta) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    theta += delta_th

    # Normalize theta to stay within [-pi, pi]
    theta = math.atan2(math.sin(theta), math.cos(theta))

    # Create the odometry message
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = 'odom'

    # Set the position
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0.0
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    odom.pose.pose.orientation.x = odom_quat[0]
    odom.pose.pose.orientation.y = odom_quat[1]
    odom.pose.pose.orientation.z = odom_quat[2]
    odom.pose.pose.orientation.w = odom_quat[3]

    # Set the velocity
    odom.child_frame_id = 'base_link'
    odom.twist.twist.linear.x = vx
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.angular.z = vth

    # Publish the odometry message
    odom_pub.publish(odom)

    # Send transform from base_link to odom
    odom_broadcaster = tf.TransformBroadcaster()
    odom_broadcaster.sendTransform(
        (x, y, 0.0),  # Position (in odom frame)
        odom_quat,    # Orientation (in odom frame)
        current_time, # Timestamp
        'base_link',  # Child frame (robot base)
        'odom'        # Parent frame (world/odom frame)
    )

    last_time = current_time

    # Publish wheel commands (for circular motion)
    wheel_fl_pub.publish(Float64(v_fl))
    wheel_fr_pub.publish(Float64(v_fr))
    wheel_b_pub.publish(Float64(v_b))

    # Sleep to maintain loop rate
    rate.sleep()

