#!/usr/bin/env python
# ============================================================
# File Name: <raise_hands.py>
# Developer: TAKKIE HALIMI
# Email: takkie8halimi@gmail.com
#
# Description: 
#     <This code defines a ROS (Robot Operating System) node 
#      that gradually raises the arms of the virtual Pepper robot 
#      Here's a brief description of its purpose and functionality. 
#      The purpose of this script is to control the movement of Pepper's arms, 
#      specifically raising them slowly to a target position. This is achieved 
#      by sending incremental position commands to the robot's left and right 
#      shoulder pitch joints. It demonstrates smooth and controlled motion for robot arms.>
#
# Version History:
#     v1.0 - Initial version
#
# ============================================================
import rospy
from std_msgs.msg import Float64

def raise_hands_slowly():
    # Initialize the ROS node
    rospy.init_node('raise_hands_slowly', anonymous=True)

    # Publishers for left and right shoulder pitch controllers
    pub_left_shoulder = rospy.Publisher('/pepper/LShoulderPitch_position_controller/command', Float64, queue_size=10)
    pub_right_shoulder = rospy.Publisher('/pepper/RShoulderPitch_position_controller/command', Float64, queue_size=10)

    # Wait a bit to make sure the publishers are connected
    rospy.sleep(1)

    # Define initial position (arms down)
    initial_position = 0.0  # Adjust this based on the initial position of the robot arms
    target_position = -1.0   # Target position (arms up)

    # Set the number of steps to control the speed of movement
    steps = 30
    step_size = (initial_position - target_position) / steps
    rate = rospy.Rate(10)  # 10 Hz (adjust this rate to control the movement speed)

    # Gradually raise both arms
    rospy.loginfo("Raising both arms slowly...")
    current_position = initial_position

    while current_position > target_position and not rospy.is_shutdown():
        current_position -= step_size
        pub_left_shoulder.publish(Float64(current_position))
        pub_right_shoulder.publish(Float64(current_position))
        
        rate.sleep()  # Sleep for a while before the next step

    # Ensure final target position is reached
    pub_left_shoulder.publish(Float64(target_position))
    pub_right_shoulder.publish(Float64(target_position))

    rospy.loginfo("Arms raised!")

if __name__ == '__main__':
    try:
        raise_hands_slowly()
    except rospy.ROSInterruptException:
        pass

