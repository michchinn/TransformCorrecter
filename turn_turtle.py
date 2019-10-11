#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import math
from sklearn.metrics import mean_squared_error
from sklearn.linear_model import LinearRegression
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3

VICON_TOPIC = '/adjusted/vicon/BUGS/BUGS'
TRANSFORMED_TOPIC = 'turtle_transformed_orientation'

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

count = 0

vicon = []
orientations = []

transform_pub = rospy.Publisher(TRANSFORMED_TOPIC, TransformStamped, queue_size=10)

def shutdown_sequence():
    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    pub.publish(twist)


def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input


def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)

    return vel


def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)

    return vel


def GoalCallBack(ros_data):
    global goal_x
    global goal_y
    goal_x = ros_data.transform.translation.x
    goal_y = ros_data.transform.translation.y

def PosCallBack(ros_data):
    global pos_x
    global pos_y
    global theta

    pos_x = ros_data.transform.translation.x
    pos_y = ros_data.transform.translation.y
	
    orientation = ros_data.transform.rotation
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    theta = -(yaw+np.random.normal(loc=0.0,scale=0.1))

    transformData = TransformStamped(
        header = Header(
            frame_id = ros_data.header.frame_id),
        transform = Transform(
            translation=Vector3(0.0,0.0,theta)))

    transform_pub.publish(transformData)
               
    print("new theta: %s"%theta)

if __name__=="__main__":
    global goal_x
    global goal_y
    global pos_x
    global pos_y
    global theta

    goal_x = 0
    goal_y = 0
    pos_x = 0
    pos_y = 0
    theta = 0
    
    #plt.plot(orientations)
    #plt.plot(vicon)
    #plt.ylabel("orientation (radians)")
    #plt.xlabel("message index")
    #plt.draw()

    rospy.init_node('turtle_chaser')
    rospy.on_shutdown(shutdown_sequence)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    #transform_pub = rospy.Publisher(TRANSFORMED_TOPIC, float, queue_size=10)
    #subscriber = rospy.Subscriber('vicon/WANDV2/WANDV2', TransformStamped, GoalCallBack)
    subscriber = rospy.Subscriber(VICON_TOPIC, TransformStamped, PosCallBack)

    set_rate = 10
    dt = 1.0/set_rate
    rate = rospy.Rate(set_rate)

    # PID
    Kp = 1.5
    Ki = 0.0
    Kd = 0.1

    previous_error = 0
    integral = 0

    try:
        while not rospy.is_shutdown():
            desired_heading = 0
            current_heading = theta

            print("Desired Heading: " + str(desired_heading))
            #print("Current Heading: " + str(current_heading))

            error = desired_heading - current_heading
            error = math.atan2(math.sin(error), math.cos(error))

            integral = integral + (error * dt)
            derivate = (error - previous_error) / dt
            theta = Kp*error + Ki*integral + Kd*derivate

            previous_error = error


            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = theta
            pub.publish(twist)
               
            #count = count +1
            #if count%10 == 0:
            #    plt.plot(orientations)
            #    plt.plot(vicon)
            #    plt.pause(0.01)
            #plt.draw()
            rate.sleep()

    except Exception as e:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        print e

    shutdown_sequence()
