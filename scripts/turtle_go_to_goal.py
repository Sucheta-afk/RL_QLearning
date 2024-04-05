#!/usr/bin/python3

import math

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

x = 0
y = 0
theta = 0  # Radians

def pose_callback(data):
    global x, y, theta
    x = data.x
    y = data.y
    theta = data.theta
    
    
def goto_goal(x_goal, y_goal, pub):

    angular_threshold = 0.01
    linear_threshold = 0.1

    rate = rospy.Rate(1) # 10 Hz

    msg = Twist()

    # Angular adjustment
    while True:

        beta = 0.75

        angular_error = math.atan2(y_goal - y, x_goal - x) - theta

        if abs(angular_error) < angular_threshold:
            msg.angular.z = 0 # Theta = 0 (stopping the rotation)
            break

        msg.angular.z = beta * angular_error # Proportional control

        pub.publish(msg)

        rate.sleep()

    # Linear adjustment
    while True:

        alpha = 0.75

        displacement_error = math.sqrt((x_goal - x) ** 2 + (y_goal - y) ** 2)

        if displacement_error < linear_threshold:
            msg.linear.x = 0
            break

        msg.linear.x = alpha * displacement_error # Proportional control

        pub.publish(msg)

        rate.sleep()

    rospy.loginfo("Goal reached")


def normalise_angle(angle):
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    return angle


def main():

    rospy.init_node('turtle_go_to_goal', anonymous=True)

    rospy.Subscriber('turtle1/pose', Pose, pose_callback)

    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)


    x_goal = float(input("Enter x goal: "))
    y_goal = float(input("Enter y goal: "))

    goto_goal(x_goal, y_goal, pub)

    rospy.spin()


if __name__ == "__main__":
    main()