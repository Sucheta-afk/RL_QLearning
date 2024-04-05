#!/usr/bin/python3

import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

x = 0
y = 0
theta = 0  # Radians

def pose_callback(data):
    global x, y, theta

    x=data.pose.pose.position.x
    y=data.pose.pose.position.y

    q_x=data.pose.pose.orientation.x
    q_y=data.pose.pose.orientation.y
    q_z=data.pose.pose.orientation.z
    q_w=data.pose.pose.orientation.w
    

    roll,pitch,yaw=euler_from_quaternion((q_x, q_y, q_z, q_w))
    theta=yaw
    
    
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
    pub.publish(msg)

    # Linear adjustment
    while True:

        alpha = 0.75

        displacement_error = math.sqrt((x_goal - x) ** 2 + (y_goal - y) ** 2)

        if displacement_error < linear_threshold:
            msg.linear.x = 0
            break

        msg.linear.x = alpha * displacement_error # Proportional control

    

        rate.sleep()
    pub.publish(msg)

    rospy.loginfo("Goal reached")


def normalise_angle(angle):
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    return angle


def main():

    rospy.init_node('turtle_go_to_goal', anonymous=True)

    rospy.Subscriber('atom/odom', Odometry, pose_callback)

    pub = rospy.Publisher('atom/cmd_vel', Twist, queue_size=10)


    x_goal = float(input("Enter x goal: "))
    y_goal = float(input("Enter y goal: "))

    goto_goal(x_goal, y_goal, pub)

    rospy.spin()


if __name__ == "__main__":
    main()

