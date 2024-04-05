#!/usr/bin/env python3
import rospy
import yaml
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

x1 = 0
y1 = 0
theta1 = 0

       

def pose_callback(pose_msg:Pose):
    global x1, y1, theta1
    x1 = pose_msg.x 
    y1 = pose_msg.y 
    theta1 = pose_msg.theta 


def Goto_goal(x_goal, y_goal):
    global x1,y1,theta1
    msg = Twist()
    rate = rospy.Rate(10)

    while(True):
        angle_to_goal = normalize_angle(theta1 - math.atan2(y_goal - y1, x_goal - x1))

        alpha = -0.75
        
        if abs(angle_to_goal) > 0.01:
            msg.angular.z = alpha * angle_to_goal
        else:
            msg.angular.z = 0
            break
        pub.publish(msg)
        rate.sleep()

    while(True):
        distance = math.sqrt((x_goal - x1)**2 + (y_goal - y1)**2)
        beta = 0.75
        msg.linear.x = beta * distance
        pub.publish(msg)
        if distance < 0.1:
            break
        rate.sleep()

    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)
    rospy.loginfo("Goal reached")


def normalize_angle(angle):
    if (angle > math.pi):
        angle = angle - 2*math.pi
    if (angle < -math.pi):
        angle = angle + 2*math.pi
    return angle


if __name__ == "__main__":

        rospy.init_node("GoTo_Controller")
        pub = rospy.Publisher("/turtle1/cmd_vel",Twist, queue_size =10)
        sub = rospy.Subscriber("/turtle1/pose",Pose, callback = pose_callback)
        rospy.loginfo("Node has been started")

        userX = float(input("Please enter X goal: "))
        userY = float(input("Please enter Y goal: "))

        Goto_goal(userX,userY)
        print(x1,y1,theta1)
