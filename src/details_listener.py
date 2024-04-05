#!/usr/bin/python3

import rospy
from slot_17.msg import details1

def callback(data):
    rospy.loginfo(f"\nName:{data.name}\nAge:{data.age}\nscore:{data.score}\n")

def listener():
    
        rospy.init_node('listener', anonymous=True)
    
        rospy.Subscriber('student_details', details1, callback)

        rospy.loginfo("Started listening on topic '/student_details'")
    
        rospy.spin()

if __name__ == "__main__":
      listener()