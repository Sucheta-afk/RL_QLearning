#!/usr/bin/python3

import rospy
from slot_17.msg import details

data = [
    {
        "name": "John",
        "age": 25,
        "score": 98
    },
    {
        "name": "Alison",
        "age": 30,
        "score": 95
    },
    {
        "name": "Peter",
        "age": 27,
        "score": 90
    },
    {
        "name": "Tom",
        "age": 22,
        "score": 85
    },
    {
        "name": "Jerry",
        "age": 28,
        "score": 80
    
    }
]

def talker():
    pub = rospy.Publisher('student_details', details, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    for i in data:
        msg = details()
        msg.name = i["name"]
        msg.age = i["age"]
        pub.publish(msg)
        rospy.loginfo(f'Published {i["name"]}\'s details')
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass