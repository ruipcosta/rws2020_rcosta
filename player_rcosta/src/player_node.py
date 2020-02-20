#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def callback(msg):

    print("received a message!")


def main():
    print("Hello player node");

    rospy.init_node('rcosta', anonymous=False)

    rospy.Subscriber("chatter",String, callback)

    rospy.spin()

if __name__ == "__main__":
    main()

