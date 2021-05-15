#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

def gazeboModelCallback(data):
    global gpsMsg

    objList = data.name
    poseList = data.pose
    
    jackal0_GPS = poseList[objList.index('jackal0')]

    gpsMsg.position.x = jackal0_GPS.position.x
    gpsMsg.position.y = jackal0_GPS.position.y
    gpsMsg.position.z = jackal0_GPS.position.z
    gpsMsg.orientation.x = jackal0_GPS.orientation.x
    gpsMsg.orientation.y = jackal0_GPS.orientation.y
    gpsMsg.orientation.z = jackal0_GPS.orientation.z
    gpsMsg.orientation.w = jackal0_GPS.orientation.w

def main():
    rospy.sleep(5)
    global gpsMsg
    gpsMsg = Pose()
    gps_pub = rospy.Publisher('/jackal0/global_pos', Pose, queue_size=10)
    goal_pub = rospy.Publisher('/jackal0/goal_pos', Pose, queue_size=10)
    rospy.Subscriber("/gazebo/model_states", ModelStates, gazeboModelCallback)
    rospy.init_node('global_pos', anonymous=True)
    rate = rospy.Rate(10)
    goalMsg = Pose()
    goalMsg.position.x = 11
    goalMsg.position.y = 0
    goalMsg.position.z = 0
    goalMsg.orientation.x = 0
    goalMsg.orientation.y = 0
    goalMsg.orientation.z = 0
    goalMsg.orientation.w = 1

    while not rospy.is_shutdown():
        gps_pub.publish(gpsMsg)
	goal_pub.publish(goalMsg)
        rate.sleep()

if __name__ == "__main__":
    main()
