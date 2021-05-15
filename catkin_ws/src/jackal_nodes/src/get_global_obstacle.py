#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseArray

def gazeboModelCallback(data):
    global obs_list
    obs_list = PoseArray()
    objList = data.name
    poseList = data.pose
    for i in range (1, 7): #(1, 4) for 3 obstacles
        obs_GPS = poseList[objList.index('jackal'+str(i))]
        gpsMsg = Pose()
        gpsMsg.position.x = obs_GPS.position.x
        gpsMsg.position.y = obs_GPS.position.y
        gpsMsg.position.z = obs_GPS.position.z
        gpsMsg.orientation.x = obs_GPS.orientation.x
        gpsMsg.orientation.y = obs_GPS.orientation.y
        gpsMsg.orientation.z = obs_GPS.orientation.z
        gpsMsg.orientation.w = obs_GPS.orientation.w
        obs_list.poses.append(gpsMsg)

def main():
    rospy.sleep(5)
    global obs_list
    obs_list = PoseArray()
    gps_pub = rospy.Publisher('/obstacle/global_pos', PoseArray, queue_size=10)
    rospy.Subscriber("/gazebo/model_states", ModelStates, gazeboModelCallback)
    rospy.init_node('obs_pos', anonymous=True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        gps_pub.publish(obs_list)
        rate.sleep()

if __name__ == "__main__":
    main()
