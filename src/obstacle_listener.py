#!/usr/bin/env python3
 
import rospy
from obstacle_detector.msg import Obstacles

def obstacles_callback(obstacle_msg):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    rospy.loginfo(obstacle_msg)

def check_obstacles(obs_msg, bounding_box_func):
    global obstacles_detected
    obstacles_detected = False
    for circle in obs_msg.circles:
        rospy.loginfo("Received circle at x: %.2f, y: %.2f, radius: %.2f", circle.center.x, circle.center.y, circle.true_radius)
        if bounding_box_func(circle.center.x, circle.center.y):
            if circle.true_radius > 0.1:
                rospy.loginfo("Received obstacle at x: %.2f, y: %.2f, radius: %.2f", circle.center.x, circle.center.y, circle.true_radius)
                obstacles_detected = True
                # if can_stop():
                # stop_robot()
                break
                # obs_msg.circles.clear()
                
        else:
            rospy.loginfo("No obstacles detected within the bounding box")        
    if not obstacles_detected:
        rospy.loginfo("No obstacles detected")

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('obsttacle_listener', anonymous=True)

    rospy.Subscriber('/obstacles', Obstacles, obstacles_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()