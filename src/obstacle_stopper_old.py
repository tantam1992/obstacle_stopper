#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from obstacle_detector.msg import Obstacles
from sound_play.libsoundplay import SoundClient

# Global variables
linear_velocity = 0.0
obstacles_detected = False
last_stop_time = rospy.Time(0)
is_stopping = False

def nav_vel_callback(nav_vel_msg):
    global linear_velocity
    linear_velocity = nav_vel_msg.linear.x

def obstacles_callback(obstacles_msg):
    global obstacles_detected, is_stopping

    if is_stopping:
        # Skip processing if the robot is currently stopping
        return

    rospy.loginfo(obstacles_msg.circles)
    if linear_velocity > 0.01:
        check_obstacles(obstacles_msg, is_within_front_bounding_box)
    elif linear_velocity < -0.01:
        check_obstacles(obstacles_msg, is_within_back_bounding_box)
    else:
        check_obstacles(obstacles_msg, is_within_still_front_bounding_box)
        check_obstacles(obstacles_msg, is_within_still_back_bounding_box)

def is_within_front_bounding_box(x, y):
    return front_bounding_box[0] <= x <= front_bounding_box[1] and front_bounding_box[2] <= y <= front_bounding_box[3]

def is_within_back_bounding_box(x, y):
    return back_bounding_box[0] <= x <= back_bounding_box[1] and back_bounding_box[2] <= y <= back_bounding_box[3]

def is_within_still_front_bounding_box(x, y):
    return still_front_bounding_box[0] <= x <= still_front_bounding_box[1] and still_front_bounding_box[2] <= y <= still_front_bounding_box[3]

def is_within_still_back_bounding_box(x, y):
    return still_back_bounding_box[0] <= x <= still_back_bounding_box[1] and still_back_bounding_box[2] <= y <= still_back_bounding_box[3]

def check_obstacles(obs_msg, bounding_box_func):
    global obstacles_detected
    obstacles_detected = False
    for circle in obs_msg.circles:
        rospy.loginfo("Received circle at x: %.2f, y: %.2f, radius: %.2f", circle.center.x, circle.center.y, circle.true_radius)
        if bounding_box_func(circle.center.x, circle.center.y):
            if circle.true_radius > 0.1:
                rospy.loginfo("Received obstacle at x: %.2f, y: %.2f, radius: %.2f", circle.center.x, circle.center.y, circle.true_radius)
                obstacles_detected = True
                stop_robot()
                break    
        else:
            rospy.loginfo("No obstacles detected within the bounding box")        
    if not obstacles_detected:
        rospy.loginfo("No obstacles detected")

def stop_robot():
    global last_stop_time, is_stopping
    is_stopping = True
    stop_vel_msg = Twist()
    stop_vel_msg.linear.x = 0.0 
    stop_vel_msg.linear.y = 0.0
    stop_vel_msg.angular.z = 0.0

    # Publish stop message
    stop_vel_pub.publish(stop_vel_msg)
    last_stop_time = rospy.Time.now()
    rospy.loginfo("Robot paused for 2 seconds due to dynamic obstacle detected.")
    
    # Set a timer to reset the stopping state after stop_duration
    rospy.Timer(stop_duration, resume_movement, oneshot=True)

def resume_movement(event):
    global is_stopping
    is_stopping = False
    rospy.loginfo("Resuming movement")

def listener():
    global stop_duration, front_bounding_box, back_bounding_box, still_front_bounding_box, still_back_bounding_box
    rospy.init_node('obstacle_stopper')
    rospy.loginfo("obstacle_stopper started!")

    # Parameters
    stop_duration = rospy.Duration(rospy.get_param('~stop_duration', 2.0))
    front_bounding_box = rospy.get_param('~front_bounding_box', [0.2, 2.2, -0.4, 0.4])
    back_bounding_box = rospy.get_param('~back_bounding_box', [-2.0, -1.0, -0.4, 0.4])
    still_front_bounding_box = rospy.get_param('~still_front_bounding_box', [0.2, 1.2, -0.4, 0.4])
    still_back_bounding_box = rospy.get_param('~still_back_bounding_box', [-2.0, -1.0, -0.4, 0.4])

    # Publishers and subscribers
    global stop_vel_pub
    stop_vel_pub = rospy.Publisher('/stop_vel', Twist, queue_size=10)
    rospy.Subscriber('/nav_vel', Twist, nav_vel_callback)
    rospy.Subscriber('/obstacles', Obstacles, obstacles_callback)

    # rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep() 

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
