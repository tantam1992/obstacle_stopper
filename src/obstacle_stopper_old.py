#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from obstacle_detector.msg import Obstacles
from sound_play.libsoundplay import SoundClient
from std_srvs.srv import Empty
from yolo_v8_ros_msgs.msg import BoundingBoxes
from time import sleep

# Sound client for playing audio
soundhandle = SoundClient()

# Global variables
linear_velocity = 0.0
# person_detected = False
obstacles_detected = False
last_stop_time = rospy.Time(0)

def nav_vel_callback(nav_vel_msg):
    global linear_velocity
    linear_velocity = nav_vel_msg.linear.x

def obstacles_callback(obstacles_msg):
    global obstacles_detected
    rospy.loginfo(obstacles_msg)
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
        # rospy.loginfo("Received circle at x: %.2f, y: %.2f, radius: %.2f", circle.center.x, circle.center.y, circle.true_radius)
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

def stop_robot():
    global last_stop_time
    # soundhandle.playWave(warning_sound_path)

    stop_vel_msg = Twist()
    stop_vel_msg.linear.x = 0.0 
    stop_vel_msg.linear.y = 0.0
    stop_vel_msg.angular.z = 0.0
    last_stop_time = rospy.Time.now()
    # stopped = True
    # consecutive_stop_count += 1
    rospy.loginfo("Robot paused for 2 seconds due to dynamic obstacle detected.")
    rate = rospy.Rate(10)
    while rospy.Time.now() - last_stop_time < stop_duration:
        stop_vel_pub.publish(stop_vel_msg)
        rate.sleep()
    # stopped = False

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

    # def yolov8_callback(self, bounding_boxes_msg):
    #     if linear_velocity >= -0.01:
    #         for box in bounding_boxes_msg.bounding_boxes:
    #             if box.object_class == "person" and is_person_in_front(box):
    #                 rospy.loginfo("Person detected at x: %.2f, y: %.2f, z: %.2f", box.x, box.y, box.z)
    #                 person_detected = True
    #                 if not can_stop():
    #                     return
    #                 stop_robot()
    #                 bounding_boxes_msg = None
    #                 return
    #         if not person_detected:
    #             rospy.loginfo("No person detected within the bounding box")

    # def is_person_in_front(self, box):
    #     return -0.2 <= box.x <= 0.2 and box.z < 2.0

# last_stop_time = rospy.Time(0)
# stopped = False
# consecutive_stop_count = 0
# stop_timer = rospy.Time(0)
    # def can_stop(self):
    #     if consecutive_stop_count >= max_consecutive_stops:
    #         if rospy.Time.now() - stop_timer < stop_interval:
    #             return False
    #         else:
    #             consecutive_stop_count = 0
    #             stop_timer = rospy.Time.now()
    #     return True