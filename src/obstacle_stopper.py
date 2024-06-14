#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from obstacle_detector.msg import Obstacles
from sound_play.libsoundplay import SoundClient
from std_srvs.srv import Empty
from yolo_v8_ros_msgs.msg import BoundingBoxes
from time import sleep

class ObstacleStopper:
    def __init__(self):
        rospy.init_node('obstacle_stopper')
        rospy.loginfo("obstacle_stopper started!")

        # Parameters
        self.stop_duration = rospy.Duration(rospy.get_param('~stop_duration', 2.0))
        self.max_consecutive_stops = rospy.get_param('~max_consecutive_stops', 5)
        self.stop_interval = rospy.Duration(rospy.get_param('~stop_interval', 3.0))
        self.front_bounding_box = rospy.get_param('~front_bounding_box', [0.2, 2.2, -0.4, 0.4])
        self.back_bounding_box = rospy.get_param('~back_bounding_box', [-2.0, -1.0, -0.4, 0.4])
        self.still_front_bounding_box = rospy.get_param('~still_front_bounding_box', [0.2, 1.2, -0.4, 0.4])
        self.still_back_bounding_box = rospy.get_param('~still_back_bounding_box', [-2.0, -1.0, -0.4, 0.4])
        self.warning_sound_path = rospy.get_param('~warning_sound_path', '/home/u/tko_ws/src/obstacle_stopper/voice/careful.wav')

        # Subscribers and Publishers
        rospy.Subscriber('/nav_vel', Twist, self.nav_vel_callback)
        rospy.Subscriber('/raw_obstacles', Obstacles, self.raw_obstacles_callback)
        # rospy.Subscriber('/yolov8/bounding_boxes', BoundingBoxes, self.yolov8_callback)
        
        self.stop_vel_pub = rospy.Publisher('/stop_vel', Twist, queue_size=10)
        self.obstacles_pub = rospy.Publisher('/raw_obstacles', Obstacles, queue_size=10)

        # Sound client for playing audio
        self.soundhandle = SoundClient()

        self.last_stop_time = rospy.Time(0)
        self.stopped = False
        self.consecutive_stop_count = 0
        self.stop_timer = rospy.Time(0)

        self.linear_velocity = 0.0
        self.person_detected = False
        self.obstacles_detected = False

    def nav_vel_callback(self, nav_vel_msg):
        self.linear_velocity = nav_vel_msg.linear.x

    def raw_obstacles_callback(self, obstacles_msg):
        if self.linear_velocity > 0.01:
            self.check_obstacles(obstacles_msg, self.is_within_front_bounding_box)
        elif self.linear_velocity < -0.01:
            self.check_obstacles(obstacles_msg, self.is_within_back_bounding_box)
        else:
            self.check_obstacles(obstacles_msg, self.is_within_still_front_bounding_box)
            self.check_obstacles(obstacles_msg, self.is_within_still_back_bounding_box)

    def check_obstacles(self, obstacles_msg, bounding_box_func):
        self.obstacles_detected = False
        for circle in obstacles_msg.circles:
                if bounding_box_func(circle.center.x, circle.center.y):
                    if circle.true_radius > 0.1:
                        rospy.loginfo("Received obstacle at x: %.2f, y: %.2f, radius: %.2f", circle.center.x, circle.center.y, circle.true_radius)
                        self.obstacles_detected = True
                        if self.can_stop():
                            self.stop_robot()
                            obstacles_msg.circles.clear()
                            rospy.loginfo("Cleared all obstacles")
                            rospy.loginfo(obstacles_msg.circles)
                            self.obstacles_pub.publish(obstacles_msg)
                            self.clear_obstacles()
                            break
                    else:
                        rospy.loginfo("No obstacles detected within the bounding box")
                else:
                    rospy.loginfo("No obstacles detected within the bounding box")
        if not self.obstacles_detected:
            rospy.loginfo("No obstacles detected")
            

    # def yolov8_callback(self, bounding_boxes_msg):
    #     if self.linear_velocity >= -0.01:
    #         for box in bounding_boxes_msg.bounding_boxes:
    #             if box.object_class == "person" and self.is_person_in_front(box):
    #                 rospy.loginfo("Person detected at x: %.2f, y: %.2f, z: %.2f", box.x, box.y, box.z)
    #                 self.person_detected = True
    #                 if not self.can_stop():
    #                     return
    #                 self.stop_robot()
    #                 bounding_boxes_msg = None
    #                 return
    #         if not self.person_detected:
    #             rospy.loginfo("No person detected within the bounding box")

    def is_person_in_front(self, box):
        return -0.2 <= box.x <= 0.2 and box.z < 2.0
    
    def is_within_front_bounding_box(self, x, y):
        return self.front_bounding_box[0] <= x <= self.front_bounding_box[1] and self.front_bounding_box[2] <= y <= self.front_bounding_box[3]
    def is_within_back_bounding_box(self, x, y):
        return self.back_bounding_box[0] <= x <= self.back_bounding_box[1] and self.back_bounding_box[2] <= y <= self.back_bounding_box[3]
    def is_within_still_front_bounding_box(self, x, y):
        return self.still_front_bounding_box[0] <= x <= self.still_front_bounding_box[1] and self.still_front_bounding_box[2] <= y <= self.still_front_bounding_box[3]
    def is_within_still_back_bounding_box(self, x, y):
        return self.still_back_bounding_box[0] <= x <= self.still_back_bounding_box[1] and self.still_back_bounding_box[2] <= y <= self.still_back_bounding_box[3]

    def stop_robot(self):
        # self.soundhandle.playWave(self.warning_sound_path)

        stop_vel_msg = Twist()
        stop_vel_msg.linear.x = 0.0 
        stop_vel_msg.linear.y = 0.0
        stop_vel_msg.angular.z = 0.0
        self.last_stop_time = rospy.Time.now()
        self.stopped = True
        self.consecutive_stop_count += 1
        rospy.loginfo("Robot paused for 2 seconds due to dynamic obstacle detected.")
        rate = rospy.Rate(10)
        while rospy.Time.now() - self.last_stop_time < self.stop_duration:
            self.stop_vel_pub.publish(stop_vel_msg)
            rate.sleep()
        self.stopped = False
        # sleep(0.5)

    def clear_obstacles(self):
        rospy.loginfo("Publishing empty obstacles message to clear obstacles")
        empty_obstacles_msg = Obstacles(
            header=rospy.Header(
                seq=0,
                stamp=rospy.Time.now(),
                frame_id='base_link'
            ),
            segments=[],
            circles=[]
        )
        self.obstacles_pub.publish(empty_obstacles_msg)
        rospy.loginfo("Published empty obstacles message to clear obstacles")

    def can_stop(self):
        if self.consecutive_stop_count >= self.max_consecutive_stops:
            if rospy.Time.now() - self.stop_timer < self.stop_interval:
                return False
            else:
                self.consecutive_stop_count = 0
                self.stop_timer = rospy.Time.now()
        return True

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_stopper = ObstacleStopper()
        obstacle_stopper.run()
    except rospy.ROSInterruptException:
        pass
