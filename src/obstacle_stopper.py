#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from obstacle_detector.msg import Obstacles
from sound_play.libsoundplay import SoundClient
from std_srvs.srv import Empty
import threading

class ObstacleStopper:
    def __init__(self):
        rospy.init_node('obstacle_stopper')
        rospy.loginfo("obstacle_stopper started!")

        # Parameters
        self.stop_duration = rospy.Duration(rospy.get_param('~stop_duration', 3.0))
        self.max_consecutive_stops = rospy.get_param('~max_consecutive_stops', 5)
        self.stop_interval = rospy.Duration(rospy.get_param('~stop_interval', 3.0))
        self.front_bounding_box = rospy.get_param('~front_bounding_box', [0.2, 2.2, -0.4, 0.4])
        self.back_bounding_box = rospy.get_param('~back_bounding_box', [-2.0, -1.0, -0.4, 0.4])
        self.still_front_bounding_box = rospy.get_param('~still_front_bounding_box', [0.2, 1.2, -0.4, 0.4])
        self.still_back_bounding_box = rospy.get_param('~still_back_bounding_box', [-2.0, -1.0, -0.4, 0.4])
        self.warning_sound_path = rospy.get_param('~warning_sound_path', '/home/u/tko_ws/src/obstacle_stopper/voice/careful.wav')

        # Initialize variable
        self.stopped = False
        self.obstacles_detected = False
        self.consecutive_stop_count = 0
        self.last_stop_time = rospy.Time(0)
        self.stop_timer = rospy.Time(0)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Initialize threading lock
        self.nav_vel_callback_lock = threading.Lock()
        self.obstacles_callback_lock = threading.Lock()
        self.check_obstacles_lock = threading.Lock()
        self.stop_robot_lock = threading.Lock()
        self.resume_movement_lock = threading.Lock()

        # Subscribers and Publishers
        rospy.Subscriber('/nav_vel', Twist, self.nav_vel_callback)
        rospy.Subscriber('/obstacles', Obstacles, self.obstacles_callback)
        self.stop_vel_pub = rospy.Publisher('/stop_vel', Twist, queue_size=10)
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pause_navigation_pub = rospy.Publisher('/pause_navigation', Bool, queue_size=10)

        # Sound client for playing audio
        self.soundhandle = SoundClient()

        rospy.loginfo("Initialization complete, lock attribute created.")

    def nav_vel_callback(self, nav_vel_msg):
        # rospy.loginfo("nav_vel call back started")
        with self.nav_vel_callback_lock:
            # rospy.loginfo("Acquired vel_lock in nav_vel_callback")
            self.linear_velocity = nav_vel_msg.linear.x
            self.angular_velocity = nav_vel_msg.angular.z

    def obstacles_callback(self, obstacles_msg):
        # rospy.loginfo("obstacles call back started")
        with self.obstacles_callback_lock:
            # rospy.loginfo("Acquired obstacles_lock in obstacles_callback")
            
            if self.stopped:
                # rospy.loginfo("Already stopped")
                return
            
            is_in_moving_front_bounding_box = self.check_obstacles(obstacles_msg, self.is_within_front_bounding_box)
            is_in_moving_back_bounding_box = self.check_obstacles(obstacles_msg, self.is_within_back_bounding_box)
            is_in_still_front_bounding_box = self.check_obstacles(obstacles_msg, self.is_within_still_front_bounding_box)
            is_in_still_back_bounding_box = self.check_obstacles(obstacles_msg, self.is_within_still_back_bounding_box)

            is_forward = self.linear_velocity > 0.01
            is_backward = self.linear_velocity < -0.01
            is_rotate_to_left = (-0.01 <= self.linear_velocity <= 0.01 and self.angular_velocity <= -0.01)
            is_rotate_to_right = (-0.01 <= self.linear_velocity <= 0.01 and self.angular_velocity >= 0.01)

            # rospy.loginfo("Velocity: linear=%.2f, angular=%.2f", self.linear_velocity, self.angular_velocity)
            rospy.loginfo("Bounding Box Status: front_moving=%s, back_moving=%s, front_still=%s, back_still=%s",
                        is_in_moving_front_bounding_box, is_in_moving_back_bounding_box,
                        is_in_still_front_bounding_box, is_in_still_back_bounding_box)

            if is_in_still_front_bounding_box or is_in_still_back_bounding_box:
                rospy.loginfo("Detected obstacle in still bounding box")
                if is_rotate_to_left or is_rotate_to_right:
                    self.stop_robot()
                else:
                    rospy.loginfo("Not rotating, no need to stop")
            elif is_in_moving_front_bounding_box:
                rospy.loginfo("Detected obstacle in front bounding box")
                if is_forward:
                    self.stop_robot()
                else:
                    rospy.loginfo("Not moving forward, no need to stop")
            elif is_in_moving_back_bounding_box:
                rospy.loginfo("Detected obstacle in back bounding box")
                if is_backward:
                    self.stop_robot()
                else:
                    rospy.loginfo("Not moving backward, no need to stop")
            else:
                rospy.loginfo("No obstacles detected")
            
    def check_obstacles(self, obstacles_msg, bounding_box_func):
        self.obstacles_detected = False
        with self.check_obstacles_lock:
            for circle in obstacles_msg.circles:
                if circle.true_radius > 0.1 and bounding_box_func(circle.center.x, circle.center.y):                
                    rospy.loginfo("Received obstacle at x: %.2f, y: %.2f, radius: %.2f", circle.center.x, circle.center.y, circle.true_radius)
                    self.obstacles_detected = True
                    return True
            rospy.loginfo("No obstacles detected within the bounding box")
            return False

    def is_within_front_bounding_box(self, x, y):
        return self.front_bounding_box[0] <= x <= self.front_bounding_box[1] and self.front_bounding_box[2] <= y <= self.front_bounding_box[3]

    def is_within_back_bounding_box(self, x, y):
        return self.back_bounding_box[0] <= x <= self.back_bounding_box[1] and self.back_bounding_box[2] <= y <= self.back_bounding_box[3]

    def is_within_still_front_bounding_box(self, x, y):
        return self.still_front_bounding_box[0] <= x <= self.still_front_bounding_box[1] and self.still_front_bounding_box[2] <= y <= self.still_front_bounding_box[3]

    def is_within_still_back_bounding_box(self, x, y):
        return self.still_back_bounding_box[0] <= x <= self.still_back_bounding_box[1] and self.still_back_bounding_box[2] <= y <= self.still_back_bounding_box[3]

    def stop_robot(self):
        with self.stop_robot_lock:
            if self.stopped:
                return
            self.stopped = True
            # self.consecutive_stop_count += 1
            self.last_stop_time = rospy.Time.now()

            stop_vel_msg = Twist()
            stop_vel_msg.linear.x = 0.0 
            stop_vel_msg.linear.y = 0.0
            stop_vel_msg.linear.z = 0.0
            stop_vel_msg.angular.x = 0.0
            stop_vel_msg.angular.y = 0.0
            stop_vel_msg.angular.z = 0.0

            self.soundhandle.playWave(self.warning_sound_path)
            # Publish stop message
            self.pause_navigation_pub.publish(True)
            self.stop_vel_pub.publish(stop_vel_msg)
            self.cmd_vel_pub.publish(stop_vel_msg)            
            rospy.loginfo("Robot paused for 3 seconds due to dynamic obstacle detected.")
            rospy.sleep(3)
            self.resume_movement()
            # rospy.Timer(self.stop_duration, self.resume_movement, oneshot=True)

    def resume_movement(self, event=None):
        with self.resume_movement_lock:
            self.stopped = False
            self.pause_navigation_pub.publish(False)
            rospy.loginfo("Resuming movement")

    # def can_stop(self):
    #     with self.lock:
    #         if self.consecutive_stop_count >= self.max_consecutive_stops:
    #             if rospy.Time.now() - self.stop_timer < self.stop_interval:
    #                 return False
    #             else:
    #                 self.consecutive_stop_count = 0
    #                 self.stop_timer = rospy.Time.now()
    #     return True

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_stopper = ObstacleStopper()
        obstacle_stopper.run()
    except rospy.ROSInterruptException:
        pass
