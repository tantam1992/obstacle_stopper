#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from obstacle_detector.msg import Obstacles
from sound_play.libsoundplay import SoundClient

class ObstacleStopper:
    def __init__(self):
        rospy.init_node('obstacle_stopper')
        rospy.loginfo("obstacle_stopper started!")

        # Parameters
        self.stop_duration = rospy.Duration(2)  # 2 seconds stop duration
        self.max_consecutive_stops = 5  # Maximum consecutive stops allowed
        self.stop_interval = rospy.Duration(3)  # Duration to wait after reaching max consecutive stops

        # Subscribers and Publishers
        rospy.Subscriber('/nav_vel', Twist, self.nav_vel_callback)
        rospy.Subscriber('/raw_obstacles', Obstacles, self.raw_obstacles_callback)
        self.tel_vel_pub = rospy.Publisher('/tel_vel', Twist, queue_size=10)
        self.obstacles_pub = rospy.Publisher('/cleared_obstacles', Obstacles, queue_size=10)

        # Sound client for playing audio
        self.soundhandle = SoundClient()

        self.last_stop_time = rospy.Time(0)
        self.stopped = False
        self.consecutive_stop_count = 0
        self.stop_timer = rospy.Time(0)

        self.linear_velocity = 0.0
        self.obstacles_detected = False

    def nav_vel_callback(self, nav_vel_msg):
        # Extract linear velocity from cmd_vel message
        self.linear_velocity = nav_vel_msg.linear.x

    def raw_obstacles_callback(self, obstacles_msg):
        if self.linear_velocity > 0.01:
            # Check if there are any circles detected within the specified bounding box
            self.obstacles_detected = False
            for circle in obstacles_msg.circles:                
                if self.is_within_front_bounding_box(circle.center.x, circle.center.y):
                    rospy.loginfo("Received obstacle at x: %.2f, y: %.2f", circle.center.x, circle.center.y)
                    rospy.loginfo("Obstacle detected in front of the robot within the front bounding box")
                    self.obstacles_detected = True
                    if not self.can_stop():
                        return
                    self.stop_robot()
                    obstacles_msg = None
                    self.clear_obstacles()
                    return
            if not self.obstacles_detected:
                rospy.loginfo("No obstacles detected within the bounding box")
        elif self.linear_velocity < -0.01:
            # Check if there are any circles detected within the specified bounding box
            self.obstacles_detected = False
            for circle in obstacles_msg.circles:                
                if self.is_within_back_bounding_box(circle.center.x, circle.center.y):
                    rospy.loginfo("Received obstacle at x: %.2f, y: %.2f", circle.center.x, circle.center.y)
                    rospy.loginfo("Obstacle detected behind of the robot within the back bounding box")
                    self.obstacles_detected = True
                    if not self.can_stop():
                        return
                    self.stop_robot()
                    obstacles_msg = None
                    self.clear_obstacles()
                    return
            if not self.obstacles_detected:
                rospy.loginfo("No obstacles detected within the bounding box")
        else:
            return
        
    def is_within_front_bounding_box(self, x, y):
        # Define the bounding box dimensions (2m length, 0.8m width)
        if 0.2 <= x <= 2.2 and -0.4 <= y <= 0.4:
            return True
        return False

    def is_within_back_bounding_box(self, x, y):
        # Define the bounding box dimensions (1m length, 0.8m width)
        if -2.0 <= x <= -1.0 and -0.4 <= y <= 0.4:
            return True
        return False

    def stop_robot(self):
        # Play sound
        self.soundhandle.playWave('/home/u/tko_ws/src/obstacle_stopper/voice/careful.wav')

        # Stop the robot motion 
        tel_vel_msg = Twist()
        tel_vel_msg.linear.x = 0.0 
        tel_vel_msg.linear.y = 0.0
        tel_vel_msg.angular.z = 0.0
        # Record the time when the robot stopped
        self.last_stop_time = rospy.Time.now()
        self.stopped = True
        self.consecutive_stop_count += 1
        rospy.loginfo("Robot paused for 2 seconds due to dynamic obstacle detected.")  # Pause navigation for 2 seconds
        # Publish zero velocities repeatedly for stop_duration seconds
        while rospy.Time.now() - self.last_stop_time < self.stop_duration:
            self.tel_vel_pub.publish(tel_vel_msg)
        # After stop_duration seconds, resume navigation
        self.stopped = False

    def clear_obstacles(self):
        # Publish an empty Obstacles message to clear the circles
        empty_obstacles_msg = Obstacles()
        self.obstacles_pub.publish(empty_obstacles_msg)
        rospy.loginfo("Published empty obstacles message to clear obstacles")

    def can_stop(self):
        # Check if the robot can stop based on the consecutive stop count and stop timer
        if self.consecutive_stop_count >= self.max_consecutive_stops:
            if rospy.Time.now() - self.stop_timer < self.stop_interval:
                return False  # Wait for the stop interval to expire
            else:
                # Reset consecutive stop count and update the stop timer
                self.consecutive_stop_count = 0
                self.stop_timer = rospy.Time.now()
        return True

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_stopper = ObstacleStopper()
        obstacle_stopper.run()
    except rospy.ROSInterruptException:
        pass
