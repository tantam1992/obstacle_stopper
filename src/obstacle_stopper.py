#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from obstacle_detector.msg import Obstacles
from nav_msgs.msg import Odometry
from sound_play.msg import SoundRequest
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
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/raw_obstacles', Obstacles, self.raw_obstacles_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Sound client for playing audio
        self.soundhandle = SoundClient()

        self.last_stop_time = rospy.Time(0)
        self.stopped = False
        self.consecutive_stop_count = 0
        self.stop_timer = rospy.Time(0)

        self.linear_velocity = 0.0       
    
    def cmd_vel_callback(self, cmd_vel_msg):
        # Extract linear velocity from cmd_vel message
        self.linear_velocity = cmd_vel_msg.linear.x

    def raw_obstacles_callback(self, obstacles_msg):
        if self.linear_velocity > 0.0:
            # Check if there are any circles detected
            if obstacles_msg.circles:
                rospy.loginfo("Obstacle detected in front of the robot")
                if not self.can_stop():
                    return
                self.stop_robot()
                return

    def stop_robot(self):
        # Play sound
        # self.soundhandle.playWave('/home/u/Documents/TKO_project/Voice/careful.wav')

        # Stop the robot motion
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.angular.z = 0.0
        # Record the time when the robot stopped
        self.last_stop_time = rospy.Time.now()
        self.stopped = True
        self.consecutive_stop_count += 1
        rospy.loginfo("Robot paused for 2 seconds due to dynamic obstacle detected.")  # Pause navigation for 2 seconds
        # Publish zero velocities repeatedly for stop_duration seconds
        while rospy.Time.now() - self.last_stop_time < self.stop_duration:
            self.cmd_vel_pub.publish(cmd_vel_msg)
        # After stop_duration seconds, resume navigation
        self.stopped = False

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
