#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from laser_line_extraction.msg import LineSegmentList
from geometry_msgs.msg import Point

class ObstacleStopper:
    def __init__(self):
        rospy.init_node('obstacle_stopper')
        rospy.loginfo("obstacle_stopper started!")

        # Parameters
        self.stop_duration = rospy.Duration(1)  # 1 second stop duration
        self.max_consecutive_stops = 5  # Maximum consecutive stops allowed
        self.stop_interval = rospy.Duration(3)  # Duration to wait after reaching max consecutive stops

        # Subscribers and Publishers
        rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback)
        rospy.Subscriber('/collisionFront/scan_filtered', LaserScan, self.front_scan_callback)
        rospy.Subscriber('/collisionBack/scan_filtered', LaserScan, self.back_scan_callback)
        rospy.Subscriber('/collisionFront/line_segments', LineSegmentList, self.front_line_segments_callback)
        rospy.Subscriber('/collisionBack/line_segments', LineSegmentList, self.back_line_segments_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.last_stop_time = rospy.Time(0)
        self.stopped = False
        self.consecutive_stop_count = 0
        self.stop_timer = rospy.Time(0)

        self.robot_width = 0.8  # Adjust as needed
        self.scan_range = 1.5  # Adjust as needed
        self.linear_velocity = 0.0

        self.front_line_segments = None
        self.back_line_segments = None

    def odometry_callback(self, odom_msg):
        # Extract linear velocity from odometry message
        self.linear_velocity = odom_msg.twist.twist.linear.x
        # rospy.loginfo(f"velocity: {self.linear_velocity}")
        
    def front_scan_callback(self, scan_msg):
        if self.linear_velocity >= 0.05:
            if self.stopped or self.front_line_segments is None:
                # If already stopped or line segments detected don't process further
                return

            # Define the dimensions of the area around the robot
            area_width = 1.0  # Width of the area (in meters)
            area_length = 1.5  # Length of the area (in meters)

            # Check if there are any obstacles detected in the specified area
            for i, range_value in enumerate(scan_msg.ranges):
                if np.isnan(range_value):
                    continue  # Ignore NaN values
                else:
                    # Assuming scan_msg.ranges contains the distance data
                    # and scan_msg.angle_increment contains the angle increment data
                    angle = scan_msg.angle_min + i * scan_msg.angle_increment

                    # Check if the obstacle is within the specified area
                    if abs(angle) < np.arctan(area_width / (2 * area_length)) and range_value < area_length:
                        for segment in self.front_line_segments:
                            start = segment.start
                            end = segment.end
                            distance = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
                            if distance < 0.8:
                                rospy.loginfo(f"Front Scan - Index: {i}, Range: {range_value}, Distance: {distance}")
                                if not self.can_stop():
                                    return
                                self.stop_robot()
                                return

    def back_scan_callback(self, scan_msg):
        if self.linear_velocity <= -0.05:
            if self.stopped or self.back_line_segments is None:
                # If already stopped or line segments detected don't process further
                return

            # Define the dimensions of the area around the robot
            area_width = 1.0  # Width of the area (in meters)
            area_length = 1.5  # Length of the area (in meters)

            # Check if there are any obstacles detected in the specified area
            for i, range_value in enumerate(scan_msg.ranges):
                if np.isnan(range_value):
                    continue  # Ignore NaN values
                else:
                    # Assuming scan_msg.ranges contains the distance data
                    # and scan_msg.angle_increment contains the angle increment data
                    angle = scan_msg.angle_min + i * scan_msg.angle_increment

                    # Check if the obstacle is within the specified area
                    if abs(angle) < np.arctan(area_width / (2 * area_length)) and range_value < area_length:
                        for segment in self.back_line_segments:
                            start = segment.start
                            end = segment.end
                            distance = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
                            if distance < 0.8:
                                rospy.loginfo(f"Back Scan - Index: {i}, Range: {range_value}, Distance: {distance}")
                                if not self.can_stop():
                                    return
                                self.stop_robot()
                                return

    def front_line_segments_callback(self, line_segments_msg):
        # Check if line segments are detected
        if line_segments_msg.line_segments:
            # Store the line segments data
            self.front_line_segments = line_segments_msg.line_segments
        else:
            # If no line segments detected, set to None
            self.front_line_segments = None

    def back_line_segments_callback(self, line_segments_msg):
        # Check if line segments are detected
        if line_segments_msg.line_segments:
            # Store the line segments data
            self.back_line_segments = line_segments_msg.line_segments
        else:
            # If no line segments detected, set to None
            self.back_line_segments = None

    def stop_robot(self):
        # Stop the robot motion
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.angular.z = 0.0
        # Record the time when the robot stopped
        self.last_stop_time = rospy.Time.now()
        self.stopped = True
        self.consecutive_stop_count += 1
        rospy.loginfo("Robot paused for 1 second due to dynamic obstacle detected.")  # Pause navigation for 1 second
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
