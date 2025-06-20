#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool

class SoundStillPlayer:
    def __init__(self):
        rospy.init_node('sound_still_player_node', anonymous=True)
        
        # Subscribe to the cmd_vel topic
        self.cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.move_base_status_subscriber = rospy.Subscriber("/move_base/status", GoalStatusArray, self.goal_status_callback)

        self.still_sound_play_pub = rospy.Publisher('/play_still_sound', Bool, queue_size=10)

        self.signaling_flag = False
        self.goal_status = None
        self.rate = rospy.Rate(1)  # 1 second interval

        # Parameters for detecting stay in place
        self.linear_threshold = 0.05
        self.angular_threshold = 0.05

    def goal_status_callback(self, GoalStatus):
        if GoalStatus.status_list:
            # Get the latest status based on timestamp
            latest_status = max(GoalStatus.status_list, key=lambda s: (s.goal_id.stamp.secs, s.goal_id.stamp.nsecs))
            self.goal_status = latest_status.status
        else:
            self.goal_status = None
    
    def cmd_vel_callback(self, msg):
        # Check if the robot is moving
        if abs(msg.linear.x) < self.linear_threshold and abs(msg.angular.z) < self.angular_threshold:
            self.signaling_flag = True
        else:
            self.signaling_flag = False

    def run(self):
        while not rospy.is_shutdown():
            if self.signaling_flag and self.goal_status in [1, 2]:
                self.still_sound_play_pub.publish(True)
                self.signaling_flag = False  # Reset after publishing
            self.rate.sleep()  # Wait for the specified rate

if __name__ == '__main__':
    player = SoundStillPlayer()
    try:
        player.run()
    except rospy.ROSInterruptException:
        pass