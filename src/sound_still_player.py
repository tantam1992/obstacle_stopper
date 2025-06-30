#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool
from std_msgs.msg import String

class SoundStillPlayer:
    def __init__(self):
        rospy.init_node('sound_still_player_node', anonymous=True)
        
        # Subscribe to the cmd_vel topic
        self.cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.move_base_status_subscriber = rospy.Subscriber("/move_base/status", GoalStatusArray, self.goal_status_callback)
        self.fold_state_sub = rospy.Subscriber('/fold_state', String, self.fold_state_callback)

        self.still_sound_play_pub = rospy.Publisher('/play_still_sound', Bool, queue_size=10)

        self.signaling_flag = False
        self.goal_status = None
        self.fold_state = None
        self.still_time = 0  # Time the robot has been still
        self.rate = rospy.Rate(1)  # 1 second interval

        # Parameters for detecting stay in place
        self.linear_threshold = 0.05
        self.angular_threshold = 0.05
        self.still_duration = 7  # Duration in seconds to stay still

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
            self.still_time = 0  # Reset the still time if moving

    def fold_state_callback(self, msg):
        self.fold_state = msg.data

    def run(self):
        while not rospy.is_shutdown():
            # Publish sound only if not in TRANSFORMED or TRANSFORMING state
            if self.fold_state not in ["TRANSFORMED", "TRANSFORMING"] and self.signaling_flag and self.goal_status in [1, 2]:
                self.still_time += 1  # Increment still time
                if self.still_time >= self.still_duration:
                    self.still_sound_play_pub.publish(True)
                    self.still_time = 0  # Reset after publishing
            else:
                self.still_time = 0  # Reset if not still or if in TRANSFORMED or TRANSFORMING state

            self.rate.sleep()  # Wait for the specified rate

if __name__ == '__main__':
    player = SoundStillPlayer()
    try:
        player.run()
    except rospy.ROSInterruptException:
        pass