#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class SoundRotationPlayer:
    def __init__(self):
        rospy.init_node('sound_rotation_player_node', anonymous=True)
        
        # Subscribe to the cmd_vel topic
        self.cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.rotation_sound_play_pub = rospy.Publisher('/play_rotation_sound', Bool, queue_size=10)

        self.signaling_flag = False
        self.rate = rospy.Rate(1)  # 1 seconds interval

        # Parameters for detecting rotation in place
        self.linear_threshold = 0.05
        self.angular_threshold = 0.1

    def cmd_vel_callback(self, msg):
        # Check if any linear component is negative
        if abs(msg.linear.x) < self.linear_threshold and abs(msg.angular.z) > self.angular_threshold:
            self.signaling_flag = True
        else:
            self.signaling_flag = False

    def run(self):
        while not rospy.is_shutdown():
            if self.signaling_flag:
                self.rotation_sound_play_pub.publish(True)
                self.signaling_flag = False
            self.rate.sleep()  # Wait for the specified rate

if __name__ == '__main__':
    player = SoundRotationPlayer()
    try:
        player.run()
    except rospy.ROSInterruptException:
        pass