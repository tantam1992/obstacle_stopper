#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import pygame

class ReverseBeepMonitor:
    def __init__(self):
        rospy.init_node('reverse_beep_node', anonymous=True)
        
        # Subscribe to the cmd_vel topic
        self.cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Initialize pygame
        pygame.mixer.init()
        self.track = pygame.mixer.Sound('/home/u/tko_ws/src/obstacle_stopper/voice/single_beep.wav')  # Replace with your audio file path
        
        # Set the volume (0.0 to 1.0)
        self.volume = 0.2  # Adjust this value as needed
        self.track.set_volume(self.volume)
        
        self.signaling_flag = False
        self.rate = rospy.Rate(1)  # 5 seconds interval

    def cmd_vel_callback(self, msg):
        # Check if any linear component is negative
        if msg.linear.x < 0:
            self.signaling_flag = True

    def run(self):
        while not rospy.is_shutdown():
            if self.signaling_flag:
                self.track.play()  # Play the audio track
                self.signaling_flag = False
            self.rate.sleep()  # Wait for the specified rate

if __name__ == '__main__':
    monitor = ReverseBeepMonitor()
    try:
        monitor.run()
    except rospy.ROSInterruptException:
        pass