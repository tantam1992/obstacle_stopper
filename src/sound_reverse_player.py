#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import pygame

class CmdVelMonitor:
    def __init__(self):
        rospy.init_node('cmd_vel_monitor', anonymous=True)
        
        # Subscribe to the cmd_vel topic
        self.cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Initialize pygame
        pygame.mixer.init()
        # self.track = pygame.mixer.Sound('path/to/your/audiofile.wav')  # Replace with your audio file path
        self.track = lambda: print("playsound") #pygame.mixer.Sound('path/to/your/audiofile.wav')  # Replace with your audio file path
        
        self.signaling_flag = False
        self.rate = rospy.Rate(0.2)  # 5 seconds interval

    def cmd_vel_callback(self, msg):
        # Check if any linear component is negative
        if msg.linear.x < 0 or msg.linear.y < 0 or msg.angular.z < 0:
            self.signaling_flag = True

    def run(self):
        while not rospy.is_shutdown():
            if self.signaling_flag:
                # self.track.play()  # Play the audio track
                self.signaling_flag = False
                self.track()
            self.rate.sleep()  # Wait for the specified rate

if __name__ == '__main__':
    monitor = CmdVelMonitor()
    try:
        monitor.run()
    except rospy.ROSInterruptException:
        pass