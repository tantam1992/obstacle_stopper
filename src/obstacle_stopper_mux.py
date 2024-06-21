#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class ObstacleStopperMux:
    def __init__(self):
        rospy.init_node('obstacle_stopper_mux')
        rospy.loginfo("obstacle_stopper_mux started!")

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sound_play_pub = rospy.Publisher('/play_warning_sound', Bool, queue_size=10)

        self.nav_vel = None
        self.tel_vel = None
        self.dock_vel = None
        self.is_in_front = False
        self.is_in_back = False
        self.is_in_rotate = False

        rospy.Subscriber('/nav_vel', Twist, self.nav_vel_callback)
        rospy.Subscriber('/tel_vel', Twist, self.tel_vel_callback)
        rospy.Subscriber('/dock_vel', Twist, self.dock_vel_callback)
        rospy.Subscriber('/is_in_front', Bool, self.is_in_front_callback)
        rospy.Subscriber('/is_in_back', Bool, self.is_in_back_callback)
        rospy.Subscriber('/is_in_rotate', Bool, self.is_in_rotate_callback)

        self.rate = rospy.Rate(7.5)

    def nav_vel_callback(self, msg):
        self.nav_vel = msg

    def tel_vel_callback(self, msg):
        self.tel_vel = msg

    def dock_vel_callback(self, msg):
        self.dock_vel = msg

    def is_in_front_callback(self, msg):
        self.is_in_front = msg.data

    def is_in_back_callback(self, msg):
        self.is_in_back = msg.data

    def is_in_rotate_callback(self, msg):
        self.is_in_rotate = msg.data

    def run(self):
        while not rospy.is_shutdown():
            stop_vel_msg = Twist()
            stop_vel_msg.linear.x = 0.0
            stop_vel_msg.linear.y = 0.0
            stop_vel_msg.linear.z = 0.0
            stop_vel_msg.angular.x = 0.0
            stop_vel_msg.angular.y = 0.0
            stop_vel_msg.angular.z = 0.0
            
            cmd_vel_msg = stop_vel_msg

            self.is_moving_forward = False
            self.is_moving_backward = False
            self.is_rotating = False

            play_sound_msg = Bool()
            play_sound_msg.data = False
            
            if self.tel_vel is not None:
                rospy.loginfo("tel_vel received")
                cmd_vel_msg = self.tel_vel

            elif self.dock_vel is not None:
                rospy.loginfo("dock_vel received")
                cmd_vel_msg = self.dock_vel

            elif self.nav_vel is not None:
                rospy.loginfo("nav_vel receiving")
                nav_vel_msg = self.nav_vel
                self.is_moving_forward = nav_vel_msg.linear.x > 0.1
                self.is_moving_backward = nav_vel_msg.linear.x < -0.1
                self.is_rotating = (-0.1 <= nav_vel_msg.linear.x <= 0.1) and (nav_vel_msg.angular.z > 0.01 or nav_vel_msg.angular.z < -0.01)
                rospy.loginfo("nav_vel received")

                self.is_able_to_move = False

                if self.is_moving_forward:
                    if not self.is_in_front:
                        rospy.loginfo("forward enabled")
                        self.is_able_to_move = True
                        
                if self.is_moving_backward:
                    if not self.is_in_back:
                        rospy.loginfo("backward enabled")
                        self.is_able_to_move = True

                if self.is_rotating:
                    if not self.is_in_rotate:
                        rospy.loginfo("rotate enabled")
                        self.is_able_to_move = True

                if self.is_able_to_move:
                    cmd_vel_msg = nav_vel_msg
                else:
                    rospy.loginfo("obstacle detected")
                    play_sound_msg.data = True
                    cmd_vel_msg = stop_vel_msg

            else:
                rospy.loginfo("no vel received")
                cmd_vel_msg = stop_vel_msg
            
            self.sound_play_pub.publish(play_sound_msg)
            self.cmd_vel_pub.publish(cmd_vel_msg)
            self.tel_vel = None
            self.nav_vel = None
            self.rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_stopper_mux = ObstacleStopperMux()
        obstacle_stopper_mux.run()
    except rospy.ROSInterruptException:
        pass
