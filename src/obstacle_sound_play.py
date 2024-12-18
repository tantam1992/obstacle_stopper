#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from collections import deque

class ObstacleStopperMux:
    def __init__(self):
        rospy.init_node('obstacle_stopper_mux')
        rospy.loginfo("obstacle_stopper_mux started!")

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sound_play_pub = rospy.Publisher('/play_warning_sound', Bool, queue_size=10)

        self.nav_vel = None
        self.is_in_front = False
        self.is_in_back = False
        self.is_in_rotate = False

        rospy.Subscriber('/nav_vel', Twist, self.nav_vel_callback)
        rospy.Subscriber('/is_in_front', Bool, self.is_in_front_callback)
        rospy.Subscriber('/is_in_back', Bool, self.is_in_back_callback)
        rospy.Subscriber('/is_in_rotate', Bool, self.is_in_rotate_callback)

        self.rate = rospy.Rate(7.5)

        # Define timeouts and thresholds
        # self.nav_vel_timeout = rospy.Duration(0.5)  # 0.5 seconds
        # self.last_nav_vel_time = rospy.Time.now()

        self.obstacle_queue_size = 3
        self.front_queue = deque([False] * self.obstacle_queue_size, maxlen=self.obstacle_queue_size)
        self.back_queue = deque([False] * self.obstacle_queue_size, maxlen=self.obstacle_queue_size)
        self.rotate_queue = deque([False] * self.obstacle_queue_size, maxlen=self.obstacle_queue_size)

    def nav_vel_callback(self, msg):
        self.nav_vel = msg
        # self.last_nav_vel_time = rospy.Time.now()

    def is_in_front_callback(self, msg):
        self.is_in_front = msg.data
        self.front_queue.append(msg.data)

    def is_in_back_callback(self, msg):
        self.is_in_back = msg.data
        self.back_queue.append(msg.data)

    def is_in_rotate_callback(self, msg):
        self.is_in_rotate = msg.data
        self.rotate_queue.append(msg.data)

    def check_obstacle(self, queue):
        return all(queue)

    def run(self):
        while not rospy.is_shutdown():
            self.is_moving_forward = False
            self.is_moving_backward = False
            self.is_rotating = False

            play_sound_msg = Bool()
            play_sound_msg.data = False

            # elif self.nav_vel is not None and (rospy.Time.now() - self.last_nav_vel_time) < self.nav_vel_timeout:
            if self.nav_vel is not None :
                # rospy.loginfo("nav_vel receiving")
                nav_vel_msg = self.nav_vel
                self.is_moving_forward = nav_vel_msg.linear.x > 0.1
                self.is_moving_backward = nav_vel_msg.linear.x < -0.1
                self.is_rotating = (-0.1 <= nav_vel_msg.linear.x <= 0.1) and (nav_vel_msg.angular.z > 0.01 or nav_vel_msg.angular.z < -0.01)
                # rospy.loginfo("nav_vel received")

                self.is_able_to_move = False

                if self.is_moving_forward:
                    if not self.check_obstacle(self.front_queue):
                        rospy.loginfo("forward enabled")
                        self.is_able_to_move = True
                        
                if self.is_moving_backward:
                    if not self.check_obstacle(self.back_queue):
                        rospy.loginfo("backward enabled")
                        self.is_able_to_move = True

                if self.is_rotating:
                    if not self.check_obstacle(self.rotate_queue):
                        rospy.loginfo("rotate enabled")
                        self.is_able_to_move = True

                if self.is_able_to_move:
                    # cmd_vel_msg = nav_vel_msg
                    pass
                else:
                    rospy.loginfo("obstacle detected")
                    play_sound_msg.data = True
                    # cmd_vel_msg = stop_vel_msg

            else:
                rospy.loginfo("no vel received")
                # cmd_vel_msg = stop_vel_msg
            
            self.sound_play_pub.publish(play_sound_msg)
            # self.tel_vel = None
            self.nav_vel = None
            self.rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_stopper_mux = ObstacleStopperMux()
        obstacle_stopper_mux.run()
    except rospy.ROSInterruptException:
        pass
