#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from obstacle_detector.msg import Obstacles
import time

class ObstacleListener:
    def __init__(self):
        rospy.init_node('obstacle_listener')
        rospy.loginfo("obstacle_listener started!")

        # Parameters
        self.front_bounding_box = rospy.get_param('~front_bounding_box', [0.2, 2.2, -0.4, 0.4])
        self.back_bounding_box = rospy.get_param('~back_bounding_box', [-2.0, -1.0, -0.4, 0.4])
        self.rotate_front_bounding_box = rospy.get_param('~rotate_front_bounding_box', [0.2, 1.2, -0.4, 0.4])
        self.rotate_back_bounding_box = rospy.get_param('~rotate_back_bounding_box', [-2.0, -1.0, -0.4, 0.4])

        # Subscribers and Publishers
        rospy.Subscriber('/obstacles', Obstacles, self.obstacles_callback)
        self.is_in_front_bounding_box_pub = rospy.Publisher('/is_in_front', Bool, queue_size=10)
        self.is_in_back_bounding_box_pub = rospy.Publisher('/is_in_back', Bool, queue_size=10)
        self.is_in_rotate_bounding_box_pub = rospy.Publisher('/is_in_rotate', Bool, queue_size=10)        

    def obstacles_callback(self, obstacles_msg):
        list_length = len(obstacles_msg.circles)
        start = time.time() 

        self.is_in_front(obstacles_msg)
        self.is_in_back(obstacles_msg)
        self.is_in_rotate(obstacles_msg)

        end = time.time()
        runtime = end -start
        rospy.loginfo("start time: {}".format(start))
        rospy.loginfo("end_time: {}".format(end))
        rospy.loginfo("runtime: {}".format(runtime))
        rospy.loginfo("circle count: {}".format(list_length))

    def is_in_front(self, obstacles_msg):
        for circle in obstacles_msg.circles:
            if self.is_within_front_bounding_box(circle.center.x, circle.center.y) and circle.true_radius > 0.1:
                rospy.loginfo("In front bounding box, received obstacle at x: %.2f, y: %.2f, radius: %.2f", circle.center.x, circle.center.y, circle.true_radius)
                self.is_in_front_bounding_box_pub.publish(True)
                return
        rospy.loginfo("No obstacles in front bounding box")
        self.is_in_front_bounding_box_pub.publish(False)

    def is_in_back(self, obstacles_msg):
        for circle in obstacles_msg.circles:
            if self.is_within_back_bounding_box(circle.center.x, circle.center.y) and circle.true_radius > 0.1:
                rospy.loginfo("In back bounding box, received obstacle at x: %.2f, y: %.2f, radius: %.2f", circle.center.x, circle.center.y, circle.true_radius)
                self.is_in_back_bounding_box_pub.publish(True)
                return
        self.is_in_back_bounding_box_pub.publish(False)

    def is_in_rotate(self, obstacles_msg):
        for circle in obstacles_msg.circles:
            if (self.is_within_rotate_front_bounding_box(circle.center.x, circle.center.y) or 
                self.is_within_rotate_back_bounding_box(circle.center.x, circle.center.y)) and circle.true_radius > 0.1:
                rospy.loginfo("In rotate bounding box, received obstacle at x: %.2f, y: %.2f, radius: %.2f", circle.center.x, circle.center.y, circle.true_radius)
                self.is_in_rotate_bounding_box_pub.publish(True)
                return
        self.is_in_rotate_bounding_box_pub.publish(False)
    
    def is_within_front_bounding_box(self, x, y):
        return self.front_bounding_box[0] <= x <= self.front_bounding_box[1] and self.front_bounding_box[2] <= y <= self.front_bounding_box[3]
    
    def is_within_back_bounding_box(self, x, y):
        return self.back_bounding_box[0] <= x <= self.back_bounding_box[1] and self.back_bounding_box[2] <= y <= self.back_bounding_box[3]
    
    def is_within_rotate_front_bounding_box(self, x, y):
        return self.rotate_front_bounding_box[0] <= x <= self.rotate_front_bounding_box[1] and self.rotate_front_bounding_box[2] <= y <= self.rotate_front_bounding_box[3]
    
    def is_within_rotate_back_bounding_box(self, x, y):
        return self.rotate_back_bounding_box[0] <= x <= self.rotate_back_bounding_box[1] and self.rotate_back_bounding_box[2] <= y <= self.rotate_back_bounding_box[3]

    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_listener = ObstacleListener()
        obstacle_listener.run()
    except rospy.ROSInterruptException:
        pass
