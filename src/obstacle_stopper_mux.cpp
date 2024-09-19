#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <deque>
#include <memory>

class ObstacleStopperMux {
public:
    ObstacleStopperMux() : rate(15.0), obstacle_queue_size(3),
                           front_queue(obstacle_queue_size, false),
                           back_queue(obstacle_queue_size, false),
                           rotate_queue(obstacle_queue_size, false),
                           last_dock_vel_time(ros::Time::now()) {
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        sound_play_pub = nh.advertise<std_msgs::Bool>("/play_warning_sound", 10);

        nav_vel_sub = nh.subscribe("/nav_vel", 10, &ObstacleStopperMux::nav_vel_callback, this);
        tel_vel_sub = nh.subscribe("/tel_vel", 10, &ObstacleStopperMux::tel_vel_callback, this);
        dock_vel_sub = nh.subscribe("/dock_vel", 10, &ObstacleStopperMux::dock_vel_callback, this);
        is_in_front_sub = nh.subscribe("/is_in_front", 10, &ObstacleStopperMux::is_in_front_callback, this);
        is_in_back_sub = nh.subscribe("/is_in_back", 10, &ObstacleStopperMux::is_in_back_callback, this);
        is_in_rotate_sub = nh.subscribe("/is_in_rotate", 10, &ObstacleStopperMux::is_in_rotate_callback, this);

        ROS_INFO("obstacle_stopper_mux started!");
    }

    void run() {
        while (ros::ok()) {
            geometry_msgs::Twist stop_vel_msg;
            stop_vel_msg.linear.x = 0.0;
            stop_vel_msg.linear.y = 0.0;
            stop_vel_msg.linear.z = 0.0;
            stop_vel_msg.angular.x = 0.0;
            stop_vel_msg.angular.y = 0.0;
            stop_vel_msg.angular.z = 0.0;

            geometry_msgs::Twist cmd_vel_msg = stop_vel_msg;
            std_msgs::Bool play_sound_msg;
            play_sound_msg.data = false;

            if (tel_vel) {
                ROS_INFO("tel_vel received");
                cmd_vel_msg = *tel_vel;
                tel_vel.reset(); // Reset tel_vel after processing
            } else if (dock_vel || (ros::Time::now() - last_dock_vel_time).toSec() < 0.5) {
                ROS_INFO("dock_vel received or within buffer period");
                if (dock_vel) {
                    last_dock_vel = dock_vel; // Store the last received dock_vel
                    last_dock_vel_time = ros::Time::now();
                }
                if (last_dock_vel) {
                    cmd_vel_msg = *last_dock_vel;
                }
                dock_vel.reset(); // Reset dock_vel after processing
            } else if (nav_vel) {
                geometry_msgs::Twist nav_vel_msg = *nav_vel;
                is_moving_forward = nav_vel_msg.linear.x > 0.1;
                is_moving_backward = nav_vel_msg.linear.x < -0.1;
                is_rotating = (-0.1 <= nav_vel_msg.linear.x && nav_vel_msg.linear.x <= 0.1) &&
                              (nav_vel_msg.angular.z > 0.01 || nav_vel_msg.angular.z < -0.01);

                is_able_to_move = false;

                if (is_moving_forward) {
                    if (!check_obstacle(front_queue)) {
                        ROS_INFO("forward enabled");
                        is_able_to_move = true;
                    }
                }

                if (is_moving_backward) {
                    if (!check_obstacle(back_queue)) {
                        ROS_INFO("backward enabled");
                        is_able_to_move = true;
                    }
                }

                if (is_rotating) {
                    if (!check_obstacle(rotate_queue)) {
                        ROS_INFO("rotate enabled");
                        is_able_to_move = true;
                    }
                }

                if (is_able_to_move) {
                    cmd_vel_msg = nav_vel_msg;
                } else {
                    ROS_INFO("obstacle detected");
                    play_sound_msg.data = true;
                    cmd_vel_msg = stop_vel_msg;
                }

                nav_vel.reset(); // Reset nav_vel after processing
            } else {
                ROS_INFO("no vel received");
                cmd_vel_msg = stop_vel_msg;
            }

            sound_play_pub.publish(play_sound_msg);
            cmd_vel_pub.publish(cmd_vel_msg);
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Publisher sound_play_pub;
    ros::Subscriber nav_vel_sub;
    ros::Subscriber tel_vel_sub;
    ros::Subscriber dock_vel_sub;
    ros::Subscriber is_in_front_sub;
    ros::Subscriber is_in_back_sub;
    ros::Subscriber is_in_rotate_sub;
    ros::Rate rate;

    std::shared_ptr<geometry_msgs::Twist> nav_vel;
    std::shared_ptr<geometry_msgs::Twist> tel_vel;
    std::shared_ptr<geometry_msgs::Twist> dock_vel;
    std::shared_ptr<geometry_msgs::Twist> last_dock_vel;
    ros::Time last_dock_vel_time;
    bool is_in_front;
    bool is_in_back;
    bool is_in_rotate;
    bool is_moving_forward;
    bool is_moving_backward;
    bool is_rotating;
    bool is_able_to_move;

    int obstacle_queue_size;
    std::deque<bool> front_queue;
    std::deque<bool> back_queue;
    std::deque<bool> rotate_queue;

    void nav_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
        nav_vel = std::make_shared<geometry_msgs::Twist>(*msg);
    }

    void tel_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
        tel_vel = std::make_shared<geometry_msgs::Twist>(*msg);
    }

    void dock_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
        dock_vel = std::make_shared<geometry_msgs::Twist>(*msg);
        last_dock_vel_time = ros::Time::now(); // Update the time when dock_vel is received
    }

    void is_in_front_callback(const std_msgs::Bool::ConstPtr& msg) {
        is_in_front = msg->data;
        front_queue.push_back(msg->data);
        if (front_queue.size() > obstacle_queue_size) {
            front_queue.pop_front();
        }
    }

    void is_in_back_callback(const std_msgs::Bool::ConstPtr& msg) {
        is_in_back = msg->data;
        back_queue.push_back(msg->data);
        if (back_queue.size() > obstacle_queue_size) {
            back_queue.pop_front();
        }
    }

    void is_in_rotate_callback(const std_msgs::Bool::ConstPtr& msg) {
        is_in_rotate = msg->data;
        rotate_queue.push_back(msg->data);
        if (rotate_queue.size() > obstacle_queue_size) {
            rotate_queue.pop_front();
        }
    }

    bool check_obstacle(const std::deque<bool>& queue) {
        return std::all_of(queue.begin(), queue.end(), [](bool v) { return v; });
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_stopper_mux");
    ObstacleStopperMux obstacle_stopper_mux;
    obstacle_stopper_mux.run();
    return 0;
}
