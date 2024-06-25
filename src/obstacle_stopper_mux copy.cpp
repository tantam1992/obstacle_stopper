#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

class ObstacleStopperMux {
public:
    ObstacleStopperMux() {
        ros::NodeHandle nh;

        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        sound_play_pub = nh.advertise<std_msgs::Bool>("/play_warning_sound", 10);

        nav_vel_sub = nh.subscribe("/nav_vel", 10, &ObstacleStopperMux::navVelCallback, this);
        tel_vel_sub = nh.subscribe("/tel_vel", 10, &ObstacleStopperMux::telVelCallback, this);
        dock_vel_sub = nh.subscribe("/dock_vel", 10, &ObstacleStopperMux::dockVelCallback, this);
        is_in_front_sub = nh.subscribe("/is_in_front", 10, &ObstacleStopperMux::isInFrontCallback, this);
        is_in_back_sub = nh.subscribe("/is_in_back", 10, &ObstacleStopperMux::isInBackCallback, this);
        is_in_rotate_sub = nh.subscribe("/is_in_rotate", 10, &ObstacleStopperMux::isInRotateCallback, this);

        is_in_front = false;
        is_in_back = false;
        is_in_rotate = false;
    }

    void run() {
        ros::Rate rate(7.5);

        while (ros::ok()) {
            geometry_msgs::Twist stop_vel_msg;
            stop_vel_msg.linear.x = 0.0;
            stop_vel_msg.linear.y = 0.0;
            stop_vel_msg.linear.z = 0.0;
            stop_vel_msg.angular.x = 0.0;
            stop_vel_msg.angular.y = 0.0;
            stop_vel_msg.angular.z = 0.0;

            geometry_msgs::Twist cmd_vel_msg = stop_vel_msg;

            is_moving_forward = false;
            is_moving_backward = false;
            is_rotating = false;

            std_msgs::Bool play_sound_msg;
            play_sound_msg.data = false;

            if (tel_vel) {
                ROS_INFO("tel_vel received");
                cmd_vel_msg = *tel_vel;
            } else if (dock_vel) {
                ROS_INFO("dock_vel received");
                cmd_vel_msg = *dock_vel;
            } else if (nav_vel) {
                ROS_INFO("nav_vel receiving");
                geometry_msgs::Twist nav_vel_msg = *nav_vel;
                is_moving_forward = nav_vel_msg.linear.x > 0.1;
                is_moving_backward = nav_vel_msg.linear.x < -0.1;
                is_rotating = (-0.1 <= nav_vel_msg.linear.x <= 0.1) && (nav_vel_msg.angular.z > 0.01 || nav_vel_msg.angular.z < -0.01);
                ROS_INFO("nav_vel received");

                is_able_to_move = false;

                if (is_moving_forward) {
                    if (!is_in_front) {
                        ROS_INFO("forward enabled");
                        is_able_to_move = true;
                    }
                }

                if (is_moving_backward) {
                    if (!is_in_back) {
                        ROS_INFO("backward enabled");
                        is_able_to_move = true;
                    }
                }

                if (is_rotating) {
                    if (!is_in_rotate) {
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
            } else {
                ROS_INFO("no vel received");
                cmd_vel_msg = stop_vel_msg;
            }

            sound_play_pub.publish(play_sound_msg);
            cmd_vel_pub.publish(cmd_vel_msg);
            tel_vel.reset();
            nav_vel.reset();
            dock_vel.reset();

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Publisher cmd_vel_pub;
    ros::Publisher sound_play_pub;
    ros::Subscriber nav_vel_sub;
    ros::Subscriber tel_vel_sub;
    ros::Subscriber dock_vel_sub;
    ros::Subscriber is_in_front_sub;
    ros::Subscriber is_in_back_sub;
    ros::Subscriber is_in_rotate_sub;

    boost::shared_ptr<geometry_msgs::Twist const> nav_vel;
    boost::shared_ptr<geometry_msgs::Twist const> tel_vel;
    boost::shared_ptr<geometry_msgs::Twist const> dock_vel;

    bool is_in_front;
    bool is_in_back;
    bool is_in_rotate;

    bool is_moving_forward;
    bool is_moving_backward;
    bool is_rotating;
    bool is_able_to_move;

    void navVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        nav_vel = msg;
    }

    void telVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        tel_vel = msg;
    }

    void dockVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        dock_vel = msg;
    }

    void isInFrontCallback(const std_msgs::Bool::ConstPtr& msg) {
        is_in_front = msg->data;
    }

    void isInBackCallback(const std_msgs::Bool::ConstPtr& msg) {
        is_in_back = msg->data;
    }

    void isInRotateCallback(const std_msgs::Bool::ConstPtr& msg) {
        is_in_rotate = msg->data;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_stopper_mux");
    ObstacleStopperMux obstacle_stopper_mux;
    obstacle_stopper_mux.run();
    return 0;
}
