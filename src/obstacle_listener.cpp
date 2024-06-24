#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <obstacle_detector/Obstacles.h>
#include <vector>

class ObstacleListener {
public:
    ObstacleListener() {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        pnh.param("front_bounding_box", front_bounding_box, std::vector<double>{0.2, 2.2, -0.4, 0.4});
        pnh.param("back_bounding_box", back_bounding_box, std::vector<double>{-2.0, -1.0, -0.4, 0.4});
        pnh.param("rotate_front_bounding_box", rotate_front_bounding_box, std::vector<double>{0.2, 1.2, -0.4, 0.4});
        pnh.param("rotate_back_bounding_box", rotate_back_bounding_box, std::vector<double>{-2.0, -1.0, -0.4, 0.4});

        obstacles_sub = nh.subscribe("/obstacles", 10, &ObstacleListener::obstaclesCallback, this);
        is_in_front_bounding_box_pub = nh.advertise<std_msgs::Bool>("/is_in_front", 10);
        is_in_back_bounding_box_pub = nh.advertise<std_msgs::Bool>("/is_in_back", 10);
        is_in_rotate_bounding_box_pub = nh.advertise<std_msgs::Bool>("/is_in_rotate", 10);
    }

    void run() {
        ros::Rate rate(15);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Subscriber obstacles_sub;
    ros::Publisher is_in_front_bounding_box_pub;
    ros::Publisher is_in_back_bounding_box_pub;
    ros::Publisher is_in_rotate_bounding_box_pub;

    std::vector<double> front_bounding_box;
    std::vector<double> back_bounding_box;
    std::vector<double> rotate_front_bounding_box;
    std::vector<double> rotate_back_bounding_box;

    void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& msg) {
        auto start = ros::Time::now();

        isInFront(msg);
        isInBack(msg);
        isInRotate(msg);

        auto end = ros::Time::now();
        auto runtime = end - start;
        ROS_INFO("start time: %.6f", start.toSec());
        ROS_INFO("end time: %.6f", end.toSec());
        ROS_INFO("runtime: %.6f", runtime.toSec());
        ROS_INFO("circle count: %zu", msg->circles.size());
    }

    void isInFront(const obstacle_detector::Obstacles::ConstPtr& msg) {
        std_msgs::Bool in_front;
        in_front.data = false;

        for (const auto& circle : msg->circles) {
            if (isWithinFrontBoundingBox(circle.center.x, circle.center.y) && circle.true_radius > 0.1) {
                ROS_INFO("In front bounding box, received obstacle at x: %.2f, y: %.2f, radius: %.2f", circle.center.x, circle.center.y, circle.true_radius);
                in_front.data = true;
                break;
            }
        }

        if (!in_front.data) {
            ROS_INFO("No obstacles in front bounding box");
        }

        is_in_front_bounding_box_pub.publish(in_front);
    }

    void isInBack(const obstacle_detector::Obstacles::ConstPtr& msg) {
        std_msgs::Bool in_back;
        in_back.data = false;

        for (const auto& circle : msg->circles) {
            if (isWithinBackBoundingBox(circle.center.x, circle.center.y) && circle.true_radius > 0.1) {
                ROS_INFO("In back bounding box, received obstacle at x: %.2f, y: %.2f, radius: %.2f", circle.center.x, circle.center.y, circle.true_radius);
                in_back.data = true;
                break;
            }
        }

        is_in_back_bounding_box_pub.publish(in_back);
    }

    void isInRotate(const obstacle_detector::Obstacles::ConstPtr& msg) {
        std_msgs::Bool in_rotate;
        in_rotate.data = false;

        for (const auto& circle : msg->circles) {
            if ((isWithinRotateFrontBoundingBox(circle.center.x, circle.center.y) || isWithinRotateBackBoundingBox(circle.center.x, circle.center.y)) && circle.true_radius > 0.1) {
                ROS_INFO("In rotate bounding box, received obstacle at x: %.2f, y: %.2f, radius: %.2f", circle.center.x, circle.center.y, circle.true_radius);
                in_rotate.data = true;
                break;
            }
        }

        is_in_rotate_bounding_box_pub.publish(in_rotate);
    }

    bool isWithinFrontBoundingBox(double x, double y) {
        return front_bounding_box[0] <= x && x <= front_bounding_box[1] && front_bounding_box[2] <= y && y <= front_bounding_box[3];
    }

    bool isWithinBackBoundingBox(double x, double y) {
        return back_bounding_box[0] <= x && x <= back_bounding_box[1] && back_bounding_box[2] <= y && y <= back_bounding_box[3];
    }

    bool isWithinRotateFrontBoundingBox(double x, double y) {
        return rotate_front_bounding_box[0] <= x && x <= rotate_front_bounding_box[1] && rotate_front_bounding_box[2] <= y && y <= rotate_front_bounding_box[3];
    }

    bool isWithinRotateBackBoundingBox(double x, double y) {
        return rotate_back_bounding_box[0] <= x && x <= rotate_back_bounding_box[1] && rotate_back_bounding_box[2] <= y && y <= rotate_back_bounding_box[3];
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_listener");
    ObstacleListener obstacle_listener;
    obstacle_listener.run();
    return 0;
}
