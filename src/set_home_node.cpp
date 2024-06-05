#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

class SetHomeNode
{
public:
    SetHomeNode()
    {
        home_position_.x = 0.0;
        home_position_.y = 0.0;
        home_position_.z = 0.0;

        current_position_.pose.position.x = 0.0;
        current_position_.pose.position.y = 0.0;
        current_position_.pose.position.z = 0.0;

        diff_position_.pose.position.x = 0.0;
        diff_position_.pose.position.y = 0.0;
        diff_position_.pose.position.z = 0.0;

        ros::NodeHandle nh;
        set_home_server_ = nh.advertiseService("/mavros/override_set_home", &SetHomeNode::setHomeCallback, this);

        // subscribe to the current position in mavros
        current_position_sub_ = nh.subscribe("/mavros/local_position/pose", 1, &SetHomeNode::currentPositionCallback, this);

        // advertise the difference between the home position and the current position
        diff_position_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/local_position/adjusted", 1);   

        ROS_INFO("Set Home Node Initialized"); 

        ros::Rate rate(60);

        while (ros::ok())
        {
            computeDifference();
            diff_position_pub_.publish(diff_position_);
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::ServiceServer set_home_server_;
    ros::Subscriber current_position_sub_;
    ros::Publisher diff_position_pub_;

    geometry_msgs::Point home_position_;
    geometry_msgs::PoseStamped current_position_;
    geometry_msgs::PoseStamped diff_position_;

    bool setHomeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        home_position_.x = current_position_.pose.position.x;
        home_position_.y = current_position_.pose.position.y;
        home_position_.z = current_position_.pose.position.z;

        ROS_INFO("Home position set to: x: %f, y: %f, z: %f", home_position_.x, home_position_.y, home_position_.z);

        return true;
    }

    void currentPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_position_ = *msg;
    }

    void computeDifference()
    {
        diff_position_ = current_position_;
        diff_position_.pose.position.x = current_position_.pose.position.x - home_position_.x;
        diff_position_.pose.position.y = current_position_.pose.position.y - home_position_.y;
        diff_position_.pose.position.z = current_position_.pose.position.z - home_position_.z;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_home_node");
    SetHomeNode set_home_node;
    return 0;
}