#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

class RosWithClass
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub_m;
    ros::Publisher pub;
    ros::Subscriber sub;
    std_msgs::String pub_msg;
    sensor_msgs::LaserScan scan_msg;

public:
    RosWithClass();
    void Publication(void);
    void Callback(const sensor_msgs::LaserScan::ConstPtr &msg);
};

RosWithClass::RosWithClass()
{
    pub_m = nh.advertise<std_msgs::String>("/pub_msg", 1);
    pub = nh.advertise<sensor_msgs::LaserScan>("/scan_set", 1);
    sub = nh.subscribe("scan", 5, &RosWithClass::Callback, this);
}

void RosWithClass::Publication(void)
{
    pub_msg.data = "message";
    pub_m.publish(pub_msg);

    //pub.publish(scan_msg);
}

void RosWithClass::Callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    int i = msg->ranges.size() / 2;
    if (msg->ranges[i] < msg->range_min || // エラー値の場合
        msg->ranges[i] > msg->range_max || // 測定範囲外の場合
        std::isnan(msg->ranges[i]))        // 無限遠の場合
    {
        ROS_INFO("front-range: measurement error");
    }
    else
    {
        ROS_INFO("front-range: %0.3f",
                 msg->ranges[msg->ranges.size() / 2]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    RosWithClass ros_with_class;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros_with_class.Publication();
        ros::spinOnce();
        loop_rate.sleep();
    }
}