#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

class PoseEstimate
{
private:
    ros::NodeHandle nh;
    // ros::Publisher pub_m;
    ros::Publisher pub;
    ros::Subscriber sub;
    //   std_msgs::String pub_msg;
    sensor_msgs::LaserScan scan_msg;
    void

        public : PoseEstimate();
    void Publication(void);
    void Callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void filter(sensor_msgs::LaserScan input);
};

PoseEstimate::PoseEstimate()
{
    //  pub_m = nh.advertise<std_msgs::String>("/pub_msg", 1);
    pub = nh.advertise<sensor_msgs::LaserScan>("/scan_set", 1);
    sub = nh.subscribe("scan", 5, &PoseEstimate::Callback, this);
}

void PoseEstimate::filter(sensor_msgs::LaserScan input)
{

    int start_pos, target_size;
    float start_angle = -2.35;
    float end_angle = 0.0;
    /*
    start_pos = (input.angle_min - start_angle) / input.angle_increment;
    target_size = (end_angle - start_angle) / input.angle_increment;
*/
    scan_msg = input;
    //start_pos = (input.angle_min - start_angle) / input.angle_increment;

    ROS_INFO("ranges.size() %d", scan_msg.ranges.size());
    ROS_INFO("angle_min %f", scan_msg.angle_min);
    ROS_INFO("angle_max %f", scan_msg.angle_max);
    start_pos = int(start_angle - input.angle_min) / input.angle_increment;
    target_size = (end_angle - start_angle) / input.angle_increment;
    ROS_INFO("start_pos %d", start_pos);
    ROS_INFO("target_size %d", target_size);
    scan_msg.ranges.erase(scan_msg.ranges.begin(), scan_msg.ranges.begin() + int(start_pos));
    scan_msg.ranges.erase(scan_msg.ranges.begin() + target_size, scan_msg.ranges.end());

    scan_msg.angle_min = start_angle;
    scan_msg.angle_max = end_angle;
    //scan_msg.ranges.erase(scan_msg.ranges.begin() + target_size, scan_msg.ranges.end());
    /*
    scan_msg.ranges.erase(scan_msg.ranges.begin(), scan_msg.ranges.begin() + start_pos);
    scan_msg.ranges.erase(scan_msg.ranges.begin() + target_size, scan_msg.ranges.end());
    scan_msg.range_min = start_angle;
    scan_msg.range_max = end_angle;
    */
    ROS_INFO("_ranges.size() %d", scan_msg.ranges.size());
    ROS_INFO("angle_min %f", scan_msg.angle_min);
    ROS_INFO("angle_max %f", scan_msg.angle_max);
    ROS_INFO("-----------------------------------------");
}

void PoseEstimate::Publication(void)
{
    // pub_msg.data = "message";
    //   pub_m.publish(pub_msg);
    pub.publish(scan_msg);
}

void PoseEstimate::Callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    int i = msg->ranges.size() / 2;
    if (msg->ranges[i] < msg->range_min || // エラー値の場合
        msg->ranges[i] > msg->range_max || // 測定範囲外の場合
        std::isnan(msg->ranges[i]))        // 無限遠の場合
    {
        //ROS_INFO("front-range: measurement error");
    }
    else
    {
        //ROS_INFO("front-range: %0.3f",msg->ranges[msg->ranges.size() / 2]);
    }
    /*
    ROS_INFO("range.size %d", msg->ranges.size());
    ROS_INFO("angle_min %f", msg->angle_min * 57.3);
    ROS_INFO("angle_max %f", msg->angle_max * 57.3);
    ROS_INFO("angle_increment %f", msg->angle_increment * 57.3);
    */
    float num = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    //ROS_INFO("angle_num %f", num);
    filter(*msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    PoseEstimate pose_estimate;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        pose_estimate.Publication();
        ros::spinOnce();
        loop_rate.sleep();
    }
}