#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

class DivideScanData
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_L;
    ros::Publisher pub_R;
    ros::Subscriber sub_;
    sensor_msgs::LaserScan scan_data_L_;
    sensor_msgs::LaserScan scan_data_R_;

public:
    DivideScanData();
    void publication(void);
    void Callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void divide(sensor_msgs::LaserScan input);
};

DivideScanData::DivideScanData()
{
    pub_L = nh_.advertise<sensor_msgs::LaserScan>("/scan_L", 1);
    pub_R = nh_.advertise<sensor_msgs::LaserScan>("/scan_R", 1);
    sub_ = nh_.subscribe("scan", 5, &DivideScanData::Callback, this);
}

void DivideScanData::divide(sensor_msgs::LaserScan input)
{

    int target_begin_L, target_size_L;
    float angle_begin_L = -2.05;
    float angle_end_L = -1.5;

    int target_begin_R, target_size_R;
    float angle_begin_R = 1.6;
    float angle_end_R = 2.09;

    scan_data_L_ = input;
    target_begin_L = (angle_begin_L - input.angle_min) / input.angle_increment;
    target_size_L = (angle_end_L - angle_begin_L) / input.angle_increment;
    scan_data_L_.ranges.erase(scan_data_L_.ranges.begin(), scan_data_L_.ranges.begin() + int(target_begin_L));
    scan_data_L_.ranges.erase(scan_data_L_.ranges.begin() + target_size_L, scan_data_L_.ranges.end());
    scan_data_L_.angle_min = angle_begin_L;
    scan_data_L_.angle_max = angle_end_L;

    scan_data_R_ = input;
    target_begin_R = (angle_begin_R - input.angle_min) / input.angle_increment;
    target_size_R = (angle_end_R - angle_begin_R) / input.angle_increment;
    scan_data_R_.ranges.erase(scan_data_R_.ranges.begin(), scan_data_R_.ranges.begin() + int(target_begin_R));
    scan_data_R_.ranges.erase(scan_data_R_.ranges.begin() + target_size_R, scan_data_R_.ranges.end());
    scan_data_R_.angle_min = angle_begin_R;
    scan_data_R_.angle_max = angle_end_R;
    /*
    ROS_INFO("input.angle_min %f", input.angle_min);
    ROS_INFO("input.angle_max %f", input.angle_max);
    ROS_INFO("scan_data_R_.angle_min %f", scan_data_R_.angle_min);
    ROS_INFO("scan_data_R_.angle_max %f", scan_data_R_.angle_max);
    ROS_INFO("scan_data_L_.angle_min %f", scan_data_L_.angle_min);
    ROS_INFO("scan_data_L_.angle_max %f", scan_data_L_.angle_max);
    ROS_INFO("____soiya___________");
    ROS_INFO("input");
    ROS_INFO("input_size_calc %f", (input.angle_max - input.angle_min) / input.angle_increment);
    ROS_INFO("input_size %d", input.ranges.size());
    ROS_INFO("L");
    ROS_INFO("scan_L_size_calc %f", (angle_end_L - angle_begin_L) / input.angle_increment);
    ROS_INFO("scan_L_size %d", scan_data_L_.ranges.size());
    ROS_INFO("R");
    ROS_INFO("scan_R_size_calc %f", (angle_end_R - angle_begin_R) / input.angle_increment);
    ROS_INFO("scan_R_size %d", scan_data_R_.ranges.size());

    ROS_INFO("___________________________________");
    */
}

void DivideScanData::publication(void)
{
    pub_L.publish(scan_data_L_);
    pub_R.publish(scan_data_R_);
}

void DivideScanData::Callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    divide(*msg);
    publication();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    DivideScanData divide_scan_data;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        //divide_scan_data.publication();
        ros::spinOnce();
        loop_rate.sleep();
    }
}