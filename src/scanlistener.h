#ifndef SCANLISTENER_H
#define SCANLISTENER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>

class ScanMsgFormatted
{
  public:
    std::vector<double> vd_scan_points_;
    double d_angle_increment_;
    double d_angle_min_;
    double d_angle_max_;
};

class ScanListener
{
  public:
    ScanListener(int argc, char **argv);
    virtual ~ScanListener();

    ScanMsgFormatted getActualScan();
  protected:
  private:
    int init(int argc, char **argv);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
    ros::NodeHandle* nh_;
    ros::Subscriber* sub_;
    ScanMsgFormatted actual_scan_;
};

#endif // SCANLISTENER_H
