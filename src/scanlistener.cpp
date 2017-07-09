#include "scanlistener.h"

ScanListener::ScanListener(int argc, char **argv)
{
  //ctor
  init(argc, argv);
}

ScanListener::~ScanListener()
{
  //dtor
}

int ScanListener::init(int argc, char **argv)
{

  ros::init(argc, argv, "scan_listener");

  // Create ROS node handle and subscriber to scan
  nh_ = new ros::NodeHandle();
  sub_ = new  ros::Subscriber();
  *sub_ = nh_->subscribe("/scan", 1, &ScanListener::scanCallback, this);

  return 0;
}


void ScanListener::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  actual_scan_.d_angle_min_ = scan_in->angle_min;
  actual_scan_.d_angle_max_ = scan_in->angle_max;
  actual_scan_.d_angle_increment_ = scan_in->angle_increment;
  actual_scan_.vd_scan_points_.assign(scan_in->ranges.begin(), scan_in->ranges.end());
}


ScanMsgFormatted ScanListener::getActualScan()
{
  ros::spinOnce();
  return actual_scan_;
}