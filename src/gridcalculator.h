#ifndef GRIDCALCULATOR_H
#define GRIDCALCULATOR_H

#include <math.h>
#include "gridmap.h"
#include <vector>
#include <geometry_msgs/Point.h>
#include "robotmodel.h"

class GridCalculator
{
  public:
    GridCalculator(GridMap* gridMap);
    virtual ~GridCalculator();
    void addScanToGrid(std::vector<double> vd_scan_points, double d_angle_increment, double d_angle_min, double d_angle_max);

    static geometry_msgs::Point transformSensorToRobotFrame(unsigned int ui_sensor_range, double d_sensor_angle);
    static geometry_msgs::Point transformRobotToWorldFrame(geometry_msgs::Point point_robot_frame, geometry_msgs::Point robot_pos_world_frame, double d_robot_angle);
    geometry_msgs::Point transformSensorToWorldFrame(unsigned int ui_sensor_range, double d_sensor_angle);

    void addMeasureToGrid(geometry_msgs::Point);
  protected:
  private:
    GridMap* gridMap_;
};

#endif // GRIDCALCULATOR_H
