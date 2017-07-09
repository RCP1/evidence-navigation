#include "gridcalculator.h"


GridCalculator::GridCalculator(GridMap* gridMap)
  : gridMap_(gridMap)
{
  //ctor
}

GridCalculator::~GridCalculator()
{
  //dtor
}

void GridCalculator::addScanToGrid(std::vector<double> vd_scan_points, double d_angle_increment, double d_angle_min, double d_angle_max)
{
  double d_actual_angle = d_angle_min; // angle_min is negative
  if(vd_scan_points.empty())
  {
    std::cout << "Empty scan" << std::endl;
    return;
  }
  for(std::vector<double>::iterator it = vd_scan_points.begin(); it != vd_scan_points.end() ; ++it) //|| d_actual_angle >= d_angle_max
  {
    if(isnan(*it))
    {
      //std::cout << "[          ] Number is NaN" << " Actual angle is: " << d_actual_angle << std::endl;
      // Nothing measured / Out of range
    }
    else
    {
      //std::cout << "[          ] Number is: " << *it << " Actual angle is: " << d_actual_angle << std::endl;
      addMeasureToGrid(transformSensorToWorldFrame((unsigned int)(M_TO_MM * (*it)), d_actual_angle));
    }
    d_actual_angle += d_angle_increment;
  }
}

geometry_msgs::Point GridCalculator::transformSensorToRobotFrame(unsigned int ui_sensor_range, double d_sensor_angle)
{
  geometry_msgs::Point point_robot_frame;
  // Sensor x and y from range and angle
  // x is facing forward, in PointCloud z direction
  // y is facing left to sensor view direction
  point_robot_frame.x = (int)(cos(d_sensor_angle) * ui_sensor_range) + SENSOR_TO_MID_X_MM;
  point_robot_frame.y = (int)(sin(d_sensor_angle) * ui_sensor_range) + SENSOR_TO_MID_Y_MM;
  //std::cout << "[          ] Measured Point in Robot Frame is:" << point_robot_frame.x << " " << point_robot_frame.y << std::endl;
  return point_robot_frame;
}



geometry_msgs::Point GridCalculator::transformRobotToWorldFrame(geometry_msgs::Point point_robot_frame, geometry_msgs::Point robot_pos_world_frame, double d_robot_angle)
{
  geometry_msgs::Point point_world_frame;
  point_world_frame.x = robot_pos_world_frame.x +
                  (int)((cos(-d_robot_angle) * point_robot_frame.x) + //Rotation applied to x of scan point
                  (sin(-d_robot_angle) * point_robot_frame.y));
  point_world_frame.y = robot_pos_world_frame.y +
                  (int)(- (sin(-d_robot_angle) * point_robot_frame.x) + //Rotation applied to y of scan point
                  (cos(-d_robot_angle) * point_robot_frame.y));

  //std::cout << "[          ] Measured Point in World Frame WITHOUT robot translation offset:" << (int)((cos(-d_robot_angle) * point_robot_frame.x) +
                  //(sin(-d_robot_angle) * point_robot_frame.y)) << " " << (int)(( - sin(-d_robot_angle) * point_robot_frame.x) +
                  //(cos(-d_robot_angle) * point_robot_frame.y)) << std::endl;
  //std::cout << "[          ] Measured Point in World Frame is:" << point_world_frame.x << " " << point_world_frame.y << std::endl;
  return point_world_frame;
}


geometry_msgs::Point GridCalculator::transformSensorToWorldFrame(unsigned int ui_sensor_range, double d_sensor_angle)
{
  geometry_msgs::Point point_world_frame;

  geometry_msgs::Point robot_pos_world_frame;
  robot_pos_world_frame.x = (int)gridMap_->getPose().x * GridMap::GRID_RES_IN_MM + OFFSET_MID_OF_CM_CELL;
  robot_pos_world_frame.y = (int)gridMap_->getPose().y * GridMap::GRID_RES_IN_MM + OFFSET_MID_OF_CM_CELL;

  //std::cout << "[          ] Robot Pos. is:" << robot_pos_world_frame.x << " " << robot_pos_world_frame.y << std::endl;
  point_world_frame = transformRobotToWorldFrame(transformSensorToRobotFrame(ui_sensor_range, d_sensor_angle),
                                                    robot_pos_world_frame,
                                                    gridMap_->getPose().theta);
  return point_world_frame;
}

void GridCalculator::addMeasureToGrid(geometry_msgs::Point measured_point)
{
  unsigned int ui_measured_x = (unsigned int)measured_point.x / GridMap::GRID_RES_IN_MM;
  unsigned int ui_measured_y = (unsigned int)measured_point.y / GridMap::GRID_RES_IN_MM;

  int x,y,sX,sY,dX,dY,h;
  unsigned int ui_robot_pos_x = (unsigned int)gridMap_->getPose().x;
  unsigned int ui_robot_pos_y = (unsigned int)gridMap_->getPose().y;

  /* Add occupancy at measured point */
  gridMap_->updateCellAsOccupied(ui_measured_x, ui_measured_y);

  /* Substract occupancy on line to measured point */
  x = ui_robot_pos_x;
  sX = (ui_measured_x < ui_robot_pos_x ? -1 : 1);
  dX = abs(ui_measured_x - ui_robot_pos_x);

  y = ui_robot_pos_y;
  sY = (ui_measured_y < ui_robot_pos_y ? -1 : 1);
  dY = abs(ui_measured_y - ui_robot_pos_y);

  if (dX >= dY) {
    h = dX/2;
    for (int i=0; i<dX; ++i) {
      gridMap_->updateCellAsFree(x + i * sX, y);
      h += dY;
      if (h >= dX) {
        h -= dX;
        y += sY;
      }
    }
  }
  else {
    h = dY / 2;
    for (int i=0; i<dY; ++i) {
      gridMap_->updateCellAsFree(x, y + i * sY);
      h += dX;
      if (h >= dY) {
        h -= dY;
        x += sX;
      }
    }
  }
}