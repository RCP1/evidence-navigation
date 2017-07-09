#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "gridmap.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include "robotmodel.h"
#include "mapsearchnode.h"
#include "stlastar.h"

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0
#define PI 3.141592654

class PathPlanner
{
  public:
    PathPlanner(GridMap* gridMap);
    virtual ~PathPlanner();
    void setGoalPoint(unsigned int ui_x, unsigned int ui_y);
    geometry_msgs::Pose2D getNextPose();
    void fromPoseToDistance(int& i_next_degree, int& i_next_distance, geometry_msgs::Pose2D& pose);
    geometry_msgs::Point transformRobotToWorldFrame(geometry_msgs::Point point_robot_frame,
                                                    geometry_msgs::Point robot_pos_world_frame,
                                                    double d_robot_angle);
    void simplePath(int& i_next_degree, int& i_next_distance, geometry_msgs::Pose2D& pose);
  protected:
  private:
    static const unsigned int RESIZE_FACTOR;
    GridMap* gridMap_;
    GridMap* gridMapFiltered_;
    AStarSearch<MapSearchNode> astarsearch_;
    unsigned int ui_goal_x_;
    unsigned int ui_goal_y_;
};

#endif // PATHPLANNER_H
