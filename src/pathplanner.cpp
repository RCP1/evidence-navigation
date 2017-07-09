#include "pathplanner.h"

/*
 *  Constants
 */
const unsigned int PathPlanner::RESIZE_FACTOR =  10; // cell size in [cm]


PathPlanner::PathPlanner(GridMap* gridMap)
  : gridMap_(gridMap)
{
  //ctor
  gridMapFiltered_ = new GridMap(gridMap_->getSize());
}

PathPlanner::~PathPlanner()
{
  //dtor
}



void PathPlanner::setGoalPoint(unsigned int ui_x, unsigned int ui_y)
{
  ui_goal_x_ = ui_x / RESIZE_FACTOR;
  ui_goal_y_ = ui_y / RESIZE_FACTOR;
}

geometry_msgs::Pose2D PathPlanner::getNextPose()
{
  geometry_msgs::Pose2D result;
  gridMapFiltered_->copy(gridMap_);
  gridMapFiltered_->writeBPM("/home/udoo/1.ppm");
  gridMapFiltered_->filterGradients();
  gridMapFiltered_->enlargeObstacles();
  //gridMapFiltered_->writeBPM("/home/udoo/2.ppm");
  GridMap gridMapResized = gridMapFiltered_->resizeOccpyForSearch(RESIZE_FACTOR);


  // Create a start state
  MapSearchNode nodeStart(&gridMapResized);
  nodeStart.x = gridMapResized.getPose().x;
  nodeStart.y = gridMapResized.getPose().y;

  // Define the goal state
  MapSearchNode nodeEnd(&gridMapResized);
  nodeEnd.x = ui_goal_x_;
  nodeEnd.y = ui_goal_y_;

  // Set Start and goal states
  astarsearch_.SetStartAndGoalStates( nodeStart, nodeEnd );

  unsigned int SearchState;

  do
  {
    SearchState = astarsearch_.SearchStep();

  }
  while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

  if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
  {
    cout << "Search found goal state\n";

      MapSearchNode* node = astarsearch_.GetSolutionStart();
      int steps = 0;
      node = astarsearch_.GetSolutionNext();
      result.x = node->x * RESIZE_FACTOR;
      result.y = node->y * RESIZE_FACTOR;
      //node->PrintNodeInfo();
      for( ;; )
      {
        node = astarsearch_.GetSolutionNext();
        if( !node )
        {
          break;
        }
        gridMapResized.setOccupancy(node->x, node->y, 100);

        //node->PrintNodeInfo();
        steps++;

      };
      // Once you're done with the solution you can free the nodes up
      astarsearch_.FreeSolutionNodes();

  }
  else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED )
  {
    result = gridMap_->getPose();
    cout << "Search terminated. Did not find goal state\n";
  }

  astarsearch_.EnsureMemoryFreed();
  //gridMapResized.writeBPM("/home/udoo/3.ppm");

  return result;
}

void PathPlanner::fromPoseToDistance(int& i_next_degree, int& i_next_distance, geometry_msgs::Pose2D& pose)
{
  i_next_distance = RESIZE_FACTOR * GridMap::GRID_RES_IN_MM;

  if(pose.x > gridMap_->getPose().x)
  {
    pose.theta = 0.0;
  }
  if(pose.x< gridMap_->getPose().x)
  {
    pose.theta = PI;
  }
  if(pose.y < gridMap_->getPose().y)
  {
    pose.theta = 1.5 * PI;
  }
  if(pose.y > gridMap_->getPose().y)
  {
    pose.theta = 0.5 * PI;
  }

  i_next_degree = (int)(360 * (pose.theta - gridMap_->getPose().theta) / (2 * PI));

  if(i_next_degree > 180)
      i_next_degree = i_next_degree - 360;
  if(i_next_degree < -180)
      i_next_degree = i_next_degree + 360;


  cout << i_next_degree << " " << 360 * pose.theta / (2 * PI) << " " << pose.x << " " << pose.y << " " << gridMap_->getPose().x << " " << gridMap_->getPose().y << endl;
}

geometry_msgs::Point PathPlanner::transformRobotToWorldFrame(geometry_msgs::Point point_robot_frame, geometry_msgs::Point robot_pos_world_frame, double d_robot_angle)
{
  geometry_msgs::Point point_world_frame;
  point_world_frame.x = robot_pos_world_frame.x +
                  (int)((cos(-d_robot_angle) * point_robot_frame.x) + //Rotation applied to x of scan point
                  (sin(-d_robot_angle) * point_robot_frame.y));
  point_world_frame.y = robot_pos_world_frame.y +
                  (int)(- (sin(-d_robot_angle) * point_robot_frame.x) + //Rotation applied to y of scan point
                  (cos(-d_robot_angle) * point_robot_frame.y));
  return point_world_frame;
}

void PathPlanner::simplePath(int& i_next_degree, int& i_next_distance, geometry_msgs::Pose2D& pose)
{
  int x,y,sX,sY,dX,dY,h;

  gridMap_->writeBPM("/home/udoo/1.ppm");
  gridMapFiltered_->copy(gridMap_);
  gridMapFiltered_->filterGradients();
  gridMapFiltered_->enlargeObstacles();
  gridMapFiltered_->writeBPM("/home/udoo/2.ppm");

  geometry_msgs::Point robot_pos_world_frame;
  robot_pos_world_frame.x = (unsigned int)gridMapFiltered_->getPose().x * GridMap::GRID_RES_IN_MM + OFFSET_MID_OF_CM_CELL;
  robot_pos_world_frame.y = (unsigned int)gridMapFiltered_->getPose().y * GridMap::GRID_RES_IN_MM + OFFSET_MID_OF_CM_CELL;

  geometry_msgs::Point goal_point_robot_frame;
  goal_point_robot_frame.x = GridMap::GRID_RES_IN_MM * RESIZE_FACTOR;
  goal_point_robot_frame.y = 0;
  geometry_msgs::Point goal_point_world_frame = transformRobotToWorldFrame(goal_point_robot_frame, robot_pos_world_frame,
                                                    gridMapFiltered_->getPose().theta);

  bool b_way_free = true;

  /* Get occupancy on line to measured point with Bresenham */
  x = robot_pos_world_frame.x;
  sX = (goal_point_world_frame.x < robot_pos_world_frame.x ? -1 : 1);
  dX = abs(goal_point_world_frame.x - robot_pos_world_frame.x);

  y = robot_pos_world_frame.y;
  sY = (goal_point_world_frame.y < robot_pos_world_frame.y ? -1 : 1);
  dY = abs(goal_point_world_frame.y - robot_pos_world_frame.y);

  if (dX >= dY) {
    h = dX/2;
    for (int i=0; i < dX; ++i) {
      if(gridMapFiltered_->getOccupancy((x + i * sX) / GridMap::GRID_RES_IN_MM, y / GridMap::GRID_RES_IN_MM) == 100)
      {
        b_way_free = false;
        cout << (x + i * sX) / GridMap::GRID_RES_IN_MM << " " << y / GridMap::GRID_RES_IN_MM << " "  << gridMapFiltered_->getOccupancy((x + i * sX) / GridMap::GRID_RES_IN_MM, y / GridMap::GRID_RES_IN_MM);
      }
      h += dY;
      if (h >= dX) {
        h -= dX;
        y += sY;
      }
    }
  }
  else {
    h = dY / 2;
    for (int i=0; i < dY; ++i) {
      if(gridMapFiltered_->getOccupancy(x / GridMap::GRID_RES_IN_MM,(y + i * sY) / GridMap::GRID_RES_IN_MM) == 100)
      {
        b_way_free = false;
        cout << x / GridMap::GRID_RES_IN_MM << " " << (y+i*sY) / GridMap::GRID_RES_IN_MM << " "  << gridMapFiltered_->getOccupancy(x / GridMap::GRID_RES_IN_MM,(y + i * sY) / GridMap::GRID_RES_IN_MM);
      }
      h += dX;
      if (h >= dY) {
        h -= dY;
        x += sX;
      }
    }
  }

  if(b_way_free)
  {
    i_next_distance = GridMap::GRID_RES_IN_MM * RESIZE_FACTOR;
    i_next_degree = 0;
    pose.x = (unsigned int)(goal_point_world_frame.x / GridMap::GRID_RES_IN_MM);
    pose.y = (unsigned int)(goal_point_world_frame.y / GridMap::GRID_RES_IN_MM);
  }
  else
  {
    i_next_distance = 0;
    // Rotate 120 degrees
    i_next_degree = 120;
    pose.theta += ((double)i_next_degree / 360) * 2 * PI;
    if(pose.theta >= 2 * PI)
      pose.theta -= 2 * PI;
    pose.x = gridMapFiltered_->getPose().x;
    pose.y = gridMapFiltered_->getPose().y;
  }
  cout << i_next_degree << " " << 360 * pose.theta / (2 * PI) << " " << pose.x << " " << pose.y << " " << gridMapFiltered_->getPose().x << " " << gridMapFiltered_->getPose().y << endl;
}