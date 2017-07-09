#include "gridmap.h"

const unsigned int GridMap::DEFAULT_OCCUPANCY = 50;
const int GridMap::MAX_OCCUPANCY = 100;
const int GridMap::MIN_OCCUPANCY = 0;
const unsigned int GridMap::GRID_RES_IN_MM = 10;
const unsigned int GridMap::OFFSET_OCCUPANCY_OCCUPIED = 2;
const unsigned int GridMap::OFFSET_OCCUPANCY_FREE = 1;

GridMap::GridMap(unsigned int ui_size)
  : ui_size_(ui_size)
{
  init();
}

GridMap::~GridMap()
{
  //dtor
}

void GridMap::copy(const GridMap* src_grid)
{
  this->setPose(src_grid->getPose());
  this->setSize(src_grid->getSize());

  evidence_grid_.clear();
  //evidence_grid_.swap(std::vector<std::vector<unsigned int> >(evidence_grid_));
  std::vector<std::vector<unsigned int> >(evidence_grid_).swap(evidence_grid_);
  for(unsigned int y = 0; y < ui_size_; ++y)
  {
    std::vector<unsigned int> row;
    for(unsigned int x = 0; x < ui_size_; ++x)
    {
      row.push_back(src_grid->getOccupancy(x, y));
    }
    evidence_grid_.push_back(row);
  }
}

void GridMap::init()
{
  for(unsigned int y = 0; y < ui_size_; ++y)
  {
    std::vector<unsigned int> row;
    for(unsigned int x = 0; x < ui_size_; ++x)
    {
      row.push_back(DEFAULT_OCCUPANCY);
    }
    evidence_grid_.push_back(row);
  }

  pose_.x = (unsigned int)(ui_size_ / 2);
  pose_.y = (unsigned int)(ui_size_ / 2);
  pose_.theta = 0;
}


std::vector<std::vector<unsigned int> > GridMap::getGrid() const
{
  return evidence_grid_;
}

void GridMap::setGrid(std::vector<std::vector<unsigned int> > evidence_grid)
{
  evidence_grid_ = evidence_grid;
}


unsigned int GridMap::getSize() const
{
  return ui_size_;
}

void GridMap::setSize(unsigned int ui_size)
{
  ui_size_ = ui_size;
}

unsigned int GridMap::getOccupancy(unsigned int ui_x, unsigned int ui_y) const
{
  if( ui_x < 0 || ui_x >= ui_size_ ||
		  ui_y < 0 || ui_y >= ui_size_)
	{
		return 100;
	}
	else
    return evidence_grid_[ui_y][ui_x];
}

int GridMap::setOccupancy(unsigned int ui_x, unsigned int ui_y, int i_occupancy)
{
  if( ui_x < 0 || ui_x >= ui_size_ ||
    ui_y < 0 || ui_y >= ui_size_)
	{
		return 1;
	}
  if(i_occupancy > MAX_OCCUPANCY)
    evidence_grid_[ui_y][ui_x] = MAX_OCCUPANCY;
  else if(i_occupancy < MIN_OCCUPANCY)
    evidence_grid_[ui_y][ui_x] = MIN_OCCUPANCY;
  else
    evidence_grid_[ui_y][ui_x] = i_occupancy;
  return 0;
}

int GridMap::updateCellAsOccupied(unsigned int ui_x, unsigned int ui_y)
{
  // Add offset to actual occupancy at point
  return this->setOccupancy(ui_x, ui_y, (int)(this->getOccupancy(ui_x, ui_y) + OFFSET_OCCUPANCY_OCCUPIED));
}

int GridMap::updateCellAsFree(unsigned int ui_x, unsigned int ui_y)
{
  // Subtract offset of actual occupancy at point
  return this->setOccupancy(ui_x, ui_y, (int)(this->getOccupancy(ui_x, ui_y) - OFFSET_OCCUPANCY_FREE));
}

geometry_msgs::Pose2D GridMap::getPose() const
{
  return pose_;
}

void GridMap::setPose(geometry_msgs::Pose2D pose)
{
  pose_ = pose;
}

int GridMap::writeBPM(std::string str_filepath)
{
  int x, y;
  FILE *fp = fopen(str_filepath.c_str(), "wb"); /* b - binary mode */
  (void) fprintf(fp, "P6\n%d %d\n%d\n", ui_size_, ui_size_, MAX_OCCUPANCY);
  for (y = ui_size_ - 1; y >= 0; --y) // 0 is top row
  {
    for (x = 0; x < ui_size_; ++x)
    {
      static unsigned char color[3];
      // (MAX value - actual value) to inverse image (white is free, black is occupied
      color[0] = MAX_OCCUPANCY - (char) evidence_grid_[y][x] ; //* 2.56;  /* red */
      color[1] = MAX_OCCUPANCY - (char) evidence_grid_[y][x] ; //* 2.56;  /* green */
      color[2] = MAX_OCCUPANCY - (char) evidence_grid_[y][x] ; //* 2.56;  /* blue */
      (void) fwrite(color, 1, 3, fp);
    }
  }
  (void) fclose(fp);
  return 0;
}

void GridMap::filterGradients()
{
  for(unsigned int y = 0; y < ui_size_; ++y)
  {
    for(unsigned int x = 0; x < ui_size_; ++x)
    {
      if(evidence_grid_[y][x] > DEFAULT_OCCUPANCY) // = obstacle
        evidence_grid_[y][x] = MAX_OCCUPANCY;
      else // = free
        evidence_grid_[y][x] = MIN_OCCUPANCY;
    }
  }
}

void GridMap::enlargeObstacles()
{
  std::map<unsigned int, unsigned int> map_obstacle_points;
  for(unsigned int y = 0; y < ui_size_; ++y)
  {
    for(unsigned int x = 0; x < ui_size_; ++x)
    {
      if(evidence_grid_[y][x] == MAX_OCCUPANCY) // = obstacle
      {
        map_obstacle_points[x] = y;

      }
    }
  }


  for(std::map<unsigned int, unsigned int>::iterator it = map_obstacle_points.begin(); it != map_obstacle_points.end(); ++it)
  {
    enlargePoint(it->first, it->second);
  }
}

void GridMap::enlargePoint(unsigned int ui_x, unsigned int ui_y)
{
  int ui_r_robot = ceil(sqrt(MID_TO_FRAME_X_MM * MID_TO_FRAME_X_MM + MID_TO_FRAME_Y_MM * MID_TO_FRAME_Y_MM) / GRID_RES_IN_MM);

  for(int y = -ui_r_robot; y <= ui_r_robot; y++)
  {
    for(int x = -ui_r_robot; x <= ui_r_robot; x++)
    {
        if(x * x + y * y <= ui_r_robot * ui_r_robot)
        {
          if(ui_y + y < 0 || ui_x + x < 0)
            break;
          if(ui_y + y >= ui_size_ || ui_x + x >= ui_size_)
            break;
          evidence_grid_[ui_y + y][ui_x + x] = MAX_OCCUPANCY;
        }
    }
  }
}

GridMap GridMap::resizeOccpyForSearch(unsigned int ui_resize_scale)
{
  GridMap resized_gridMap(ui_size_ / ui_resize_scale);
  geometry_msgs::Pose2D resized_pose;
  resized_pose.x = (int) (pose_.x / ui_resize_scale);
  resized_pose.y = (int) (pose_.y / ui_resize_scale);
  resized_pose.theta = pose_.theta;
  resized_gridMap.setPose(resized_pose);

  for(unsigned int y_resized = 0; y_resized < ui_size_ / ui_resize_scale; ++y_resized)
  {
    for(unsigned int x_resized = 0; x_resized < ui_size_ / ui_resize_scale; ++x_resized)
    {
      for(unsigned int y = y_resized * ui_resize_scale; y < (y_resized + 1) * ui_resize_scale; ++y)
      {
        for(unsigned int x = x_resized * ui_resize_scale; x < (x_resized + 1) * ui_resize_scale; ++x)
        {
          if(evidence_grid_[y][x] == MAX_OCCUPANCY) // = obstacle
          {
            resized_gridMap.setOccupancy(x_resized, y_resized, 9); // 9 tells astar search, that this can not be passed
          }
          else
          {
            resized_gridMap.setOccupancy(x_resized, y_resized, 1); // 1 tells astar search, that this is free
          }
        }
      }
    }
  }

  return resized_gridMap;
}

