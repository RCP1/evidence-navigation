#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <vector>
#include <map>
#include <geometry_msgs/Pose2D.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include "robotmodel.h"

class GridMap
{
  public:
    static const unsigned int GRID_RES_IN_MM;

    GridMap(unsigned int ui_size);
    virtual ~GridMap();
    void copy(const GridMap* src_grid);
    std::vector<std::vector<unsigned int> > getGrid() const;
    void setGrid(std::vector<std::vector<unsigned int> > evidence_grid);
    unsigned int getSize() const;
    void setSize(unsigned int ui_size);
    int updateCellAsOccupied(unsigned int ui_x, unsigned int ui_y);
    int updateCellAsFree(unsigned int ui_x, unsigned int ui_y);
    geometry_msgs::Pose2D getPose() const;
    void setPose(geometry_msgs::Pose2D pose);
    unsigned int getOccupancy(unsigned int ui_x, unsigned int ui_y) const; // For gtest pub
    int writeBPM(std::string str_filepath);
    void filterGradients();
    void enlargeObstacles();
    void enlargePoint(unsigned int ui_x, unsigned int ui_y);
    GridMap resizeOccpyForSearch(unsigned int ui_resize_scale);
    int setOccupancy(unsigned int ui_x, unsigned int ui_y, int i_occupancy);
  protected:

  private:
    static const unsigned int DEFAULT_OCCUPANCY;
    static const int MAX_OCCUPANCY;
    static const int MIN_OCCUPANCY;
    static const unsigned int OFFSET_OCCUPANCY_OCCUPIED;
    static const unsigned int OFFSET_OCCUPANCY_FREE;

    void init();


    std::vector<std::vector<unsigned int> > evidence_grid_;
    unsigned int ui_size_;
    geometry_msgs::Pose2D pose_;
};

#endif // GRIDMAP_H
