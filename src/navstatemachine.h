#ifndef NAVSTATEMACHINE_H
#define NAVSTATEMACHINE_H

#include "movemanager.h"
#include "scanlistener.h"
#include "gridcalculator.h"
#include "pathplanner.h"
#include "gridmap.h"
#include "signalhandler.h"
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>

enum States {Init, GetScan, ActualizeMap, PlanPath, MoveRobot, CheckSignals};

class NavStateMachine
{
  public:
    NavStateMachine(int argc, char **argv);
    virtual ~NavStateMachine();
  protected:
  private:
    static const unsigned int MAP_SIZE;
    static const unsigned int RATE;
    void transition(int argc, char **argv);
    void init(int argc, char **argv);
    void getScan();
    void actualizeMap();
    void planPath();
    void moveRobot();
    void checkSignals();

    enum States state_;
    SignalHandler* signalHandler_;
    MoveManager* moveManager_;
    ScanListener* scanListener_;
    GridCalculator* gridCalculator_;
    PathPlanner* pathPlanner_;
    GridMap* gridMap_;
    ScanMsgFormatted actual_scan_;
    geometry_msgs::Pose2D pose_;
    int i_next_degree_;
    int i_next_distance_;
};

#endif // NAVSTATEMACHINE_H
