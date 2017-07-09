#include "navstatemachine.h"

/*
 *  Constants
 */
const unsigned int NavStateMachine::MAP_SIZE =  1000; // cell count in x and y
const unsigned int NavStateMachine::RATE =  30; // hz

NavStateMachine::NavStateMachine(int argc, char **argv)
{
  transition(argc, argv);
}

NavStateMachine::~NavStateMachine()
{
  //dtor
}

void NavStateMachine::transition(int argc, char **argv) {
  // For ros slepping, init time
  ros::Time::init();
  // Rate for sleep each state
  ros::Rate loop_rate(RATE);
	while(true)
	{
    switch(state_)
    {
      case Init:
        std::cout << "Init" << std::endl;
        init(argc, argv);
        state_ = GetScan;
        break;
      case GetScan:
        std::cout << "GetScan" << std::endl;
        getScan();
        state_ = ActualizeMap;
        break;
      case ActualizeMap:
        std::cout << "ActualizeMap" << std::endl;
        actualizeMap();
        state_ = PlanPath;
        break;
      case PlanPath:
        std::cout << "PlanPath" << std::endl;
        planPath();
        state_ = MoveRobot;
        break;
      case MoveRobot:
        std::cout << "MoveRobot" << std::endl;
        moveRobot();
        state_ = CheckSignals;
        break;
      case CheckSignals:
        std::cout << "CheckSignals" << std::endl;
        checkSignals();
        state_ = GetScan;
        break;
      default:
        state_ = Init;
        break;
    }
    loop_rate.sleep();
	}
}

void NavStateMachine::init(int argc, char **argv)
{
  signalHandler_ = new SignalHandler();
  signalHandler_->setupSignalHandlers();
  // Disable from here for disabling RP6

  moveManager_ = new MoveManager();
  if(moveManager_->initConn())
  {
    std::cout << "Can not establish serial connection! Exiting." << std::endl;
    exit(1);
  }
  // Disable to here */
  gridMap_ = new GridMap(MAP_SIZE);
  scanListener_ = new ScanListener(argc, argv);
  gridCalculator_ = new GridCalculator(gridMap_);
  pathPlanner_ = new PathPlanner(gridMap_);
}

void NavStateMachine::getScan()
{
  actual_scan_ = scanListener_->getActualScan();
}

void NavStateMachine::actualizeMap()
{

  gridCalculator_->addScanToGrid(actual_scan_.vd_scan_points_,
                                  actual_scan_.d_angle_increment_,
                                  actual_scan_.d_angle_min_,
                                  actual_scan_.d_angle_max_);

}

void NavStateMachine::planPath()
{
  //pathPlanner_->setGoalPoint(gridMap_->getPose().x + 100, gridMap_->getPose().y + 100);
  //pose_ = pathPlanner_->getNextPose();
  // Simple path, avoid collision
  pose_ = gridMap_->getPose();
  pathPlanner_->simplePath(i_next_degree_,i_next_distance_,pose_);
}

void NavStateMachine::moveRobot()
{
  moveManager_->rotateAndMove(i_next_degree_, i_next_distance_);
  gridMap_->setPose(pose_);
}

void NavStateMachine::checkSignals()
{
  // signal ctrl-c (KILL) received
  if(signalHandler_->gotExitSignal())
  {
    moveManager_->closeConn();
    delete moveManager_;
    delete scanListener_;
    delete gridCalculator_;
    delete pathPlanner_;
    delete gridMap_;
    exit(1);
  }
}
