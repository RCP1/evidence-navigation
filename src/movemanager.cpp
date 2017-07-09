#include "movemanager.h"

using namespace std;

MoveManager::MoveManager()
{
  // port "/dev/ttyUSB0", 38400 (standard of RP6), timeout in milliseconds
  serialhandler_ = new serial::Serial("/dev/ttyUSB0", 38400, serial::Timeout::simpleTimeout(1000));
  // Set RTS pin on low for not resetting RP6
  serialhandler_->setRTS(false);

}

MoveManager::~MoveManager()
{
  //dtor
}

bool MoveManager::initConn()
{
  bool b_opened = !serialhandler_->isOpen();
  cout << "Press start now!" << endl;
  sleep(4);
  cout << "Done!" << endl;
  return b_opened;
}

void MoveManager::closeConn()
{
  serialhandler_->close();
}

int MoveManager::rotate(int int_degree)
{
 //TODO
 return 0;
}

int MoveManager::move(int int_distance_mm)
{
 //TODO
 return 0;
}

int MoveManager::rotateAndMove(int int_degree, int int_distance_mm)
{
  string str_resp;
  size_t bytes_txrx = 0;
  char buffer[4];
  int int_error_cnt = -1;

  if(abs(int_degree) > 360 || abs(int_distance_mm) >= 1000) // Not possible
    return 1;

  /**
   * Move command:
   * ui16_rotation as angle: char[3]
   * rot_dir: char //2:LEFT, 3:RIGHT
   * ui16_distance in mm: char[3]
   * dist_dir: char //0:FWD, 1:BWD
   * < As End of Command!
   * > Is sent, if cmd was successful
   */
  sprintf(buffer, "%03d", abs(int_degree));
  str_cmd_ = string(buffer);

  if(int_degree >= 0)
    str_cmd_ += "2";
  else
    str_cmd_ += "3";

  sprintf(buffer, "%03d", abs(int_distance_mm));
  str_cmd_ += string(buffer);

  if(int_distance_mm >= 0)
    str_cmd_ += "0";
  else
    str_cmd_ += "1";

  str_cmd_ += "<";



  serialhandler_->flushInput();
  // Send
  do
  {
    bytes_txrx = serialhandler_->write(str_cmd_);
    int_error_cnt++;
    // Wait for received bytes and read those into string
    do
    {
      bytes_txrx = serialhandler_->read(str_resp, 1);
    }while(bytes_txrx < 1);
    // Answer is error? Return error 2
    if(int_error_cnt >= 100 || str_resp == "<")
      break;
  }while(str_resp != ">");

  if(str_resp == ">")
    return 0;
  else
  {
    cout << "Unable to send command, either wrong format or RP6 did not synchronized" << endl;
    return 2;
  }
}