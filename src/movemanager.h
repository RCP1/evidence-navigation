#ifndef MOVEMANAGER_H
#define MOVEMANAGER_H

#include "serial/serial.h"
#include <stdlib.h>
#include <string>
#include <iostream>
#include <unistd.h>

class MoveManager
{
  public:
    MoveManager();
    virtual ~MoveManager();
    /* Opens and closes connection to RP6, returns 0, if successful */
    bool initConn();
    void closeConn();
    /* Rotate or move or do both with following functions */
    int rotate(int int_degree);
    int move(int int_distance_mm);
    int rotateAndMove(int int_degree, int int_distance_mm);
  protected:
  private:
    std::string str_cmd_;
    serial::Serial* serialhandler_;
};

#endif // MOVEMANAGER_H
