#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "Config.h"
#include "Sensor.h"
#include "Motors.h"
#include <cstdint>

enum SprintCmd : uint8_t { 
    CMD_FORWARD = 0, 
    CMD_TURN_R  = 1, 
    CMD_TURN_L  = 2, 
    CMD_FINISH  = 3 
};

struct SprintMove {
    SprintCmd command;
    uint8_t cells; // How many cells to we can go through once.
};

class Navigator {
public:
    static void init();   
    static void update(); 
    
    static void moveForwardOneCell();
    static void turn90(uint8_t clockwise); // 1 for cw and 0 for ccw
    static void alignWithWalls();
    
    // Mapping Logic
    static void updateMap();      
    static void computePath();    
    static void determineNextDirection();
    static void updateCoordinates();
    static void faceHeading(int16_t requiredHeading);

    static void generateSprintPath();
    static void executeSprint();
    static void sprintForward(uint8_t cells);

    void setState(Config::State newState) { _currentState = newState; }
    
private:
  static Config::State _currentState;
  static uint8_t  _posX, _posY;      // Current grid coordinates (0-15)
  static int16_t  _targetHeading;    // In 1024-scale
  static bool _isReturning;

  // 1 byte per cell. 
  // Bits [0-3]: Walls, Bit [4]: Visited, Bit [5-7]: Reserved
  static uint8_t  _maze[Config::MAZE_SIZE][Config::MAZE_SIZE];
  static uint8_t  _distMap[Config::MAZE_SIZE][Config::MAZE_SIZE];

  // sprint variables
  static SprintMove _sprintPath[150]; // Buffer to hold the compressed route
  static uint8_t _totalSprintMoves;   // How many steps in the final route
  static uint8_t _currentMoveIndex;   // Which step we are currently executing

  // util
  static bool canMove(uint8_t x, uint8_t y, uint8_t direction);
  static void setTarget(bool returnToStart);
};

#endif