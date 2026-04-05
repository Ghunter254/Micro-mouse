#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "Config.h"
#include "Sensor.h"
#include "Motors.h"
#include <cstdint>



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

    void setState(Config::State newState) { _currentState = newState; }
    
private:
  static Config::State _currentState;
  static uint8_t  _posX, _posY;      // Current grid coordinates (0-15)
  static int16_t  _targetHeading;    // In 1024-scale

  // 1 byte per cell. 
  // Bits [0-3]: Walls, Bit [4]: Visited, Bit [5-7]: Reserved
  static uint8_t  _maze[Config::MAZE_SIZE][Config::MAZE_SIZE];
  static uint8_t  _distMap[Config::MAZE_SIZE][Config::MAZE_SIZE];

  // util
  static bool canMove(uint8_t x, uint8_t y, uint8_t direction);
};

#endif