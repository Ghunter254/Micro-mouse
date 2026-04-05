#include "Navigation.h"

Config::State Navigator::_currentState = Config::IDLE;
uint8_t Navigator::_posX = 0;   
uint8_t Navigator::_posY = 0;   
int16_t Navigator::_targetHeading = 0;

uint8_t Navigator::_distMap[Config::MAZE_SIZE][Config::MAZE_SIZE] = {0}; 
uint8_t Navigator::_maze[Config::MAZE_SIZE][Config::MAZE_SIZE] = {0};

void Navigator::init()
{
  _posX = 0; _posY = 0;
  _targetHeading = 0; // Start facing North
  _currentState = Config::IDLE;

  for (uint8_t y=0; y< Config::MAZE_SIZE; y++) {
    for (uint8_t x=0; x < Config::MAZE_SIZE; x++){
      // Manhattan distance to center (7,7) to (8,8)
      uint8_t dx = (x < 8) ? (7 - x) : (x - 8);
      uint8_t dy = (y < 8) ? (7 - y) : (y - 8);
      _distMap[x][y] = dx + dy;
      _maze[x][y] = 0;
    }
  }
}

void Navigator::update()
{
  switch(_currentState){
    case Config::IDLE:
      break;
    
    case Config::EXPLORE:
      // scan walls at the current position.
      updateMap();

      // Decide next move based on flood-fill.
      computePath();

      determineNextDirection();
      break;

    case Config::SPRINT:
      // write logic for high-speed runs here .. todo
      break;

    case Config::CALIBRATE:
      alignWithWalls();
      _currentState = Config::IDLE;
      break;

  }
}

void Navigator::moveForwardOneCell()
{
  SensorManager::resetEncoders();
  int32_t startTicks = SensorManager::getEncoderTicks();

  while ((SensorManager::getEncoderTicks() - startTicks) < Config::TICKS_PER_CELL) {
    // Use Gyro to stay on _targetHeading
    int16_t currentHeading = SensorManager::getHeading();
    int16_t error = _targetHeading - currentHeading; 

    // Wrap around fix.
    if (error > 512)  error -= 1024;
    if (error < -512) error += 1024;
    
    // p controller.
    int16_t correction = (error * 5) >> 2; 
    MotorController::setSpeed(Config::SPEED_EXPLORE + correction, Config::SPEED_EXPLORE - correction);
    
    //  If front is too close we break.
    if (SensorManager::getDistance(Config::FRONT) < 60) break;
  }

  MotorController::hardBrake();
  delay(100);

  updateCoordinates();
}

void Navigator::alignWithWalls()
{
  uint16_t L = SensorManager::getDistance(Config::LEFT);
  uint16_t R = SensorManager::getDistance(Config::RIGHT);

  if (L< 100 && R < 100) {
    int16_t diff = L - R;

    while (abs(diff) > 5) {
      if (diff > 0) MotorController::setSpeed(-200, 200); // Turn Left
        else MotorController::setSpeed(200, -200);         // Turn Right
        L = SensorManager::getDistance(Config::LEFT);
        R = SensorManager::getDistance(Config::RIGHT);
        diff = L - R;
    }
  }

  MotorController::hardBrake();
}

void Navigator::updateMap() {
  uint8_t relativeWalls = 0;
  if (SensorManager::isWallDetected(Config::FRONT)) relativeWalls |= 0x01; // "Up"
  if (SensorManager::isWallDetected(Config::RIGHT)) relativeWalls |= 0x02; // "Right"
  if (SensorManager::isWallDetected(Config::BACK))  relativeWalls |= 0x04; // "Down"
  if (SensorManager::isWallDetected(Config::LEFT))  relativeWalls |= 0x08; // "Left"

  // Shift based on heading: 0=N, 256=E, 512=S, 768=W
  uint8_t shift = _targetHeading >> 8; // 0, 1, 2, or 3
  
  // Circular shift the bits to align with North/East/South/West
  uint8_t absoluteWalls = ((relativeWalls << shift) | (relativeWalls >> (4 - shift))) & 0x0F;
  
  _maze[_posX][_posY] |= (absoluteWalls | Config::VISITED);

}

void Navigator::computePath() {
  bool changed = true;
  while (changed) {
      changed = false;
      for (uint8_t x = 0; x < 16; x++) {
          for (uint8_t y = 0; y < 16; y++) {
              if (_distMap[x][y] == 0) continue; // Goal is always 0

              uint8_t minVal = 255;
              uint8_t walls = _maze[x][y];

              // Check North neighbor
              if (!(walls & Config::WALL_N) && y < 15)
                  if (_distMap[x][y+1] < minVal) minVal = _distMap[x][y+1];
              // Check East
              if (!(walls & Config::WALL_E) && x < 15)
                  if (_distMap[x+1][y] < minVal) minVal = _distMap[x+1][y];
              // Check South
              if (!(walls & Config::WALL_S) && y > 0)
                  if (_distMap[x][y-1] < minVal) minVal = _distMap[x][y-1];
              // Check West
              if (!(walls & Config::WALL_W) && x > 0)
                  if (_distMap[x-1][y] < minVal) minVal = _distMap[x-1][y];

              if (_distMap[x][y] != minVal + 1) {
                  _distMap[x][y] = minVal + 1;
                  changed = true;
              }
          }
      }
  }
}

void Navigator::turn90(uint8_t clockwise) {
  if (clockwise) _targetHeading = (_targetHeading + 256) & 1023;
  else  _targetHeading = (_targetHeading - 256) & 1023;

  while (abs(_targetHeading - SensorManager::getHeading()) > 10) {
      int16_t error = _targetHeading - SensorManager::getHeading();
      // Handle the wrap-around for the shortest turn
      if (error > 512) error -= 1024;
      if (error < -512) error += 1024;

      int16_t p_speed = (error * 4); // Fixed point gain
      MotorController::setSpeed(p_speed, -p_speed);
  }
  MotorController::hardBrake();
}

void Navigator::determineNextDirection() {
    uint8_t x = _posX;
    uint8_t y = _posY;
    uint8_t currentDist = _distMap[x][y];
    uint8_t walls = _maze[x][y];

    if (currentDist == 0) {
      _currentState = Config::IDLE;
      MotorController::hardBrake();
      // todo ...add some sort of feedback here ...visual oor haptic or something
      return;
    }

    // We look for a neighbor whose distance is exactly currentDist - 1

    // Try North
    if (!(walls & Config::WALL_N) && y < 15 && _distMap[x][y+1] == currentDist - 1) {
        faceHeading(0); // North
    }
    //Try East
    else if (!(walls & Config::WALL_E) && x < 15 && _distMap[x+1][y] == currentDist - 1) {
        faceHeading(256); // East
    }
    // Try South
    else if (!(walls & Config::WALL_S) && y > 0 && _distMap[x][y-1] == currentDist - 1) {
        faceHeading(512); // South
    }
    //Try West
    else if (!(walls & Config::WALL_W) && x > 0 && _distMap[x-1][y] == currentDist - 1) {
        faceHeading(768); // West
    }

    // Now that we are facing the right way, move!
    moveForwardOneCell();
}


void Navigator::updateCoordinates() {
    // 0 = North, 256 = East, 512 = South, 768 = West
    if (_targetHeading == 0)   _posY++;
    if (_targetHeading == 256) _posX++;
    if (_targetHeading == 512) _posY--;
    if (_targetHeading == 768) _posX--;
}

void Navigator::faceHeading(int16_t requiredHeading) {
    if (_targetHeading == requiredHeading) {
        return;
    }
    int16_t diff = (requiredHeading - _targetHeading) & 1023;

    if (diff == 256) {
        turn90(1); // Clockwise
    } else if (diff == 768) {
        turn90(0); // Counter-Clockwise
    } else if (diff == 512) {
        turn90(1); // U-Turn (Two 90s)
        turn90(1);
    }
}



























