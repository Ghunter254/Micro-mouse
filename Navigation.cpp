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
  _isReturning = false;

  for (uint8_t x = 0; x < Config::MAZE_SIZE; x++) {
      for (uint8_t y = 0; y < Config::MAZE_SIZE; y++) {
          _maze[x][y] = 0;
      }
  }

  setTarget(false);
  computePath();
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
      executeSprint();
      break;

    case Config::CALIBRATE:
      alignWithWalls();
      _currentState = Config::IDLE;
      break;

  }
}

void Navigator::moveForwardOneCell()
{
  Position startingPosition = SensorManager::getPosition();
  int32_t posY = startingPosition.y;

  while ((SensorManager::getForwardDistance() - posY) < Config::TICKS_PER_CELL) {
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
  
  // Update the current wall.
  _maze[_posX][_posY] |= (absoluteWalls | Config::VISITED);

  // Updating the neighbouring walls.
  // If we have a North wall, the cell above us has a South wall.
  if ((absoluteWalls & Config::WALL_N) && (_posY < Config::MAZE_SIZE - 1)) {
      _maze[_posX][_posY + 1] |= Config::WALL_S;
  }

  // If we have an East wall, the cell to the right has a West wall.
  if ((absoluteWalls & Config::WALL_E) && (_posX < Config::MAZE_SIZE - 1)) {
      _maze[_posX + 1][_posY] |= Config::WALL_W;
  }

  // If we have a South wall, the cell below us has a North wall.
  if ((absoluteWalls & Config::WALL_S) && (_posY > 0)) {
      _maze[_posX][_posY - 1] |= Config::WALL_N;
  }

  // If we have a West wall, the cell to the left has an East wall.
  if ((absoluteWalls & Config::WALL_W) && (_posX > 0)) {
      _maze[_posX - 1][_posY] |= Config::WALL_E;
  }

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

              if (minVal != 255 && _distMap[x][y] != minVal + 1) {
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
        if (!_isReturning) {
            //  We reached the center!
            MotorController::hardBrake();
            
            // TODO: Play a beep or flash an LED here .. haptic or visual feedback or something
            delay(1000); 

            // Flip the state and set the goal to the Start (0,0)
            _isReturning = true;
            setTarget(true);

            // Wash the new gradient over the maze
            computePath();
            
            // Return immediately so the next loop starts driving back
            return; 
        } 
        else {
            MotorController::hardBrake();
            
            // Reset the target to the center for the Sprint run
            _isReturning = false;
            setTarget(false);
            computePath(); 

            generateSprintPath();

            // The maze is mapped. We are ready for the high-speed run.
            _currentState = Config::SPRINT; 
            
            // TODO: Wait for user input (button press) before starting SPRINT
            return;
        }
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

void Navigator::setTarget(bool returnToStart)
{
  for (uint8_t x = 0; x < Config::MAZE_SIZE; x++) {
      for (uint8_t y = 0; y < Config::MAZE_SIZE; y++) {
          _distMap[x][y] = 255; // 255 represents "Uncalculated"
      }
  }

  if (returnToStart) {
        _distMap[0][0] = 0; // The start cell is the new goal
    } else {
        // The standard 4 center cells are the goal
        _distMap[7][7] = 0;
        _distMap[7][8] = 0;
        _distMap[8][7] = 0;
        _distMap[8][8] = 0;
    }

}


void Navigator::generateSprintPath() {
    uint8_t x = 0;
    uint8_t y = 0;
    int16_t simHeading = 0; // 0=N, 256=E, 512=S, 768=W
    
    _totalSprintMoves = 0;
    _currentMoveIndex = 0;

    // Follow the gradient downhill until we reach the center (0)
    while (_distMap[x][y] != 0) {
        uint8_t currentDist = _distMap[x][y];
        uint8_t walls = _maze[x][y];
        int16_t nextHeading = simHeading;
        uint8_t nextX = x, nextY = y;

        // Find the downhill neighbor
        if (!(walls & Config::WALL_N) && y < 15 && _distMap[x][y+1] == currentDist - 1) {
            nextHeading = 0; nextY = y + 1;
        } else if (!(walls & Config::WALL_E) && x < 15 && _distMap[x+1][y] == currentDist - 1) {
            nextHeading = 256; nextX = x + 1;
        } else if (!(walls & Config::WALL_S) && y > 0 && _distMap[x][y-1] == currentDist - 1) {
            nextHeading = 512; nextY = y - 1;
        } else if (!(walls & Config::WALL_W) && x > 0 && _distMap[x-1][y] == currentDist - 1) {
            nextHeading = 768; nextX = x - 1;
        }

        // Calculate if a turn is required to face that neighbor
        if (nextHeading != simHeading) {
            int16_t diff = (nextHeading - simHeading) & 1023;
            
            if (diff == 256) {
                _sprintPath[_totalSprintMoves++] = {CMD_TURN_R, 0};
            } else if (diff == 768) {
                _sprintPath[_totalSprintMoves++] = {CMD_TURN_L, 0};
            } else if (diff == 512) {
                // Should mathematically never happen on an optimal path, but safe to trap
                _sprintPath[_totalSprintMoves++] = {CMD_TURN_R, 0};
                _sprintPath[_totalSprintMoves++] = {CMD_TURN_R, 0};
            }
            simHeading = nextHeading; // Update our simulated heading
        }

        // Move Forward (With Compression)
        // If the previous command was ALSO a forward, just increment its cell multiplier
        if (_totalSprintMoves > 0 && _sprintPath[_totalSprintMoves - 1].command == CMD_FORWARD) {
            _sprintPath[_totalSprintMoves - 1].cells++;
        } else {
            // Otherwise, start a new forward command
            _sprintPath[_totalSprintMoves++] = {CMD_FORWARD, 1};
        }

        // Move to the next cell for the next loop iteration
        x = nextX;
        y = nextY;
    }

    // Terminate the array with the finish command
    _sprintPath[_totalSprintMoves++] = {CMD_FINISH, 0};
}

void Navigator::sprintForward(uint8_t cells) {
    // We stop 40 counts early to account for braking inertia
    int32_t targetCounts = (cells * Config::COUNTS_PER_CELL) - 40; 
    
    // Trapezoidal Parameters
    int32_t accelCounts = Config::COUNTS_PER_CELL / 2; // Accelerate over half a cell
    int32_t decelCounts = Config::COUNTS_PER_CELL;     // Decelerate over a full cell
    
    // If the straightaway is too short for the full profile, adjust to a triangle profile
    if (targetCounts < accelCounts + decelCounts) {
        accelCounts = targetCounts / 3;
        decelCounts = targetCounts / 2;
    }

    int16_t minSpeed = 250; 
    int16_t maxSpeed = 850; // SPRINT TOP SPEED

    SensorManager::resetDistance();

    while (abs(SensorManager::getForwardDistance()) < targetCounts) {
        int32_t dist = abs(SensorManager::getForwardDistance());
        int16_t baseSpeed = maxSpeed;

        // Acceleration Phase (Ramp Up)
        if (dist < accelCounts) {
            baseSpeed = minSpeed + ((maxSpeed - minSpeed) * dist) / accelCounts;
        } 
        // Deceleration Phase (Ramp Down)
        else if (dist > targetCounts - decelCounts) {
            int32_t decelProgress = dist - (targetCounts - decelCounts);
            baseSpeed = maxSpeed - ((maxSpeed - minSpeed) * decelProgress) / decelCounts;
            
            // Clamp so we don't go backwards or stall
            if (baseSpeed < minSpeed) baseSpeed = minSpeed; 
        }

        // Heading PID (Keep it perfectly straight)
        int16_t currentHeading = SensorManager::getHeading();
        int16_t error = _targetHeading - currentHeading; 
        if (error > 512)  error -= 1024;
        if (error < -512) error += 1024;
        
        int16_t correction = (error * 5) >> 2; 
        MotorController::setSpeed(baseSpeed + correction, baseSpeed - correction);
        
        // Catastrophic Crash Safety (Only engage if near a wall)
        if (SensorManager::getDistance(Config::FRONT) < 50 && dist > targetCounts - 400) {
            break; 
        }
    }

    MotorController::hardBrake();
    delay(50); // Let the chassis settle before executing the next turn
}

void Navigator::executeSprint() {
    if (_currentMoveIndex >= _totalSprintMoves) return; // Safety bounds

    SprintMove currentMove = _sprintPath[_currentMoveIndex];

    switch(currentMove.command) {
        case CMD_FORWARD:
            sprintForward(currentMove.cells); 
            break;
        case CMD_TURN_R:
            turn90(1); 
            break;
        case CMD_TURN_L:
            turn90(0); 
            break;
        case CMD_FINISH:
            MotorController::hardBrake();
            _currentState = Config::IDLE; // We won!
            return;
    }

    _currentMoveIndex++;
}














