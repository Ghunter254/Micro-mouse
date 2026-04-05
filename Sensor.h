#ifndef SENSOR_H
#define SENSOR_H

#include "Config.h"
#include <cstdint>

/**
 * @brief SensorData using 1024-scale Fixed Point
 * For Angles: 1024 = 360 degrees. 
 * This means 1 unit = ~0.35 degrees.
 * For Distances: We keep these in raw mm (uint16_t) as they don't need scaling.
 */
struct SensorData {
  uint16_t distance[4];
  int16_t  heading;     
  int16_t  accelX;     
  int16_t  accelY;
};

class SensorManager {
  public:
    static void init();
    static void update();

    /**
     * @param sensorIndex: 
     * 0 = Config::FRONT
     * 1 = Config::LEFT
     * 2 = Config::RIGHT
     * 3 = Config::BACK
     */
    static uint16_t getDistance(uint8_t sensorIndex);
    static int16_t getHeading();

    static  int32_t getEncoderTicks();
    static void resetEncoders();
    
    static bool isWallDetected(uint8_t direction);
    static void calibrateGyro();

  private:
    static SensorData _currentData;
    static uint8_t    _currentSensorIdx; 
    static uint32_t   _lastPingTime;
    static int16_t _gyroBias;


};

#endif