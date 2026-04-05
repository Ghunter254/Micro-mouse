#ifndef MOTORS_H
#define MOTORS_H

#include "Config.h"
#include <cstdint>

class MotorController {

  public:

    static void init();

    /**
     * @brief Sets raw motor speeds using 1024-scale logic.
     * @param leftSpeed  -1024 to 1024 (Full Reverse to Full Forward)
     * @param rightSpeed -1024 to 1024
     */
    static void setSpeed(int16_t leftSpeed, int16_t rightSpeed);

    /**
     * @brief Gradual stop to prevent skiddinnggg
     */
    static void stop();

    /**
     * @brief Forces motor pins to LOW/LOW or HIGH/HIGH for immediate halt.
     */
    static void hardBrake();

    // Util function for motor calibration. Will be used if needded.
    void setCalibration(float bias);

  private:
    static uint16_t _motorBias; // Stored in 1024-scale
    
    // Internal helper to map 1024-scale to the MCU's PWM range (e.g., 0-255)
    static void writeHardwarePWM(uint8_t pin1, uint8_t pin2, int16_t speed);
};

#endif