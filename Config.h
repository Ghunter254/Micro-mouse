#ifndef CONFIG
#define CONFIG

#include <Arduino.h>
#include <cstdint>

namespace Config {

// --- MOTOR PINS ---
// Both PWMs are on Hardware Timer 2 (TIM2_CH1 and TIM2_CH4)
constexpr uint8_t M1_PWM = PA0; 
constexpr uint8_t M1_IN1 = PA1;
constexpr uint8_t M1_IN2 = PA2;

constexpr uint8_t M2_PWM = PA3;
constexpr uint8_t M2_IN1 = PA4;
constexpr uint8_t M2_IN2 = PA5;

// --- ULTRASONIC PINS ---
// WARNING: PB1 is NOT 5V tolerant.... use a voltage divider here.
struct UltrasonicPin { uint8_t trig; uint8_t echo; };
constexpr UltrasonicPin US_PINS[4] = {
  {PB0, PB1}, // Front
  {PB2, PB3}, // Left
  {PB4, PB5}, // Right
  {PB6, PB7}  // Back
};

constexpr uint8_t FRONT = 0;
constexpr uint8_t LEFT  = 1;
constexpr uint8_t RIGHT = 2;
constexpr uint8_t BACK  = 3;

// --- MAZE BITMASKS ---
constexpr uint8_t WALL_N   = 0x01; // 0000 0001
constexpr uint8_t WALL_E   = 0x02; // 0000 0010
constexpr uint8_t WALL_S   = 0x04; // 0000 0100
constexpr uint8_t WALL_W   = 0x08; // 0000 1000
constexpr uint8_t VISITED  = 0x10; // 0001 0000

enum State : uint8_t { 
    IDLE = 0, 
    EXPLORE = 1, 
    SPRINT = 2, 
    CALIBRATE = 3, 
    CRASHED = 4 
};

// --- IMU PINS (I2C1) ---
constexpr uint8_t SCL_PIN = PB8; 
constexpr uint8_t SDA_PIN = PB9;
constexpr uint8_t IMU_ADDR = 0x68;

// Both are 5V Tolerant (FT). 
constexpr uint8_t MOUSE_CLK = PA6; 
constexpr uint8_t MOUSE_DAT = PA7;

// Based on an 8-count-per-mm optical sensor resolution
constexpr uint16_t COUNTS_PER_CELL = 1440; // 180mm * 8
constexpr int16_t SPEED_EXPLORE = 400;

// --- MAZE DIMENSIONS ---
constexpr uint8_t  MAZE_SIZE   = 16;
constexpr uint16_t CELL_DIM_MM = 180;

// For fixed point math
constexpr int16_t FIXED_UNIT = 1024;

}

#endif