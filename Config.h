#ifndef CONFIG
#define CONFIG

#include <Arduino.h>
#include <cstdint>

namespace Config {

// Definition for the motor pins.
constexpr uint8_t M1_PWM = PA0;
constexpr uint8_t M1_IN1 = PA1;
constexpr uint8_t M1_IN2 = PA2;

constexpr uint8_t M2_PWM = PA3;
constexpr uint8_t M2_IN1 = PA4;
constexpr uint8_t M2_IN2 = PA5;

// Ultrasonic sensors.
// Fr, L, R, B
struct UltrasonicPin { uint8_t trig; uint8_t echo; };
constexpr UltrasonicPin US_PINS[4] = {
  {PB0, PB1}, {PB2, PB3}, {PB4, PB5}, {PB6, PB7}
};

constexpr uint8_t FRONT = 0;
constexpr uint8_t LEFT  = 1;
constexpr uint8_t RIGHT = 2;
constexpr uint8_t BACK  = 3;

constexpr uint8_t WALL_N   = 0x01; // 0000 0001
constexpr uint8_t WALL_E   = 0x02; // 0000 0010
constexpr uint8_t WALL_S   = 0x04; // 0000 0100
constexpr uint8_t WALL_W   = 0x08; // 0000 1000
constexpr uint8_t VISITED  = 0x10; // 0001 0000 (Bit 4)

enum State : uint8_t { 
    IDLE = 0, 
    EXPLORE = 1, 
    SPRINT = 2, 
    CALIBRATE = 3, 
    CRASHED = 4 
};

// Imu pins
constexpr uint8_t SCL_PIN = PB8; 
constexpr uint8_t SDA_PIN = PB9;
constexpr uint8_t IMU_ADDR = 0x68;

// Encoder pins

constexpr uint8_t ENC_A = PA6; 
constexpr uint8_t ENC_B = PA7;

constexpr uint16_t TICKS_PER_CELL = 1200;
constexpr int16_t SPEED_EXPLORE = 400;

// Maze 
constexpr uint8_t  MAZE_SIZE   = 16;
constexpr uint16_t CELL_DIM_MM = 180;

//  For fixed point math
constexpr int16_t FIXED_UNIT = 1024;

}

#endif