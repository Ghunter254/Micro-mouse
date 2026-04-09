#include "Sensor.h"
#include <HardwareTimer.h>
#include <Wire.h>


SensorData SensorManager::_currentData = { {0,0,0,0}, 0, 0, 0 };
uint8_t    SensorManager::_currentSensorIdx = 0;
uint32_t   SensorManager::_lastPingTime = 0;
int16_t SensorManager::_gyroBias = 0;
int32_t SensorManager::_currentPosition = {0,0};


HardwareTimer *MyTim;
PS2Mouse opticalMouse(Config::MOUSE_CLK, Config::MOUSE_DAT);


void SensorManager::init() 
{
  // setup for ultrasonics
  for(uint8_t i=0; i < 4; i++){
    pinMode(Config::US_PINS[i].trig, OUTPUT);
    pinMode(Config::US_PINS[i].echo, INPUT);
    digitalWrite(Config::US_PINS[i].trig, LOW);
  }

  // Mouse initialization.
  MyTim = new HardwareTimer(TIM3);
  opticalMouse.initiliaze();

  MyTim->pause();
  MyTim->setOverflow(200, HERTZ_FORMAT);
  MyTim->attachInterrupt(onHeartBeat);
  MyTim->resume();

  // IMU
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(Config::IMU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  calibrateGyro();
}

void SensorManager::update()
{
  uint32_t now = micros();

  // Check to  see if the current echo has returned... 
  if (now - _lastPingTime > 30000) { // 30ms

    uint32_t duration = pulseIn(Config::US_PINS[_currentSensorIdx].echo, HIGH, 15000);

    if (duration > 0) {
      _currentData.distance[_currentSensorIdx] = (uint16_t)((duration * 175) >> 10);
    } else {
      _currentData.distance[_currentSensorIdx] = 999; // Where 999 is infinite distance.
    }

    // Then we increment to the next sensor.
    // Using bitwise add for circular buffer.
    _currentSensorIdx = (_currentSensorIdx + 1) & 0x03;

    // Trig the next sensot
    digitalWrite(Config::US_PINS[_currentSensorIdx].trig, HIGH);
    delayMicroseconds(10); // Standard HC-SR04 trigger pulse
    digitalWrite(Config::US_PINS[_currentSensorIdx].trig, LOW);
    
    _lastPingTime = now;
  }

  Wire.beginTransmission(Config::IMU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(Config::IMU_ADDR, (uint8_t)2);

  if (Wire.available() >= 2) {
    int16_t rawGyroZ = (Wire.read() << 8 | Wire.read());
    int16_t correctedGyro = rawGyroZ - _gyroBias;

    // Replace division with Bit-Shift (Approx 131 sensitivity)
    // (correctedGyro * Scale) >> 10 
    // For 100Hz loop, use a scale factor of ~8
    _currentData.heading += (correctedGyro >> 7); 
    _currentData.heading &= 1023; 
  }

}

Position SensorManager::getPosition(){

  noInterrupts();
  Position position =  _currentPosition;
  interrupts();
  return position;

}

int32_t SensorManager::getForwardDistance() {
   return _currentPosition.y;
}

int32_t SensorManager::getLateralDrift() {
    return _currentPosition.x;
}
void SensorManager::resetPosition() {
    noInterrupts();
    _currentPosition = { 0,0 }
    interrupts();
}

uint16_t SensorManager::getDistance(uint8_t sensorIndex) 
{
  return _currentData.distance[sensorIndex & 0x03];
}

bool SensorManager::isWallDetected(uint8_t sensorIndex) 
{
  return (_currentData.distance[sensorIndex & 0X03] < 120);
}

void SensorManager::resetMouseTimer() {
  MyTim->setCount(0);
}


void SensorManager::calibrateGyro() {
    digitalWrite(PC13, LOW);
    int32_t sum = 0;
    for (int i = 0; i < 200; i++) {
        Wire.beginTransmission(0x68);
        Wire.write(0x47);
        Wire.endTransmission(false);
        Wire.requestFrom(0x68, 2);
        sum += (Wire.read() << 8 | Wire.read());
        delay(2);
    }
    _gyroBias = sum / 200; 
    digitalWrite(PC13, HIGH);
}

int16_t SensorManager::getHeading() {
    return _currentData.heading;
}

void onHeartBeat () {
  PS2Mouse::MouseData mData = opticalMouse.readData();

  // Accumulate the read data.
  _currentPosition.globalMouseX += mData.position.x;
  _currentPosition.globalMouseY += mData.position.y;

}














