#include "Motors.h"

uint16_t MotorController::_motorBias = 1024; // No bias

void MotorController::init()
{

  pinMode(Config::M1_IN1, OUTPUT);
  pinMode(Config::M1_IN2, OUTPUT);
  pinMode(Config::M2_IN1, OUTPUT);
  pinMode(Config::M2_IN2, OUTPUT);

  analogWriteFrequency(20000);

  stop();


}

void MotorController::setSpeed(int16_t leftSpeed, int16_t rightSpeed)
{
  int32_t calibratedLeft = (static_cast<int32_t>(leftSpeed) * _motorBias) >> 10;

  // Clamping the values to 1024 scale.
  if (calibratedLeft > 1024)  calibratedLeft = 1024;
  if (calibratedLeft < -1024) calibratedLeft = -1024;
  if (rightSpeed > 1024)      rightSpeed = 1024;
  if (rightSpeed < -1024)     rightSpeed = -1024;

  writeHardwarePWM(Config::M1_IN1, Config::M1_IN2, (int16_t)calibratedLeft);
  writeHardwarePWM(Config::M2_IN1, Config::M2_IN2, rightSpeed);

}

void MotorController::writeHardwarePWM(uint8_t pin1, uint8_t pin2, int16_t speed)
{
  uint8_t pwmValue = abs(speed) >> 2;

  if (speed > 0) {
    
    analogWrite(pin1, pwmValue);
    digitalWrite(pin2, LOW);

  } else if (speed < 0 ) {
    digitalWrite(pin1, LOW);
    analogWrite(pin2, pwmValue);
  }
  else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
}

void MotorController::hardBrake() {
  digitalWrite(Config::M1_IN1, HIGH);
  digitalWrite(Config::M1_IN2, HIGH);
  digitalWrite(Config::M2_IN1, HIGH);
  digitalWrite(Config::M2_IN2, HIGH);
}

void MotorController::stop() {
    setSpeed(0, 0);
}





















