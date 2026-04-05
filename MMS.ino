#include "Config.h"
#include "Motors.h"
#include "Sensor.h"
#include "Navigation.h"

Navigator mouse;

void setup() {
  Serial.begin(115200);
  SensorManager::init(); 
  MotorController::init();

  mouse.init();
  delay(5000); 

  mouse.setState(Config::EXPLORE);
}

void loop() {

  SensorManager::update();
  mouse.update();
  delay(1); 
}