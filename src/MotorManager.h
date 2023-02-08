#ifndef MOTORMANAGER_h
#define MOTORMANAGER_h
#include "Arduino.h"
#include <LinkedList.h>
#include <DebugOut.h>
#include <NullCheck.h>

#include <MotorSensor.h>
#include <SensorManager.h>
#include <DistanceDevice.h>
#include <DistanceSensor.h>
#include <RegisteredSensors.h>
#include "MotorController.h"

typedef void(*SensorTriggered) (MotorSensor *sensor);
typedef void(*PositionReached) (uint16_t pos);

class MotorManager {
public:
  MotorManager(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM);
  MotorManager(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM, MotorSensor **sensors, uint16_t sensorCount, DistanceDevice *distanceDevice);
  MotorManager(BTS7960 *motor);
  MotorManager(BTS7960 *motor, MotorSensor **sensors, uint16_t sensorCount, DistanceDevice *distanceDevice);

  SensorManager *sensorManager;

  void begin();
  void begin(uint16_t *steps, uint16_t posCount);
  void setSteps(uint16_t *steps, uint16_t posCount);

  void read();

  void gotoPos(uint16_t pos);
  void stop();

  void forward();
  void forward(bool clear);
  void reverse();
  void reverse(bool clear);

  uint16_t *getSteps();
  uint16_t getStepCount();

  void move();

  bool initPositions();

  Direction currentDirection();
  Direction lastDirection();
  uint16_t currentDistance();
  uint16_t lastTriggeredDistance();

  void setSpeed(uint16_t value);
  uint16_t getSpeed();
  void printSteps();
  void printSensors();

  void addSensorListener(SensorTriggered listener);
  void addPositionListener(PositionReached listener);

private:
  MotorController _motorController;

  uint16_t speed = 255;
  bool initialized = false;
  bool initializing = false;
  bool travelToStepStarted = false;
  int16_t travelToStepTrigger = -1;
  uint16_t lastSensor;

  void clearPositions();
  void motorOnSensorReachedOnce(uint16_t sensorIndex);
  void motorOnSensorReached(uint16_t sensorIndex);

  bool checkForPos(MotorSensor *sensor, uint16_t sensorIndex);

  bool checkForForce(MotorSensor *sensor, uint16_t sensorIndex);

  void motorOnError(String error);
  uint16_t stepCount;
  uint16_t *steps;
  uint16_t defaultStep;

  unsigned long _lastDebounceTime = 0;  // the last time the output pin was toggled
  unsigned long _debounceDelay = 500;    // the debounce time; increase if the output flickers

  LinkedList<SensorTriggered> stListeners;
  LinkedList<PositionReached> pListeners;
};

#endif