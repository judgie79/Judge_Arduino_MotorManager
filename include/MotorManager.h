#ifndef MOTORMANAGER_h
#define MOTORMANAGER_h
#include "Arduino.h"
#include <BTS7960.h>
#include <LinkedList.h>
#include <DebugOut.h>
#include <NullCheck.h>

#include <DirectionTriggerSensor.h>
#include <DirectionSensorManager.h>
#include <DistanceDevice.h>
#include <RegisteredSensors.h>

typedef void(*SensorTriggered) (DirectionTriggerSensor *sensor);
typedef void(*PositionReached) (uint8_t pos);

const uint8_t MAX_SPEED = 255;
const uint8_t MIN_SPEED = 0;

enum class Direction : int8_t {
  FORWARD = 1,
  BACKWARD = -1,
  NONE = 0
};

class MotorManager {
public:
  MotorManager(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM);
  MotorManager(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM, DirectionTriggerSensor **sensors, uint8_t sensorCount, DistanceDevice *distanceDevice);
  MotorManager(BTS7960 *motor);
  MotorManager(BTS7960 *motor, DirectionTriggerSensor **sensors, uint8_t sensorCount, DistanceDevice *distanceDevice);

  DirectionSensorManager *sensorManager;

  void begin();
  void begin(uint8_t *steps, uint8_t posCount);
  void setSteps(uint8_t *steps, uint8_t posCount);

  void read();

  void gotoPos(uint8_t pos);
  void stop();

  void forward();
  void forward(bool clear);
  void reverse();
  void reverse(bool clear);

  uint8_t *getSteps();
  uint8_t getStepCount();

  void move();

  bool initPositions();

  Direction currentDirection();
  Direction lastDirection();
  uint8_t currentDistance();
  void setDirection(Direction newDirection);
  uint8_t lastTriggeredDistance();

  void setSpeed(uint8_t value);
  uint8_t getSpeed();

  void addSensorListener(SensorTriggered listener);
  void addPositionListener(PositionReached listener);

private:
  BTS7960 * _motorController;

  uint8_t speed = 255;
  Direction _currentDirection;
  Direction _lastDirection;
  bool initialized = false;
  bool initializing = false;
  bool travelToStepStarted = false;
  int8_t travelToStepTrigger = -1;
  uint8_t lastSensor;

  void clearPositions();
  void motorOnSensorReachedOnce(uint8_t sensorIndex);
  void motorOnSensorReached(uint8_t sensorIndex);

  bool checkForPos(DirectionTriggerSensor *sensor, uint8_t sensorIndex);

  bool checkForForce(DirectionTriggerSensor *sensor, uint8_t sensorIndex);

  void motorOnError(String error);
  uint8_t stepCount;
  uint8_t *steps;
  uint8_t defaultStep;
  bool _isRunning;

  unsigned long _lastDebounceTime = 0;  // the last time the output pin was toggled
  unsigned long _debounceDelay = 500;    // the debounce time; increase if the output flickers

  LinkedList<SensorTriggered> stListeners;
  LinkedList<PositionReached> pListeners;
};

#endif