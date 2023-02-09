#ifndef MotorController_h
#define MotorController_h

#include "Arduino.h"
#include "BTS7960.h"

enum class Direction : int8_t {
  FORWARD = 1,
  BACKWARD = -1,
  NONE = 0
};

class MotorController {
public:
  MotorController(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM);
  MotorController(BTS7960 *motor);
  void begin();
  void setSpeed(uint8_t speed);
  void setMaxSpeed(uint8_t speed);
  void setMinSpeed(uint8_t speed);
  void forward();
  void backward();
  void setDirection(Direction newDirection);
  void move();
  void stop();
  void end();

  bool isRunning();
  Direction currentDirection();
  Direction lastDirection();
private:
  BTS7960 *_motorShield;
  uint8_t _speed;
  uint8_t _minSpeed;
  uint8_t _maxSpeed;
  bool _isRunning;
  Direction _currentDirection;
  Direction _lastDirection;

  uint8_t _L_EN;
  uint8_t _R_EN;
  uint8_t _L_PWM;
  uint8_t _R_PWM;
};

#endif