#include "Arduino.h"
#include "MotorController.h"

const uint8_t MAX_SPEED = 255;
const uint8_t MIN_SPEED = 0;

MotorController::MotorController(BTS7960 *motor) {
  _motorShield = motor;
}

MotorController::MotorController(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM) {
  _motorShield = new BTS7960(L_EN, R_EN, L_PWM, R_PWM);
}

void MotorController::begin() {
  _motorShield->Enable();
}

void MotorController::end() {
  _motorShield->Disable();
}

void MotorController::setSpeed(uint8_t speed) {
 
  _speed = speed;
}

void MotorController::setMaxSpeed(uint8_t speed) {
  if (speed < MIN_SPEED) {
    speed = MIN_SPEED;
  } else if (speed > MAX_SPEED) {
    speed = MAX_SPEED;
  }

  _maxSpeed = speed;
}

void MotorController::setMinSpeed(uint8_t speed) {
  if (speed < MIN_SPEED) {
    speed = MIN_SPEED;
  } else if (speed > (uint8_t)MAX_SPEED) {
    speed = MAX_SPEED;
  }

  _minSpeed = speed;
}

void MotorController::forward() {
  if (!_isRunning) {
    _isRunning = true;
    _motorShield->TurnLeft(_speed);
  }
}

void MotorController::backward() {
  if (!_isRunning) {
    _isRunning = true;
    _motorShield->TurnRight(_speed);
  }
}

void MotorController::setDirection(Direction newDirection) {
  _lastDirection = _currentDirection;
  _currentDirection = newDirection;
}

void MotorController::move() {
  if (_currentDirection == Direction::FORWARD) {
    _lastDirection = _currentDirection;
    forward();
  } else if (_currentDirection == Direction::BACKWARD) {
    _lastDirection = _currentDirection;
    backward();
  } else {
    stop();
  }
}

void MotorController::stop() {
  _isRunning = false;
  _motorShield->Stop();
}

bool MotorController::isRunning() {
  return _isRunning;
}

Direction MotorController::lastDirection() {
  return _lastDirection;
}

Direction MotorController::currentDirection() {
  return _currentDirection;
}