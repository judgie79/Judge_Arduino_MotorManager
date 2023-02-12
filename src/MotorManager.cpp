#include "MotorManager.h"
#include "BTS7960.h"


MotorManager::MotorManager(BTS7960 *motor)
{
  this->sensorManager = nullptr;
  _motorController = motor;
}

MotorManager::MotorManager(BTS7960 *motor, DirectionTriggerSensor **sensors, uint8_t sensorCount, DistanceDevice *distanceDevice)
{
  this->sensorManager = new DirectionSensorManager(sensors, sensorCount, distanceDevice);
  _motorController = motor;
}

MotorManager::MotorManager(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM)
{
  this->sensorManager = nullptr;
  _motorController = new BTS7960(L_EN, R_EN, L_PWM, R_PWM);;
}

MotorManager::MotorManager(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM, DirectionTriggerSensor **sensors, uint8_t sensorCount, DistanceDevice *distanceDevice)
{
  // this->sensors = sensors;
  // this->sensorCount = sensorCount;
  this->sensorManager = new DirectionSensorManager(sensors, sensorCount, distanceDevice);
  _motorController = new BTS7960(L_EN, R_EN, L_PWM, R_PWM);;
}

void MotorManager::begin()
{
  _motorController->Enable();

  if (norNull(sensorManager))
  {
    sensorManager->begin();
  }
  else
  {
    LOGD_DEBUG("no sensormanager");
  }
}

void MotorManager::begin(uint8_t *steps, uint8_t posCount)
{
  this->stepCount = posCount;
  this->steps = NULL;
  this->steps = steps;

  begin();
  if (norNull(sensorManager))
  {
    uint8_t sc = sensorManager->getSensorCount();

    for (size_t i = 0; i < this->stepCount; i++)
    {
      uint8_t id = sc + i;
      sensorManager->addDistanceSensor(id, this->steps[i], SensorTriggerType::Info, SensorTriggerDirection::Both);
    }
  }
}

void MotorManager::setSteps(uint8_t *steps, uint8_t posCount)
{
  this->stepCount = posCount;
  this->steps = NULL;
  this->steps = steps;
  if (norNull(sensorManager))
  {

    sensorManager->removeSensors(SensorTriggerType::Info);
    uint8_t sc = sensorManager->getSensorCount();
    for (size_t i = 0; i < this->stepCount; i++)
    {
      uint8_t id = sc + i;
      sensorManager->addDistanceSensor(id, this->steps[i], SensorTriggerType::Info, SensorTriggerDirection::Both);
    }
  }
}

void MotorManager::read()
{
  if (norNull(sensorManager))
  {
    sensorManager->read();
  }
  else
  {
    return;
  }

  if (norNull(sensorManager))
  {
    DirectionTriggerSensor *sensor = sensorManager->lastTriggeredSensor();
    if (norNull(sensor))
    {
      if (sensor->isTriggered())
      {
        if (sensorManager->lastTriggeredIndex() > -1)
        {
          motorOnSensorReachedOnce(sensorManager->lastTriggeredIndex());
        }
      }
      else if (sensor->isStillTriggered())
      {
        if (sensorManager->lastTriggeredIndex() > -1)
        {
          motorOnSensorReached(sensorManager->lastTriggeredIndex());
        }
      }
    }
  }
}

Direction MotorManager::currentDirection()
{
  return _currentDirection;
}

Direction MotorManager::lastDirection()
{
  return _lastDirection;
}

void MotorManager::clearPositions()
{
  travelToStepStarted = false;
  travelToStepTrigger = -1;
}

void MotorManager::forward()
{
  forward(false);
}

void MotorManager::forward(bool clear)
{
  if (clear)
  {
    clearPositions();
  }
  setDirection(Direction::FORWARD);
}

void MotorManager::setDirection(Direction newDirection)
{
  _lastDirection = _currentDirection;
  _currentDirection = newDirection;
}

void MotorManager::stop()
{
  setDirection(Direction::NONE);
  _isRunning = false;
  _motorController->Stop();
}

void MotorManager::reverse()
{
  reverse(false);
}

void MotorManager::reverse(bool clear)
{
  if (clear)
  {
    clearPositions();
  }

  setDirection(Direction::BACKWARD);
}

void MotorManager::gotoPos(uint8_t pos)
{
  RegisteredSensors<DirectionTriggerSensor> posSensors = sensorManager->getSensors(SensorTriggerType::Info);

  DirectionTriggerSensor *posSensor = posSensors.sensors[pos];
  if (norNull(posSensor))
  {
    travelToStepStarted = true;
    travelToStepTrigger = pos;
    uint8_t currentDistance = sensorManager->currentDistance();
    uint8_t distanceToGo = steps[pos];
    if (currentDistance > distanceToGo)
    {
      reverse();
    }
    else if (currentDistance < distanceToGo)
    {
      forward();
    }
    else
    {
      stop();
    }
  }
}

void MotorManager::move()
{

  SensorTriggerDirection triggerDirection = SensorTriggerDirection::None;

  if (_currentDirection == Direction::FORWARD)
  {
    triggerDirection = SensorTriggerDirection::Forward;
  }
  else if (_currentDirection == Direction::BACKWARD)
  {
    triggerDirection = SensorTriggerDirection::Backward;
  }

  if (triggerDirection != SensorTriggerDirection::None)
  {
    RegisteredSensors<DirectionTriggerSensor> forceStopSensors = sensorManager->getSensors(SensorTriggerType::Force, new SensorTriggerDirection[1]{triggerDirection}, 1);

    bool foundTriggered = false;
    if (forceStopSensors.sensorCount > 0 && norNull(forceStopSensors.sensors))
    {
      for (size_t i = 0; i < forceStopSensors.sensorCount; i++)
      {
        DirectionTriggerSensor *sensor = forceStopSensors.sensors[i];

        if (norNull(sensor))
        {
          if (sensor->isTriggered() || sensor->isStillTriggered())
          {
            foundTriggered = true;
          }
        }
      }
    }
    if (!foundTriggered)
    {
      if (_currentDirection == Direction::FORWARD) {
        _lastDirection = _currentDirection;
      if (!_isRunning) {
        _isRunning = true;
        _motorController->TurnLeft(speed);
      }
      } else if (_currentDirection == Direction::BACKWARD) {
        _lastDirection = _currentDirection;
        if (!_isRunning) {
          _isRunning = true;
          _motorController->TurnRight(speed);
        }
      } else {
        stop();
      }
    }
    else
    {
      // do not move, force stop trigger in direction
      stop();
    }
  }
}

void MotorManager::motorOnSensorReachedOnce(uint8_t sensorIndex)
{

  DirectionTriggerSensor *sensor = sensorManager->getSensors().sensors[sensorIndex];

  if (isNull(sensor))
  {
    return;
  }

  if (checkForForce(sensor, sensorIndex))
  {
  }
  else if (checkForPos(sensor, sensorIndex))
  {
  }
}

bool MotorManager::checkForForce(DirectionTriggerSensor *sensor, uint8_t sensorIndex)
{
  if (sensor->getTriggerType() != SensorTriggerType::Force)
  {
    return false;
  }

  if (
      (sensor->getTriggerDirection() == SensorTriggerDirection::Both) ||
      ((this->currentDirection() == Direction::BACKWARD) && (sensor->getTriggerDirection() == SensorTriggerDirection::Backward)) ||
      ((this->currentDirection() == Direction::FORWARD) && (sensor->getTriggerDirection() == SensorTriggerDirection::Forward)))
  {
    clearPositions();
    stop();
  }

  if (stListeners.moveToStart())
  {
    do
    {
      SensorTriggered sl = stListeners.getCurrent();
      sl(sensor);
    } while (stListeners.next());
  }

  return true;
}

bool MotorManager::checkForPos(DirectionTriggerSensor *sensor, uint8_t sensorIndex)
{
  if (sensor->getTriggerType() != SensorTriggerType::Info)
  {
    return false;
  }

  if (travelToStepStarted)
  {
    RegisteredSensors<DirectionTriggerSensor> posSensors = sensorManager->getSensors(SensorTriggerType::Info);

    DirectionTriggerSensor *posSensor = posSensors.sensors[travelToStepTrigger];

    if (norNull(posSensor))
    {
      if (posSensor->getId() == sensor->getId())
      {
        LOGD_INFO("reached sensor index" + String(sensorIndex));
        clearPositions();
        stop();
        if (!initialized)
        {
          LOGD_INFO("initialized");
          initialized = true;
          initializing = false;
        }
        if (pListeners.moveToStart())
          do
          {
            PositionReached pr = pListeners.getCurrent();
            pr(posSensor->getId());
          } while (pListeners.next());
      }
    }
  }

  if (stListeners.moveToStart())
    do
    {
      SensorTriggered sl = stListeners.getCurrent();
      sl(sensor);
    } while (stListeners.next());
  return true;
}

void MotorManager::motorOnSensorReached(uint8_t sensorIndex)
{

  DirectionTriggerSensor *sensor = sensorManager->getSensors().sensors[sensorIndex];

  if (sensor == NULL || sensor == nullptr)
  {
    return;
  }

  if (checkForForce(sensor, sensorIndex))
  {
  }
  else if (checkForPos(sensor, sensorIndex))
  {
  }
}

void MotorManager::motorOnError(String error)
{
}

bool MotorManager::initPositions()
{
  // check init
  if (!initialized)
  {
    if (!initializing)
    {

      LOGD_INFO("initialize");
      // start init and move to back pos
      initializing = true;
      gotoPos(defaultStep);
    }
    else if (initializing && !initialized)
    {
      move();
    }
  }

  // do nothing else
  return initialized;
}

uint8_t MotorManager::getSpeed()
{
  return speed;
}

void MotorManager::setSpeed(uint8_t value)
{
  speed = value;
}

uint8_t MotorManager::currentDistance()
{
  return sensorManager->currentDistance();
}

uint8_t *MotorManager::getSteps()
{
  return steps;
}

uint8_t MotorManager::getStepCount()
{
  return stepCount;
}

void MotorManager::addSensorListener(SensorTriggered listener)
{
  stListeners.Append(listener);
}

void MotorManager::addPositionListener(PositionReached listener)
{
  pListeners.Append(listener);
}