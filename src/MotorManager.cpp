#include "MotorManager.h"



MotorManager::MotorManager(BTS7960 *motor) : _motorController(motor)
{
  this->sensorManager = nullptr;
}

MotorManager::MotorManager(BTS7960 *motor, MotorSensor **sensors, uint16_t sensorCount, DistanceDevice *distanceDevice) : _motorController(motor)
{
  this->sensorManager = new SensorManager(sensors, sensorCount, distanceDevice);
}

MotorManager::MotorManager(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM) : _motorController(L_EN, R_EN, L_PWM, R_PWM)
{
  this->sensorManager = nullptr;
}

MotorManager::MotorManager(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM, MotorSensor **sensors, uint16_t sensorCount, DistanceDevice *distanceDevice) : _motorController(L_EN, R_EN, L_PWM, R_PWM)
{
  // this->sensors = sensors;
  // this->sensorCount = sensorCount;
  this->sensorManager = new SensorManager(sensors, sensorCount, distanceDevice);
}

void MotorManager::begin()
{
  _motorController.begin();
  _motorController.setSpeed(speed);

  if (norNull(sensorManager))
  {
    sensorManager->begin();
  }
  else
  {
    LOGD_DEBUG("no sensormanager");
  }
}

void MotorManager::begin(uint16_t *steps, uint16_t posCount)
{
  this->stepCount = posCount;
  this->steps = NULL;
  this->steps = steps;

  begin();
  if (norNull(sensorManager))
  {
    uint16_t sc = sensorManager->getSensorCount();

    for (size_t i = 0; i < this->stepCount; i++)
    {
      uint16_t id = sc + i;
      sensorManager->addDistanceSensor(id, "POS" + String(i), this->steps[i], SensorTriggerType::Position, SensorTriggerDirection::Both);
    }
  }
}

void MotorManager::setSteps(uint16_t *steps, uint16_t posCount)
{
  this->stepCount = posCount;
  this->steps = NULL;
  this->steps = steps;
  if (norNull(sensorManager))
  {

    sensorManager->removeSensors(SensorTriggerType::Position);
    uint16_t sc = sensorManager->getSensorCount();
    for (size_t i = 0; i < this->stepCount; i++)
    {
      uint16_t id = sc + i;
      sensorManager->addDistanceSensor(id, "POS" + String(i), this->steps[i], SensorTriggerType::Position, SensorTriggerDirection::Both);
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
    MotorSensor *sensor = sensorManager->lastTriggeredSensor();
    if (norNull(sensor))
    {
      if (sensor->isTriggered())
      {
        int16_t index = sensorManager->lastTriggeredIndex();
        if (index > -1)
        {
          motorOnSensorReachedOnce(index);
        }
      }
      else if (sensor->isStillTriggered())
      {
        int16_t index = sensorManager->lastTriggeredIndex();
        if (index > -1)
        {
          motorOnSensorReached(index);
        }
      }
    }
  }
}

void MotorManager::printSteps()
{
  for (size_t i = 0; i < stepCount; i++)
  {
    LOGD_INFO(String(steps[i]));
  }
}
void MotorManager::printSensors()
{
  sensorManager->printSensors();
}

Direction MotorManager::currentDirection()
{
  return _motorController.currentDirection();
}

Direction MotorManager::lastDirection()
{
  return _motorController.lastDirection();
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

  if (_motorController.currentDirection() != Direction::FORWARD)
  {
    _motorController.setDirection(Direction::FORWARD);
  }

  _motorController.setSpeed(speed);

  LOGD_INFO("set motor FORWARD");
}

void MotorManager::stop()
{
  _motorController.setDirection(Direction::NONE);
  _motorController.setSpeed(0);
  _motorController.stop();
  LOGD_INFO("motor STOP");
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

  if (_motorController.currentDirection() != Direction::BACKWARD)
  {
    _motorController.setDirection(Direction::BACKWARD);
  }
  _motorController.setSpeed(speed);

  LOGD_INFO("set motor BACKWARD");
}

void MotorManager::gotoPos(uint16_t pos)
{
  LOGD_INFO("goto " + String(pos));
  RegisteredSensors posSensors = sensorManager->getSensors(SensorTriggerType::Position);

  MotorSensor *posSensor = posSensors.sensors[pos];
  if (norNull(posSensor))
  {
    travelToStepStarted = true;
    travelToStepTrigger = pos;
    uint16_t currentDistance = sensorManager->currentDistance();
    uint16_t distanceToGo = steps[pos];
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

  if (_motorController.currentDirection() == Direction::FORWARD)
  {
    triggerDirection = SensorTriggerDirection::Forward;
  }
  else if (_motorController.currentDirection() == Direction::BACKWARD)
  {
    triggerDirection = SensorTriggerDirection::Backward;
  }

  if (triggerDirection != SensorTriggerDirection::None)
  {
    RegisteredSensors forceStopSensors = sensorManager->getSensors(SensorTriggerType::ForceStop, new SensorTriggerDirection[1]{triggerDirection}, 1);

    bool foundTriggered = false;
    if (forceStopSensors.sensorCount > 0 && norNull(forceStopSensors.sensors))
    {
      for (size_t i = 0; i < forceStopSensors.sensorCount; i++)
      {
        MotorSensor *sensor = forceStopSensors.sensors[i];

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
      _motorController.move();
    }
    else
    {
      // do not move, force stop trigger in direction
      stop();
    }
  }
}

void MotorManager::motorOnSensorReachedOnce(uint16_t sensorIndex)
{

  MotorSensor *sensor = sensorManager->getSensors().sensors[sensorIndex];

  if (isNull(sensor))
  {
    LOGD_ERROR("sensor from index is NULL");
    return;
  }

  if (checkForForce(sensor, sensorIndex))
  {
  }
  else if (checkForPos(sensor, sensorIndex))
  {
  }
}

bool MotorManager::checkForForce(MotorSensor *sensor, uint16_t sensorIndex)
{
  if (sensor->getTriggerType() != SensorTriggerType::ForceStop)
  {
    return false;
  }

  if (this->currentDirection() == Direction::BACKWARD)
  {
    if (sensor->getTriggerDirection() == SensorTriggerDirection::Backward ||
        sensor->getTriggerDirection() == SensorTriggerDirection::Both)
    {
      LOGD_INFO("reached force stop backward");
      clearPositions();
      stop();
    }
  }
  else if (this->currentDirection() == Direction::FORWARD)
  {
    //  DebugOut.debug("check FORCESTOP FORWARD ");
    if (sensor->getTriggerDirection() == SensorTriggerDirection::Forward ||
        sensor->getTriggerDirection() == SensorTriggerDirection::Both)
    {
      LOGD_INFO("reached force stop forward");
      clearPositions();
      stop();
    }
  }

  if(stListeners.moveToStart())
        do{
            SensorTriggered sl = stListeners.getCurrent();
            sl(sensor);
        }while(stListeners.next());
  return true;

  return true;
}

bool MotorManager::checkForPos(MotorSensor *sensor, uint16_t sensorIndex)
{
  if (sensor->getTriggerType() != SensorTriggerType::Position)
  {
    return false;
  }

  if (travelToStepStarted)
  {
    RegisteredSensors posSensors = sensorManager->getSensors(SensorTriggerType::Position);

    MotorSensor *posSensor = posSensors.sensors[travelToStepTrigger];

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
        if(pListeners.moveToStart())
        do{
            PositionReached pr = pListeners.getCurrent();
            pr(posSensor->getId());
        }while(pListeners.next());
      }
    }
    
  }
  
  if(stListeners.moveToStart())
        do{
            SensorTriggered sl = stListeners.getCurrent();
            sl(sensor);
        }while(stListeners.next());
  return true;
}

void MotorManager::motorOnSensorReached(uint16_t sensorIndex)
{

  MotorSensor *sensor = sensorManager->getSensors().sensors[sensorIndex];

  if (sensor == NULL || sensor == nullptr)
  {
    LOGD_ERROR("sensor from index is NULL");
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

      // MotorSensor **distanceSensors = sensorManager->getSensors(SensorTriggerType::Position);

      gotoPos(defaultStep);
    }
    else if (initializing && !initialized)
    {
      LOGD_DEBUG("move");
      move();
    }
  }

  // do nothing else
  return initialized;
}

uint16_t MotorManager::getSpeed()
{
  return speed;
}

void MotorManager::setSpeed(uint16_t value)
{
  speed = value;
}

uint16_t MotorManager::currentDistance()
{
  return sensorManager->currentDistance();
}

uint16_t *MotorManager::getSteps()
{
  return steps;
}

uint16_t MotorManager::getStepCount()
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