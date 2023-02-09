#include "MotorManager.h"
#include <SonarDistanceDevice.h>
#include <ButtonSensor.h>
#include <DebugOut.h>



const int distanceEchoPin = 12;    // the number of the echo pin connected to the distance sensor
const int distanceTriggerPin = 13; // the number of the trigger pin connected to the distance sensor
SonarDistanceDevice sonar(distanceTriggerPin, distanceEchoPin, 30);

const int backEndSensorPin = 11; // the number of the pushbutton pin of the backend sensor
const int frontEndSensorPin = 7; // the number of the pushbutton pin of the front end sensorJudge_MotorManager
ButtonSensor backEndSensor(0, "START", SensorType::Trigger, SensorTriggerType::ForceStop, SensorTriggerDirection::Backward, backEndSensorPin, INPUT_PULLUP);
ButtonSensor frontEndSensor(1, "END", SensorType::Trigger, SensorTriggerType::ForceStop, SensorTriggerDirection::Forward, frontEndSensorPin, INPUT_PULLUP);

TriggerSensor** defaultSensors = new TriggerSensor * [2] { &backEndSensor, & frontEndSensor };

uint8_t R_EN = A0;  // the number of the R_EN pin connected to the motor
uint8_t L_EN = A1;  // the number of the L_EN pin connected to the motor
uint8_t R_PWM = 9;  // the number of the R_PWM pin connected to the motor
uint8_t L_PWM = 10; // the number of the L_PWM pin connected to the motor
MotorManager mManager(L_EN, R_EN, L_PWM, R_PWM, defaultSensors, 2, &sonar);

int steps[2] = { 7, 15 };
const int defaultPos = 0;

void setup() {
	Serial.begin(9600);
	DebugOut.setOutput(&Serial);
	DebugOut.setLogLevel(LogLevel::Debug);
	DebugOut.enable();
	// put your setup code here, to run once:
	mManager.begin(steps, 2);
	delay(2000);
	mManager.read();
	mManager.printSensors();
}

void loop() {

}
