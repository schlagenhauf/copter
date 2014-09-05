/* Multicopter Flight Code #1
 * Copyright Jonas Schlagenhauf, 2013
 */

#define NULL 0

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <string.h>

#include "Actuators.h"
#include "Conversion.h"
#include "Sensors.h"
#include "Control.h"
#include "Quaternion.h"
#include "Vec3.h"

using namespace Geometry;

Quaternion copterRot;

unsigned long lastTime;
float factor = PI / (14.375f * 180) * 0.125f;

void setup()
{
  Wire.begin();
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Sensors::initMagSensor();
  Sensors::initAcclSensor();
  Sensors::initGyroSensor();
  SensorsST.setIMUNeutral();

  uint8_t pins[]{20,21,22,23};
  ActuatorsST.init(pins);

  ControlST.init();

  lastTime = millis();
}

void loop()
{
  // convert IMU readout to quaternion rotation
  Vec3<short> gyroVals;
  Sensors::getGyroData((short*)&gyroVals.x);

  float dTime = (float)(millis() - lastTime) / 1000;
  lastTime = millis();

  float x = (gyroVals.x - SensorsST._neutralGyro.x) * factor * dTime;
  float y = (gyroVals.y - SensorsST._neutralGyro.y) * factor * dTime;
  float z = (gyroVals.z - SensorsST._neutralGyro.z) * factor * dTime;

  Quaternion gyro(z,y,x);
  gyro.normalize();

  Vec3<short> acclVals;
  Sensors::getAcclData((short*)&acclVals.x);
  Quaternion accl(acclVals, SensorsST._neutralAccl);
  accl.normalize();

  copterRot = (copterRot * gyro).slerp(accl, 0.08);
  copterRot.normalize();

  // control motors
  Quaternion target (1, 0, 0, 0);
  ActuatorsST.generateMotorValues(copterRot, target, ControlST.getSpeed());
  /*
  ActuatorsST.zeroMotorValues();
  ActuatorsST.setAscend(ControlST.getSpeed());
  ActuatorsST.setForward(ControlST.getPitch() - 0.5f);
  ActuatorsST.setLeft(ControlST.getRoll() - 0.5f);
  ActuatorsST.setTurn(ControlST.getYaw() - 0.5f);
  */
  //ActuatorsST.applyMotorValues();

  /*
  Serial.print(ControlST.getRoll());
  Serial.print(" ");
  Serial.print(ControlST.getPitch());
  Serial.print(" ");
  Serial.print(ControlST.getYaw());
  Serial.print(" ");
  Serial.println(ControlST.getSpeed());
  */

  /*
  Serial.print(ActuatorsST.powers_[0]);
  Serial.print(" ");
  Serial.print(ActuatorsST.powers_[1]);
  Serial.print(" ");
  Serial.print(ActuatorsST.powers_[2]);
  Serial.print(" ");
  Serial.println(ActuatorsST.powers_[3]);
  */

  /*
  Serial.print(copterRot.w == copterRot.w ? "test" : "NaN");
  Serial.print(" ");
  Serial.print(copterRot.w * 1000);
  Serial.print(" ");
  Serial.print(copterRot.x * 1000);
  Serial.print(" ");
  Serial.print(copterRot.y * 1000);
  Serial.print(" ");
  Serial.println(copterRot.z * 1000);
  */

  // write to serial out
  char byteArray[16];
  copterRot.toByteArray(byteArray);

  Serial.write(0x02); // STX (start of text)
  Serial.write(byteArray, 16); // sensor data

  digitalWrite(13, HIGH);
  delay(20);
  digitalWrite(13, LOW);
  delay(20);
}
