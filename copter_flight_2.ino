/* Multicopter Flight Code #2
 * Copyright Jonas Schlagenhauf, 2013
 */

#define NULL 0

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <string.h>

#include "Actuators.h"
#include "Conversion.h"
#include "Imu.h"
#include "Control.h"
#include "Quaternion.h"
#include "Vec3.h"

using namespace Geometry;

Quaternion copterRot;

unsigned long lastTime;
//float factor = PI / (14.375f * 180) * 0.125f;

Imu imu;

void setup()
{
  Wire.begin();
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  imu.init();
  imu.setNeutral(10, 10);


  uint8_t pins[]{20,21,22,23};
  ActuatorsST.init(pins);

  ControlST.init();

  lastTime = millis();
}

void loop()
{
  // convert IMU readout to quaternion rotation
  double dTime = (double)(millis() - lastTime) / 1000.0;
  lastTime = millis();

  imu.getData();
  Vec3<double> corrGyro = (imu.gyro - imu.neutralGyro_) * (PI / 180 * 0.125 * dTime);
  Quaternion gyro(-corrGyro.x, corrGyro.y, corrGyro.z);
  gyro.normalize();

  Quaternion accl(imu.accl, imu.neutralAccl_, copterRot.getYaw());
  accl.normalize();

  copterRot = (copterRot * gyro).slerp(accl, 0.05);
  copterRot.normalize();

  // control motors
  Quaternion target (0, (ControlST.getPitch() - 0.5) * PI, 0);
  ActuatorsST.generateMotorValues(copterRot, target, ControlST.getSpeed());

  Serial.print(ActuatorsST.powers_[0]);
  Serial.print(" ");
  Serial.print(ActuatorsST.powers_[1]);
  Serial.print(" ");
  Serial.print(ActuatorsST.powers_[2]);
  Serial.print(" ");
  Serial.print(ActuatorsST.powers_[3]);
  Serial.print("\n");

  // write to serial out
  /*
  char byteArray[16];
  copterRot.toByteArray(byteArray);

  Serial.write(0x02); // STX (start of text)
  Serial.write(byteArray, 16); // sensor data
  */

  digitalWrite(13, HIGH);
  delay(20);
  digitalWrite(13, LOW);
  delay(20);
}
