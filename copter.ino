/* Multicopter Flight Code #2
 * Copyright Jonas Schlagenhauf, 2013
 */

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <string.h>

#include "Actuators.h"
//#include "Conversion.h"
#include "Imu.h"
#include "Control.h"
#include "Quaternion.h"
#include "Vec3.h"
#include "pb_encode.h"
#include "copcom.pb.h"

using namespace Geometry;

Quaternion copterRot;

unsigned long lastTime;

Imu imu;

PbCopterState stateMsg;

void setup()
{
  Wire.begin();
  Serial.begin(9600);

  imu.init();
  imu.setNeutral(10, 10);
  ActuatorsST.init();
  ControlST.init();

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  lastTime = millis();

  // pre-set protobuf field markers
  stateMsg.has_orientation = true;
  stateMsg.has_motorSpeed = true;
}


void loop()
{
  // convert IMU readout to quaternion rotation
  double dTime = (double)(millis() - lastTime) / 1000.0;
  lastTime = millis();

  imu.getData();
  Vec3<double> corrGyro = (imu.gyro - imu.neutralGyro_) * (PI / 180 * 0.125 * dTime);
  Quaternion gyro(corrGyro.x, corrGyro.y, corrGyro.z);
  gyro.normalize();

  Quaternion accl(imu.accl, imu.neutralAccl_, copterRot.getYaw());
  accl.normalize();

  copterRot = (copterRot * gyro).slerp(accl, 0.004);
  copterRot.normalize();

  // control motors
  Quaternion target (-(ControlST.getPitch() - 0.5) * PI / 4.0, (ControlST.getRoll() - 0.5) * PI / 4.0, (ControlST.getYaw() - 0.5) * PI / 4.0);
  ActuatorsST.generateMotorValues(copterRot, target, ControlST.getSpeed());
  ActuatorsST.applyMotorValues();

  stateMsg.orientation.x = copterRot.x;
  stateMsg.orientation.y = copterRot.y;
  stateMsg.orientation.z = copterRot.z;
  stateMsg.orientation.w = copterRot.w;
  stateMsg.motorSpeed.w = ActuatorsST.powers_[0];
  stateMsg.motorSpeed.x = ActuatorsST.powers_[1];
  stateMsg.motorSpeed.y = ActuatorsST.powers_[2];
  stateMsg.motorSpeed.z = ActuatorsST.powers_[3];


  uint8_t buffer[128];
  pb_ostream_t outstream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  if(pb_encode(&outstream, PbCopterState_fields, &stateMsg)) {
    Serial.write(0xa3);
    Serial.write(0xa3);
    Serial.write(0xa3);
    Serial.write(buffer, outstream.bytes_written); // sensor data
  }


  digitalWrite(13, HIGH);
  delay(1);
  digitalWrite(13, LOW);
  delay(9);
}
