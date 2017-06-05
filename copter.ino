/* Multicopter Flight Code #2
 * Copyright Jonas Schlagenhauf, 2013
 */

#include <Arduino.h>
#include <EEPROM.h>
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
#include "pb_decode.h"
#include "copcom.pb.h"

#define MAX_CMD_MSG_SIZE 512

using namespace Geometry;

unsigned long lastTime;

Quaternion copterRot;
Imu imu;
PbCopterState stateMsg;
PbCopterCommand cmdMsg;
bool firstCmdMsg = false;

void writeCmdMsgToEEPROM() {
  uint8_t buffer[MAX_CMD_MSG_SIZE];
  pb_ostream_t outstream = pb_ostream_from_buffer(buffer, MAX_CMD_MSG_SIZE);

  if(!pb_encode(&outstream, PbCopterCommand_fields, &cmdMsg))
    return;

  // write size of pb message into first 4 bytes
  EEPROM.write(0, (outstream.bytes_written >> 24) & 0xFF);
  EEPROM.write(1, (outstream.bytes_written >> 16) & 0xFF);
  EEPROM.write(2, (outstream.bytes_written >> 8) & 0xFF);
  EEPROM.write(3, outstream.bytes_written & 0xFF);

  // write pb message after length header
  for (size_t i = 0; i < outstream.bytes_written; ++i) {
    EEPROM.write(4 + i, buffer[i]); // sensor data
  }
}

void readCmdMsgFromEEPROM() {
  // read size of saved message from the first 4 bytes
  size_t numBytes =
      (EEPROM.read(0) << 24) & (EEPROM.read(1) << 16)
    & (EEPROM.read(2) << 8) & EEPROM.read(3);

  uint8_t* buffer = new uint8_t[numBytes];
  for (size_t i = 0; i < numBytes; ++i) {
    buffer[i] = EEPROM.read(4 + i);
  }

  pb_istream_t instream = pb_istream_from_buffer(buffer, numBytes);
  pb_decode(&instream, PbCopterCommand_fields, &cmdMsg);
  delete[] buffer;
}

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  Wire.begin();
  Serial.begin(115200);

  imu.init();
  imu.setNeutral(10, 10);

  readCmdMsgFromEEPROM();

  digitalWrite(13, HIGH);
  ActuatorsST.init(); // takes some time (10s)
  ActuatorsST.setParams(&cmdMsg);
  digitalWrite(13, LOW);

  ControlST.init();

  lastTime = millis();

  // pre-set protobuf field markers
  stateMsg.has_orientation = true;
  stateMsg.has_controls = true;
  stateMsg.has_refOrientation = true;
  stateMsg.has_motorSpeed = true;
  stateMsg.has_accl = true;
}

void loop()
{
  double dTime = (double)(millis() - lastTime) / 1000.0;
  lastTime = millis();

  imu.getData();

  // set up quaternion from gyro data
  Vec3<double> corrGyro = (imu.gyro - imu.neutralGyro_) * (PI / 180 * 0.125 * dTime);
  Quaternion gyro(corrGyro.x, corrGyro.y, corrGyro.z);
  gyro.normalize();

  // set up quaternion from smoothed accelerometer data
  imu.computeMeanAccl();
  Quaternion accl(imu.meanAccl, imu.neutralAccl_, copterRot.getYaw());
  accl.normalize();

  // interpolate between accl and gyro with a complementary filter
  copterRot = (copterRot * gyro).slerp(accl, 0.004);
  copterRot.normalize();

  // set up reference orientation based on roll / pitch / yaw
  Quaternion ref (-(ControlST.getPitch() - 0.5) * PI / 4.0, (ControlST.getRoll() - 0.5) * PI / 4.0, (ControlST.getYaw() - 0.5) * PI / 4.0);
  ActuatorsST.generateMotorValues(copterRot, ref, ControlST.getSpeed());
  ActuatorsST.applyMotorValues();

  // Check for serial input
  int numBytes = Serial.available();
  if (numBytes > 0) {
    uint8_t* buf = new uint8_t[numBytes];
    Serial.readBytes((char*) buf, numBytes);
    pb_istream_t instream = pb_istream_from_buffer(buf, numBytes);
    pb_decode(&instream, PbCopterCommand_fields, &cmdMsg);
    delete[] buf;

    if (firstCmdMsg) {
      writeCmdMsgToEEPROM();
      firstCmdMsg = false;
    }

    ActuatorsST.setParams(&cmdMsg);
  }

  // Fill state message
  stateMsg.accl.x = imu.meanAccl.x;
  stateMsg.accl.y = imu.meanAccl.y;
  stateMsg.accl.z = imu.meanAccl.z;
  stateMsg.orientation.x = copterRot.x;
  stateMsg.orientation.y = copterRot.y;
  stateMsg.orientation.z = copterRot.z;
  stateMsg.orientation.w = copterRot.w;
  stateMsg.refOrientation.x = ref.x;
  stateMsg.refOrientation.y = ref.y;
  stateMsg.refOrientation.z = ref.z;
  stateMsg.refOrientation.w = ref.w;
  stateMsg.controls.x = ControlST.getRoll();
  stateMsg.controls.y = ControlST.getPitch();
  stateMsg.controls.z = ControlST.getYaw();
  stateMsg.controls.w = ControlST.getSpeed();
  stateMsg.motorSpeed.w = ActuatorsST.powers_[0];
  stateMsg.motorSpeed.x = ActuatorsST.powers_[1];
  stateMsg.motorSpeed.y = ActuatorsST.powers_[2];
  stateMsg.motorSpeed.z = ActuatorsST.powers_[3];

  uint8_t buffer[512];
  pb_ostream_t outstream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  if(pb_encode(&outstream, PbCopterState_fields, &stateMsg)) {
    Serial.write(0xa3);
    Serial.write(0xa3);
    Serial.write(0xa3);
    Serial.write(buffer, outstream.bytes_written); // sensor data
  }

  digitalWrite(13, HIGH);
  delay(10);
  digitalWrite(13, LOW);
  delay(90);
}
