#ifndef _ACTUATORS_H
#define _ACTUATORS_H

#include "Quaternion.h"
#include "Vec3.h"

// Motor specs
#define NUM_ACTUATORS 4
#define MIN_SIGNAL 1100
#define MAX_SIGNAL 1250
//#define MAX_SIGNAL 1953
#define SIGNAL_RANGE (MAX_SIGNAL - MIN_SIGNAL)
#define MAP_SIGNAL(x) (MIN_SIGNAL + x * SIGNAL_RANGE)

// Motor mappings
#define MOTOR_FRONT 0
#define MOTOR_LEFT 1
#define MOTOR_BACK 2
#define MOTOR_RIGHT 3

// Handy singleton define
#define ActuatorsST (*(Actuators::getInst()))

using namespace Geometrics;

class Actuators
{
 private:
  // Actual properties
  uint8_t pins_[NUM_ACTUATORS];
  Servo actuators_[NUM_ACTUATORS];
  float intRollError_, intPitchError_, intYawError_;
  float dRollError_, dPitchError_, dYawError_;

  float p_, i_, d_;

 public:

  float powers_[NUM_ACTUATORS];

  // Singleton instance
  static Actuators* _inst;
  // Singleton functions
  static Actuators* getInst();

  // Creates the given amount of actuators and starts the calibration
  void init(const uint8_t* pins);

  // Applies the values in <_power> to the actuators
  void applyMotorValues();

  // Sets all motor powers to zero
  void zeroMotorValues();

  // Sets the PID parameters
  void setPIDParameters(float p, float i, float d);

  // Compensates unwanted roations by countersteering
  void pid(Quaternion& actual, Quaternion& target);

  // Increases the power of all actuators by <amount>.
  // negative values result in a descend
  void setAscend(float speed);

  // Pitches the copter, which results in a forward movement.
  // negative values result a backward movement
  void setForward(float speed);

  // Rolls the copter, which results in a movement to the left (positive values)
  // or right (negative values);
  void setLeft(float speed);

  // Rotates the copter around the vertical axis. Positive values result
  // in a clockwise rotation, negative anticlockwise (viewed from top).
  void setTurn(float speed);
};

// Singleton instance
Actuators* Actuators::_inst = NULL;

Actuators* Actuators::getInst()
{
  if (!_inst)
    _inst = new Actuators();

  return _inst;
}

void Actuators::init(const uint8_t* pins)
{
  // copy pins
  memcpy(pins_, pins, NUM_ACTUATORS * sizeof(uint8_t));

  // attach actuators to pins
  for (uint8_t i = 0; i < NUM_ACTUATORS; ++i)
    actuators_[i].attach(pins_[i]);

  digitalWrite(13, HIGH);

  // calibrate motors
  for (uint8_t i = 0; i < NUM_ACTUATORS; ++i)
    actuators_[i].writeMicroseconds(MAX_SIGNAL);

  bool on = true;
  for (int i = 0; i < 5; ++i)
  {
    digitalWrite(13, on);
    on = !on;
    delay(700);
  }

  for (uint8_t i = 0; i < NUM_ACTUATORS; ++i)
    actuators_[i].writeMicroseconds(MIN_SIGNAL);

  for (int i = 0; i < 20; ++i)
  {
    digitalWrite(13, on);
    on = !on;
    delay(100);
  }

  intRollError_ = 0;
  intPitchError_ = 0;
  intYawError_ = 0;
}

void Actuators::applyMotorValues()
{
  for (uint8_t i = 0; i < NUM_ACTUATORS; ++i)
    actuators_[i].writeMicroseconds(MAP_SIGNAL(powers_[i]));
}

void Actuators::zeroMotorValues()
{
  for (uint8_t i = 0; i < NUM_ACTUATORS; ++i)
    powers_[i] = 0;
}

void Actuators::setPIDParameters(float p, float i, float d)
{
  p_ = p;
  i_ = i;
  d_ = d;
}

void Actuators::pid(Quaternion& actual, Quaternion& target)
{
  // get the difference in rotation
  Quaternion delta = target * actual.inverse();

  float dRoll = atan2(2 * delta.y*delta.w - 2 * delta.x*delta.z, 1 - 2 * delta.y * delta.y - 2 * delta.z * delta.z);
  float dPitch = asin(2 * delta.x*delta.y + 2 * delta.z*delta.w);
  float dYaw = atan2(2 * delta.x*delta.w - 2 * delta.y*delta.z, 1 - 2 * delta.x * delta.x - 2 * delta.z * delta.z);

  intRollError_ += dRoll;
  intPitchError_ += dPitch;
  intYawError_ += dYaw;

  float rollAction = p_ * dRoll + i_ * intRollError_ + d_ * (dRoll-dRollError_);
  float yawAction = p_ * dYaw + i_ * intYawError_ + d_ * (dYaw-dYawError_);
  float pitchAction = p_ * dPitch + i_ * intPitchError_ + d_ * (dPitch-dPitchError_);

  zeroMotorValues();
  setForward(pitchAction);
  setLeft(rollAction);
  setTurn(yawAction);
  applyMotorValues();
}

void Actuators::setAscend(float speed)
{
  powers_[MOTOR_FRONT] += speed;
  powers_[MOTOR_BACK] += speed;
  powers_[MOTOR_LEFT] += speed;
  powers_[MOTOR_RIGHT] += speed;
}

void Actuators::setForward(float speed)
{
  powers_[MOTOR_FRONT] += -speed;
  powers_[MOTOR_BACK] += speed;
}

void Actuators::setLeft(float speed)
{
  powers_[MOTOR_LEFT] += -speed;
  powers_[MOTOR_RIGHT] += speed;
}

void Actuators::setTurn(float speed)
{
  powers_[MOTOR_LEFT] += -speed;
  powers_[MOTOR_RIGHT] += -speed;
  powers_[MOTOR_FRONT] += speed;
  powers_[MOTOR_BACK] += speed;
}

#endif //  _ACTUATORS_H
