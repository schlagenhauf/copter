#ifndef _ACTUATORS_H
#define _ACTUATORS_H

#include "Quaternion.h"
#include "Vec3.h"
#include "Pid.h"

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

using namespace Geometry;

class Actuators
{
 private:
  // Actual properties
  uint8_t pins_[NUM_ACTUATORS];
  Servo actuators_[NUM_ACTUATORS];
  float intRollError_, intPitchError_, intYawError_;
  float dRollError_, dPitchError_, dYawError_;

  Pid pid_;


 public:

  float powers_[NUM_ACTUATORS];

  Actuators (float p, float i, float d) : pid_(p,i,d) {};

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

  // Compensates unwanted roations by countersteering
  void generateMotorValues(Quaternion& actual, Quaternion& target, float scale);

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
    _inst = new Actuators(0.5, 0.0, 1.0);

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

void Actuators::generateMotorValues(Quaternion& actual, Quaternion& target, float scale)
{
  Vec3<float> action = pid_(actual, target);

  zeroMotorValues();
  setLeft(action.x * scale);
  setForward(action.y * scale);
  setTurn(action.z * scale);
  //applyMotorValues();
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
