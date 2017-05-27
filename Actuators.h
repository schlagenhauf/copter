#ifndef _ACTUATORS_H
#define _ACTUATORS_H

#include "Quaternion.h"
#include "Vec3.h"
#include "Pid.h"
#include "QPid.h"

// Motor specs
#define NUM_ACTUATORS 4
#define MIN_SIGNAL 1100
#define MAX_SIGNAL 1400
//#define MAX_SIGNAL 1750
//#define MAX_SIGNAL 1953
#define SIGNAL_RANGE (MAX_SIGNAL - MIN_SIGNAL)
#define MAP_SIGNAL(x) (MIN_SIGNAL + x * SIGNAL_RANGE)

// motor pins
#define MOTOR_FRONT_PIN 20
#define MOTOR_LEFT_PIN 21
#define MOTOR_BACK_PIN 22
#define MOTOR_RIGHT_PIN 23

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
  Servo actuators_[NUM_ACTUATORS];

  Pid rollPid_;
  Pid pitchPid_;
  Pid yawPid_;


 public:

  float powers_[NUM_ACTUATORS];

  Actuators () : rollPid_(1,0,0), pitchPid_(1,0,0), yawPid_(1,0,0) {};

  // Singleton instance
  static Actuators* _inst;
  // Singleton functions
  static Actuators* getInst();

  // Creates the given amount of actuators and starts the calibration
  void init();

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
    _inst = new Actuators();

  return _inst;
}

void Actuators::init()
{
  // init PIDs
  rollPid_ = Pid(0.5, 0.000, 3.5);
  pitchPid_ = Pid(0.5, 0.000, 3.5);
  yawPid_ = Pid(0.5, 0.000, 3.5);

  /*
  rollPid_ = Pid(0.5, 0.005, 4.2);
  pitchPid_ = Pid(0.5, 0.005, 4.2);
  yawPid_ = Pid(0.5, 0.005, 4.2);
  */

  // attach actuators to pins
  actuators_[MOTOR_FRONT].attach(MOTOR_FRONT_PIN);
  actuators_[MOTOR_LEFT].attach(MOTOR_LEFT_PIN);
  actuators_[MOTOR_BACK].attach(MOTOR_BACK_PIN);
  actuators_[MOTOR_RIGHT].attach(MOTOR_RIGHT_PIN);

  digitalWrite(13, HIGH);

  // calibrate motors
  actuators_[MOTOR_FRONT].writeMicroseconds(MAX_SIGNAL);
  actuators_[MOTOR_LEFT].writeMicroseconds(MAX_SIGNAL);
  actuators_[MOTOR_BACK].writeMicroseconds(MAX_SIGNAL);
  actuators_[MOTOR_RIGHT].writeMicroseconds(MAX_SIGNAL);

  /*
  bool on = true;
  for (int i = 0; i < 5; ++i)
  {
    digitalWrite(13, on);
    on = !on;
    delay(700);
  }
  */
  delay(3500);

  actuators_[MOTOR_FRONT].writeMicroseconds(MIN_SIGNAL);
  actuators_[MOTOR_LEFT].writeMicroseconds(MIN_SIGNAL);
  actuators_[MOTOR_BACK].writeMicroseconds(MIN_SIGNAL);
  actuators_[MOTOR_RIGHT].writeMicroseconds(MIN_SIGNAL);

  /*
  for (int i = 0; i < 20; ++i)
  {
    digitalWrite(13, on);
    on = !on;
    delay(100);
  }
  */
  delay(7000);

  digitalWrite(13, LOW);
}

void Actuators::applyMotorValues()
{
  actuators_[MOTOR_FRONT].writeMicroseconds(MAP_SIGNAL(constrain(powers_[MOTOR_FRONT],0,1)));
  actuators_[MOTOR_LEFT].writeMicroseconds(MAP_SIGNAL(constrain(powers_[MOTOR_LEFT],0,1)));
  actuators_[MOTOR_BACK].writeMicroseconds(MAP_SIGNAL(constrain(powers_[MOTOR_BACK],0,1)));
  actuators_[MOTOR_RIGHT].writeMicroseconds(MAP_SIGNAL(constrain(powers_[MOTOR_RIGHT],0,1)));
}

void Actuators::zeroMotorValues()
{
  for (uint8_t i = 0; i < NUM_ACTUATORS; ++i)
    powers_[i] = 0;
}

void Actuators::generateMotorValues(Quaternion& actual, Quaternion& target, float scale)
{
  //Vec3<float> action = pid_(actual, target);
  Quaternion deltaRot = actual * target.inverse();
  double dRoll = deltaRot.getRoll();
  double dPitch = deltaRot.getPitch();
  double dYaw = deltaRot.getYaw();

  double rollAction = rollPid_(dRoll, 0);
  double pitchAction = pitchPid_(dPitch, 0);
  double yawAction = yawPid_(dYaw, 0);

  zeroMotorValues();
  /*
  if (scale > 0.1)
  {
    setLeft(pitchAction * 0.6f); // somehow switched
    setForward(rollAction * 0.6f); // somehow switched
    setTurn(yawAction * 0.6f); // somehow switched
  }
  */

  setAscend(scale * 0.5);

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
