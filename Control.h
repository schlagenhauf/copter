#ifndef _CONTROL_H
#define _CONTROL_H

#include <algorithm>
#include <mk20dx128.h>
#include <wiring.h>
#include <Arduino.h>
#include <inttypes.h>

#define RCV_CHAN1 6
#define RCV_CHAN2 7
#define RCV_CHAN3 8
#define RCV_CHAN4 9

#define CHAN_ROLL 0
#define CHAN_PITCH 1
#define CHAN_YAW 3
#define CHAN_SPEED 2

#define ROLL_MIN (int16_t)1050
#define ROLL_MAX (int16_t)2050
#define ROLL_RANGE ((ROLL_MAX - ROLL_MIN))
#define PITCH_MIN (int16_t)1050
#define PITCH_MAX (int16_t)2050
#define PITCH_RANGE (PITCH_MAX - PITCH_MIN)
#define YAW_MIN (int16_t)1050
#define YAW_MAX (int16_t)2050
#define YAW_RANGE (YAW_MAX - YAW_MIN)
#define SPEED_MIN (int16_t)1050
#define SPEED_MAX (int16_t)2050
#define SPEED_RANGE (SPEED_MAX - SPEED_MIN)

// Handy singleton define
#define ControlST (*(Control::getInst()))

using namespace std;

class Control
{
 private:
  float roll, pitch, yaw, speed;

  static Control* _inst;

 public:
  int16_t chanVal[4];
  int16_t elapsed[4];

  static Control* getInst();

  void init();

  float getRoll();
  float getPitch();
  float getYaw();
  float getSpeed();
};

// Singleton instance
Control* Control::_inst = NULL;

Control* Control::getInst()
{
  if (!_inst)
    _inst = new Control;

  return _inst;
}

void isr1()
{
  if (digitalRead(RCV_CHAN1))
    ControlST.elapsed[0] = micros();
  else
    ControlST.chanVal[0] = micros() - ControlST.elapsed[0];
}

void isr2()
{
  if (digitalRead(RCV_CHAN2))
    ControlST.elapsed[1] = micros();
  else
    ControlST.chanVal[1] = micros() - ControlST.elapsed[1];
}

void isr3()
{
  if (digitalRead(RCV_CHAN3))
    ControlST.elapsed[2] = micros();
  else
    ControlST.chanVal[2] = micros() - ControlST.elapsed[2];
}

void isr4()
{
  if (digitalRead(RCV_CHAN4))
    ControlST.elapsed[3] = micros();
  else
    ControlST.chanVal[3] = micros() - ControlST.elapsed[3];
}


void Control::init()
{
  pinMode(RCV_CHAN1, INPUT);
  pinMode(RCV_CHAN2, INPUT);
  pinMode(RCV_CHAN3, INPUT);
  pinMode(RCV_CHAN4, INPUT);

  attachInterrupt(RCV_CHAN1, isr1, CHANGE);
  attachInterrupt(RCV_CHAN2, isr2, CHANGE);
  attachInterrupt(RCV_CHAN3, isr3, CHANGE);
  attachInterrupt(RCV_CHAN4, isr4, CHANGE);
}

float Control::getRoll()
{
  return max(min((float) (chanVal[CHAN_ROLL] - ROLL_MIN) / ROLL_RANGE, 1.0f), 0.0f);
}

float Control::getPitch()
{
  return max(min((float) (chanVal[CHAN_PITCH] - PITCH_MIN) / PITCH_RANGE, 1.0f), 0.0f);
}

float Control::getYaw()
{
  return max(min((float) (chanVal[CHAN_YAW] - YAW_MIN) / YAW_RANGE, 1.0f), 0.0f);
}

float Control::getSpeed()
{
  return max(min((float) (chanVal[CHAN_SPEED] - SPEED_MIN) / SPEED_RANGE, 1.0f), 0.0f);
}

#endif  // _CONTROL_H
