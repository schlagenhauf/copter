#ifndef _SENSORS_H
#define _SENSORS_H

// I2C Addresses
#define ADDR_GYRO B1101000
#define ADDR_ACCL B1010011
#define ADDR_MAG B0011110

// Register Addresses
#define ACCL_POWER_CTL 0x2D
#define ACCL_DATA_FORMAT 0x31
#define ACCL_DATAXYZ 0x32
#define GYRO_DATAXYZ 0x1D
#define MAG_CONFIG1 0x00
#define MAG_CONFIG2 0x01
#define MAG_MODE 0x02

// Sensor Specs, TODO: actually use them
#define GYRO_SNSTV 14.375f
#define ACCL_SNSTV 256.f
#define MAG_SNSTV 1090.f

#include "Vec3.h"

// Handy singleton define
#define SensorsST (*(Sensors::getInst()))

struct SensorData
{
  Vec3<short> accl;
  Vec3<short> gyro;
  Vec3<short> mag;
};

class Sensors
{
 private:
  static Sensors* _inst;

 public:
  static Sensors* getInst();
  static void del();

  static inline void setRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value, uint8_t stop = true);
  static inline void initAcclSensor();
  static inline void initGyroSensor();
  static inline void initMagSensor();
  static inline void getAcclData(short* array);
  static inline void getGyroData(short* array);
  static inline void getMagData(short* array);
  void setIMUNeutral(unsigned short iterations = 10, unsigned short dly = 20);

  Vec3<float> _neutralAccl;
  Vec3<float> _neutralGyro;
  Vec3<float> _neutralMag;
};

// Singleton instance
Sensors* Sensors::_inst = NULL;

Sensors* Sensors::getInst()
{
  if (!_inst)
    _inst = new Sensors;

  return _inst;
}

void Sensors::del()
{
  if (_inst)
  {
    delete _inst;
    _inst = NULL;
  }
}

inline void Sensors::setRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value, uint8_t stop)
{
  // set a register at <regAddr> of device at <devAddr> with value <value>.
  // set stop to false to prepare a register for reading
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  if (stop)
    Wire.write(value);
  Wire.endTransmission(stop);
}

inline void Sensors::initAcclSensor()
{
  // set the accelerometer config
  setRegister(ADDR_ACCL, ACCL_POWER_CTL, BIT3); // enable measurement
  setRegister(ADDR_ACCL, ACCL_DATA_FORMAT, BIT3); // dynamic resolution
}

inline void Sensors::initGyroSensor()
{
  // set the gyrometer config
  /*
  Wire.beginTransmission(ADDR_GYRO);
  Wire.write(0x38); // 0x38 is the register FIFO_CTL
  Wire.write(BIT7); // set bit7 for Stream Mode
  Wire.endTransmission();
  Wire.beginTransmission(ADDR_GYRO);
  Wire.write(0x31); // 0x38 is the register DATA_FORMAT
  Wire.write(BIT7); // set bit3 for full resolution. Bit1 and 0 are for the range setting
  Wire.endTransmission();
  */
}

inline void Sensors::initMagSensor()
{
  // setting to single output mode
  setRegister(ADDR_MAG, MAG_CONFIG1, 0x70);
  setRegister(ADDR_MAG, MAG_CONFIG2, 0xA0);
  setRegister(ADDR_MAG, MAG_MODE, 0x00); // continous mode (0x00)
}

inline void Sensors::getAcclData(short* array)
{
  array[0] = 0;
  array[1] = 0;
  array[2] = 0;

  // read out accelerometer
  setRegister(ADDR_ACCL, ACCL_DATAXYZ, 0, false);

  Wire.requestFrom(ADDR_ACCL, 6);
  int cursor = 0;
  while (Wire.available() >= 2)
  {
    uint8_t first = Wire.read();
    uint8_t second = Wire.read();
    array[cursor] = second * TWOPOWEIGHT + first;
    ++cursor;
  }
  Wire.endTransmission(true);
}

inline void Sensors::getGyroData(short* array)
{
  array[0] = 0;
  array[1] = 0;
  array[2] = 0;

  // read out gyrometer
  setRegister(ADDR_GYRO, GYRO_DATAXYZ, 0, false);

  Wire.requestFrom(ADDR_GYRO, 6);
  int cursor = 0;
  while (Wire.available() >= 2)
  {
    byte first = Wire.read();
    byte second = Wire.read();
    array[cursor] = first * TWOPOWEIGHT + second;
    ++cursor;
  }
  Wire.endTransmission(true);
}

inline void Sensors::getMagData(short* array)
{
  array[0] = 0;
  array[1] = 0;
  array[2] = 0;

  Wire.requestFrom(ADDR_MAG, 6);
  int cursor = 0;
  while (Wire.available() >= 2)
  {
    byte first = Wire.read();
    byte second = Wire.read();
    array[cursor] = second * TWOPOWEIGHT + first;
    ++cursor;
  }
  Wire.endTransmission(true);

  // reset read cursor to the first data field
  setRegister(ADDR_MAG,0x03, 0);
}

void Sensors::setIMUNeutral(unsigned short iterations, unsigned short dly)
{
  int acclVals[3];
  int gyroVals[3];
  int magVals[3];

  short buffer[3];
  for (int i = 0; i < iterations; ++i)
  {
    getAcclData(buffer);
    acclVals[0] += buffer[0];
    acclVals[1] += buffer[1];
    acclVals[2] += buffer[2];
    getGyroData(buffer);
    gyroVals[0] += buffer[0];
    gyroVals[1] += buffer[1];
    gyroVals[2] += buffer[2];
    getMagData(buffer);
    magVals[0] += buffer[0];
    magVals[1] += buffer[1];
    magVals[2] += buffer[2];
    delay(dly);
  }

  acclVals[0] /= iterations;
  acclVals[1] /= iterations;
  acclVals[2] /= iterations;
  gyroVals[0] /= iterations;
  gyroVals[1] /= iterations;
  gyroVals[2] /= iterations;
  magVals[0] /= iterations;
  magVals[1] /= iterations;
  magVals[2] /= iterations;

  _neutralAccl = Vec3<float>(acclVals[0], acclVals[1], acclVals[2]);
  _neutralGyro = Vec3<float>(gyroVals[0], gyroVals[1], gyroVals[2]);
  _neutralMag = Vec3<float>(magVals[0], magVals[1], magVals[2]);
}

#endif  // _SENSORS_H
