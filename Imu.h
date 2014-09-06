#ifndef _IMU_H
#define _IMU_H

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

// sensor sensitivities
#define GYRO_SNSTV (1.0 / 14.375)
#define ACCL_SNSTV (1.0 / 256.0)
#define MAG_SNSTV (1.0 / 1090.0)

#include "Vec3.h"


class Imu
{
 private:

 public:
  static inline void setRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value, uint8_t stop = true);
  void init();
  void getData();
  void setNeutral(unsigned short iterations = 10, unsigned short dly = 20);

  Vec3<double> accl;
  Vec3<double> gyro;
  Vec3<double> mag;

  Vec3<double> neutralAccl_;
  Vec3<double> neutralGyro_;
  Vec3<double> neutralMag_;
};

inline void Imu::setRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value, uint8_t stop)
{
  // set a register at <regAddr> of device at <devAddr> with value <value>.
  // set stop to false to prepare a register for reading
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  if (stop)
    Wire.write(value);
  Wire.endTransmission(stop);
}

inline void Imu::init()
{
  // accelerometer config
  setRegister(ADDR_ACCL, ACCL_POWER_CTL, BIT3); // enable measurement
  setRegister(ADDR_ACCL, ACCL_DATA_FORMAT, BIT3); // dynamic resolution

  // gyrometer config
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

  // mag config
  // setting to single output mode
  setRegister(ADDR_MAG, MAG_CONFIG1, 0x70);
  setRegister(ADDR_MAG, MAG_CONFIG2, 0xA0);
  setRegister(ADDR_MAG, MAG_MODE, 0x00); // continous mode (0x00)
}

inline void Imu::getData()
{
  // read out accelerometer
  setRegister(ADDR_ACCL, ACCL_DATAXYZ, 0, false);
  Wire.requestFrom(ADDR_ACCL, 6);
  accl.x = (double) ((short) (Wire.read() + Wire.read() * TWOPOWEIGHT)) * ACCL_SNSTV;
  accl.y = (double) ((short) (Wire.read() + Wire.read() * TWOPOWEIGHT)) * ACCL_SNSTV;
  accl.z = (double) ((short) (Wire.read() + Wire.read() * TWOPOWEIGHT)) * ACCL_SNSTV;
  Wire.endTransmission(true);

  // read out gyrometer
  setRegister(ADDR_GYRO, GYRO_DATAXYZ, 0, false);
  Wire.requestFrom(ADDR_GYRO, 6);
  gyro.x = (double) ((short) (Wire.read() * TWOPOWEIGHT + Wire.read())) * GYRO_SNSTV;
  gyro.y = (double) ((short) (Wire.read() * TWOPOWEIGHT + Wire.read())) * GYRO_SNSTV;
  gyro.z = (double) ((short) (Wire.read() * TWOPOWEIGHT + Wire.read())) * GYRO_SNSTV;
  Wire.endTransmission(true);

  // read out magnetometer
  setRegister(ADDR_MAG,0x03, 0); // reset read cursor to the first data field
  Wire.requestFrom(ADDR_MAG, 6);
  mag.x = (double) ((short) (Wire.read() + Wire.read() * TWOPOWEIGHT)) * MAG_SNSTV;
  mag.y = (double) ((short) (Wire.read() + Wire.read() * TWOPOWEIGHT)) * MAG_SNSTV;
  mag.z = (double) ((short) (Wire.read() + Wire.read() * TWOPOWEIGHT)) * MAG_SNSTV;
  Wire.endTransmission(true);
}

void Imu::setNeutral(unsigned short iterations, unsigned short dly)
{
  neutralAccl_ = Vec3<double>(0,0,0);
  neutralGyro_ = Vec3<double>(0,0,0);
  neutralMag_ = Vec3<double>(0,0,0);

  for (int i = 0; i < iterations; ++i)
  {
    getData();
    neutralAccl_ += accl;
    neutralGyro_ += gyro;
    neutralMag_ += mag;
    delay(dly);
  }

  neutralAccl_ /= iterations;
  neutralGyro_ /= iterations;
  neutralMag_ /= iterations;
}

#endif  // _IMU_H
