#ifndef _COPTER_CONVERSION
#define _COPTER_CONVERSION

// Constants
#define TWOPOWEIGHT (short)256
#define TWOPOWFIFTEEN (short)32768
#define TWOPOWSIXTEEN 65536
#define BIT0 1
#define BIT1 2
#define BIT2 4
#define BIT3 8
#define BIT4 16
#define BIT5 32
#define BIT6 64
#define BIT7 128

class Conversion
{
 public:
  static inline void shortToByteArray(short* shortArray, uint8_t* byteArray)
  {
    byteArray[0] = shortArray[0] >> 8;
    byteArray[1] = shortArray[0] & 255;

    byteArray[2] = shortArray[1] >> 8;
    byteArray[3] = shortArray[1] & 255;

    byteArray[4] = shortArray[2] >> 8;
    byteArray[5] = shortArray[2] & 255;
  }

  static inline short toTwosComplement(short value)
  {
    if (value > TWOPOWFIFTEEN)
      value = TWOPOWSIXTEEN - value;
    return value;
  }

  static inline uint8_t switchEndian(uint8_t value)
  {
    uint8_t retVal = 0;
    retVal |= (value & BIT7) >> 7;
    retVal |= (value & BIT6) >> 5;
    retVal |= (value & BIT5) >> 3;
    retVal |= (value & BIT4) >> 1;
    retVal |= (value & BIT3) << 1;
    retVal |= (value & BIT2) << 3;
    retVal |= (value & BIT1) << 5;
    retVal |= (value & BIT0) << 7;
    return retVal;
  }
};

#endif  // _COPTER_CONVERSION
