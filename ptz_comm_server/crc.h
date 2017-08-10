#ifndef CRC_H_
#define CRC_H_

#include <cstdint>

class CRC{
public:
  CRC();
  virtual ~CRC();

  uint8_t crc8PushBlock(uint8_t *pcrc, uint8_t *block, uint16_t count);

protected:

private:
  void crc8PushByte(uint8_t *crc, uint8_t ch);
};

#endif    // CRC_H_


