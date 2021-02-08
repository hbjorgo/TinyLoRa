#ifndef AES_CMAC_H
#define AES_CMAC_H

#include <Arduino.h>
#include "AES.h"

class AES_CMAC {
  public:
    void Calculate(unsigned char *Data,
                   unsigned char *Final_MIC,
                   unsigned char Data_Length,
                   uint8_t *NwkSKey,
                   uint8_t &randomNum,
                   uint8_t &txrandomNum);
  private:
    AES aes;
    void Generate_Keys(unsigned char *K1, unsigned char *K2, uint8_t *NwkSKey);
    void XOR(unsigned char *New_Data, unsigned char *Old_Data);
    void Shift_Left(unsigned char *Data);
};

#endif
