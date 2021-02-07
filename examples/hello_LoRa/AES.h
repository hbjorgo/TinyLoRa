#ifndef Encryption_H
#define Encryption_H

#include <Arduino.h>
#if defined(ARDUINO_ARCH_AVR)
#include <avr/pgmspace.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <pgmspace.h>
#endif

class AES {
  public:
    AES();
    void AES_Encrypt(unsigned char *Data, unsigned char *Key);

  private:
    static const unsigned char S_Table[16][16];
    void AES_Add_Round_Key(unsigned char *Round_Key, unsigned char (*State)[4]);
    void AES_Calculate_Round_Key(unsigned char Round, unsigned char *Round_Key);
    unsigned char AES_Sub_Byte(unsigned char Byte);
    void AES_Shift_Rows(unsigned char (*State)[4]);
    void AES_Mix_Collums(unsigned char (*State)[4]);
};

#endif
