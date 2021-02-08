#include "AES_CMAC.h"

void AES_CMAC::Calculate(unsigned char *Data,
                             unsigned char *Final_MIC,
                             unsigned char Data_Length,
                             uint8_t *NwkSKey,
                             uint8_t &randomNum,
                             uint8_t &txrandomNum) {
  unsigned char i;

  unsigned char Key_K1[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  unsigned char Key_K2[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  unsigned char Old_Data[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  unsigned char New_Data[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  unsigned char Number_of_Blocks = 0x00;
  unsigned char Incomplete_Block_Size = 0x00;
  unsigned char Block_Counter = 0x01;

  // Calculate number of Blocks and blocksize of last block
  Number_of_Blocks = Data_Length / 16;
  Incomplete_Block_Size = Data_Length % 16;

  if (Incomplete_Block_Size != 0) {
    Number_of_Blocks++;
  }

  Generate_Keys(Key_K1, Key_K2, NwkSKey);

  // Perform full calculation until n-1 messsage blocks
  while (Block_Counter < Number_of_Blocks) {
    // Copy data into array
    for (i = 0; i < 16; i++) {
      New_Data[i] = *Data;
      Data++;
    }

    // Preform XOR with old data
    XOR(New_Data, Old_Data);

    // Preform AES encryption
    aes.Encrypt(New_Data, NwkSKey);

    // Copy New_Data to Old_Data
    for (i = 0; i < 16; i++) {
      Old_Data[i] = New_Data[i];
    }

    // Raise Block counter
    Block_Counter++;

    for (uint8_t x = 0; x < 16; x++) {
      Serial.print(Old_Data[i]);
    }
  }

  // Perform calculation on last block
  // Check if Datalength is a multiple of 16
  if (Incomplete_Block_Size == 0) {
    // Copy last data into array
    for (i = 0; i < 16; i++) {
      New_Data[i] = *Data;
      Data++;
    }

    // Preform XOR with Key 1
    XOR(New_Data, Key_K1);

    // Preform XOR with old data
    XOR(New_Data, Old_Data);

    // Preform last AES routine
    // read NwkSkey from PROGMEM
    aes.Encrypt(New_Data, NwkSKey);
  } else {
    // Copy the remaining data and fill the rest
    for (i = 0; i < 16; i++) {
      if (i < Incomplete_Block_Size) {
        New_Data[i] = *Data;
        Data++;
      }
      if (i == Incomplete_Block_Size) {
        New_Data[i] = 0x80;
      }
      if (i > Incomplete_Block_Size) {
        New_Data[i] = 0x00;
      }
    }

    // Preform XOR with Key 2
    XOR(New_Data, Key_K2);

    // Preform XOR with Old data
    XOR(New_Data, Old_Data);

    // Preform last AES routine
    aes.Encrypt(New_Data, NwkSKey);
  }

  Final_MIC[0] = New_Data[0];
  Final_MIC[1] = New_Data[1];
  Final_MIC[2] = New_Data[2];
  Final_MIC[3] = New_Data[3];

  // Generate a random number between 0 and 7 to select next transmit channel
  randomNum = Final_MIC[3] & 0x07;

  // Generate a random number between 0 and 7 to randomise next transmit message
  // schedule
  txrandomNum = Final_MIC[2] & 0x07;
}

/**************************************************************************/
/*!
    @brief    Function used to generate keys for the MIC calculation.
    @param    *K1
              Pointer to Key1.
    @param    *K2
              Pointer to Key2.
*/
/**************************************************************************/
void AES_CMAC::Generate_Keys(unsigned char *K1, unsigned char *K2, uint8_t *NwkSKey) {
  unsigned char i;
  unsigned char MSB_Key;

  // Encrypt the zeros in K1 with the NwkSkey
  aes.Encrypt(K1, NwkSKey);

  // Create K1
  // Check if MSB is 1
  if ((K1[0] & 0x80) == 0x80) {
    MSB_Key = 1;
  } else {
    MSB_Key = 0;
  }

  // Shift K1 one bit left
  Shift_Left(K1);

  // if MSB was 1
  if (MSB_Key == 1) {
    K1[15] = K1[15] ^ 0x87;
  }

  // Copy K1 to K2
  for (i = 0; i < 16; i++) {
    K2[i] = K1[i];
  }

  // Check if MSB is 1
  if ((K2[0] & 0x80) == 0x80) {
    MSB_Key = 1;
  } else {
    MSB_Key = 0;
  }

  // Shift K2 one bit left
  Shift_Left(K2);

  // Check if MSB was 1
  if (MSB_Key == 1) {
    K2[15] = K2[15] ^ 0x87;
  }
}

/**************************************************************************/
/*!
    @brief    Function to XOR two character arrays.
    @param    *New_Data
              A pointer to the calculated data.
    @param    *Old_Data
              A pointer to the data to be xor'd.
*/
/**************************************************************************/
void AES_CMAC::XOR(unsigned char *New_Data, unsigned char *Old_Data) {
  unsigned char i;

  for (i = 0; i < 16; i++) {
    New_Data[i] = New_Data[i] ^ Old_Data[i];
  }
}

void AES_CMAC::Shift_Left(unsigned char *Data) {
  unsigned char i;
  unsigned char Overflow = 0;
  // unsigned char High_Byte, Low_Byte;

  for (i = 0; i < 16; i++) {
    // Check for overflow on next byte except for the last byte
    if (i < 15) {
      // Check if upper bit is one
      if ((Data[i + 1] & 0x80) == 0x80) {
        Overflow = 1;
      } else {
        Overflow = 0;
      }
    } else {
      Overflow = 0;
    }

    // Shift one left
    Data[i] = (Data[i] << 1) + Overflow;
  }
}
