#include "AT24C512.h"
#include "EXT_FUNCTION.H"

/* Memory usage 2 byte*/
idata char tempData = 0;

/**********************/
/* Implement function */
/**********************/
char read(unsigned short idata index){
		//char tempData;
    //Write device information
    I2C_Start_EEPROM();
    I2C_SentByte_EEPROM(0xA0); //Device address
    I2C_SentByte_EEPROM(index >> 8); //MSB
    I2C_SentByte_EEPROM(index & 0xFF); //LSB

    //Read data
    I2C_Start_EEPROM();
    I2C_SentByte_EEPROM(0xA0 | 0x01); //Switch to read mode
    tempData = I2C_ReceiveByte_EEPROM();
    SendAcknowledge_EEPROM(1);
    I2C_Stop_EEPROM();
    
    return tempData;
}

void write(unsigned short idata index, char idata datain){
    I2C_Init_EEPROM();
    I2C_Start_EEPROM();
    I2C_SentByte_EEPROM(0xA0); //Device address
    I2C_SentByte_EEPROM(index >> 8); //MSB
    I2C_SentByte_EEPROM(index & 0xFF); //LSB
    I2C_SentByte_EEPROM(datain);
    I2C_Stop_EEPROM();
}
