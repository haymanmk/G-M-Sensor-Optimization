/******************************************************************************/
/*                                                                            */
/*   "ext_function.h"                specific eternal function                */
/*                                                                            */
/******************************************************************************/
#ifndef EXT_FUNCTION_H
#define EXT_FUNCTION_H

/* Function in Serial.c */

extern void delay(unsigned int end_count);
extern void putbuf (unsigned char c);
extern char putchar (unsigned char c);
extern void putline (char idata Dataline[], unsigned char DataCount);
extern void serial (void); 

unsigned char GetCommandByte(unsigned char index);
unsigned char GetSerialCount();

extern bit CheckCommandFlag;   // UART interrupt to check command and set flag

extern char* sensor_type_name;

/* Function in IIC_Function.c */

extern void I2C_Wait(void);
extern void I2C_Init(void);
extern void I2C_Init_EEPROM(void);
extern void I2C_Start(void);
extern void I2C_Start_EEPROM(void);
extern void I2C_Stop(void);
extern void I2C_Stop_EEPROM(void);

extern bit I2C_SentByte(unsigned char bytedata);
extern bit I2C_SentByte_EEPROM(unsigned char bytedata);
extern unsigned char I2C_ReceiveByte(void);
extern unsigned char I2C_ReceiveByte_EEPROM(void);
extern void SendAcknowledge(bit ack);
extern void SendAcknowledge_EEPROM(bit ack);
extern void I2C_ByteWrite(unsigned char device,unsigned char address,unsigned char bytedata);
extern unsigned char I2C_ByteRead(unsigned char device,unsigned char address);

extern void I2C_M_Data(unsigned char device,unsigned char address,bit ReadHighResolution);	
extern void I2C_T_Data(unsigned char device,unsigned char address);	
extern void I2C_Gyro_Data(unsigned char device, unsigned char address);
extern void I2C_Acc_Data(unsigned char device, unsigned char address);
extern unsigned char I2cReadTrm2(signed char device,unsigned char address);	
	
extern void I2CReadMultiBytes(signed char device, unsigned char address, unsigned char* _data, char amount);
extern unsigned short TwoBytes2Int(unsigned char HighByte, unsigned char LowByte);
extern void LowerResolution2MatchOldSensor(unsigned char* DataArray);
extern void CalibrationCaculate(unsigned char *Xorigin, unsigned char *Yorigin, unsigned char *Zorigin,
						 short *GainMatrix);
// extern bit StringCompare(char idata **strA, char idata strB[], char num_char);
//extern float Gauss_Data(unsigned char* DataArray);

#endif