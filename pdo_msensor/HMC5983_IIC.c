#include <AT89X52.h>
#include <INTRINS.H>

#include "Ext_function.h"
#include "MCU_GPIO.h"
#include "IIC_Address.h"
#include "AT24C512.h"
#include "LSM9DS1_calibration.h"

//===========================//
//=== Define Sensor Board ===//
//===========================//
//#define HMC5983 0
//#define MMC5883MA 1
#define ACK 0x06

// //===========================//
// //=== Address parameters ====//
// //===========================//
// idata char ADD_I2C;		 //I2C control address
// idata char ADD_CONFIG_A; //Configuration Register A   Read/Write
// idata char ADD_CONFIG_B; //Configuration Register B   Read/Write
// idata char ADD_MODE;	 //Mode Register  Read/Write
// idata char ADD_X_MSB;	 //Data Output X MSB Register  Read
// idata char ADD_X_LSB;	 //Data Output X LSB Register  Read
// idata char ADD_Z_MSB;	 //Data Output Z MSB Register  Read
// idata char ADD_Z_LSB;	 //Data Output Z LSB Register  Read
// idata char ADD_Y_MSB;	 //Data Output Y MSB Register  Read
// idata char ADD_Y_LSB;	 //Data Output Y LSB Register  Read
// idata char ADD_STATUS;	 //Status Register  Read
// idata char ADD_ID_A;	 //Identification Register A  Read
// idata char ADD_ID_B;	 //Identification Register B  Read
// idata char ADD_ID_C;	 //Identification Register C  Read
// idata char ADD_TMP_MSB;	 //Temperature Output MSB Register  Read
// idata char ADD_TMP_LSB;	 //Temperature Output LSB Register  Read
// idata char ADD_ID;		 //Product ID 0b0010 1111

// /**********************************************************************/
// //#define MMC5883MA
// idata char ADD_CONFIG_0; //Control register 0
// idata char ADD_CONFIG_1; //Control register 1
// idata char ADD_CONFIG_2; //Control register 2
// //extern char ADD_X_THR;    //Motion detection threshold of X
// //extern char ADD_Y_THR;    //Motion detection threshold of Y
// //extern char ADD_Z_THR;    //Motion detection threshold of Z
// /**********************************************************************/
// //#define MAG_LSM9DS1
// idata char CTRL_REG1_M; //Control register 1
// idata char CTRL_REG2_M; //Control register 2
// idata char CTRL_REG3_M; //Control register 3
// idata char CTRL_REG4_M; //Control register 4
// idata char CTRL_REG5_M; //Control register 5

// //#define ACGY_LSM9DS1
// idata char ACGY_ADD_I2C;	 //I2C control address 0x60 0110 0000
// idata char ACGY_GYR_ADD_X_LSB;	 //Xout LSB
// idata char ACGY_GYR_ADD_X_MSB;	 //Xout MSB
// idata char ACGY_GYR_ADD_Y_LSB;	 //Yout LSB
// idata char ACGY_GYR_ADD_Y_MSB;	 //Yout MSB
// idata char ACGY_GYR_ADD_Z_LSB;	 //Zout LSB
// idata char ACGY_GYR_ADD_Z_MSB;	 //Zout MSB
// idata char ACGY_ADD_STATUS;	 //Device status
// idata char ACGY_CTRL_REG1_G;	 //Angular rate sensor Control Register1
// idata char ACGY_ORIENT_CFG_G; //Angular rate sensor sign & orientation
// idata char ACGY_CTRL_REG4;	 //GYRO XYZ enable
// idata char ACGY_CTRL_REG5;	 //ACC XYZ enable
// idata char ACGY_CTRL_REG6;	 //ACC full-scale, bandwidth
// idata char ACGY_CTRL_REG7;	 //ACC high resolution mode
// idata char ACGY_CTRL_REG8;	 //Control register 8 >> reset all registers SW_reset
// idata char ACGY_CTRL_REG9;	 //FIFO enable
// idata char ACGY_ADD_TMP_MSB;	 //Temperature Output MSB Register  Read
// idata char ACGY_ADD_TMP_LSB;	 //Temperature Output LSB Register  Read
// idata char ACGY_ADD_ID;		 //Product ID 0b0011 1101

// //#define ACC_LSM9DS1
// idata char ACGY_ACC_ADD_X_LSB;  //Xout LSB
// idata char ACGY_ACC_ADD_X_MSB;  //Xout MSB
// idata char ACGY_ACC_ADD_Y_LSB;  //Yout LSB
// idata char ACGY_ACC_ADD_Y_MSB;  //Yout MSB
// idata char ACGY_ACC_ADD_Z_LSB;  //Zout LSB
// idata char ACGY_ACC_ADD_Z_MSB;  //Zout MSB
// idata char ACGY_FIFO_CTRL; //FIFO mode select

//For RS232 Communication
unsigned char incount;
unsigned char inline[16];					 /* storage for command input line    */
unsigned char code Ending[2] = {0x0D, 0x0A}; /* Ending char CR, LF                */
unsigned char ReadCommandFlag;				 /*  set flag when command is read     */

unsigned char OneByte[1];
//unsigned char TempByte[2];

//unsigned char M_Data[9];
unsigned char T_Data[3]; /* storage for temperature data T[T,,]    */
unsigned char X_Data[2]; /* storage for magnet X data X[MSB,LSB]    */
unsigned char Y_Data[2]; /* storage for magnet Y data Y[MSB,LSB]    */
unsigned char Z_Data[2]; /* storage for magnet Z data Z[MSB,LSB]    */
unsigned char Rx3;		 /* count times  */

unsigned char GyroX_Data[2]; /* storage for gyro X data X[MSB,LSB]    */
unsigned char GyroY_Data[2]; /* storage for gyro Y data Y[MSB,LSB]    */
unsigned char GyroZ_Data[2]; /* storage for gyro Z data Z[MSB,LSB]    */

unsigned char AccX_Data[2]; /* storage for acc X data X[MSB,LSB]    */
unsigned char AccY_Data[2]; /* storage for acc Y data Y[MSB,LSB]    */
unsigned char AccZ_Data[2]; /* storage for acc Z data Z[MSB,LSB]    */

bit DRY_Bit;
bit Gyro_DRY_Bit;
bit Acc_DRY_Bit;
char MODE = 0;
char GyroMODE = 0;
char AccMODE = 0;
// char *sensor_type_name;
bit ReadHighResolution;
unsigned short indexEEPROM; //used to record current index when reading/ writing data in EEPROM

struct SENSOR_TYPE
{
	char type[15];		 //sensor name
	char Device_Address; //sensor address
	char ID_Address;	 //ID address
	char Num_Registers;
	char ID[3]; //ID data
};

// code struct SENSOR_TYPE sensor_type[] = {
// 	// {"HMC5983", 0x3C, 0x0A, 3, "H43"},
// 	// {"MMC5883MA", 0x60, 0x2F, 1, 0x0C},
// 	{"MAG_LSM9DS1", 0x38, 0x0F, 1, 0x3D},
// 	{"ACGY_LSM9DS1", 0xD4, 0x0F, 1, 0x68},
// 	{"LSM9DS1", 0, 0, 0, 0},

// };

/******************************************************************************/
/*       Test	                                                                */
/******************************************************************************/
/*
typedef union{
	idata char _char[4];
	idata float _float;
}test1;
*/

/******************************************************************************/
/*       Initialize                                                           */
/******************************************************************************/
void serial_init(void);
void timer0_init(void);
void Initial_board(void);
void Initial_relay(void);
void Initial_register(void);

/******************************************************************************/
/*       ReadCommand                                                          */
/******************************************************************************/
unsigned char ReadCommand(void);
void COM_Command(void);
void EOL(void);

/******************************************************************************/
/*       Sub-Function for COM port command                                    */
/******************************************************************************/

void ChangeMeasurementMode(unsigned char idata *_inline);
void ReadXYZDataSequentially(void);
void RelayControl_1(unsigned char idata *_inline);
void RelayControl_2(unsigned char idata *_inline);
void SetReadingOutputMode(unsigned char idata *_inline);
void SetReadingAccOutputMode(unsigned char idata *_inline);
void SetReadingGyroOutputMode(unsigned char idata *_inline);
void ControlAlertLight(unsigned char idata *_inline);
void ReadSensorID(unsigned char idata *_inline);
void SetReadingTempMode(void);
//void DetectConnectedSensor(unsigned char xdata *_inline);
// char CheckSensorCorrect(void);
bit CheckArrayEqual(unsigned char idata *_data, char idata **sensor_ID);

/******************************************************************************/
/*       Main                                                                 */
/******************************************************************************/
void main(void)
{
	unsigned int i; //Loop counter
	unsigned int iRx3;
	unsigned int GyroiRx3;
	unsigned int AcciRx3;
	char index_sensor_type_struct;

	Initial_board();
	Initial_relay();

	LED1_ON;
	delay(10000);
	LED1_OFF;
	delay(10000);
	LED2_ON;
	delay(10000);
	LED2_OFF;
	delay(10000);
	LED3_ON;
	delay(10000);
	LED3_OFF;
	delay(10000);

	// detect the connected type of sensor
	// index_sensor_type_struct = CheckSensorCorrect();

	// sensor_type_name = &sensor_type[index_sensor_type_struct].type;

	// SetI2CParameters();
	Initial_register();

	putline("READY", 5);
	EOL();

	//M_Data[0]='X';M_Data[3]='Y';M_Data[6]='Z';
	T_Data[0] = 'T';

	//===================================//
	//============ Main Loop ============//
	//===================================//
	while (1)
	{

		COM_Command();
		//--------------------------------------------------------------------

		//=== MODE 3 ===//

		while (MODE == 3) //MCK<CR><LF>
		{
			RY4_ON;
			putline("MCK", 3);
			EOL();
			if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
			{
				I2C_ByteWrite(ADD_I2C, CTRL_REG3_M, 0x00); //REG3 0b00000000 bit6 bit7>> operation mode>>continuous mode
			}
			// else if (StringCompare(sensor_type_name, "MMC5883MA", 9) == 1)
			// {
			// 	I2C_ByteWrite(ADD_I2C, ADD_CONFIG_0, 0x01); //Internal control 0 >> Initiate magnetic field measurement
			// 	I2C_ByteWrite(ADD_I2C, ADD_CONFIG_1, 0x00); //Internal control 1 >> 0b00000000, turning SW_RST on will reset all registers
			// 	I2C_ByteWrite(ADD_I2C, ADD_CONFIG_2, 0x41); //Internal control 2 >> 0b0100 0001=0x41 Continuous Mode at sampling rate 14Hz
			// }
			// else if (StringCompare(sensor_type_name, "HMC5983", 7) == 1)
			// {
			// 	I2C_ByteWrite(ADD_I2C, ADD_CONFIG_A, 0xFC); //CRA  >> Temp Enable, 8 AVG per Sample, output rate: 220 Hz 0xFC, 15Hz 0xF0
			// 	I2C_ByteWrite(ADD_I2C, ADD_CONFIG_B, 0x00); //CRB  >> +/- 0.88 Gauss  0.73 mG/LSB
			// 	I2C_ByteWrite(ADD_I2C, ADD_MODE, 0x00);		//Mode >> Continuous Mode
			// }
			LED2_ON;

			while (MODE == 3)
			{
				if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
				{
					DRY_Bit = 0;
					while (DRY_Bit == 0)
					{
						DRY_Bit = I2C_ByteRead(ADD_I2C, ADD_STATUS) & 0x08; //0b00001000 polling to check the magnetic field measurement ready
					}
				}
				// else if (StringCompare(sensor_type_name, "MMC5883MA", 9) == 1)
				// {
				// 	DRY_Bit = 0;
				// 	while (DRY_Bit == 0)
				// 	{
				// 		DRY_Bit = I2C_ByteRead(ADD_I2C, ADD_STATUS) & 0x01; //polling to check the magnetic field measurement ready
				// 	}
				// }
				// else if (StringCompare(sensor_type_name, "HMC5983", 7) == 1)
				// {
				// 	while (DRDY == 0)
				// 	{
				// 		_nop_();
				// 	}
				// }
				COM_Command();
				if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
					I2C_M_Data(ADD_I2C, ADD_X_LSB, ReadHighResolution);
				// else if (StringCompare(sensor_type_name, "MMC5883MA", 9) == 1)
				// 	I2C_M_Data(ADD_I2C, ADD_X_LSB, ReadHighResolution);
				// else if (StringCompare(sensor_type_name, "HMC5983", 7) == 1)
				// 	I2C_M_Data(ADD_I2C, ADD_X_MSB, ReadHighResolution);
			}

			if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
			{
				//reset all registers
				I2C_ByteWrite(ADD_I2C, CTRL_REG2_M, 0x24); //REG2 >> 0b00100100, turning SOFT_RST on will reset all registers
			}
			// else if (StringCompare(sensor_type_name, "MMC5883MA", 9) == 1)
			// {
			// 	//reset all registers
			// 	I2C_ByteWrite(ADD_I2C, ADD_CONFIG_1, 0x80); //Internal control 1 >> 0b10000000, turning SW_RST on will reset all registers
			// }
			// else if (StringCompare(sensor_type_name, "HMC5983", 7) == 1)
			// 	I2C_ByteWrite(ADD_I2C, ADD_MODE, 0x03); //Go to Sleep

			putline("ACK", 3);
			EOL();
			putline("READY", 5);
			EOL();
		} //while(MODE == 3)

		//----------------------------------------------------------------------------

		//=== MODE 2 ===//
		// read specified number, Rx3, of magnetic field measurement which is defined by external controller.
		if (MODE == 2) //MR
		{
			RY4_ON;

			putline("MRN", 3);
			EOL();

			iRx3 = Rx3 * 6;

			for (i = 0; i <= iRx3; i++)
			{
				if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
				{
					I2C_ByteWrite(ADD_I2C, CTRL_REG3_M, 0x01); //REG3 0b00000001 bit6 bit7>> operation mode>>single mode
				}
				// else if (StringCompare(sensor_type_name, "MMC5883MA", 9) == 1)
				// {
				// 	I2C_ByteWrite(ADD_I2C, ADD_CONFIG_0, 0x01); //Internal control 0
				// 	I2C_ByteWrite(ADD_I2C, ADD_CONFIG_1, 0x00); //Internal control 1 >> 0b00000000, turning SW_RST on will reset all registers
				// 												//I2C_ByteWrite(ADD_I2C,ADD_CONFIG_2,0x41); //Internal control 2 >> 0b0100 0001=0x41 Continuous Mode at sampling rate 14Hz
				// }
				// else if (StringCompare(sensor_type_name, "HMC5983", 7) == 1)
				// {
				// 	I2C_ByteWrite(ADD_I2C, ADD_CONFIG_A, 0xF0); //CRA  >> Temp Enable, 8 AVG per Sample, output rate: 220 Hz 0xFC, 15Hz 0xF0
				// 	I2C_ByteWrite(ADD_I2C, ADD_CONFIG_B, 0x00); //CRB  >> +/- 0.88 Gauss  0.73 mG/LSB
				// 	I2C_ByteWrite(ADD_I2C, ADD_MODE, 0x80);		//Mode >> Continuous Mode >> Enable the I2C high speed mode, 3400 kHz.
				// }
				//COM_Command();

				if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
					DRY_Bit = I2C_ByteRead(ADD_I2C, ADD_STATUS) & 0x08;
				// else if (StringCompare(sensor_type_name, "MMC5883MA", 9) == 1)
				// 	DRY_Bit = I2C_ByteRead(ADD_I2C, ADD_STATUS) & 0x01;

				if (DRY_Bit == 0)
				{
					LED2_ON;
					delay(1000);
				}

				while (DRY_Bit == 0)
				{
					if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
						DRY_Bit = I2C_ByteRead(ADD_I2C, ADD_STATUS) & 0x08;
					// else if (StringCompare(sensor_type_name, "MMC5883MA", 9) == 1)
					// 	DRY_Bit = I2C_ByteRead(ADD_I2C, ADD_STATUS) & 0x01;

					//if (DRY_Bit == 1)
					if (DRY_Bit == 1)
					{
						LED2_OFF;
					}

					delay(1000);
				}
				if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
					I2C_M_Data(ADD_I2C, ADD_X_LSB, ReadHighResolution);
				// else if (StringCompare(sensor_type_name, "MMC5883MA", 9) == 1)
				// 	I2C_M_Data(ADD_I2C, ADD_X_LSB, ReadHighResolution);
				// else if (StringCompare(sensor_type_name, "HMC5983", 7) == 1)
				// 	I2C_M_Data(ADD_I2C, ADD_X_MSB, ReadHighResolution);
			}

			//I2C_ByteWrite(0x3C,0x02,0x03); //Go to Sleep

			putline("ACK", 3);
			EOL();
			putline("READY", 5);
			EOL();

			MODE = 0;
		}

		//=== MODE 1 ===//
		while (MODE == 1) //AL Pass
		{
			COM_Command();
			RY4_ON;
			delay(15000);
			RY4_OFF;
			delay(20000);
		}

		//=== MODE 4 ===//
		while (MODE == 4) //AL Fail
		{
			COM_Command();
			RY4_OFF;
		}

		//=== MODE 5 ===//
		while (MODE == 5) //AL Test
		{
			COM_Command();
			RY4_ON;
		}
		//=== MODE 6 ===//
		/**
		 * Continuously write data into EEPROM until all the elements in array are refreshed.
		 */

		if (MODE == 6)
		{
			COM_Command();
		}

		//--------------------------------------------------------------------

		//=== MODE 3 ===//

		while (GyroMODE == 3) //MCK<CR><LF>
		{

			putline("YCK", 3);
			EOL();

			while (GyroMODE == 3)
			{

				Gyro_DRY_Bit = 0;

				while (Gyro_DRY_Bit == 0)
				{
					Gyro_DRY_Bit = I2C_ByteRead(ACGY_ADD_I2C, ACGY_ADD_STATUS) & 0x02; //0b00000010 polling to check gyro measurement ready
				}

				COM_Command();

				I2C_Gyro_Data(ACGY_ADD_I2C, ACGY_GYR_ADD_X_LSB);
			}

			//reset all registers
			I2C_ByteWrite(ADD_I2C, CTRL_REG2_M, 0x24); //REG2 >> 0b00100100, turning SOFT_RST on will reset all registers

			putline("ACK", 3);
			EOL();
			putline("READY", 5);
			EOL();
		} //while(MODE == 3)

		//----------------------------------------------------------------------------

		//=== MODE 2 ===//
		// read specified number, Rx3, of magnetic field measurement which is defined by external controller.
		if (GyroMODE == 2) //GR
		{

			putline("YRN", 3);
			EOL();

			GyroiRx3 = Rx3 * 6;

			for (i = 0; i <= GyroiRx3; i++)
			{
				Gyro_DRY_Bit = I2C_ByteRead(ACGY_ADD_I2C, ACGY_ADD_STATUS) & 0x02; // 0b00000010

				if (Gyro_DRY_Bit == 0)
				{
					delay(1000);
				}

				while (Gyro_DRY_Bit == 0)
				{
					Gyro_DRY_Bit = I2C_ByteRead(ACGY_ADD_I2C, ACGY_ADD_STATUS) & 0x02;
					delay(1000);
				}
				I2C_Gyro_Data(ACGY_ADD_I2C, ACGY_GYR_ADD_X_LSB);
			}

			putline("ACK", 3);
			EOL();
			putline("READY", 5);
			EOL();

			GyroMODE = 0;
		}

		//--------------------------------------------------------------------

		//=== MODE 3 ===//

		while (AccMODE == 3) //MCK<CR><LF>
		{

			putline("ACCK", 4);
			EOL();

			while (AccMODE == 3)
			{

				Acc_DRY_Bit = 0;

				while (Acc_DRY_Bit == 0)
				{
					Acc_DRY_Bit = I2C_ByteRead(ACGY_ADD_I2C, ACGY_ADD_STATUS) & 0x01; //0b00000010 polling to check acc ready
				}

				COM_Command();

				I2C_Acc_Data(ACGY_ADD_I2C, ACGY_ACC_ADD_X_LSB);
			}

			//reset all registers
			I2C_ByteWrite(ADD_I2C, CTRL_REG2_M, 0x24); //REG2 >> 0b00100100, turning SOFT_RST on will reset all registers

			putline("ACK", 3);
			EOL();
			putline("READY", 5);
			EOL();
		} //while(MODE == 3)

		//----------------------------------------------------------------------------

		//=== MODE 2 ===//
		// read specified number, Rx3, of magnetic field measurement which is defined by external controller.
		if (AccMODE == 2) //GR
		{

			putline("ACRN", 4);
			EOL();

			AcciRx3 = Rx3 * 6;

			for (i = 0; i <= AcciRx3; i++)
			{
				Acc_DRY_Bit = I2C_ByteRead(ACGY_ADD_I2C, ACGY_ADD_STATUS) & 0x01; // 0b00000010

				if (Acc_DRY_Bit == 0)
				{
					delay(1000);
				}

				while (Acc_DRY_Bit == 0)
				{
					Acc_DRY_Bit = I2C_ByteRead(ACGY_ADD_I2C, ACGY_ADD_STATUS) & 0x01;
					delay(1000);
				}
				I2C_Acc_Data(ACGY_ADD_I2C, ACGY_ACC_ADD_X_LSB);
			}

			putline("ACK", 3);
			EOL();
			putline("READY", 5);
			EOL();

			AccMODE = 0;
		}

	} //Main loop
}

/******************************************************************************/
/*     Command from COM Port                                                  */
/******************************************************************************/

void COM_Command(void)
{
	char _index;
	unsigned char command_count = 0;
	// CHAR_TO_FLOAT idata char_float_conv;

	if (CheckCommandFlag == 1)
	{
		command_count = ReadCommand() - 2; // -2 means ignore CRLF
	}
	if (ReadCommandFlag == 1)
	{
		if (MODE == 6 && command_count >= 2 && indexEEPROM < 12)
		{
			//push in bytes in little-endian approach
			write(indexEEPROM, inline[0]);
			indexEEPROM++;
			putchar(ACK);
		}
		switch (inline[0])
		{

		case ('S'):
			ChangeMeasurementMode(inline);
			break;

		case ('X'):
			ReadXYZDataSequentially();
			break;

		case ('T'):
			SetReadingTempMode();
			break;

		case ('R'): //Read Register
			if (inline[1] == 'S')
			{
				// if (StringCompare(sensor_type_name, "MMC5883MA", 9) == 1)
				// {
				// 	// SET the sensor momentarily to restore the sensor characteristic.
				// 	I2C_ByteWrite(ADD_I2C, ADD_CONFIG_0, 0x08);
				// 	delay(10000);
				// 	I2C_ByteWrite(ADD_I2C, ADD_CONFIG_0, 0x00);
				// }
				// putline("RS", 2);
				// EOL();
			}
			else
			{
				putline("R", 1);
				OneByte[0] = I2C_ByteRead(ADD_I2C, inline[1]);
				putline(OneByte, 1);
				EOL();
			}

			break;

		case ('W'): //WRITE Register
			I2C_ByteWrite(ADD_I2C, inline[1], inline[2]);
			putline("WRITE BYTE", 10);
			EOL();
			break;

		case ('F'): //FFF, FFN, FNF, FNN, where F for OFF and N for ON.
			RelayControl_1(inline);
			break;

		case ('N'): //NFF, NFN, NNF, NNN, where F for OFF and N for ON.
			RelayControl_2(inline);
			break;

		case ('A'):
			if (inline[1] == 'C')
				SetReadingAccOutputMode(inline); //Read acc data
			else
				ControlAlertLight(inline);
			break;

		case ('M'):
			SetReadingOutputMode(inline); //Read magnet data
			break;
		case ('Y'):
			SetReadingGyroOutputMode(inline); //Read gyro data
			break;

		case ('I'):
			ReadSensorID(inline);
			break;

		case ('D'):
			// if (inline[1] == 'S')
			// {
			// 	_index = CheckSensorCorrect();
			// 	sensor_type_name = &sensor_type[_index].type;
			// 	SetI2CParameters();
			// 	putline(sensor_type[_index].type, sizeof(sensor_type[_index].type));
			// 	EOL();
			// }
			break;

		case ('G'):
			ReadSensorID(inline);
			break;

		/* For gain matrix operation */
		case ('E'):
			switch (inline[1])
			{
			// case ('A'): //EEPROM W/R
			// 	switch (inline[2])
			// 	{
			// 		case ('R'):
			// 			if(command_count >= 7 && inline[3] == '0' && (inline[4] == 'X' || inline[4] == 'x')){
			// 				char_float_conv._float = read(16*(inline[5]-'0')+(inline[6]-'0'));
			// 				EOL();
			// 			}
			// 			break;
			// 		case ('W'):
			// 			if(command_count >= 8 && inline[3] == '0' && (inline[4] == 'X' || inline[4] == 'x')){
			// 				write(16*(inline[5]-'0')+(inline[6]-'0'), inline[7]);
			// 				putline("Write 0x", 8);
			// 				putline(inline+5, 2);
			// 				EOL();
			// 			}
			// 			break;
			// 		default:
			// 			putline("Invalid command for EEPROM R/W.", 31);
			// 			EOL();
			// 			break;
			// 	}
			// 	break;
			case ('W'): //Write gain and offset matrix
				// Procedure: PC sends ['EW'] -> MCU returns [<ACK>] -> PC writes datas recursively -> MCU returns [<ACK>]
				// -> until all elements refreshed -> MCU replies [<ACK>] and ['Done']
				putchar(ACK); // acknowledge pc request has been received,
							  // and it is ready to receive array data sequentially.
				MODE = 6;
				indexEEPROM = 0;
				break;
				//EW32=0.123;
				// if(inline[2]>='0' && inline[2]<=
				// '3' && inline[3]>='0' && inline[3]<='2' && inline[4] == '='){
				// 	 char_float_conv._float = byte_to_float(inline, 5, 15);
				// 	 if(char_float_conv._float != -999.0){
				// 		write_gain_and_offset_matrix(inline[2]-'0', inline[3]-'0', char_float_conv._float);
				// 	 	putline("Write success", 13);
				// 	 }
				// 	 else{
				// 		 putline("Write failure", 13);
				// 		 //fail
				// 	 }
				// }
				// else{
				// fail
				// }
				// break;

			// case ('R'): //Read gain and offset matrix
			// 	char_float_conv._float = read_gain_and_offset_matrix(inline[3]-'0', inline[4]-'0');
			// 	break;
			// case ('?'):
			// 	putline("EEPROM operation introduction.", 30);
			// 	EOL();
			// 	break;
			default:
				break;
			}
		} //switch 1

		ReadCommandFlag = 0;
	} // if(ReadCommandFlag  == 1)
}

//MODE = 0 Nothing /= 1 Pass/= 2 MR Output /=3 Contiue MR Output
//----------------------------------------------------------------------------

/******************************************************************************/
/*     Relay Initial Status                                                   */
/******************************************************************************/
void Initial_relay(void)
{
	RY1_OFF;
	RY2_OFF;
	RY3_OFF;
	RY4_ON;
}

/******************************************************************************/
/*       serial_init: initialize serial interface                             */
/******************************************************************************/
void serial_init(void)
{

	SCON = 0x52; // SCON=01010011 SM0=0 SM1=1 SM2=0 REN=1 TB8=0 RB8=0 TI=	1 RI= 0
	TMOD = 0x21;
	TH1 = 256 - (28800 / 9600); //Fosc=11.059M   baudrate=9600
	TR1 = 1;					//TIMRE 1 star Up
}

/******************************************************************************/
/*       timer_init: initialize timer interrupt                               */
/******************************************************************************/
void timer0_init(void)
{

	TMOD = (TMOD & 0xF0) | 0x01; /* Set T/C0 Mode */

	ET0 = 1; /* Enable Timer 0 Interrupts */
	TR0 = 1; /* Start Timer 0 Running */

	//TH0=0xDB;TL0=0xFF;//0.01sec
	TH0 = 0x4B;
	TL0 = 0xFF; //0.05sec

	EA = 1; /* Global Interrupt Enable */ //Later
}

/******************************************************************************/
/*      Timer 0 Interrupt Service Routine.                                    */
/******************************************************************************/
void timer0_ISR(void) interrupt 1
{
	TR0 = 0; // STOP TIMER
	//TH0=0xDB;TL0=0xFF;//0.01 sec
	TH0 = 0x4B;
	TL0 = 0xFF; //0.05sec

	TR0 = 1; //START TIMER
}

/******************************************************************************/
/*       Initialize                                                           */
/******************************************************************************/
void Initial_board(void)
{

	serial_init(); /* Initialize the serial interface  */
				   // timer0_init ();		/* Initialize the timer0 interface  */

	ES = 1; /* Enable serial interrupts */
	PS = 1; /* Set serial interrupts to Hign priority */

	EA = 1; /* Enable Global interrupt*/
}

/******************************************************************************/
/*       ReadCommand                                                          */
/******************************************************************************/
unsigned char ReadCommand(void)
{

	unsigned char index = 0;

	ES = 0; //disable UART interrupt

	incount = GetSerialCount();

	while (index != incount)
	{
		inline[index] = GetCommandByte(index);
		index = index + 1;
	}

	CheckCommandFlag = 0; //Command is checkec and then clear flag
	ReadCommandFlag = 1;  //Command is in inline now, then set new flag

	ES = 1; //enable UART interrupt

	return index;
}

/****************************************************************************/
/*   Sub-Function for COM port command                                      */
/****************************************************************************/

void ChangeMeasurementMode(unsigned char idata *_inline)
{
	if (_inline[1] == '0') //'S0'
	{
		I2C_ByteWrite(ADD_I2C, 0x00, 0xF0);
		I2C_ByteWrite(ADD_I2C, 0x01, 0xA0);
		I2C_ByteWrite(ADD_I2C, 0x02, 0x00);
	}
	else if (_inline[1] == '1') //'S1'
	{
		I2C_ByteWrite(ADD_I2C, 0x00, 0xF1); //poisitive bias
		I2C_ByteWrite(ADD_I2C, 0x01, 0xA0);
		I2C_ByteWrite(ADD_I2C, 0x02, 0x00);
	}
	else if (_inline[1] == '2') //'S2'
	{
		I2C_ByteWrite(ADD_I2C, 0x00, 0xF2); //Negative bias
		I2C_ByteWrite(ADD_I2C, 0x01, 0xA0);
		I2C_ByteWrite(ADD_I2C, 0x02, 0x00);
	}
}

void ReadXYZDataSequentially(void)
{
	if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
		I2C_M_Data(ADD_I2C, ADD_X_LSB, ReadHighResolution);
	// else if (StringCompare(sensor_type_name, "MMC5883MA", 9) == 1)
	// 	I2C_M_Data(ADD_I2C, ADD_X_LSB, ReadHighResolution);
	// else if (StringCompare(sensor_type_name, "HMC5983", 7) == 1)
	// 	I2C_M_Data(ADD_I2C, ADD_X_MSB, ReadHighResolution);

	putline("X", 1);
	OneByte[0] = I2cReadTrm2(ADD_I2C, ADD_X_MSB);
	putline(OneByte, 1);
	delay(10);
	OneByte[0] = I2cReadTrm2(ADD_I2C, ADD_X_LSB);
	putline(OneByte, 1);
	delay(10);
	putline("Y", 1);
	OneByte[0] = I2cReadTrm2(ADD_I2C, ADD_Y_MSB);
	putline(OneByte, 1);
	delay(10);
	OneByte[0] = I2cReadTrm2(ADD_I2C, ADD_Y_LSB);
	putline(OneByte, 1);
	delay(10);
	putline("Z", 1);
	OneByte[0] = I2cReadTrm2(ADD_I2C, ADD_Z_MSB);
	putline(OneByte, 1);
	delay(10);
	OneByte[0] = I2cReadTrm2(ADD_I2C, ADD_Z_LSB);
	putline(OneByte, 1);
	delay(10);
}

void RelayControl_1(unsigned char idata *_inline)
{
	switch (_inline[1])
	{
	case ('F'):
		switch (_inline[2])
		{
		case ('F'): //FFF
			RY1_OFF;
			RY2_OFF;
			RY3_OFF;
			putline("FFF", 3);
			EOL();
			putline("ACK", 3);
			EOL();
			putline("READY", 5);
			EOL();
			break;

		case ('N'): //FFN
			RY1_OFF;
			RY2_OFF;
			RY3_ON;
			putline("FFN", 3);
			EOL();
			putline("ACK", 3);
			EOL();
			putline("READY", 5);
			EOL();
			break;
		} //switch 3
		break;

	case ('N'):
		switch (_inline[2])
		{
		case ('F'): //FNF
			RY1_OFF;
			RY2_ON;
			RY3_OFF;
			putline("FNF", 3);
			EOL();
			putline("ACK", 3);
			EOL();
			putline("READY", 5);
			EOL();
			break;

		case ('N'): //FNN
			RY1_OFF;
			RY2_ON;
			RY3_ON;
			putline("FNN", 3);
			EOL();
			putline("ACK", 3);
			EOL();
			putline("READY", 5);
			EOL();
			break;
		} //switch 3
		break;
	} //switch 2
}

void RelayControl_2(unsigned char idata *_inline)
{
	switch (_inline[1])
	{
	case ('F'):
		switch (inline[2])
		{
		case ('F'): //NFF
			RY1_ON;
			RY2_OFF;
			RY3_OFF;
			putline("NFF", 3);
			EOL();
			putline("ACK", 3);
			EOL();
			putline("READY", 5);
			EOL();
			break;

		case ('N'): //NFN
			RY1_ON;
			RY2_OFF;
			RY3_ON;
			putline("NFN", 3);
			EOL();
			putline("ACK", 3);
			EOL();
			putline("READY", 5);
			EOL();
			break;
		} //switch 3
		break;

	case ('N'):
		switch (inline[2])
		{
		case ('F'): //NNF
			RY1_ON;
			RY2_ON;
			RY3_OFF;
			putline("NNF", 3);
			EOL();
			putline("ACK", 3);
			EOL();
			putline("READY", 5);
			EOL();
			break;

		case ('N'): //NNN
			RY1_ON;
			RY2_ON;
			RY3_ON;
			putline("NNN", 3);
			EOL();
			putline("ACK", 3);
			EOL();
			putline("READY", 5);
			EOL();
			break;
		} //switch 3
		break;
	} //switch 2
}

void SetReadingOutputMode(unsigned char idata *_inline)
{
	switch (_inline[1])
	{
	case ('H'): //MH >> read data in high resolution for MMC5883MA
		ReadHighResolution = 1;
		putline("MH", 2);
		EOL();
		break;

	case ('L'): //ML >> read data in low resolution for MMC5883MA
		ReadHighResolution = 0;
		putline("ML", 2);
		EOL();
		break;

	case ('R'): //MR >> counting mode
		Rx3 = _inline[2];
		MODE = 2;
		GyroMODE = 0;
		AccMODE = 0;
		break;

	case ('C'): //MC >> continue mode
		MODE = 3;
		GyroMODE = 0;
		AccMODE = 0;
		break;
	} //switch 2
}
void SetReadingAccOutputMode(unsigned char idata *_inline)
{
	switch (_inline[2])
	{
	case ('R'): //ACR >> counting mode
		Rx3 = _inline[3];
		MODE = 0;
		GyroMODE = 0;
		AccMODE = 2;
		break;

	case ('C'): //ACC >> continue mode
		MODE = 0;
		GyroMODE = 0;
		AccMODE = 3;
		break;
	} //switch 2
}
void SetReadingTempMode(void)
{
	char T_status = 0;
	if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
	{
		T_status = I2C_ByteRead(ACGY_ADD_I2C, ACGY_ADD_STATUS) & 0x04; //0b00000100 polling to check temperature ready
		while (T_status == 0)
		{
			T_status = I2C_ByteRead(ACGY_ADD_I2C, ACGY_ADD_STATUS) & 0x04;
		}

		I2C_T_Data(ACGY_ADD_I2C, ACGY_ADD_TMP_LSB); //read temperature data
	}
	else
	{
		I2C_T_Data(ADD_I2C, ADD_TMP_MSB);
	}
}

void SetReadingGyroOutputMode(unsigned char idata *_inline)
{
	switch (_inline[1])
	{
	case ('R'): //YR >> gyro counting mode
		Rx3 = _inline[2];
		MODE = 0;
		GyroMODE = 2;
		AccMODE = 0;
		break;

	case ('C'): //YC >> gyro continue mode
		MODE = 0;
		GyroMODE = 3;
		AccMODE = 0;
		break;
	} //switch 2
}

void ControlAlertLight(unsigned char idata *_inline)
{
	switch (_inline[2])
	{
	case ('F'): //A_F >> Fail
		MODE = 4;
		GyroMODE = 0;
		AccMODE = 0;
		//RY4_OFF;
		putline("ALF", 3);
		EOL();
		putline("ACK", 3);
		EOL();
		putline("READY", 5);
		EOL();
		break;

	case ('P'): //A_P >> Pass
		MODE = 1;
		GyroMODE = 0;
		AccMODE = 0;
		putline("ALP", 3);
		EOL();
		putline("ACK", 3);
		EOL();
		putline("READY", 5);
		EOL();
		break;

	case ('T'): //A_T >> Test, Turn on the green alert lamp
		MODE = 5;
		GyroMODE = 0;
		AccMODE = 0;
		//RY4_ON;
		putline("ALT", 3);
		EOL();
		putline("ACK", 3);
		EOL();
		putline("READY", 5);
		EOL();
		break;
	} //switch 2
}

void ReadSensorID(unsigned char idata *_inline)
{
	//unsigned char* _data[3];
	unsigned char ID_Byte;

	if (_inline[1] == 'D')
	{

		//I2CReadMultiBytes(ADD_I2C, ADD_ID, *_data, 3);
		ID_Byte = I2C_ByteRead(ADD_I2C, ADD_ID);
		putline("ID:", 3);
		putchar(ID_Byte);
		EOL();
	}
}

/*
void DetectConnectedSensor(unsigned char xdata *_inline)
{
	char _index;
	
	if (_inline[1]=='S')
	{
		_index = CheckSensorCorrect();
		*sensor_type_name = sensor_type[_index].type;
		SetI2CParameters(sensor_type_name);
		putline(sensor_type[_index].type, sizeof(sensor_type[_index].type)); EOL();
	}
}
*/

// char CheckSensorCorrect(void)
// {
// 	// This function is used to find the correct type of sensor connected with 8051 board.
// 	// If there is a sensor defined in the sensor_type struct table found out,
// 	// it will return the index of the sensor in sensor_type variable.
// 	// If not, return 255, the size of char.

// 	short len_sensor_type = sizeof(sensor_type) / sizeof(sensor_type[0]);
// 	char _i;
// 	unsigned char _data[3];
// 	unsigned char _data2[3];
// 	unsigned char ID_Byte;
// 	unsigned char ID2_Byte;

// 	for (_i = 0; _i < len_sensor_type; _i++)
// 	{
// 		if (sensor_type[_i].Num_Registers > 1)
// 		{
// 			I2CReadMultiBytes(sensor_type[_i].Device_Address, sensor_type[_i].ID_Address, &_data, sensor_type[_i].Num_Registers);
// 			I2CReadMultiBytes(sensor_type[_i + 1].Device_Address, sensor_type[_i + 1].ID_Address, &_data2, sensor_type[_i + 1].Num_Registers);
// 			if (CheckArrayEqual(_data, &sensor_type[_i].ID))
// 			{
// 				if (CheckArrayEqual(_data2, &sensor_type[_i + 1].ID)) //inorder to check the LSM9DS1 mag(i) && acc/gyro(i+1) sensor
// 					return _i + 2;									  //return LSM9DS1 (i+2)
// 				else
// 					return _i;
// 			}
// 		}
// 		else
// 		{
// 			ID_Byte = I2C_ByteRead(sensor_type[_i].Device_Address, sensor_type[_i].ID_Address);
// 			ID2_Byte = I2C_ByteRead(sensor_type[_i + 1].Device_Address, sensor_type[_i + 1].ID_Address);
// 			if (ID_Byte == sensor_type[_i].ID[0])
// 			{
// 				if (ID2_Byte == sensor_type[_i + 1].ID[0]) //inorder to check the LSM9DS1 mag(i) && acc/gyro(i+1) sensor
// 				{
// 					return _i + 2; //return LSM9DS1 (i+2)
// 				}
// 				else
// 				{
// 					return _i;
// 				}
// 			}
// 		}
// 	}
// 	return 0xFF;
// }

bit CheckArrayEqual(unsigned char idata *_data, char idata **sensor_ID)
{
	char len_data = sizeof(_data);
	char _i;

	for (_i = 0; _i < len_data; _i++)
	{
		if (_data[_i] != *(sensor_ID + _i))
			return 0;
	}
	return 1;
}

bit StringCompare(char idata **strA, char idata strB[], char num_char)
{

	char _i;
	char len_str = sizeof(strB);

	if (num_char > len_str)
		num_char = len_str;

	for (_i = 0; _i < num_char; _i++)
	{
		char *_c = (strA + _i);
		if (*_c != strB[_i])
			return 0;
	}

	return 1;
}

void Initial_register(void)
{
	if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
	{
		I2C_ByteWrite(ACGY_ADD_I2C, ACGY_CTRL_REG8, 0x05); //SW_RESET on

		I2C_ByteWrite(ADD_I2C, CTRL_REG1_M, 0x10); //0b00010000 >> ODR=10Hz
		I2C_ByteWrite(ADD_I2C, CTRL_REG2_M, 0x20); //0b00100000 >> +-8 gauss, bit5 → SOFT_RST
		I2C_ByteWrite(ADD_I2C, CTRL_REG4_M, 0x00); //0b00000000 >> Z Low-power mode
		I2C_ByteWrite(ADD_I2C, CTRL_REG5_M, 0x00); //0b00000000 >> data continue update

		I2C_ByteWrite(ACGY_ADD_I2C, ACGY_CTRL_REG1_G, 0xC0);  //0b11000000>> GYRO ODR:952Hz,+-245dps,Cutoff:33Hz
		I2C_ByteWrite(ACGY_ADD_I2C, ACGY_ORIENT_CFG_G, 0x00); //0b00000000 >> GYO sign postive
		I2C_ByteWrite(ACGY_ADD_I2C, ACGY_CTRL_REG4, 0x38);	  //0b00111000 >> Gyro output enable
		I2C_ByteWrite(ACGY_ADD_I2C, ACGY_FIFO_CTRL, 0xC0);	  //0b1100 0000>> FIFO mode >> continuos mode
		I2C_ByteWrite(ACGY_ADD_I2C, ACGY_CTRL_REG5, 0x38);	  //ACC XYZ enable
		I2C_ByteWrite(ACGY_ADD_I2C, ACGY_CTRL_REG6, 0xC0);	  //ACC +-2g, ODR=952Hz, BW=408Hz
		I2C_ByteWrite(ACGY_ADD_I2C, ACGY_CTRL_REG9, 0x1A);	  //0b00011010 >> Gyro Temp FIFO enable
		putline("LSM", 3);
		EOL();
	}
	// else if (StringCompare(sensor_type_name, "MMC5883MA", 9) == 1)
	// {
	// 	// SET the sensor momentarily to restore the sensor characteristic.
	// 	I2C_ByteWrite(ADD_I2C, ADD_CONFIG_0, 0x08); //Set on 0000 1000
	// 	delay(10000);
	// 	I2C_ByteWrite(ADD_I2C, ADD_CONFIG_0, 0x00);
	// 	putline("MMC", 3);
	// 	EOL();
	// }
	// else if (StringCompare(sensor_type_name, "HMC5983", 7) == 1)
	// {
	// 	putline("HMC", 3);
	// 	EOL();
	// }
}

void EOL(void)
{
	putline(Ending, 2);
}
