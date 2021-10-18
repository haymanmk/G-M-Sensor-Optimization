/************************************************/
/*    General Functions for IIC operation       */
/*								                */
/*									            */
/************************************************/
#include <AT89X52.h>
#include <INTRINS.H> /* special function register 8052       */

#include "MCU_GPIO.h"
#include "EXT_FUNCTION.H"

//#include "IIC_Address.h"

// Function protocol definition //
void SignInvertor2SComplement(char axis);

//extern unsigned char M_Data[9];
extern unsigned char T_Data[3];
extern unsigned char X_Data[2];
extern unsigned char Y_Data[2];
extern unsigned char Z_Data[2];
extern unsigned char code Ending[2]; /* Ending char CR, LF */

extern unsigned char GyroX_Data[2];
extern unsigned char GyroY_Data[2];
extern unsigned char GyroZ_Data[2];

extern unsigned char AccX_Data[2];
extern unsigned char AccY_Data[2];
extern unsigned char AccZ_Data[2];

unsigned char ID[3];

short R1[3] = {100, 200, 300};
short R2[3] = {400, 500, 600};
short R3[3] = {700, 800, 900};
short R4[3] = {100, 200, 300};

//char* sensor_type_name;

//*************************************************************************************
//  I2C_Wate: delay time only
//*************************************************************************************
void I2C_Wait(void)
{
	int _i;
	for (_i = 0; _i < 2; _i++)
	{
		_nop_();
	}
}

//*************************************************************************************
//	I2C_Init: Initial status of IIC Bus
//*************************************************************************************
void I2C_Init(void)
{
	ISDA = 1;
	ISCL = 1;
}

void I2C_Init_EEPROM(void)
{
	ISDA2 = 1;
	ISCL2 = 1;
}

//*************************************************************************************
//
//	I2C_Star:  ISCL =0 & ISDA =0 (Star condition by Master)
//
//*************************************************************************************
void I2C_Start(void)
{
	ISDA = 1;
	I2C_Wait(); //modified by hayman
	ISCL = 1;
	I2C_Wait();
	ISDA = 0;
	I2C_Wait();
	ISCL = 0;
}

void I2C_Start_EEPROM(void)
{
	ISDA2 = 1;
	I2C_Wait();
	ISCL2 = 1;
	I2C_Wait();
	ISDA2 = 0;
	I2C_Wait();
	ISCL2 = 0;
}
//*************************************************************************************
//
//	I2C_Stop:  ISCL =1 & ISDA =1 (Stop condition by Master)
//
//*************************************************************************************
void I2C_Stop(void)
{
	ISDA = 0;
	I2C_Wait();
	ISCL = 1;
	I2C_Wait();
	ISDA = 1;
}
void I2C_Stop_EEPROM(void)
{
	ISDA2 = 0;
	I2C_Wait();
	ISCL2 = 1;
	I2C_Wait();
	ISDA2 = 1;
}
//*************************************************************************************
//
// I2C_SentByte: clock out by data to Slave (Only operate ISCL H/L and read ISDA)
//				 return ack (make sure Slave receiving correctly)
//
//*************************************************************************************
bit I2C_SentByte(unsigned char bytedata)
{
	unsigned char i;
	bit ack;
	for (i = 0; i < 8; i++)
	{
		bytedata = _crol_(bytedata, 1);
		if (bytedata & 0x01)
		{
			ISDA = 1;
		}
		else
		{
			ISDA = 0;
		}

		ISCL = 1;
		I2C_Wait();
		ISCL = 0;
		I2C_Wait();
	}

	//ISDA=1; modified by hayman
	I2C_Wait();
	ISCL = 1;
	I2C_Wait();
	ack = ISDA;

	ISCL = 0;

	I2C_Wait();
	return ack;
}

bit I2C_SentByte_EEPROM(unsigned char bytedata)
{
	unsigned char i;
	bit ack;
	for (i = 0; i < 8; i++)
	{

		bytedata = _crol_(bytedata, 1);
		if (bytedata & 0x01)
		{
			ISDA2 = 1;
		}
		else
		{
			ISDA2 = 0;
		}

		ISCL2 = 1;
		I2C_Wait();
		ISCL2 = 0;
		I2C_Wait();
	}
	ISCL2 = 1;
	I2C_Wait();
	ack = ISDA2;
	ISCL2 = 0;
	_nop_();
	return ack;
}
//*************************************************************************************
//
// I2C_ReceiveByte: clock out byte data from Slave (Only operate ISCL H/L and read ISDA)
//
//
//*************************************************************************************
unsigned char I2C_ReceiveByte(void)
{
	unsigned char i;
	unsigned char bytedata = 0x00;
	// Receive byte (MSB first)

	for (i = 0; i < 8; i++)
	{
		ISCL = 1;
		I2C_Wait();

		bytedata <<= 1;
		if (ISDA)
			bytedata |= 0x01;

		ISCL = 0;
		I2C_Wait();
	}
	return bytedata;
}

unsigned char I2C_ReceiveByte_EEPROM(void)
{
	unsigned char i;
	unsigned char bytedata = 0x00;
	// Receive byte (MSB first)

	for (i = 0; i < 8; i++)
	{
		ISCL2 = 1;
		I2C_Wait();

		bytedata <<= 1;
		if (ISDA2)
			bytedata |= 0x01;

		ISCL2 = 0;
		I2C_Wait();
	}
	return bytedata;
}
//*************************************************************************************
//
// SendAcknowledge: "NACK"  Master --> Slave (ack==1) for ending
//
//*************************************************************************************
void SendAcknowledge(bit ack)
{
	ISDA = ack;
	ISCL = 1;
	I2C_Wait();
	ISCL = 0;
	ISDA = 1; //Recover ISDA LINE
}
void SendAcknowledge_EEPROM(bit ack)
{
	ISDA2 = ack;
	ISCL2 = 1;
	I2C_Wait();
	ISCL2 = 0;
	ISDA2 = 1; //Recover ISDA LINE
}
//*************************************************************************************
//
// I2C_ByteWrite: Write One Register to a Device
// I2C_Start--> SentByte(device)-->SentByte(address)--> SentByte(Data)--> Stop
//
//*************************************************************************************
void I2C_ByteWrite(unsigned char device, unsigned char address, unsigned char bytedata)
{
	unsigned char i;
	bit ack;

	for (i = 0; i < 10; i++) //time out,retry=10
	{
		I2C_Start();

		ack = I2C_SentByte(device);
		if (ack == 1) //FAIL then STOP
		{
			I2C_Stop();
			continue;
		}

		ack = I2C_SentByte(address);
		if (ack == 1) //FAIL then STOP
		{
			I2C_Stop();
			continue;
		}

		ack = I2C_SentByte(bytedata);
		if (ack == 1) //FAIL then STOP
		{
			I2C_Stop();
			continue;
		}

		I2C_Stop();

		if (ack == 0) //ACK PASS  Go Out
		{
			break;
		}
	}
	delay(10);

	I2C_Start();
	I2C_SentByte(device);
	I2C_SentByte(address);
	I2C_SentByte(bytedata);
	I2C_Stop();
}

//*************************************************************************************
//
//	I2C_ByteRead: Read "One" Register from Slave
//	  I2C_Start --> I2C_SentByte(device) --> I2C_SentByte(address)
//--> I2C_Start --> I2C_SentByte(device|0x01, read)--> I2C_ReceiveByte --> NACK --> Stop
//
//*************************************************************************************
unsigned char I2C_ByteRead(unsigned char device, unsigned char address)
{
	unsigned char bytedata = 0x00;

	I2C_Start();

	I2C_SentByte(device);
	I2C_SentByte(address);
	I2C_Start();
	I2C_SentByte(device | 0x01); //Write-->Read
	bytedata = I2C_ReceiveByte();
	SendAcknowledge(1);

	I2C_Stop();

	return bytedata;
}

//*************************************************************************************
//
//	I2C_ByteRead: Read "One" Register from Slave
//	  I2C_Start --> I2C_SentByte(device) --> I2C_SentByte(address)
//--> I2C_Start --> I2C_SentByte(device|0x01, read)--> I2C_ReceiveByte --> NACK --> Stop
//
//*************************************************************************************

void I2C_M_Data(unsigned char device, unsigned char address, bit ReadHighResolution)
{
	unsigned char bytedata = 0x00;
	//unsigned char TempByte[1];
	//char* TransformedResult[2];
	bit ack;

	I2C_Start();
	I2C_SentByte(device);
	I2C_SentByte(address);

	I2C_Start();
	ack = I2C_SentByte(device | 0x01); //Write-->Read

	/*
	if (StringCompare(sensor_type_name, "HMC5983", 7) == 1)
	{
		X_Data[0] = I2C_ReceiveByte();
		SendAcknowledge(0);
		X_Data[1] = I2C_ReceiveByte();
		SendAcknowledge(0);

		Z_Data[0] = I2C_ReceiveByte(); // DATA SHEET INDICATE THIS IS Z, Please note it is for  M_Data[7] not [5]
		SendAcknowledge(0);
		Z_Data[1] = I2C_ReceiveByte(); // DATA SHEET INDICATE THIS IS Z
		SendAcknowledge(0);

		Y_Data[0] = I2C_ReceiveByte(); // DATA SHEET INDICATE THIS IS Y
		SendAcknowledge(0);
		Y_Data[1] = I2C_ReceiveByte(); // DATA SHEET INDICATE THIS IS Y
		SendAcknowledge(1);

		//render sign inverse at X and Z data
		SignInvertor2SComplement(3);
	}
	
	else if (StringCompare(sensor_type_name, "MMC5883MA", 9) == 1)	
	{
		X_Data[1] = I2C_ReceiveByte(); //0xcb; //LSB
		SendAcknowledge(0);
		X_Data[0] = I2C_ReceiveByte(); //0x7c; //MSB
		SendAcknowledge(0);

		Y_Data[1] = I2C_ReceiveByte();
		SendAcknowledge(0);
		Y_Data[0] = I2C_ReceiveByte();
		SendAcknowledge(0);

		Z_Data[1] = I2C_ReceiveByte();
		SendAcknowledge(0);
		Z_Data[0] = I2C_ReceiveByte();
		SendAcknowledge(1);

		if (!ReadHighResolution)
		{
			//lower the resolution of M-sensor to be in compliance with the old version of software.
			LowerResolution2MatchOldSensor(&X_Data);
			LowerResolution2MatchOldSensor(&Y_Data);
			LowerResolution2MatchOldSensor(&Z_Data);
			//render sign inverse at X and Z data
			SignInvertor2SComplement(1);
		}
	}
	*/
	if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
	{
		X_Data[1] = I2C_ReceiveByte(); //0xcb; //LSB
		SendAcknowledge(0);
		X_Data[0] = I2C_ReceiveByte(); //0x7c; //MSB
		SendAcknowledge(0);

		Y_Data[1] = I2C_ReceiveByte();
		SendAcknowledge(0);
		Y_Data[0] = I2C_ReceiveByte();
		SendAcknowledge(0);

		Z_Data[1] = I2C_ReceiveByte();
		SendAcknowledge(0);
		Z_Data[0] = I2C_ReceiveByte();
		SendAcknowledge(1); //ending

		if (!ReadHighResolution)
		{
			//lower the resolution of M-sensor to be in compliance with the old version of software.
			LowerResolution2MatchOldSensor(&X_Data);
			LowerResolution2MatchOldSensor(&Y_Data);
			LowerResolution2MatchOldSensor(&Z_Data);
			//render sign inverse at X and Z data
			SignInvertor2SComplement(3);
		}
	}

	I2C_Stop();

	//putline(M_Data,9);
	putline("X", 1);
	putline(X_Data, 2);
	putline("Y", 1);
	putline(Y_Data, 2);
	putline("Z", 1);
	putline(Z_Data, 2);
}

//*************************************************************************************
//
//	I2C_ByteRead: Read "One" Register from Slave
//	  I2C_Start --> I2C_SentByte(device) --> I2C_SentByte(address)
//--> I2C_Start --> I2C_SentByte(device|0x01, read)--> I2C_ReceiveByte --> NACK --> Stop
//
//*************************************************************************************
void I2C_T_Data(unsigned char device, unsigned char address)
{

	unsigned char bytedata = 0x00;

	I2C_Start();
	I2C_SentByte(device);
	I2C_SentByte(address);

	I2C_Start();
	I2C_SentByte(device | 0x01); //Write-->Read

	/*
	if (StringCompare(sensor_type_name, "HMC5983", 7) == 1)
	{
		T_Data[1] = I2C_ReceiveByte();
		SendAcknowledge(0);
		T_Data[2] = I2C_ReceiveByte();
		SendAcknowledge(1);
	}
	else if (StringCompare(sensor_type_name, "MMC5883MA", 9) == 1)
	{
		T_Data[1] = 0x00;
		T_Data[2] = I2C_ReceiveByte();
		SendAcknowledge(1);
	}
	*/
	if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
	{
		T_Data[2] = I2C_ReceiveByte(); //LSB
		SendAcknowledge(0);
		T_Data[1] = I2C_ReceiveByte(); //MSB
		SendAcknowledge(1);
	}

	I2C_Stop();

	putline(T_Data, 3);
	putline(Ending, 2);
}
//*************************************************************************************
//
//	I2C_ByteRead: Read "One" Register from Slave
//	  I2C_Start --> I2C_SentByte(device) --> I2C_SentByte(address)
//--> I2C_Start --> I2C_SentByte(device|0x01, read)--> I2C_ReceiveByte --> NACK --> Stop
//
//*************************************************************************************

void I2C_Gyro_Data(unsigned char device, unsigned char address)
{
	unsigned char bytedata = 0x00;
	//unsigned char TempByte[1];
	//char* TransformedResult[2];
	bit ack;

	I2C_Start();
	I2C_SentByte(device);
	I2C_SentByte(address);

	I2C_Start();
	ack = I2C_SentByte(device | 0x01); //Write-->Read

	if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
	{
		GyroX_Data[1] = I2C_ReceiveByte(); //LSB
		SendAcknowledge(0);
		GyroX_Data[0] = I2C_ReceiveByte(); //MSB
		SendAcknowledge(0);

		GyroY_Data[1] = I2C_ReceiveByte();
		SendAcknowledge(0);
		GyroY_Data[0] = I2C_ReceiveByte();
		SendAcknowledge(0);

		GyroZ_Data[1] = I2C_ReceiveByte();
		SendAcknowledge(0);
		GyroZ_Data[0] = I2C_ReceiveByte();
		SendAcknowledge(1); //ending

		//SignInvertor2SComplement(1);
	}

	I2C_Stop();

	//putline(M_Data,9);
	putline("X", 1);
	putline(GyroX_Data, 2);
	putline("Y", 1);
	putline(GyroY_Data, 2);
	putline("Z", 1);
	putline(GyroZ_Data, 2);
}
//*************************************************************************************
//
//	I2C_ByteRead: Read "One" Register from Slave
//	  I2C_Start --> I2C_SentByte(device) --> I2C_SentByte(address)
//--> I2C_Start --> I2C_SentByte(device|0x01, read)--> I2C_ReceiveByte --> NACK --> Stop
//
//*************************************************************************************

void I2C_Acc_Data(unsigned char device, unsigned char address)
{
	unsigned char bytedata = 0x00;
	//unsigned char TempByte[1];
	//char* TransformedResult[2];
	bit ack;

	I2C_Start();
	I2C_SentByte(device);
	I2C_SentByte(address);

	I2C_Start();
	ack = I2C_SentByte(device | 0x01); //Write-->Read

	if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
	{
		AccX_Data[1] = I2C_ReceiveByte(); //LSB
		SendAcknowledge(0);
		AccX_Data[0] = I2C_ReceiveByte(); //MSB
		SendAcknowledge(0);

		AccY_Data[1] = I2C_ReceiveByte();
		SendAcknowledge(0);
		AccY_Data[0] = I2C_ReceiveByte();
		SendAcknowledge(0);

		AccZ_Data[1] = I2C_ReceiveByte();
		SendAcknowledge(0);
		AccZ_Data[0] = I2C_ReceiveByte();
		SendAcknowledge(1); //ending

		SignInvertor2SComplement(2);
	}

	I2C_Stop();

	//putline(M_Data,9);
	putline("X", 1);
	putline(AccX_Data, 2);
	putline("Y", 1);
	putline(AccY_Data, 2);
	putline("Z", 1);
	putline(AccZ_Data, 2);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
unsigned char I2cReadTrm2(signed char device,unsigned char address)	//KEY FUNCTION in MAIN
{
    unsigned char bytedata=0x00;

    I2C_Start( );
    I2C_SentByte(device);
    I2C_SentByte(address);
    I2C_Start( );
    I2C_SentByte(device|0x01);
    bytedata=I2C_ReceiveByte( );
    SendAcknowledge(1);
    I2C_Stop( );
    return bytedata;  
}
*/
void I2CReadMultiBytes(signed char device, unsigned char address, unsigned char *_data, char amount)
{
	char _i;

	for (_i = 0; _i < amount; _i++)
	{
		_data[_i] = I2C_ByteRead(device, address + _i);
	}
}

unsigned short TwoBytes2Int(unsigned char HighByte, unsigned char LowByte)
{
	return (LowByte | (unsigned int)HighByte << 8);
}

void LowerResolution2MatchOldSensor(unsigned char *DataArray)
{

	unsigned short TwoBytesInt = TwoBytes2Int(DataArray[0], DataArray[1]);
	short SignedTwoBytesInt;
	float TransFactor;
	short TransformedSignedTwoBytesInt;

	/*
	if (StringCompare(sensor_type_name, "MMC5883MA", 9) == 1)
	{
		SignedTwoBytesInt = TwoBytesInt - 32768;	//origin LSB range 0 ~ 65535 >>-32768~32767
		TransFactor =  1370.0 / 4096.0;				//MMC resolution:4096LSB/gauss
	}
	*/
	if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
	{
		SignedTwoBytesInt = TwoBytesInt; //origin LSB range -32768 ~ 32767
		TransFactor = 1370.0 / 3448.0;	 //LSM resolution: 0.29 mgauss/LSB >> 3448 LSB/gauss
	}

	TransformedSignedTwoBytesInt = (short)((float)SignedTwoBytesInt * TransFactor);
	DataArray[1] = (char)(TransformedSignedTwoBytesInt);	  //Low byte
	DataArray[0] = (char)(TransformedSignedTwoBytesInt >> 8); //High byte
}

void CalibrationCaculate(unsigned char *Xorigin, unsigned char *Yorigin, unsigned char *Zorigin,
						 unsigned char *Xcal, unsigned char *Ycal, unsigned char *Zcal)
{


	// unsigned short TwoBytesIntX = TwoBytes2Int(Xorigin[0], Xorigin[1]);
	// unsigned short TwoBytesIntY = TwoBytes2Int(Yorigin[0], Yorigin[1]);
	// unsigned short TwoBytesIntZ = TwoBytes2Int(Zorigin[0], Zorigin[1]);

	short SignedTwoBytesIntX, SignedTwoBytesIntY, SignedTwoBytesIntZ;
	short TransformedSignedTwoBytesIntX, TransformedSignedTwoBytesIntY, TransformedSignedTwoBytesIntZ;

	SignedTwoBytesIntX = (short)TwoBytes2Int(Xorigin[0], Xorigin[1]);
	SignedTwoBytesIntY = (short)TwoBytes2Int(Yorigin[0], Yorigin[1]);
	SignedTwoBytesIntZ = (short)TwoBytes2Int(Zorigin[0], Zorigin[1]);

	TransformedSignedTwoBytesIntX = (SignedTwoBytesIntX * R1[0] +
											SignedTwoBytesIntY * R2[0] + SignedTwoBytesIntZ * R3[0] + R4[0])/100;

	TransformedSignedTwoBytesIntY = (SignedTwoBytesIntX * R1[1] +
											SignedTwoBytesIntY * R2[1] + SignedTwoBytesIntZ * R3[1] + R4[1])/100;

	TransformedSignedTwoBytesIntZ = (SignedTwoBytesIntX * R1[2] +
											SignedTwoBytesIntY * R2[2] + SignedTwoBytesIntZ * R3[2] + R4[2])/100;

	Xcal[1] = (char)(TransformedSignedTwoBytesIntX);	  //Low byte
	Xcal[0] = (char)(TransformedSignedTwoBytesIntX >> 8); //High byte
	Ycal[1] = (char)(TransformedSignedTwoBytesIntY);	  //Low byte
	Ycal[0] = (char)(TransformedSignedTwoBytesIntY >> 8); //High byte
	Zcal[1] = (char)(TransformedSignedTwoBytesIntZ);	  //Low byte
	Zcal[0] = (char)(TransformedSignedTwoBytesIntZ >> 8); //High byte
}

void SignInvertor2SComplement(char axis)
{
	// Argument 'axis' specifies which axis is going to be inverted the +/- sign,
	// where axis=1 for X, axis=2 for Z, and axis=3 for both.
	unsigned char carry;
	//------------------------------------------Sign Invert
	if (axis == 1 || axis == 3)
	{
		if (X_Data[1] == 0x00) //LSB
		{
			carry = 0x01;
		}
		else
		{
			X_Data[1] = ~X_Data[1] + 0x01;
			carry = 0x00;
		}
		X_Data[0] = ~X_Data[0] + carry;
	}
	//------------------------------------------Sign Invert
	if (axis == 2 || axis == 3)
	{
		if (Z_Data[1] == 0x00) //LSB
		{
			carry = 0x01;
		}
		else
		{
			Z_Data[1] = ~Z_Data[1] + 0x01;
			carry = 0x00;
		}

		Z_Data[0] = ~Z_Data[0] + carry;
	}
}

// where axis=1 for X, axis=2 for Z, and axis=3 for Y.
/*
float Gauss_Data(unsigned char *DataArray)
{
	short MinData = -10960;
	float resolu = 1 / 1370; // >> gauss/LSB

	unsigned short GaussTwoBytesInt = TwoBytes2Int(DataArray[0], DataArray[1]);
	short GaussSignedTwoBytesInt = GaussTwoBytesInt;
	float gauss = -8 + (GaussSignedTwoBytesInt - MinData) * resolu;

	return gauss;
}
*/