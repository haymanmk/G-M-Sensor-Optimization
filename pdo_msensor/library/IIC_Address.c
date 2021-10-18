#include <AT89X52.h>
#include <INTRINS.H>

#include "IIC_Address.h"
#include "Ext_function.h"

//char* sensor_type_name;

void SetI2CParameters(void)
{
	
	
	if (StringCompare(sensor_type_name, "LSM9DS1", 7) == 1)
	{
		ADD_I2C = 0x38; 	//Magnet I2C control address 0x38 0011 1000 (Deven)
		//ADD_I2C = 0x3C;		//BlueBoard
		ADD_X_LSB = 0x28;		 //mag Xout LSB
		ADD_X_MSB = 0x29;		 //mag Xout MSB
		ADD_Y_LSB = 0x2A;		 //mag Yout LSB
		ADD_Y_MSB = 0x2B;		 //mag Yout MSB
		ADD_Z_LSB = 0x2C;		 //mag Zout LSB
		ADD_Z_MSB = 0x2D;		 //mag Zout MSB
		ADD_STATUS = 0x27;		 //mag Device status
		CTRL_REG1_M = 0x20; //Control register 1 >> ODR select
		CTRL_REG2_M = 0x21; //Control register 2 >> gauss full-scale &  reset Configuration registers and user register
		CTRL_REG3_M = 0x22; //Control register 3 >> continue/single mode
		CTRL_REG4_M = 0x23; //Control register 4 >> mag Z operation mode 
		CTRL_REG5_M = 0x24; //Control register 5 >> block data updata
		ADD_ID = 0x0F;			 //Product ID 0b0011 1101 >> 0x3D

		ACGY_ADD_I2C = 0xD4; 			//Acc & Gyro I2C control address 0xD4 1101 0100 (Deven)
		//ACGY_ADD_I2C = 0xD6; //Blue Board
		ACGY_GYR_ADD_X_LSB = 0x18;	 //gyro Xout LSB
		ACGY_GYR_ADD_X_MSB = 0x19;	 //gyro Xout MSB
		ACGY_GYR_ADD_Y_LSB = 0x1A;	 //gyro Yout LSB
		ACGY_GYR_ADD_Y_MSB = 0x1B;	 //gyro Yout MSB
		ACGY_GYR_ADD_Z_LSB = 0x1C;	 //gyro Zout LSB
		ACGY_GYR_ADD_Z_MSB = 0x1D;	 //gyro Zout MSB
		ACGY_ADD_STATUS = 0x17;	 	 //Device status>> bit5:Temp, bit6:Gyro, bit7:Acc
		ACGY_ADD_TMP_LSB = 0x15;	//Temperature Output LSB Register  Read
		ACGY_ADD_TMP_MSB = 0x16;	//Temperature Output MSB Register  Read
		ACGY_CTRL_REG1_G = 0x10;	 //Control Register1 >> Gyro full-scale, ODR, BW 
		ACGY_ORIENT_CFG_G = 0x13; 	//Gyro XYZ sign 
		ACGY_CTRL_REG4 = 0x1E; //Control Register4 >> GYRO XYZ enable
		ACGY_CTRL_REG5 = 0x1F; //Control Register5 >> ACC XYZ enable
		ACGY_CTRL_REG6 = 0x20; //Control Register6 >> ACC full-scale, ODR, BW
		ACGY_CTRL_REG7 = 0x21; //Control Register7 >>ACC high resolution mode
		ACGY_CTRL_REG8 = 0x22;	//Control Register8 >> Softreset → claer by hardware
		ACGY_CTRL_REG9 = 0x23;  //Control Register9 >> FIFO enable
		ACGY_FIFO_CTRL = 0x2E;	//FIFO mode select
		ACGY_ACC_ADD_X_LSB = 0x28; //Acc Xout LSB
		ACGY_ACC_ADD_X_MSB = 0x29; //Acc Xout MSB
		ACGY_ACC_ADD_Y_LSB = 0x2A; //Acc Yout LSB
		ACGY_ACC_ADD_Y_MSB = 0x2B; //Acc Yout MSB
		ACGY_ACC_ADD_Z_LSB = 0x2C; //Acc Zout LSB
		ACGY_ACC_ADD_Z_MSB = 0x2D; //Acc Zout MSB
		ACGY_ADD_ID = 0x0F;	  //Product ID 0b01101000 >> 0x68
		
	}
	/*
	if (StringCompare(sensor_type_name, "MMC5883MA", 9) == 1)
	{
		ADD_I2C = 0x60;		 //I2C control address 0x60 0110 0000
		ADD_X_LSB = 0x00;	 //Xout LSB
		ADD_X_MSB = 0x01;	 //Xout MSB
		ADD_Y_LSB = 0x02;	 //Yout LSB
		ADD_Y_MSB = 0x03;	 //Yout MSB
		ADD_Z_LSB = 0x04;	 //Zout LSB
		ADD_Z_MSB = 0x05;	 //Zout MSB
		ADD_TMP_MSB = 0x06;	 //Temperature output
		ADD_STATUS = 0x07;	 //Device status
		ADD_CONFIG_0 = 0x08; //Control register 0
		ADD_CONFIG_1 = 0x09; //Control register 1
		ADD_CONFIG_2 = 0x0A; //Control register 2
		//ADD_X_THR = 0x0B;  //Motion detection threshold of X
		//ADD_Y_THR = 0x0C;  //Motion detection threshold of Y
		//ADD_Z_THR = 0x0D;  //Motion detection threshold of Z
		ADD_ID = 0x2F; //Product ID 0b0010 1111
	}
	*/
	// else if (StringCompare(sensor_type_name, "HMC5983", 7) == 1)
	// {
	// 	ADD_I2C = 0x3C;		 //I2C control address
	// 	ADD_CONFIG_A = 0x00; //Configuration Register A   Read/Write
	// 	ADD_CONFIG_B = 0x01; //Configuration Register B   Read/Write
	// 	ADD_MODE = 0x02;	 //Mode Register  Read/Write
	// 	ADD_X_MSB = 0x03;	 //Data Output X MSB Register  Read
	// 	ADD_X_LSB = 0x04;	 //Data Output X LSB Register  Read
	// 	ADD_Z_MSB = 0x05;	 //Data Output Z MSB Register  Read
	// 	ADD_Z_LSB = 0x06;	 //Data Output Z LSB Register  Read
	// 	ADD_Y_MSB = 0x07;	 //Data Output Y MSB Register  Read
	// 	ADD_Y_LSB = 0x08;	 //Data Output Y LSB Register  Read
	// 	ADD_STATUS = 0x09;	 //Status Register  Read
	// 	ADD_ID_A = 0x0A;	 //Identification Register A  Read
	// 	ADD_ID_B = 0x0B;	 //Identification Register B  Read
	// 	ADD_ID_C = 0x0C;	 //Identification Register C  Read
	// 	ADD_TMP_MSB = 0x31;	 //Temperature Output MSB Register  Read
	// 	ADD_TMP_LSB = 0x32;	 //Temperature Output LSB Register  Read
	// }
}