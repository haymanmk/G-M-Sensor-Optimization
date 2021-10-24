/***************************
 *
 * This header file defines the address at M-Sensor for I2C communication.
 *
 ***************************/
#ifndef IIC_ADDRESS_H
#define IIC_ADDRESS_H

#define ADD_I2C 0x38 //Magnet I2C control address 0x38 0011 1000 (Deven)
//ADD_I2C = 0x3C;		//BlueBoard
#define ADD_X_LSB 0x28	 //mag Xout LSB
#define ADD_X_MSB 0x29	 //mag Xout MSB
#define ADD_Y_LSB 0x2A	 //mag Yout LSB
#define ADD_Y_MSB 0x2B	 //mag Yout MSB
#define ADD_Z_LSB 0x2C	 //mag Zout LSB
#define ADD_Z_MSB 0x2D	 //mag Zout MSB
#define ADD_STATUS 0x27	 //mag Device status
#define CTRL_REG1_M 0x20 //Control register 1 >> ODR select
#define CTRL_REG2_M 0x21 //Control register 2 >> gauss full-scale &  reset Configuration registers and user register
#define CTRL_REG3_M 0x22 //Control register 3 >> continue/single mode
#define CTRL_REG4_M 0x23 //Control register 4 >> mag Z operation mode
#define CTRL_REG5_M 0x24 //Control register 5 >> block data updata
#define ADD_ID 0x0F		 //Product ID 0b0011 1101 >> 0x3D

#define ACGY_ADD_I2C 0xD4 //Acc & Gyro I2C control address 0xD4 1101 0100 (Deven)
//ACGY_ADD_I2C = 0xD6; //Blue Board
#define ACGY_GYR_ADD_X_LSB 0x18 //gyro Xout LSB
#define ACGY_GYR_ADD_X_MSB 0x19 //gyro Xout MSB
#define ACGY_GYR_ADD_Y_LSB 0x1A //gyro Yout LSB
#define ACGY_GYR_ADD_Y_MSB 0x1B //gyro Yout MSB
#define ACGY_GYR_ADD_Z_LSB 0x1C //gyro Zout LSB
#define ACGY_GYR_ADD_Z_MSB 0x1D //gyro Zout MSB
#define ACGY_ADD_STATUS 0x17	//Device status>> bit5:Temp, bit6:Gyro, bit7:Acc
#define ACGY_ADD_TMP_LSB 0x15	//Temperature Output LSB Register  Read
#define ACGY_ADD_TMP_MSB 0x16	//Temperature Output MSB Register  Read
#define ACGY_CTRL_REG1_G 0x10	//Control Register1 >> Gyro full-scale, ODR, BW
#define ACGY_ORIENT_CFG_G 0x13	//Gyro XYZ sign
#define ACGY_CTRL_REG4 0x1E		//Control Register4 >> GYRO XYZ enable
#define ACGY_CTRL_REG5 0x1F		//Control Register5 >> ACC XYZ enable
#define ACGY_CTRL_REG6 0x20		//Control Register6 >> ACC full-scale, ODR, BW
#define ACGY_CTRL_REG7 0x21		//Control Register7 >>ACC high resolution mode
#define ACGY_CTRL_REG8 0x22		//Control Register8 >> Softreset → claer by hardware
#define ACGY_CTRL_REG9 0x23		//Control Register9 >> FIFO enable
#define ACGY_FIFO_CTRL 0x2E		//FIFO mode select
#define ACGY_ACC_ADD_X_LSB 0x28 //Acc Xout LSB
#define ACGY_ACC_ADD_X_MSB 0x29 //Acc Xout MSB
#define ACGY_ACC_ADD_Y_LSB 0x2A //Acc Yout LSB
#define ACGY_ACC_ADD_Y_MSB 0x2B //Acc Yout MSB
#define ACGY_ACC_ADD_Z_LSB 0x2C //Acc Zout LSB
#define ACGY_ACC_ADD_Z_MSB 0x2D //Acc Zout MSB
#define ACGY_ADD_ID 0x0F		//Product ID 0b01101000 >> 0x68

// extern idata char ADD_I2C;		//I2C control address
// extern idata char ADD_CONFIG_A; //Configuration Register A   Read/Write
// extern idata char ADD_CONFIG_B; //Configuration Register B   Read/Write
// extern idata char ADD_MODE;		//Mode Register  Read/Write
// extern idata char ADD_X_MSB;	//Data Output X MSB Register  Read
// extern idata char ADD_X_LSB;	//Data Output X LSB Register  Read
// extern idata char ADD_Z_MSB;	//Data Output Z MSB Register  Read
// extern idata char ADD_Z_LSB;	//Data Output Z LSB Register  Read
// extern idata char ADD_Y_MSB;	//Data Output Y MSB Register  Read
// extern idata char ADD_Y_LSB;	//Data Output Y LSB Register  Read
// extern idata char ADD_STATUS;	//Status Register  Read
// extern idata char ADD_ID_A;		//Identification Register A  Read
// extern idata char ADD_ID_B;		//Identification Register B  Read
// extern idata char ADD_ID_C;		//Identification Register C  Read
// extern idata char ADD_TMP_MSB;	//Temperature Output MSB Register  Read
// extern idata char ADD_TMP_LSB;	//Temperature Output LSB Register  Read
// extern idata char ADD_CONFIG_0; //Control register 0
// extern idata char ADD_CONFIG_1; //Control register 1
// extern idata char ADD_CONFIG_2; //Control register 2
// //extern char ADD_X_THR;    //Motion detection threshold of X
// //extern char ADD_Y_THR;    //Motion detection threshold of Y
// //extern char ADD_Z_THR;    //Motion detection threshold of Z
// extern idata char ADD_ID; //Product ID

// // LSM
// //define LSM9DS1 mag
// extern idata char CTRL_REG1_M; 
// extern idata char CTRL_REG2_M; 
// extern idata char CTRL_REG3_M; 
// extern idata char CTRL_REG4_M; 
// extern idata char CTRL_REG5_M; 

// //define LSM9DS1 acc gyro
// extern idata char ACGY_ADD_I2C;		
// extern idata char ACGY_GYR_ADD_X_LSB;	
// extern idata char ACGY_GYR_ADD_X_MSB;	
// extern idata char ACGY_GYR_ADD_Y_LSB;	
// extern idata char ACGY_GYR_ADD_Y_MSB;	
// extern idata char ACGY_GYR_ADD_Z_LSB;	
// extern idata char ACGY_GYR_ADD_Z_MSB;	
// extern idata char ACGY_ADD_STATUS;	
// extern idata char ACGY_CTRL_REG1_G;	
// extern idata char ACGY_ORIENT_CFG_G; 
// extern idata char ACGY_ADD_ID;		
// extern idata char ACGY_FIFO_CTRL;
// extern idata char ACGY_CTRL_REG4;
// extern idata char ACGY_CTRL_REG5;
// extern idata char ACGY_CTRL_REG6;
// extern idata char ACGY_CTRL_REG7; 
// extern idata char ACGY_CTRL_REG8;
// extern idata char ACGY_CTRL_REG9;
// extern idata char ACGY_ACC_ADD_X_LSB; 
// extern idata char ACGY_ACC_ADD_X_MSB; 
// extern idata char ACGY_ACC_ADD_Y_LSB; 
// extern idata char ACGY_ACC_ADD_Y_MSB; 
// extern idata char ACGY_ACC_ADD_Z_LSB; 
// extern idata char ACGY_ACC_ADD_Z_MSB; 
// extern idata char ACGY_ADD_TMP_MSB;	
// extern idata char ACGY_ADD_TMP_LSB;	

// extern void SetI2CParameters(void);

#endif
//#define MMC5883MA
/*
#ifdef HMC5983
	#define ADD_I2C      0x3C  //I2C control address
	#define ADD_CONFIG_A 0x00  //Configuration Register A   Read/Write 
	#define ADD_CONFIG_B 0x01  //Configuration Register B   Read/Write 
	#define ADD_MODE     0x02  //Mode Register  Read/Write 
	#define ADD_X_MSB    0x03  //Data Output X MSB Register  Read 
	#define ADD_X_LSB    0x04  //Data Output X LSB Register  Read 
	#define ADD_Z_MSB    0x05  //Data Output Z MSB Register  Read 
	#define ADD_Z_LSB    0x06  //Data Output Z LSB Register  Read 
	#define ADD_Y_MSB    0x07  //Data Output Y MSB Register  Read 
	#define ADD_Y_LSB    0x08  //Data Output Y LSB Register  Read 
	#define ADD_STATUS   0x09  //Status Register  Read 
	#define ADD_ID_A     0x0A  //Identification Register A  Read 
	#define ADD_ID_B     0x0B  //Identification Register B  Read 
	#define ADD_ID_C     0x0C  //Identification Register C  Read 
	#define ADD_TMP_MSB  0x31  //Temperature Output MSB Register  Read 
	#define ADD_TMP_LSB  0x32  //Temperature Output LSB Register  Read
#endif

#ifdef MMC5883MA
	#define ADD_I2C      0x60  //I2C control address 0x60 0110 0000
	#define ADD_X_LSB    0x00  //Xout LSB 
	#define ADD_X_MSB    0x01  //Xout MSB 
	#define ADD_Y_LSB    0x02  //Yout LSB 
	#define ADD_Y_MSB    0x03  //Yout MSB 
	#define ADD_Z_LSB    0x04  //Zout LSB 
	#define ADD_Z_MSB    0x05  //Zout MSB 
	#define ADD_TMP_MSB  0x06  //Temperature output 
	#define ADD_STATUS   0x07  //Device status 
	#define ADD_CONFIG_0 0x08  //Control register 0 
	#define ADD_CONFIG_1 0x09  //Control register 1 
	#define ADD_CONFIG_2 0x0A  //Control register 2 
	#define ADD_X_THR    0x0B  //Motion detection threshold of X 
	#define ADD_Y_THR    0x0C  //Motion detection threshold of Y 
	#define ADD_Z_THR    0x0D  //Motion detection threshold of Z 
	#define ADD_ID       0x2F  //Product ID 0b0010 1111
#endif
*/