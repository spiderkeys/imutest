#pragma once
/* EM7180_LSM9DS0_MS5637_t3 Basic Example Code
 by: Kris Winer
 date: January 24, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.

 The EM7180 SENtral sensor hub is not a motion sensor, but rather takes raw sensor data from a variety of motion sensors,
 in this case the LSM9DS0, and does sensor fusion with quaternions as its output. The SENtral loads firmware from the
 on-board M24512DFMC 512 kbit EEPROM upon startup, configures and manages the sensors on its dedicated master I2C bus,
 and outputs scaled sensor data (accelerations, rotation rates, and magnetic fields) as well as quaternions and
 heading/pitch/roll, if selected.

 This sketch demonstrates basic EM7180 SENtral functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and
 Mahony filter algorithms to compare with the hardware sensor fusion results.
 Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

 This sketch is specifically for the Teensy 3.1 Mini Add-On shield with the EM7180 SENtral sensor hub as master,
 the LSM9DS0 9-axis motion sensor (accel/gyro/mag) as slave, an LPS25H pressure/temperature sensor, and an M24512DFM
 512kbit (64 kByte) EEPROM as slave all connected via I2C. The SENtral can use the pressure data in the sensor fusion
 and there is currently a driver for the LPS25H in the SENtral firmware.

 This sketch uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 The LPS25H is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application. The LPS25H is connected to the EM7180 I2C master bus and has an interrupt
 connected to the EM7180 just like the LSM9DS0. The driver will use the data ready interrupt from the LPS25H to signal
 to the EM7180 that it should read and process the pressure/temperature data.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 4k7 resistors are on the EM7180+LSM9DS0+LPS25H+M24512DFM Mini Add-On board for Teensy 3.1.

 Hardware setup:
 EM7180 Mini Add-On ------- Teensy 3.1
 VDD ---------------------- 3.3V
 SDA ----------------------- 17
 SCL ----------------------- 16
 GND ---------------------- GND
 INT------------------------ 8
*/

#include "CI2C.h"

#define PI 3.14159f

// See LPS25H "MEMS pressure sensor: 260-1260 hPa absolute digital output barometer" Data Sheet
#define LPS25H_REF_P_XL      0x08
#define LPS25H_REF_P_L       0x09
#define LPS25H_REF_P_H       0x0A
#define LPS25H_WHOAMI        0x0F // should return 0xBD
#define LPS25H_RES_CONF      0x10
#define LPS25H_CTRL_REG1     0x20
#define LPS25H_CTRL_REG2     0x21
#define LPS25H_CTRL_REG3     0x22
#define LPS25H_CTRL_REG4     0x23
#define LPS25H_INT_CFG       0x24
#define LPS25H_INT_SOURCE    0x25
#define LPS25H_STATUS_REG    0x27
#define LPS25H_PRESS_OUT_XL  0x28
#define LPS25H_PRESS_OUT_L   0x29
#define LPS25H_PRESS_OUT_H   0x2A
#define LPS25H_TEMP_OUT_L    0x2B
#define LPS25H_TEMP_OUT_H    0x2C
#define LPS25H_FIFO_CTRL     0x2E
#define LPS25H_FIFO_STATUS   0x2F
#define LPS25H_THS_P_L       0x30
#define LPS25H_THS_P_H       0x31
#define LPS25H_RPDS_L        0x39
#define LPS25H_RPDS_H        0x3A

// See also LSM9DS0 Register Map and Descriptions,http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00087365.pdf
//
////////////////////////////
// LSM9DS0 Gyro Registers //
////////////////////////////
#define  LSM9DS0G_WHO_AM_I_G		0x0F
#define  LSM9DS0G_CTRL_REG1_G		0x20
#define  LSM9DS0G_CTRL_REG2_G		0x21
#define  LSM9DS0G_CTRL_REG3_G		0x22
#define  LSM9DS0G_CTRL_REG4_G		0x23
#define  LSM9DS0G_CTRL_REG5_G		0x24
#define  LSM9DS0G_REFERENCE_G		0x25
#define  LSM9DS0G_STATUS_REG_G		0x27
#define  LSM9DS0G_OUT_X_L_G		0x28
#define  LSM9DS0G_OUT_X_H_G		0x29
#define  LSM9DS0G_OUT_Y_L_G		0x2A
#define  LSM9DS0G_OUT_Y_H_G		0x2B
#define  LSM9DS0G_OUT_Z_L_G		0x2C
#define  LSM9DS0G_OUT_Z_H_G		0x2D
#define  LSM9DS0G_FIFO_CTRL_REG_G	0x2E
#define  LSM9DS0G_FIFO_SRC_REG_G	0x2F
#define  LSM9DS0G_INT1_CFG_G		0x30
#define  LSM9DS0G_INT1_SRC_G		0x31
#define  LSM9DS0G_INT1_THS_XH_G		0x32
#define  LSM9DS0G_INT1_THS_XL_G		0x33
#define  LSM9DS0G_INT1_THS_YH_G		0x34
#define  LSM9DS0G_INT1_THS_YL_G		0x35
#define  LSM9DS0G_INT1_THS_ZH_G		0x36
#define  LSM9DS0G_INT1_THS_ZL_G		0x37
#define  LSM9DS0G_INT1_DURATION_G	0x38

//////////////////////////////////////////
//  LSM9DS0XM Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define  LSM9DS0XM_OUT_TEMP_L_XM	0x05
#define  LSM9DS0XM_OUT_TEMP_H_XM	0x06
#define  LSM9DS0XM_STATUS_REG_M		0x07
#define  LSM9DS0XM_OUT_X_L_M		0x08
#define  LSM9DS0XM_OUT_X_H_M		0x09
#define  LSM9DS0XM_OUT_Y_L_M		0x0A
#define  LSM9DS0XM_OUT_Y_H_M		0x0B
#define  LSM9DS0XM_OUT_Z_L_M		0x0C
#define  LSM9DS0XM_OUT_Z_H_M		0x0D
#define  LSM9DS0XM_WHO_AM_I_XM		0x0F
#define  LSM9DS0XM_INT_CTRL_REG_M	0x12
#define  LSM9DS0XM_INT_SRC_REG_M	0x13
#define  LSM9DS0XM_INT_THS_L_M		0x14
#define  LSM9DS0XM_INT_THS_H_M		0x15
#define  LSM9DS0XM_OFFSET_X_L_M		0x16
#define  LSM9DS0XM_OFFSET_X_H_M		0x17
#define  LSM9DS0XM_OFFSET_Y_L_M		0x18
#define  LSM9DS0XM_OFFSET_Y_H_M		0x19
#define  LSM9DS0XM_OFFSET_Z_L_M		0x1A
#define  LSM9DS0XM_OFFSET_Z_H_M		0x1B
#define  LSM9DS0XM_REFERENCE_X		0x1C
#define  LSM9DS0XM_REFERENCE_Y		0x1D
#define  LSM9DS0XM_REFERENCE_Z		0x1E
#define  LSM9DS0XM_CTRL_REG0_XM		0x1F
#define  LSM9DS0XM_CTRL_REG1_XM		0x20
#define  LSM9DS0XM_CTRL_REG2_XM		0x21
#define  LSM9DS0XM_CTRL_REG3_XM		0x22
#define  LSM9DS0XM_CTRL_REG4_XM		0x23
#define  LSM9DS0XM_CTRL_REG5_XM		0x24
#define  LSM9DS0XM_CTRL_REG6_XM		0x25
#define  LSM9DS0XM_CTRL_REG7_XM		0x26
#define  LSM9DS0XM_STATUS_REG_A		0x27
#define  LSM9DS0XM_OUT_X_L_A		0x28
#define  LSM9DS0XM_OUT_X_H_A		0x29
#define  LSM9DS0XM_OUT_Y_L_A		0x2A
#define  LSM9DS0XM_OUT_Y_H_A		0x2B
#define  LSM9DS0XM_OUT_Z_L_A		0x2C
#define  LSM9DS0XM_OUT_Z_H_A		0x2D
#define  LSM9DS0XM_FIFO_CTRL_REG	0x2E
#define  LSM9DS0XM_FIFO_SRC_REG		0x2F
#define  LSM9DS0XM_INT_GEN_1_REG	0x30
#define  LSM9DS0XM_INT_GEN_1_SRC	0x31
#define  LSM9DS0XM_INT_GEN_1_THS	0x32
#define  LSM9DS0XM_INT_GEN_1_DURATION	0x33
#define  LSM9DS0XM_INT_GEN_2_REG	0x34
#define  LSM9DS0XM_INT_GEN_2_SRC	0x35
#define  LSM9DS0XM_INT_GEN_2_THS	0x36
#define  LSM9DS0XM_INT_GEN_2_DURATION	0x37
#define  LSM9DS0XM_CLICK_CFG		0x38
#define  LSM9DS0XM_CLICK_SRC		0x39
#define  LSM9DS0XM_CLICK_THS		0x3A
#define  LSM9DS0XM_TIME_LIMIT		0x3B
#define  LSM9DS0XM_TIME_LATENCY		0x3C
#define  LSM9DS0XM_TIME_WINDOW		0x3D
#define  LSM9DS0XM_ACT_THS		0x3E
#define  LSM9DS0XM_ACT_DUR		0x3F


// EM7180 SENtral register map
// see http://www.emdeveloper.com/downloads/7180/EMSentral_EM7180_Register_Map_v1_3.pdf
//
#define EM7180_QX                 0x00  // this is a 32-bit normalized floating point number read from registers 0x00-03
#define EM7180_QY                 0x04  // this is a 32-bit normalized floating point number read from registers 0x04-07
#define EM7180_QZ                 0x08  // this is a 32-bit normalized floating point number read from registers 0x08-0B
#define EM7180_QW                 0x0C  // this is a 32-bit normalized floating point number read from registers 0x0C-0F
#define EM7180_QTIME              0x10  // this is a 16-bit unsigned integer read from registers 0x10-11
#define EM7180_MX                 0x12  // int16_t from registers 0x12-13
#define EM7180_MY                 0x14  // int16_t from registers 0x14-15
#define EM7180_MZ                 0x16  // int16_t from registers 0x16-17
#define EM7180_MTIME              0x18  // uint16_t from registers 0x18-19
#define EM7180_AX                 0x1A  // int16_t from registers 0x1A-1B
#define EM7180_AY                 0x1C  // int16_t from registers 0x1C-1D
#define EM7180_AZ                 0x1E  // int16_t from registers 0x1E-1F
#define EM7180_ATIME              0x20  // uint16_t from registers 0x20-21
#define EM7180_GX                 0x22  // int16_t from registers 0x22-23
#define EM7180_GY                 0x24  // int16_t from registers 0x24-25
#define EM7180_GZ                 0x26  // int16_t from registers 0x26-27
#define EM7180_GTIME              0x28  // uint16_t from registers 0x28-29
#define EM7180_Baro               0x2A  // start of two-byte LPS25H pressure data, 16-bit signed interger
#define EM7180_BaroTIME           0x2C  // start of two-byte LPS25H pressure timestamp, 16-bit unsigned
#define EM7180_Temp               0x2E  // start of two-byte LPS25H temperature data, 16-bit signed interger
#define EM7180_TempTIME           0x30  // start of two-byte LPS25H temperature timestamp, 16-bit unsigned
#define EM7180_QRateDivisor       0x32  // uint8_t
#define EM7180_EnableEvents       0x33
#define EM7180_HostControl        0x34
#define EM7180_EventStatus        0x35
#define EM7180_SensorStatus       0x36
#define EM7180_SentralStatus      0x37
#define EM7180_AlgorithmStatus    0x38
#define EM7180_FeatureFlags       0x39
#define EM7180_ParamAcknowledge   0x3A
#define EM7180_SavedParamByte0    0x3B
#define EM7180_SavedParamByte1    0x3C
#define EM7180_SavedParamByte2    0x3D
#define EM7180_SavedParamByte3    0x3E
#define EM7180_ActualMagRate      0x45
#define EM7180_ActualAccelRate    0x46
#define EM7180_ActualGyroRate     0x47
#define EM7180_ActualBaroRate     0x48
#define EM7180_ActualTempRate     0x49
#define EM7180_ErrorRegister      0x50
#define EM7180_AlgorithmControl   0x54
#define EM7180_MagRate            0x55
#define EM7180_AccelRate          0x56
#define EM7180_GyroRate           0x57
#define EM7180_BaroRate           0x58
#define EM7180_TempRate           0x59
#define EM7180_LoadParamByte0     0x60
#define EM7180_LoadParamByte1     0x61
#define EM7180_LoadParamByte2     0x62
#define EM7180_LoadParamByte3     0x63
#define EM7180_ParamRequest       0x64
#define EM7180_ROMVersion1        0x70
#define EM7180_ROMVersion2        0x71
#define EM7180_RAMVersion1        0x72
#define EM7180_RAMVersion2        0x73
#define EM7180_ProductID          0x90
#define EM7180_RevisionID         0x91
#define EM7180_RunStatus          0x92
#define EM7180_UploadAddress      0x94 // uint16_t registers 0x94 (MSB)-5(LSB)
#define EM7180_UploadData         0x96
#define EM7180_CRCHost            0x97  // uint32_t from registers 0x97-9A
#define EM7180_ResetRequest       0x9B
#define EM7180_PassThruStatus     0x9E
#define EM7180_PassThruControl    0xA0

// Using the Teensy Mini Add-On board, LSM9DS0 SDOG = SDOXM = GND as designed
// Seven-bit LSM9DS0 device addresses are ACC = 0x1E, GYRO = 0x6A, MAG = 0x1E

// Using the EM7180+LSM9DS0+LPS25H Teensy 3.1 Add-On shield, ADO is set to 0
#define ADO 0
#if ADO
#define LSM9DS0XM_ADDRESS        0x1D // Address of accel/magnetometer when ADO = 1
#define LSM9DS0G_ADDRESS         0x6B // Address of gyro when ADO = 1
#else
#define LSM9DS0XM_ADDRESS        0x1E // Address of accel/magnetometer when ADO = 0
#define LSM9DS0G_ADDRESS         0x6A // Address of gyro when ADO = 0
#endif

#define LPS25H_ADDRESS           0x5D   // Address of altimeter with LPS25H

#define EM7180_ADDRESS           0x28   // Address of the EM7180 SENtral sensor hub

#define M24512DFM_DATA_ADDRESS   0x50   // Address of the 500 page M24512DFM EEPROM data buffer, 1024 bits (128 8-bit bytes) per page
#define M24512DFM_IDPAGE_ADDRESS 0x58   // Address of the single M24512DFM lockable EEPROM ID page

#define SerialDebug true  // set to true to get Serial output for debugging

// Set initial input parameters
enum Ascale
{
	// set of allowable accel full scale settings
	AFS_2G = 0,
	AFS_4G,
	AFS_6G,
	AFS_8G,
	AFS_16G
};

enum Aodr
{
	// set of allowable gyro sample rates
  AODR_PowerDown = 0,
  AODR_3_125Hz,
  AODR_6_25Hz,
  AODR_12_5Hz,
  AODR_25Hz,
  AODR_50Hz,
  AODR_100Hz,
  AODR_200Hz,
  AODR_400Hz,
  AODR_800Hz,
  AODR_1600Hz
};

enum Abw
{
	// set of allowable accewl bandwidths
   ABW_773Hz = 0,
   ABW_194Hz,
   ABW_362Hz,
   ABW_50Hz
};

enum Gscale
{
	// set of allowable gyro full scale settings
  GFS_245DPS = 0,
  GFS_500DPS,
  GFS_NoOp,
  GFS_2000DPS
};

enum Godr
{
	// set of allowable gyro sample rates
  GODR_95Hz = 0,
  GODR_190Hz,
  GODR_380Hz,
  GODR_760Hz
};

enum Gbw
{
	// set of allowable gyro data bandwidths
  GBW_low = 0,  // 12.5 Hz at Godr = 95 Hz, 12.5 Hz at Godr = 190 Hz,  30 Hz at Godr = 760 Hz
  GBW_med,      // 25 Hz   at Godr = 95 Hz, 25 Hz   at Godr = 190 Hz,  35 Hz at Godr = 760 Hz
  GBW_high,     // 25 Hz   at Godr = 95 Hz, 50 Hz   at Godr = 190 Hz,  50 Hz at Godr = 760 Hz
  GBW_highest   // 25 Hz   at Godr = 95 Hz, 70 Hz   at Godr = 190 Hz, 100 Hz at Godr = 760 Hz
};

enum Mscale
{
	// set of allowable mag full scale settings
  MFS_2G = 0,
  MFS_4G,
  MFS_8G,
  MFS_12G
};

enum Mres
{
  MRES_LowResolution = 0,
  MRES_NoOp,
  MRES_HighResolution
};

enum Modr
{
	// set of allowable mag sample rates
  MODR_3_125Hz = 0,
  MODR_6_25Hz,
  MODR_12_5Hz,
  MODR_25Hz,
  MODR_50Hz,
  MODR_100Hz
};

// Specify sensor full scale
uint8_t Gscale 	= GFS_245DPS; 			// gyro full scale
uint8_t Godr 	= GODR_380Hz;   		// gyro data sample rate
uint8_t Gbw 	= GBW_low;      		// gyro data bandwidth
uint8_t Ascale 	= AFS_2G;     			// accel full scale
uint8_t Aodr 	= AODR_400Hz;   		// accel data sample rate
uint8_t Abw 	= ABW_50Hz;     		// accel data bandwidth
uint8_t Mscale 	= MFS_12G;     			// mag full scale
uint8_t Modr 	= MODR_100Hz;   		// mag data sample rate
uint8_t Mres 	= MRES_LowResolution;  	// magnetometer operation mode

float aRes, gRes, mRes;            		// scale resolutions per LSB for the sensors

float Quat[4] 		= {0, 0, 0, 0}; 		// quaternion data register
float Yaw, Pitch, Roll;

uint8_t param[4];                         		// used for param transfer

uint8_t ReadByte( uint8_t deviceAddress, uint8_t registerAddressIn )
{

	// Set address to request from
	uint8_t ret = I2c.read( deviceAddress, ( uint8_t )registerAddressIn, ( uint8_t )1 );

	// Non-zero failure
	if( ret )
	{
		return 0xFF;
	}

	// Request single byte from slave
	return I2c.receive();
}

int count = 0;

static inline float uint32_reg_to_float (uint8_t *buf)
{
  union {
    uint32_t ui32;
    float f;
  } u;

  u.ui32 =     (((uint32_t)buf[0]) +
               (((uint32_t)buf[1]) <<  8) +
               (((uint32_t)buf[2]) << 16) +
               (((uint32_t)buf[3]) << 24));
  return u.f;
}


void float_to_bytes (float param_val, uint8_t *buf) {
  union {
    float f;
    uint8_t comp[sizeof(float)];
  } u;
  u.f = param_val;
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = u.comp[i];
  }
  //Convert to LITTLE ENDIAN
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = buf[(sizeof(float)-1) - i];
  }
}

void EM7180_set_gyro_FS (uint16_t gyro_fs) {
  uint8_t bytes[4], STAT;
  bytes[0] = gyro_fs & (0xFF);
  bytes[1] = (gyro_fs >> 8) & (0xFF);
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte0, bytes[0]); //Gyro LSB
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte1, bytes[1]); //Gyro MSB
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte2, bytes[2]); //Unused
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte3, bytes[3]); //Unused

  I2c.write(EM7180_ADDRESS, EM7180_ParamRequest, 0xCB); //Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
  I2c.write(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
  STAT = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==0xCB)) {
    STAT = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  I2c.write(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  I2c.write(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_mag_acc_FS (uint16_t mag_fs, uint16_t acc_fs) {
  uint8_t bytes[4], STAT;
  bytes[0] = mag_fs & (0xFF);
  bytes[1] = (mag_fs >> 8) & (0xFF);
  bytes[2] = acc_fs & (0xFF);
  bytes[3] = (acc_fs >> 8) & (0xFF);
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte0, bytes[0]); //Mag LSB
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte1, bytes[1]); //Mag MSB
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte2, bytes[2]); //Acc LSB
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte3, bytes[3]); //Acc MSB
  I2c.write(EM7180_ADDRESS, EM7180_ParamRequest, 0xCA); //Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
  I2c.write(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
  STAT = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==0xCA)) {
    STAT = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  I2c.write(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  I2c.write(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_integer_param (uint8_t param, uint32_t param_val) {
  uint8_t bytes[4], STAT;
  bytes[0] = param_val & (0xFF);
  bytes[1] = (param_val >> 8) & (0xFF);
  bytes[2] = (param_val >> 16) & (0xFF);
  bytes[3] = (param_val >> 24) & (0xFF);
  param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte0, bytes[0]); //Param LSB
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte1, bytes[1]);
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte2, bytes[2]);
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte3, bytes[3]); //Param MSB
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_ParamRequest, param);
  I2c.write(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
  STAT = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==param)) {
    STAT = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  I2c.write(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  I2c.write(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_float_param (uint8_t param, float param_val) {
  uint8_t bytes[4], STAT;
  float_to_bytes (param_val, &bytes[0]);
  param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte0, bytes[0]); //Param LSB
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte1, bytes[1]);
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte2, bytes[2]);
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_LoadParamByte3, bytes[3]); //Param MSB
  I2c.write((uint8_t)EM7180_ADDRESS, (uint8_t)EM7180_ParamRequest, param);
  I2c.write(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
  STAT = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==param)) {
    STAT = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  I2c.write(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  I2c.write(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}
void readSENtralQuatData(float * destination)
{
  uint8_t rawData[16];  // x/y/z quaternion register data stored here
  I2c.read(EM7180_ADDRESS, EM7180_QX, 16, &rawData[0]);       // Read the sixteen raw data registers into data array
  destination[0] = uint32_reg_to_float (&rawData[0]);
  destination[1] = uint32_reg_to_float (&rawData[4]);
  destination[2] = uint32_reg_to_float (&rawData[8]);
  destination[3] = uint32_reg_to_float (&rawData[12]);

}

void readSENtralAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  I2c.read(EM7180_ADDRESS, EM7180_AX, 6, &rawData[0]);       // Read the six raw data registers into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]);
}

void readSENtralGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  I2c.read(EM7180_ADDRESS, EM7180_GX, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]);
}

void readSENtralMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  I2c.read(EM7180_ADDRESS, EM7180_MX, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]);
}




// I2C communication with the M24512DFM EEPROM is a little different from I2C communication with the usual motion sensor
// since the address is defined by two bytes

uint8_t M24512DFMwriteByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t  data)
{
	uint8_t returnStatus = 0;

	// Start transmission
	returnStatus = I2c.start();

	// Return if non-zero
	if( returnStatus )
	{
		return returnStatus;
	}

	// Signal the slave that it will be written to
	returnStatus = I2c.sendAddress( SLA_W( device_address ) );

	// Return if non-zero
	if( returnStatus )
	{
		// TODO: This is dumb
		if( returnStatus == 1 )
		{
			return( 2 );
		}

		return( returnStatus );
	}

	// Send register address byte
	returnStatus = I2c.sendByte( data_address1 );

	// Return if non-zero
	if( returnStatus )
	{
		if( returnStatus == 1 )
		{
			return( 3 );
		}

		return( returnStatus );
	}

	// Send register address byte
	returnStatus = I2c.sendByte( data_address2 );

	// Return if non-zero
	if( returnStatus )
	{
		if( returnStatus == 1 )
		{
			return( 3 );
		}

		return( returnStatus );
	}

	// Send data byte
	returnStatus = I2c.sendByte( data );

	// Return if non-zero
	if( returnStatus )
	{
		if( returnStatus == 1 )
		{
			return( 3 );
		}

		return( returnStatus );
	}

	// End transmission
	returnStatus = I2c.stop();

	// Return if non-zero
	if( returnStatus )
	{
		if( returnStatus == 1 )
		{
			return( 7 );
		}

		return( returnStatus );
	}

	// Zero signifies success
	return( returnStatus );
}


uint8_t M24512DFMwriteBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{
	if(count > 128)
	{
        count = 128;
        Serial.print("Page count cannot be more than 128 bytes!");
        return 99;
	}

	uint8_t returnStatus = 0;
	returnStatus = I2c.start();

	if( returnStatus )
	{
		return( returnStatus );
	}

	returnStatus = I2c.sendAddress( SLA_W( device_address ) );

	if( returnStatus )
	{
		if( returnStatus == 1 )
		{
			return( 2 );
		}

		return( returnStatus );
	}

	returnStatus = I2c.sendByte( data_address1 );

	if( returnStatus )
	{
		if( returnStatus == 1 )
		{
			return( 3 );
		}

		return( returnStatus );
	}

	returnStatus = I2c.sendByte( data_address2 );

	if( returnStatus )
	{
		if( returnStatus == 1 )
		{
			return( 3 );
		}

		return( returnStatus );
	}

	for( uint8_t i = 0; i < count; i++ )
	{
		returnStatus = I2c.sendByte( dest[i] );

		if( returnStatus )
		{
			if( returnStatus == 1 )
			{
				return( 3 );
			}

			return( returnStatus );
		}
	}

	returnStatus = I2c.stop();

	if( returnStatus )
	{
		if( returnStatus == 1 )
		{
			return( 7 );
		}

		return( returnStatus );
	}

	return( returnStatus );
}

uint8_t M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{
	uint8_t bytesAvailable = 0;
	uint8_t bufferIndex = 0;
	uint8_t totalBytes = 0;

	if( count == 0 )
	{
		count++;
	}

	uint8_t nack = count - 1;
	uint8_t returnStatus = 0;

	returnStatus = I2c.start();

	if( returnStatus )
	{
		return( returnStatus );
	}

	returnStatus = I2c.sendAddress( SLA_W( device_address ) );

	if( returnStatus )
	{
		if( returnStatus == 1 )
		{
			return( 2 );
		}

		return( returnStatus );
	}

	returnStatus = I2c.sendByte( data_address1 );

	if( returnStatus )
	{
		if( returnStatus == 1 )
		{
			return( 3 );
		}

		return( returnStatus );
	}

	returnStatus = I2c.sendByte( data_address2 );

	if( returnStatus )
	{
		if( returnStatus == 1 )
		{
			return( 3 );
		}

		return( returnStatus );
	}


	returnStatus = I2c.start();

	if( returnStatus )
	{
		if( returnStatus == 1 )
		{
			return( 4 );
		}

		return( returnStatus );
	}

	returnStatus = I2c.sendAddress( SLA_R( device_address ) );

	if( returnStatus )
	{
		if( returnStatus == 1 )
		{
			return( 5 );
		}

		return( returnStatus );
	}

	for( uint8_t i = 0; i < count; i++ )
	{


		if( i == nack )
		{
			returnStatus = I2c.receiveByte( 0 );

			if( returnStatus == 1 )
			{
				return( 6 );
			}

			if( returnStatus != MR_DATA_NACK )
			{
				return( returnStatus );
			}
		}
		else
		{
			returnStatus = I2c.receiveByte( 1 );

			if( returnStatus == 1 )
			{
				return( 6 );
			}

			if( returnStatus != MR_DATA_ACK )
			{
				return( returnStatus );
			}
		}

		dest[i] = TWDR;
		bytesAvailable = i + 1;
		totalBytes = i + 1;
	}

	returnStatus = I2c.stop();

	if( returnStatus )
	{
		if( returnStatus == 1 )
		{
			return( 7 );
		}

		return( returnStatus );
	}


	return( returnStatus );
}

uint8_t M24512DFMreadByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2)
{
	uint8_t data; // `data` will store the register data

	return M24512DFMreadBytes( device_address, data_address1, data_address2, 1, &data );
}

void Initialize()
{

	// Read SENtral device information
	uint16_t ROM1 = ReadByte(EM7180_ADDRESS, EM7180_ROMVersion1);
	uint16_t ROM2 = ReadByte(EM7180_ADDRESS, EM7180_ROMVersion2);
	Serial.print("EM7180 ROM Version: 0x"); Serial.print(ROM1, HEX); Serial.println(ROM2, HEX); Serial.println("Should be: 0xE609");
	uint16_t RAM1 = ReadByte(EM7180_ADDRESS, EM7180_RAMVersion1);
	uint16_t RAM2 = ReadByte(EM7180_ADDRESS, EM7180_RAMVersion2);
	Serial.print("EM7180 RAM Version: 0x"); Serial.print(RAM1); Serial.println(RAM2);
	uint8_t PID = ReadByte(EM7180_ADDRESS, EM7180_ProductID);
	Serial.print("EM7180 ProductID: 0x"); Serial.print(PID, HEX); Serial.println(" Should be: 0x80");
	uint8_t RID = ReadByte(EM7180_ADDRESS, EM7180_RevisionID);
	Serial.print("EM7180 RevisionID: 0x"); Serial.print(RID, HEX); Serial.println(" Should be: 0x02");

	delay(2000); // give some time to read the screen

	// Check SENtral status, make sure EEPROM upload of firmware was accomplished
	byte STAT = (ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
	if(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("EEPROM detected on the sensor bus!");
	if(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("EEPROM uploaded config file!");
	if(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("EEPROM CRC incorrect!");
	if(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("EM7180 in initialized state!");
	if(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("No EEPROM detected!");

	int count = 0;

	while(!STAT)
	{
		I2c.write(EM7180_ADDRESS, EM7180_ResetRequest, 0x01);

		delay(500);

		count++;
		STAT = (ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);

		if(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("EEPROM detected on the sensor bus!");
		if(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("EEPROM uploaded config file!");
		if(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("EEPROM CRC incorrect!");
		if(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("EM7180 in initialized state!");
		if(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("No EEPROM detected!");

		if(count > 10) break;
	}

	if(!(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04))  Serial.println("EEPROM upload successful!");

	delay(1000); // give some time to read the screen

	// Enter EM7180 initialized state
	I2c.write(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers
	I2c.write(EM7180_ADDRESS, EM7180_PassThruControl, 0x00); // make sure pass through mode is off

	// Set accel/gyro/mage desired ODR rates
	I2c.write(EM7180_ADDRESS, EM7180_QRateDivisor, 0x02); // 95 Hz
	I2c.write(EM7180_ADDRESS, EM7180_MagRate, 0x1E); // 30 Hz
	I2c.write(EM7180_ADDRESS, EM7180_AccelRate, 0x14); // 200/10 Hz
	I2c.write(EM7180_ADDRESS, EM7180_GyroRate, 0x13);  // 190/10 Hz

	// Configure operating mode
	I2c.write(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data

	// Enable interrupt to host upon certain events
	// choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
	// new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
	I2c.write(EM7180_ADDRESS, EM7180_EnableEvents, 0x7F);

	// Enable EM7180 run mode
	I2c.write(EM7180_ADDRESS, EM7180_HostControl, 0x01); // set SENtral in normal run mode

	delay(100);

	// EM7180 parameter adjustments
	Serial.println("Beginning Parameter Adjustments");

	//Disable stillness mode
	EM7180_set_integer_param (0x49, 0x00);

	//Write desired sensor full scale ranges to the EM7180
	EM7180_set_mag_acc_FS (0x3E8, 0x08); // 1000 uT, 8 g
	EM7180_set_gyro_FS (0x7D0); // 2000 dps

	// Read EM7180 status
	uint8_t runStatus = ReadByte(EM7180_ADDRESS, EM7180_RunStatus);

	if(runStatus & 0x01) Serial.println(" EM7180 run status = normal mode");

	uint8_t algoStatus = ReadByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);

	if(algoStatus & 0x01) Serial.println(" EM7180 standby status");
	if(algoStatus & 0x02) Serial.println(" EM7180 algorithm slow");
	if(algoStatus & 0x04) Serial.println(" EM7180 in stillness mode");
	if(algoStatus & 0x08) Serial.println(" EM7180 mag calibration completed");
	if(algoStatus & 0x10) Serial.println(" EM7180 magnetic anomaly detected");
	if(algoStatus & 0x20) Serial.println(" EM7180 unreliable sensor data");

	uint8_t passthruStatus = ReadByte(EM7180_ADDRESS, EM7180_PassThruStatus);

	if(passthruStatus & 0x01) Serial.print(" EM7180 in passthru mode!");

	uint8_t eventStatus = ReadByte(EM7180_ADDRESS, EM7180_EventStatus);

	if(eventStatus & 0x01) Serial.println(" EM7180 CPU reset");
	if(eventStatus & 0x02) Serial.println(" EM7180 Error");
	if(eventStatus & 0x04) Serial.println(" EM7180 new quaternion result");
	if(eventStatus & 0x08) Serial.println(" EM7180 new mag result");
	if(eventStatus & 0x10) Serial.println(" EM7180 new accel result");
	if(eventStatus & 0x20) Serial.println(" EM7180 new gyro result");
	if(eventStatus & 0x40) Serial.println(" EM7180 new baro result");

	delay(1000); // give some time to read the screen

	// Check sensor status
	uint8_t sensorStatus = ReadByte(EM7180_ADDRESS, EM7180_SensorStatus);

	Serial.print(" EM7180 sensor status = "); Serial.println(sensorStatus);

	if(sensorStatus == 0x00) Serial.println("All sensors OK!");
	if(sensorStatus & 0x01) Serial.println("Magnetometer not acknowledging!");
	if(sensorStatus & 0x02) Serial.println("Accelerometer not acknowledging!");
	if(sensorStatus & 0x04) Serial.println("Gyro not acknowledging!");
	if(sensorStatus & 0x10) Serial.println("Magnetometer ID not recognized!");
	if(sensorStatus & 0x20) Serial.println("Accelerometer ID not recognized!");
	if(sensorStatus & 0x40) Serial.println("Gyro ID not recognized!");

	Serial.print("Actual MagRate = "); Serial.print(ReadByte(EM7180_ADDRESS, EM7180_ActualMagRate)); Serial.println(" Hz");
	Serial.print("Actual AccelRate = "); Serial.print(10*ReadByte(EM7180_ADDRESS, EM7180_ActualAccelRate)); Serial.println(" Hz");
	Serial.print("Actual GyroRate = "); Serial.print(10*ReadByte(EM7180_ADDRESS, EM7180_ActualGyroRate)); Serial.println(" Hz");

	delay(3000); // give some time to read the screen

}


void Update()
{
	// Check event status register, way to check data ready by polling rather than interrupt
	uint8_t eventStatus = ReadByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register

	// Check for errors
	if(eventStatus & 0x02)
	{
		// error detected, what is it?
		uint8_t errorStatus = ReadByte(EM7180_ADDRESS, EM7180_ErrorRegister);
		if(!errorStatus)
		{
			Serial.print(" EM7180 sensor status = "); Serial.println(errorStatus);

			if(errorStatus == 0x11) Serial.print("Magnetometer failure!");
			if(errorStatus == 0x12) Serial.print("Accelerometer failure!");
			if(errorStatus == 0x14) Serial.print("Gyro failure!");
			if(errorStatus == 0x21) Serial.print("Magnetometer initialization failure!");
			if(errorStatus == 0x22) Serial.print("Accelerometer initialization failure!");
			if(errorStatus == 0x24) Serial.print("Gyro initialization failure!");
			if(errorStatus == 0x30) Serial.print("Math error!");
			if(errorStatus == 0x80) Serial.print("Invalid sample rate!");
		}

		// Handle errors ToDo
	}

	if(ReadByte(EM7180_ADDRESS, EM7180_EventStatus) & 0x04)
	{
		// new quaternion data available
		readSENtralQuatData(Quat);
	}

	//Hardware AHRS:
	Yaw   = atan2(2.0f * (Quat[0] * Quat[1] + Quat[3] * Quat[2]), Quat[3] * Quat[3] + Quat[0] * Quat[0] - Quat[1] * Quat[1] - Quat[2] * Quat[2]);
	Pitch = -asin(2.0f * (Quat[0] * Quat[2] - Quat[3] * Quat[1]));
	Roll  = atan2(2.0f * (Quat[3] * Quat[0] + Quat[1] * Quat[2]), Quat[3] * Quat[3] - Quat[0] * Quat[0] - Quat[1] * Quat[1] + Quat[2] * Quat[2]);
	Pitch *= 180.0f / PI;
	Yaw   *= 180.0f / PI;
	Yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	Roll  *= 180.0f / PI;

	// Or define output variable according to the Android system, where heading (0 to 260) is defined by the angle between the y-axis
	// and True North, pitch is rotation about the x-axis (-180 to +180), and roll is rotation about the y-axis (-90 to +90)
	// In this systen, the z-axis is pointing away from Earth, the +y-axis is at the "top" of the device (cellphone) and the +x-axis
	// points toward the right of the device.
	//
    int delt_t = millis() - count;

    if (delt_t > 10)
    { // update LCD once per half-second independent of read rate

    	if(SerialDebug)
    	{
    		Serial.print(Roll, 2);
    		Serial.print('\t');
    		Serial.print(Pitch, 2);
    		Serial.print('\t');
    		Serial.println(Yaw, 2);
    	}

    	count = millis();
    }


}


