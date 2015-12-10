/* EM7180_BMX055_MS5637_t3 Basic Example Code
 by: Kris Winer
 date: December 24, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.

 The EM7180 SENtral sensor hub is not a motion sensor, but rather takes raw sensor data from a variety of motion sensors,
 in this case the BMX0655, and does sensor fusion with quaternions as its output. The SENtral loads firmware from the
 on-board M24512DFMC 512 kbit EEPROM upon startup, configures and manages the sensors on its dedicated master I2C bus,
 and outputs scaled sensor data (accelerations, rotation rates, and magnetic fields) as well as quaternions and
 heading/pitch/roll, if selected.

 This sketch demonstrates basic EM7180 SENtral functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and
 Mahony filter algorithms to compare with the hardware sensor fusion results.
 Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

 This sketch is specifically for the Teensy 3.1 Mini Add-On shield with the EM7180 SENtral sensor hub as master,
 the BMX-055 9-axis motion sensor (accel/gyro/mag) as slave, an MS5637 pressure/temperature sensor, and an M24512DFM
 512kbit (64 kByte) EEPROM as slave all connected via I2C. The SENtral cannot use the pressure data in the sensor fusion
 yet and there is currently no driver for the MS5637 in the SENtral firmware. However, like the MAX21100, the SENtral
 can be toggled into a bypass mode where the pressure sensor (and EEPROM and BMX055) may be read directly by the
 Teensy 3.1 host micrcontroller. If the read rate is infrequent enough (2 Hz is sufficient since pressure and temperature
 do not change very fast), then the sensor fusion rate is not significantly affected.

 This sketch uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 The MS5637 is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 4k7 resistors are on the EM7180+BMX055+MS5637+M24512DFM Mini Add-On board for Teensy 3.1.

 Hardware setup:
 EM7180 Mini Add-On ------- Teensy 3.1
 VDD ---------------------- 3.3V
 SDA ----------------------- 17
 SCL ----------------------- 16
 GND ---------------------- GND
 INT------------------------ 8


 Note: The BMX055 is an I2C sensor and uses the Teensy 3.1 i2c_t3.h Wire library.
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 */

#include "Config.h"

#if IMU_SEL == IMU_BMX


//#include "Wire.h"
#include "CI2C.h"
#include "CBMX.h"

namespace bmx
{
	// BMX055 data sheet http://ae-bst.resource.bosch.com/media/products/dokumente/bmx055/BST-BMX055-DS000-01v2.pdf
	// The BMX055 is a conglomeration of three separate motion sensors packaged together but
	// addressed and communicated with separately by design
	// Accelerometer registers
	#define BMX055_ACC_WHOAMI        0x00   // should return 0xFA
	//#define BMX055_ACC_Reserved    0x01
	#define BMX055_ACC_D_X_LSB       0x02
	#define BMX055_ACC_D_X_MSB       0x03
	#define BMX055_ACC_D_Y_LSB       0x04
	#define BMX055_ACC_D_Y_MSB       0x05
	#define BMX055_ACC_D_Z_LSB       0x06
	#define BMX055_ACC_D_Z_MSB       0x07
	#define BMX055_ACC_D_TEMP        0x08
	#define BMX055_ACC_INT_STATUS_0  0x09
	#define BMX055_ACC_INT_STATUS_1  0x0A
	#define BMX055_ACC_INT_STATUS_2  0x0B
	#define BMX055_ACC_INT_STATUS_3  0x0C
	//#define BMX055_ACC_Reserved    0x0D
	#define BMX055_ACC_FIFO_STATUS   0x0E
	#define BMX055_ACC_PMU_RANGE     0x0F
	#define BMX055_ACC_PMU_BW        0x10
	#define BMX055_ACC_PMU_LPW       0x11
	#define BMX055_ACC_PMU_LOW_POWER 0x12
	#define BMX055_ACC_D_HBW         0x13
	#define BMX055_ACC_BGW_SOFTRESET 0x14
	//#define BMX055_ACC_Reserved    0x15
	#define BMX055_ACC_INT_EN_0      0x16
	#define BMX055_ACC_INT_EN_1      0x17
	#define BMX055_ACC_INT_EN_2      0x18
	#define BMX055_ACC_INT_MAP_0     0x19
	#define BMX055_ACC_INT_MAP_1     0x1A
	#define BMX055_ACC_INT_MAP_2     0x1B
	//#define BMX055_ACC_Reserved    0x1C
	//#define BMX055_ACC_Reserved    0x1D
	#define BMX055_ACC_INT_SRC       0x1E
	//#define BMX055_ACC_Reserved    0x1F
	#define BMX055_ACC_INT_OUT_CTRL  0x20
	#define BMX055_ACC_INT_RST_LATCH 0x21
	#define BMX055_ACC_INT_0         0x22
	#define BMX055_ACC_INT_1         0x23
	#define BMX055_ACC_INT_2         0x24
	#define BMX055_ACC_INT_3         0x25
	#define BMX055_ACC_INT_4         0x26
	#define BMX055_ACC_INT_5         0x27
	#define BMX055_ACC_INT_6         0x28
	#define BMX055_ACC_INT_7         0x29
	#define BMX055_ACC_INT_8         0x2A
	#define BMX055_ACC_INT_9         0x2B
	#define BMX055_ACC_INT_A         0x2C
	#define BMX055_ACC_INT_B         0x2D
	#define BMX055_ACC_INT_C         0x2E
	#define BMX055_ACC_INT_D         0x2F
	#define BMX055_ACC_FIFO_CONFIG_0 0x30
	//#define BMX055_ACC_Reserved    0x31
	#define BMX055_ACC_PMU_SELF_TEST 0x32
	#define BMX055_ACC_TRIM_NVM_CTRL 0x33
	#define BMX055_ACC_BGW_SPI3_WDT  0x34
	//#define BMX055_ACC_Reserved    0x35
	#define BMX055_ACC_OFC_CTRL      0x36
	#define BMX055_ACC_OFC_SETTING   0x37
	#define BMX055_ACC_OFC_OFFSET_X  0x38
	#define BMX055_ACC_OFC_OFFSET_Y  0x39
	#define BMX055_ACC_OFC_OFFSET_Z  0x3A
	#define BMX055_ACC_TRIM_GPO      0x3B
	#define BMX055_ACC_TRIM_GP1      0x3C
	//#define BMX055_ACC_Reserved    0x3D
	#define BMX055_ACC_FIFO_CONFIG_1 0x3E
	#define BMX055_ACC_FIFO_DATA     0x3F

	// BMX055 Gyroscope Registers
	#define BMX055_GYRO_WHOAMI           0x00  // should return 0x0F
	//#define BMX055_GYRO_Reserved       0x01
	#define BMX055_GYRO_RATE_X_LSB       0x02
	#define BMX055_GYRO_RATE_X_MSB       0x03
	#define BMX055_GYRO_RATE_Y_LSB       0x04
	#define BMX055_GYRO_RATE_Y_MSB       0x05
	#define BMX055_GYRO_RATE_Z_LSB       0x06
	#define BMX055_GYRO_RATE_Z_MSB       0x07
	//#define BMX055_GYRO_Reserved       0x08
	#define BMX055_GYRO_INT_STATUS_0  0x09
	#define BMX055_GYRO_INT_STATUS_1  0x0A
	#define BMX055_GYRO_INT_STATUS_2  0x0B
	#define BMX055_GYRO_INT_STATUS_3  0x0C
	//#define BMX055_GYRO_Reserved    0x0D
	#define BMX055_GYRO_FIFO_STATUS   0x0E
	#define BMX055_GYRO_RANGE         0x0F
	#define BMX055_GYRO_BW            0x10
	#define BMX055_GYRO_LPM1          0x11
	#define BMX055_GYRO_LPM2          0x12
	#define BMX055_GYRO_RATE_HBW      0x13
	#define BMX055_GYRO_BGW_SOFTRESET 0x14
	#define BMX055_GYRO_INT_EN_0      0x15
	#define BMX055_GYRO_INT_EN_1      0x16
	#define BMX055_GYRO_INT_MAP_0     0x17
	#define BMX055_GYRO_INT_MAP_1     0x18
	#define BMX055_GYRO_INT_MAP_2     0x19
	#define BMX055_GYRO_INT_SRC_1     0x1A
	#define BMX055_GYRO_INT_SRC_2     0x1B
	#define BMX055_GYRO_INT_SRC_3     0x1C
	//#define BMX055_GYRO_Reserved    0x1D
	#define BMX055_GYRO_FIFO_EN       0x1E
	//#define BMX055_GYRO_Reserved    0x1F
	//#define BMX055_GYRO_Reserved    0x20
	#define BMX055_GYRO_INT_RST_LATCH 0x21
	#define BMX055_GYRO_HIGH_TH_X     0x22
	#define BMX055_GYRO_HIGH_DUR_X    0x23
	#define BMX055_GYRO_HIGH_TH_Y     0x24
	#define BMX055_GYRO_HIGH_DUR_Y    0x25
	#define BMX055_GYRO_HIGH_TH_Z     0x26
	#define BMX055_GYRO_HIGH_DUR_Z    0x27
	//#define BMX055_GYRO_Reserved    0x28
	//#define BMX055_GYRO_Reserved    0x29
	//#define BMX055_GYRO_Reserved    0x2A
	#define BMX055_GYRO_SOC           0x31
	#define BMX055_GYRO_A_FOC         0x32
	#define BMX055_GYRO_TRIM_NVM_CTRL 0x33
	#define BMX055_GYRO_BGW_SPI3_WDT  0x34
	//#define BMX055_GYRO_Reserved    0x35
	#define BMX055_GYRO_OFC1          0x36
	#define BMX055_GYRO_OFC2          0x37
	#define BMX055_GYRO_OFC3          0x38
	#define BMX055_GYRO_OFC4          0x39
	#define BMX055_GYRO_TRIM_GP0      0x3A
	#define BMX055_GYRO_TRIM_GP1      0x3B
	#define BMX055_GYRO_BIST          0x3C
	#define BMX055_GYRO_FIFO_CONFIG_0 0x3D
	#define BMX055_GYRO_FIFO_CONFIG_1 0x3E

	// BMX055 magnetometer registers
	#define BMX055_MAG_WHOAMI         0x40  // should return 0x32
	#define BMX055_MAG_Reserved       0x41
	#define BMX055_MAG_XOUT_LSB       0x42
	#define BMX055_MAG_XOUT_MSB       0x43
	#define BMX055_MAG_YOUT_LSB       0x44
	#define BMX055_MAG_YOUT_MSB       0x45
	#define BMX055_MAG_ZOUT_LSB       0x46
	#define BMX055_MAG_ZOUT_MSB       0x47
	#define BMX055_MAG_ROUT_LSB       0x48
	#define BMX055_MAG_ROUT_MSB       0x49
	#define BMX055_MAG_INT_STATUS     0x4A
	#define BMX055_MAG_PWR_CNTL1      0x4B
	#define BMX055_MAG_PWR_CNTL2      0x4C
	#define BMX055_MAG_INT_EN_1       0x4D
	#define BMX055_MAG_INT_EN_2       0x4E
	#define BMX055_MAG_LOW_THS        0x4F
	#define BMX055_MAG_HIGH_THS       0x50
	#define BMX055_MAG_REP_XY         0x51
	#define BMX055_MAG_REP_Z          0x52
	/* Trim Extended Registers */
	#define BMM050_DIG_X1             0x5D // needed for magnetic field calculation
	#define BMM050_DIG_Y1             0x5E
	#define BMM050_DIG_Z4_LSB         0x62
	#define BMM050_DIG_Z4_MSB         0x63
	#define BMM050_DIG_X2             0x64
	#define BMM050_DIG_Y2             0x65
	#define BMM050_DIG_Z2_LSB         0x68
	#define BMM050_DIG_Z2_MSB         0x69
	#define BMM050_DIG_Z1_LSB         0x6A
	#define BMM050_DIG_Z1_MSB         0x6B
	#define BMM050_DIG_XYZ1_LSB       0x6C
	#define BMM050_DIG_XYZ1_MSB       0x6D
	#define BMM050_DIG_Z3_LSB         0x6E
	#define BMM050_DIG_Z3_MSB         0x6F
	#define BMM050_DIG_XY2            0x70
	#define BMM050_DIG_XY1            0x71

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
	#define EM7180_ErrorRegister      0x50
	#define EM7180_AlgorithmControl   0x54
	#define EM7180_MagRate            0x55
	#define EM7180_AccelRate          0x56
	#define EM7180_GyroRate           0x57
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

	// Using the Teensy Mini Add-On board, BMX055 SDO1 = SDO2 = CSB3 = GND as designed
	// Seven-bit BMX055 device addresses are ACC = 0x18, GYRO = 0x68, MAG = 0x10
	#define BMX055_ACC_ADDRESS  0x18   // Address of BMX055 accelerometer
	#define BMX055_GYRO_ADDRESS 0x68   // Address of BMX055 gyroscope
	#define BMX055_MAG_ADDRESS  0x10   // Address of BMX055 magnetometer
	#define MS5637_ADDRESS      0x76   // Address of MS5637 altimeter
	#define EM7180_ADDRESS      0x28   // Address of the EM7180 SENtral sensor hub
	#define M24512DFM_DATA_ADDRESS   0x50   // Address of the 500 page M24512DFM EEPROM data buffer, 1024 bits (128 8-bit bytes) per page
	#define M24512DFM_IDPAGE_ADDRESS 0x58   // Address of the single M24512DFM lockable EEPROM ID page

	#define SerialDebug true  // set to true to get Serial output for debugging

	// Set initial input parameters
	// define X055 ACC full scale options
	#define AFS_2G  0x03
	#define AFS_4G  0x05
	#define AFS_8G  0x08
	#define AFS_16G 0x0C

	enum ACCBW {    // define BMX055 accelerometer bandwidths
	  ABW_8Hz,      // 7.81 Hz,  64 ms update time
	  ABW_16Hz,     // 15.63 Hz, 32 ms update time
	  ABW_31Hz,     // 31.25 Hz, 16 ms update time
	  ABW_63Hz,     // 62.5  Hz,  8 ms update time
	  ABW_125Hz,    // 125   Hz,  4 ms update time
	  ABW_250Hz,    // 250   Hz,  2 ms update time
	  ABW_500Hz,    // 500   Hz,  1 ms update time
	  ABW_100Hz     // 1000  Hz,  0.5 ms update time
	};

	enum Gscale {
	  GFS_2000DPS = 0,
	  GFS_1000DPS,
	  GFS_500DPS,
	  GFS_250DPS,
	  GFS_125DPS
	};

	enum GODRBW {
	  G_2000Hz523Hz = 0, // 2000 Hz ODR and unfiltered (bandwidth 523Hz)
	  G_2000Hz230Hz,
	  G_1000Hz116Hz,
	  G_400Hz47Hz,
	  G_200Hz23Hz,
	  G_100Hz12Hz,
	  G_200Hz64Hz,
	  G_100Hz32Hz  // 100 Hz ODR and 32 Hz bandwidth
	};

	enum MODR {
	  MODR_10Hz = 0,   // 10 Hz ODR
	  MODR_2Hz     ,   // 2 Hz ODR
	  MODR_6Hz     ,   // 6 Hz ODR
	  MODR_8Hz     ,   // 8 Hz ODR
	  MODR_15Hz    ,   // 15 Hz ODR
	  MODR_20Hz    ,   // 20 Hz ODR
	  MODR_25Hz    ,   // 25 Hz ODR
	  MODR_30Hz        // 30 Hz ODR
	};

	enum Mmode {
	  lowPower         = 0,   // rms noise ~1.0 microTesla, 0.17 mA power
	  Regular             ,   // rms noise ~0.6 microTesla, 0.5 mA power
	  enhancedRegular     ,   // rms noise ~0.5 microTesla, 0.8 mA power
	  highAccuracy            // rms noise ~0.3 microTesla, 4.9 mA power
	};

	// MS5637 pressure sensor sample rates
	#define ADC_256  0x00 // define pressure and temperature conversion rates
	#define ADC_512  0x02
	#define ADC_1024 0x04
	#define ADC_2048 0x06
	#define ADC_4096 0x08
	#define ADC_8192 0x0A
	#define ADC_D1   0x40
	#define ADC_D2   0x50

	// Specify sensor full scale
	uint8_t OSR    = ADC_8192;         // set pressure amd temperature oversample rate
	uint8_t Gscale = GFS_125DPS;       // set gyro full scale
	uint8_t GODRBW = G_200Hz23Hz;      // set gyro ODR and bandwidth
	uint8_t Ascale = AFS_2G;           // set accel full scale
	uint8_t ACCBW  = 0x08 & ABW_16Hz;  // Choose bandwidth for accelerometer
	uint8_t Mmode  = Regular;          // Choose magnetometer operation mode
	uint8_t MODR   = MODR_10Hz;        // set magnetometer data rate
	float aRes, gRes, mRes;            // scale resolutions per LSB for the sensors

	// Parameters to hold BMX055 trim values
	signed char   dig_x1;
	signed char   dig_y1;
	signed char   dig_x2;
	signed char   dig_y2;
	uint16_t      dig_z1;
	int16_t       dig_z2;
	int16_t       dig_z3;
	int16_t       dig_z4;
	unsigned char dig_xy1;
	signed char   dig_xy2;
	uint16_t      dig_xyz1;

	// Pin definitions
	int myLed     = 13;  // LED on the Teensy 3.1

	int lastTime = 0;

	// BMX055 variables
	int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
	int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
	int16_t magCount[3];    // Stores the 13/15-bit signed magnetometer sensor output
	float Quat[4] = {0, 0, 0, 0}; // quaternion data register
	float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};  // Bias corrections for gyro, accelerometer, mag
	int16_t tempCount;            // temperature raw count output
	float   temperature;          // Stores the BMX055 internal chip temperature in degrees Celsius
	float SelfTest[6];            // holds results of gyro and accelerometer self test

	float Yaw, Pitch, Roll;

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

	//===================================================================================================================
	//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
	//===================================================================================================================

	void getGres() {
	  switch (Gscale)
	  {
		// Possible gyro scales (and their register bit settings) are:
		// 125 DPS (100), 250 DPS (011), 500 DPS (010), 1000 DPS (001), and 2000 DPS (000).
		case GFS_125DPS:
			  gRes = 124.87/32768.0; // per data sheet, not exactly 125!?
			  break;
		case GFS_250DPS:
			  gRes = 249.75/32768.0;
			  break;
		case GFS_500DPS:
			  gRes = 499.5/32768.0;
			  break;
		case GFS_1000DPS:
			  gRes = 999.0/32768.0;
			  break;
		case GFS_2000DPS:
			  gRes = 1998.0/32768.0;
			  break;
	  }
	}

	void getAres() {
	  switch (Ascale)
	  {
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (0011), 4 Gs (0101), 8 Gs (1000), and 16 Gs  (1100).
			// BMX055 ACC data is signed 12 bit
		case AFS_2G:
			  aRes = 2.0/2048.0;
			  break;
		case AFS_4G:
			  aRes = 4.0/2048.0;
			  break;
		case AFS_8G:
			  aRes = 8.0/2048.0;
			  break;
		case AFS_16G:
			  aRes = 16.0/2048.0;
			  break;
	  }
	}

	float uint32_reg_to_float (uint8_t *buf)
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

	void readAccelData(int16_t * destination)
	{
	  uint8_t rawData[6];  // x/y/z accel register data stored here
	  I2c.read(BMX055_ACC_ADDRESS, BMX055_ACC_D_X_LSB, 6, &rawData[0]);       // Read the six raw data registers into data array
	  if((rawData[0] & 0x01) && (rawData[2] & 0x01) && (rawData[4] & 0x01)) {  // Check that all 3 axes have new data
	  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 4;  // Turn the MSB and LSB into a signed 12-bit value
	  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 4;
	  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 4;
	  }
	}

	void readGyroData(int16_t * destination)
	{
	  uint8_t rawData[6];  // x/y/z gyro register data stored here
	  I2c.read(BMX055_GYRO_ADDRESS, BMX055_GYRO_RATE_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
	  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);
	  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]);
	}

	void readMagData(int16_t * magData)
	{
	  int16_t mdata_x = 0, mdata_y = 0, mdata_z = 0, temp = 0;
	  uint16_t data_r = 0;
	  uint8_t rawData[8];  // x/y/z hall magnetic field data, and Hall resistance data
	  I2c.read(BMX055_MAG_ADDRESS, BMX055_MAG_XOUT_LSB, 8, &rawData[0]);  // Read the eight raw data registers sequentially into data array
		if(rawData[6] & 0x01) { // Check if data ready status bit is set
		mdata_x = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 3;  // 13-bit signed integer for x-axis field
		mdata_y = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 3;  // 13-bit signed integer for y-axis field
		mdata_z = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 1;  // 15-bit signed integer for z-axis field
		data_r = (uint16_t) (((uint16_t)rawData[7] << 8) | rawData[6]) >> 2;  // 14-bit unsigned integer for Hall resistance

	   // calculate temperature compensated 16-bit magnetic fields
	   temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14)/(data_r != 0 ? data_r : dig_xyz1))) - ((uint16_t)0x4000)));
	   magData[0] = ((int16_t)((((int32_t)mdata_x) *
					((((((((int32_t)dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
					 (((int32_t)temp) * ((int32_t)(((int16_t)dig_xy1) << 7)))) >> 9) +
				   ((int32_t)0x100000)) * ((int32_t)(((int16_t)dig_x2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
				(((int16_t)dig_x1) << 3);

	   temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14)/(data_r != 0 ? data_r : dig_xyz1))) - ((uint16_t)0x4000)));
	   magData[1] = ((int16_t)((((int32_t)mdata_y) *
					((((((((int32_t)dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
					 (((int32_t)temp) * ((int32_t)(((int16_t)dig_xy1) << 7)))) >> 9) +
					   ((int32_t)0x100000)) * ((int32_t)(((int16_t)dig_y2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
				(((int16_t)dig_y1) << 3);
	   magData[2] = (((((int32_t)(mdata_z - dig_z4)) << 15) - ((((int32_t)dig_z3) * ((int32_t)(((int16_t)data_r) -
		((int16_t)dig_xyz1))))>>2))/(dig_z2 + ((int16_t)(((((int32_t)dig_z1) * ((((int16_t)data_r) << 1)))+(1<<15))>>16))));
		}
	  }

	int16_t readACCTempData()
	{
	  uint8_t c =  ReadByte(BMX055_ACC_ADDRESS, BMX055_ACC_D_TEMP);  // Read the raw data register
	  return ((int16_t)((int16_t)c << 8)) >> 8 ;  // Turn the byte into a signed 8-bit integer
	}




	void trimBMX055()  // get trim values for magnetometer sensitivity
	{
	  uint8_t rawData[2];  //placeholder for 2-byte trim data
	  dig_x1 = ReadByte(BMX055_ACC_ADDRESS, BMM050_DIG_X1);
	  dig_x2 = ReadByte(BMX055_ACC_ADDRESS, BMM050_DIG_X2);
	  dig_y1 = ReadByte(BMX055_ACC_ADDRESS, BMM050_DIG_Y1);
	  dig_y2 = ReadByte(BMX055_ACC_ADDRESS, BMM050_DIG_Y2);
	  dig_xy1 = ReadByte(BMX055_ACC_ADDRESS, BMM050_DIG_XY1);
	  dig_xy2 = ReadByte(BMX055_ACC_ADDRESS, BMM050_DIG_XY2);
		I2c.read(BMX055_MAG_ADDRESS, BMM050_DIG_Z1_LSB, 2, &rawData[0]);
	  dig_z1 = (uint16_t) (((uint16_t)rawData[1] << 8) | rawData[0]);
		I2c.read(BMX055_MAG_ADDRESS, BMM050_DIG_Z2_LSB, 2, &rawData[0]);
	  dig_z2 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
		I2c.read(BMX055_MAG_ADDRESS, BMM050_DIG_Z3_LSB, 2, &rawData[0]);
	  dig_z3 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
		I2c.read(BMX055_MAG_ADDRESS, BMM050_DIG_Z4_LSB, 2, &rawData[0]);
	  dig_z4 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
		I2c.read(BMX055_MAG_ADDRESS, BMM050_DIG_XYZ1_LSB, 2, &rawData[0]);
	  dig_xyz1 = (uint16_t) (((uint16_t)rawData[1] << 8) | rawData[0]);
	}


	void initBMX055()
	{
	   // start with all sensors in default mode with all registers reset
	   I2c.write(BMX055_ACC_ADDRESS,  BMX055_ACC_BGW_SOFTRESET, 0xB6);  // reset accelerometer
	   delay(1000); // Wait for all registers to reset

	   // Configure accelerometer
	   I2c.write(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_RANGE, Ascale & 0x0F); // Set accelerometer full range
	   I2c.write(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_BW, ACCBW & 0x0F);     // Set accelerometer bandwidth
	   I2c.write(BMX055_ACC_ADDRESS, BMX055_ACC_D_HBW, 0x00);              // Use filtered data

	//   I2c.write(BMX055_ACC_ADDRESS, BMX055_ACC_INT_EN_1, 0x10);           // Enable ACC data ready interrupt
	//   I2c.write(BMX055_ACC_ADDRESS, BMX055_ACC_INT_OUT_CTRL, 0x04);       // Set interrupts push-pull, active high for INT1 and INT2
	//   I2c.write(BMX055_ACC_ADDRESS, BMX055_ACC_INT_MAP_1, 0x02);        // Define INT1 (intACC1) as ACC data ready interrupt
	//   I2c.write(BMX055_ACC_ADDRESS, BMX055_ACC_INT_MAP_1, 0x80);          // Define INT2 (intACC2) as ACC data ready interrupt

	//   I2c.write(BMX055_ACC_ADDRESS, BMX055_ACC_BGW_SPI3_WDT, 0x06);       // Set watchdog timer for 50 ms

	 // Configure Gyro
	 // start by resetting gyro, better not since it ends up in sleep mode?!
	// I2c.write(BMX055_GYRO_ADDRESS, BMX055_GYRO_BGW_SOFTRESET, 0xB6); // reset gyro
	// delay(100);
	 // Three power modes, 0x00 Normal,
	 // set bit 7 to 1 for suspend mode, set bit 5 to 1 for deep suspend mode
	 // sleep duration in fast-power up from suspend mode is set by bits 1 - 3
	 // 000 for 2 ms, 111 for 20 ms, etc.
	//  I2c.write(BMX055_GYRO_ADDRESS, BMX055_GYRO_LPM1, 0x00);  // set GYRO normal mode
	//  set GYRO sleep duration for fast power-up mode to 20 ms, for duty cycle of 50%
	//  I2c.write(BMX055_ACC_ADDRESS, BMX055_GYRO_LPM1, 0x0E);
	 // set bit 7 to 1 for fast-power-up mode,  gyro goes quickly to normal mode upon wake up
	// can set external wake-up interrupts on bits 5 and 4
	// auto-sleep wake duration set in bits 2-0, 001 4 ms, 111 40 ms
	//  I2c.write(BMX055_GYRO_ADDRESS, BMX055_GYRO_LPM2, 0x00);  // set GYRO normal mode
	// set gyro to fast wake up mode, will sleep for 20 ms then run normally for 20 ms
	// and collect data for an effective ODR of 50 Hz, other duty cycles are possible but there
	// is a minimum wake duration determined by the bandwidth duration, e.g.,  > 10 ms for 23Hz gyro bandwidth
	//  I2c.write(BMX055_ACC_ADDRESS, BMX055_GYRO_LPM2, 0x87);

	 I2c.write((uint8_t)BMX055_GYRO_ADDRESS, (uint8_t)BMX055_GYRO_RANGE, Gscale);  // set GYRO FS range
	 I2c.write((uint8_t)BMX055_GYRO_ADDRESS, (uint8_t)BMX055_GYRO_BW, GODRBW);     // set GYRO ODR and Bandwidth

	// I2c.write(BMX055_GYRO_ADDRESS, BMX055_GYRO_INT_EN_0, 0x80);  // enable data ready interrupt
	// I2c.write(BMX055_GYRO_ADDRESS, BMX055_GYRO_INT_EN_1, 0x04);  // select push-pull, active high interrupts
	// I2c.write(BMX055_GYRO_ADDRESS, BMX055_GYRO_INT_MAP_1, 0x80); // select INT3 (intGYRO1) as GYRO data ready interrupt

	// I2c.write(BMX055_GYRO_ADDRESS, BMX055_GYRO_BGW_SPI3_WDT, 0x06); // Enable watchdog timer for I2C with 50 ms window


	// Configure magnetometer
	I2c.write(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x82);  // Softreset magnetometer, ends up in sleep mode
	delay(100);
	I2c.write(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x01); // Wake up magnetometer
	delay(100);

	I2c.write(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MODR << 3); // Normal mode
	//I2c.write(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MODR << 3 | 0x02); // Forced mode

	//I2c.write(BMX055_MAG_ADDRESS, BMX055_MAG_INT_EN_2, 0x84); // Enable data ready pin interrupt, active high

	// Set up four standard configurations for the magnetometer
	  switch (Mmode)
	  {
		case lowPower:
			 // Low-power
			  I2c.write(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x01);  // 3 repetitions (oversampling)
			  I2c.write(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x02);  // 3 repetitions (oversampling)
			  break;
		case Regular:
			  // Regular
			  I2c.write(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x04);  //  9 repetitions (oversampling)
			  I2c.write(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x16);  // 15 repetitions (oversampling)
			  break;
		case enhancedRegular:
			  // Enhanced Regular
			  I2c.write(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x07);  // 15 repetitions (oversampling)
			  I2c.write(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x22);  // 27 repetitions (oversampling)
			  break;
		case highAccuracy:
			  // High Accuracy
			  I2c.write(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x17);  // 47 repetitions (oversampling)
			  I2c.write(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x51);  // 83 repetitions (oversampling)
			  break;
	  }
	}

	void fastcompaccelBMX055(float * dest1)
	{
	  I2c.write(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x80); // set all accel offset compensation registers to zero
	  I2c.write(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_SETTING, 0x20);  // set offset targets to 0, 0, and +1 g for x, y, z axes
	  I2c.write(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x20); // calculate x-axis offset

	  byte c = ReadByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
	  while(!(c & 0x10)) {   // check if fast calibration complete
	  c = ReadByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
	  delay(10);
	}
	  I2c.write(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x40); // calculate y-axis offset

	  c = ReadByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
	  while(!(c & 0x10)) {   // check if fast calibration complete
	  c = ReadByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
	  delay(10);
	}
	  I2c.write(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x60); // calculate z-axis offset

	  c = ReadByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
	  while(!(c & 0x10)) {   // check if fast calibration complete
	  c = ReadByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
	  delay(10);
	}

	  int8_t compx = ReadByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_X);
	  int8_t compy = ReadByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_Y);
	  int8_t compz = ReadByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_Z);

	  dest1[0] = (float) compx/128.; // accleration bias in g
	  dest1[1] = (float) compy/128.; // accleration bias in g
	  dest1[2] = (float) compz/128.; // accleration bias in g
	}

	/*
	// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
	// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
	void accelgyrocalBMX055(float * dest1, float * dest2)
	{
	  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	  uint16_t samples, ii;

	  Serial.println("Calibrating gyro...");

	  // First get gyro bias
	  byte c = ReadByte(BMX055G_ADDRESS, BMX055G_CTRL_REG5_G);
	  I2c.write(BMX055G_ADDRESS, BMX055G_CTRL_REG5_G, c | 0x40);     // Enable gyro FIFO
	  delay(200);                                                       // Wait for change to take effect
	  I2c.write(BMX055G_ADDRESS, BMX055G_FIFO_CTRL_REG_G, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
	  delay(1000);  // delay 1000 milliseconds to collect FIFO samples

	  samples = (ReadByte(BMX055G_ADDRESS, BMX055G_FIFO_SRC_REG_G) & 0x1F); // Read number of stored samples

	  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
		int16_t gyro_temp[3] = {0, 0, 0};
		I2c.read(BMX055G_ADDRESS, BMX055G_OUT_X_L_G, 6, &data[0]);
		gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
		gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
		gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

		gyro_bias[0] += (int32_t) gyro_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		gyro_bias[1] += (int32_t) gyro_temp[1];
		gyro_bias[2] += (int32_t) gyro_temp[2];
	  }

	  gyro_bias[0] /= samples; // average the data
	  gyro_bias[1] /= samples;
	  gyro_bias[2] /= samples;

	  dest1[0] = (float)gyro_bias[0]*gRes;  // Properly scale the data to get deg/s
	  dest1[1] = (float)gyro_bias[1]*gRes;
	  dest1[2] = (float)gyro_bias[2]*gRes;

	  c = ReadByte(BMX055G_ADDRESS, BMX055G_CTRL_REG5_G);
	  I2c.write(BMX055G_ADDRESS, BMX055G_CTRL_REG5_G, c & ~0x40);   //Disable gyro FIFO
	  delay(200);
	  I2c.write(BMX055G_ADDRESS, BMX055G_FIFO_CTRL_REG_G, 0x00);  // Enable gyro bypass mode

	   Serial.println("Calibrating accel...");

	  // now get the accelerometer bias
	  c = ReadByte(BMX055XM_ADDRESS, BMX055XM_CTRL_REG0_XM);
	  I2c.write(BMX055XM_ADDRESS, BMX055XM_CTRL_REG0_XM, c | 0x40);     // Enable gyro FIFO
	  delay(200);                                                       // Wait for change to take effect
	  I2c.write(BMX055XM_ADDRESS, BMX055XM_FIFO_CTRL_REG, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
	  delay(1000);  // delay 1000 milliseconds to collect FIFO samples

	  samples = (ReadByte(BMX055XM_ADDRESS, BMX055XM_FIFO_SRC_REG) & 0x1F); // Read number of stored samples

	  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
		int16_t accel_temp[3] = {0, 0, 0};
		I2c.read(BMX055XM_ADDRESS, BMX055XM_OUT_X_L_A, 6, &data[0]);
		accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
		accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
	  }

	  accel_bias[0] /= samples; // average the data
	  accel_bias[1] /= samples;
	  accel_bias[2] /= samples;

	  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) (1.0/aRes);}  // Remove gravity from the z-axis accelerometer bias calculation
	  else {accel_bias[2] += (int32_t) (1.0/aRes);}

	  dest2[0] = (float)accel_bias[0]*aRes;  // Properly scale the data to get g
	  dest2[1] = (float)accel_bias[1]*aRes;
	  dest2[2] = (float)accel_bias[2]*aRes;

	  c = ReadByte(BMX055XM_ADDRESS, BMX055XM_CTRL_REG0_XM);
	  I2c.write(BMX055XM_ADDRESS, BMX055XM_CTRL_REG0_XM, c & ~0x40);   //Disable accel FIFO
	  delay(200);
	  I2c.write(BMX055XM_ADDRESS, BMX055XM_FIFO_CTRL_REG, 0x00);  // Enable accel bypass mode
	}
	*/
	void magcalBMX055(float * dest1)
	{
	  uint16_t ii = 0, sample_count = 0;
	  int32_t mag_bias[3] = {0, 0, 0};
	  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

	  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
	  delay(4000);

	   sample_count = 128;
	   for(ii = 0; ii < sample_count; ii++) {
		int16_t mag_temp[3] = {0, 0, 0};
		readMagData(mag_temp);
		for (int jj = 0; jj < 3; jj++) {
		  if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
		  if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		delay(105);  // at 10 Hz ODR, new mag data is available every 100 ms
	   }

	//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
	//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
	//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

		mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
		mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
		mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

		dest1[0] = (float) mag_bias[0]*mRes;  // save mag biases in G for main program
		dest1[1] = (float) mag_bias[1]*mRes;
		dest1[2] = (float) mag_bias[2]*mRes;

	 /* //write biases to accelerometermagnetometer offset registers as counts);
	  I2c.write(BMX055M_ADDRESS, BMX055M_OFFSET_X_REG_L_M, (int16_t) mag_bias[0]  & 0xFF);
	  I2c.write(BMX055M_ADDRESS, BMX055M_OFFSET_X_REG_H_M, ((int16_t)mag_bias[0] >> 8) & 0xFF);
	  I2c.write(BMX055M_ADDRESS, BMX055M_OFFSET_Y_REG_L_M, (int16_t) mag_bias[1] & 0xFF);
	  I2c.write(BMX055M_ADDRESS, BMX055M_OFFSET_Y_REG_H_M, ((int16_t)mag_bias[1] >> 8) & 0xFF);
	  I2c.write(BMX055M_ADDRESS, BMX055M_OFFSET_Z_REG_L_M, (int16_t) mag_bias[2] & 0xFF);
	  I2c.write(BMX055M_ADDRESS, BMX055M_OFFSET_Z_REG_H_M, ((int16_t)mag_bias[2] >> 8) & 0xFF);
	 */
	   Serial.println("Mag Calibration done!");
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

		delay(1000); // give some time to read the screen

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

		// Set up the SENtral as sensor bus in normal operating mode
		// Enter EM7180 initialized state
		I2c.write(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers
		I2c.write(EM7180_ADDRESS, EM7180_PassThruControl, 0x00); // make sure pass through mode is off

		// Set accel/gyro/mage desired ODR rates
		I2c.write(EM7180_ADDRESS, EM7180_QRateDivisor, 0x02); // 100 Hz
		I2c.write(EM7180_ADDRESS, EM7180_MagRate, 0x1E); // 30 Hz
		I2c.write(EM7180_ADDRESS, EM7180_AccelRate, 0x0A); // 100/10 Hz
		I2c.write(EM7180_ADDRESS, EM7180_GyroRate, 0x14); // 200/10 Hz

		// Configure operating mode
		I2c.write(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data

		// Enable interrupt to host upon certain events
		// choose interrupts when quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
		I2c.write(EM7180_ADDRESS, EM7180_EnableEvents, 0x07);

		// Enable EM7180 run mode
		I2c.write(EM7180_ADDRESS, EM7180_HostControl, 0x01); // set SENtral in normal run mode
		delay(100);

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

		delay(1000); // give some time to read the screen

		// Check sensor status
		uint8_t sensorStatus = ReadByte(EM7180_ADDRESS, EM7180_SensorStatus);
		Serial.print(" EM7180 sensor status = "); Serial.println(sensorStatus);

		if(sensorStatus & 0x01) Serial.print("Magnetometer not acknowledging!");
		if(sensorStatus & 0x02) Serial.print("Accelerometer not acknowledging!");
		if(sensorStatus & 0x04) Serial.print("Gyro not acknowledging!");
		if(sensorStatus & 0x10) Serial.print("Magnetometer ID not recognized!");
		if(sensorStatus & 0x20) Serial.print("Accelerometer ID not recognized!");
		if(sensorStatus & 0x40) Serial.print("Gyro ID not recognized!");

		Serial.print("Actual MagRate = "); Serial.print(ReadByte(EM7180_ADDRESS, EM7180_ActualMagRate)); Serial.println(" Hz");
		Serial.print("Actual AccelRate = "); Serial.print(10*ReadByte(EM7180_ADDRESS, EM7180_ActualAccelRate)); Serial.println(" Hz");
		Serial.print("Actual GyroRate = "); Serial.print(10*ReadByte(EM7180_ADDRESS, EM7180_ActualGyroRate)); Serial.println(" Hz");

		delay(1000); // give some time to read the screen
	}


	void Update()
	{
	  // Check event status register, way to chech data ready by polling rather than interrupt
	  uint8_t eventStatus = ReadByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register

	   // Check for errors
	  if(eventStatus & 0x02) { // error detected, what is it?

	  uint8_t errorStatus = ReadByte(EM7180_ADDRESS, EM7180_ErrorRegister);
	  if(!errorStatus) {
	  Serial.print(" EM7180 sensor status = "); Serial.println(errorStatus);
	  if(errorStatus & 0x11) Serial.print("Magnetometer failure!");
	  if(errorStatus & 0x12) Serial.print("Accelerometer failure!");
	  if(errorStatus & 0x14) Serial.print("Gyro failure!");
	  if(errorStatus & 0x21) Serial.print("Magnetometer initialization failure!");
	  if(errorStatus & 0x22) Serial.print("Accelerometer initialization failure!");
	  if(errorStatus & 0x24) Serial.print("Gyro initialization failure!");
	  if(errorStatus & 0x30) Serial.print("Math error!");
	  if(errorStatus & 0x80) Serial.print("Invalid sample rate!");
	  }

	  // Handle errors ToDo

	  }




		if(ReadByte(EM7180_ADDRESS, EM7180_EventStatus) & 0x04)
		{ // new quaternion data available
		readSENtralQuatData(Quat);
	   }

		// Serial print and/or display at 0.5 s rate independent of data rates
		int delt_t = millis() - lastTime;

		if (delt_t > 500)
		{

			if(SerialDebug)
			{

				Serial.println("Hardware quaternions:");
				Serial.print("Q0 = "); Serial.print(Quat[3]);
				Serial.print(" Qx = "); Serial.print(Quat[0]);
				Serial.print(" Qy = "); Serial.print(Quat[1]);
				Serial.print(" Qz = "); Serial.println(Quat[2]);
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

			if(SerialDebug)
			{
				Serial.print(Roll, 2);
				Serial.print('\t');
				Serial.print(Pitch, 2);
				Serial.print('\t');
				Serial.println(Yaw, 2);
			}

			lastTime = millis();
		}
	}
}


void CBMX::Initialize()
{
	bmx::Initialize();
}

void CBMX::Update()
{
	bmx::Update();
}

#endif

