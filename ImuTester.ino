#include <Arduino.h>

#include "CI2C.h"
#include "CBMX.cpp"
#include "CBNO055.h"
#include "CLSD.cpp"
#include "CPin.h"

#define I2CPOWER_PIN			48

//CLSD lsdImu;
//CBMX bmxImu;
CBNO055 bno;

CModule *imu = &bno;

CPin i2cpower( "i2cpower", I2CPOWER_PIN, CPin::kDigital, CPin::kOutput );

//The setup function is called once at startup of the sketch
void setup()
{
	Serial.begin( 115200 );

	// Start I2C Device
	I2c.begin();

	// Set 10ms timeout on I2C transmissions
	I2c.timeOut( 50 );

	// 400Khz
	I2c.setSpeed( 1 );

	i2cpower.Reset();
	i2cpower.Write( 0 );
	delay( 10 );
	i2cpower.Write( 1 );

	delay( 1000 );

	imu->Initialize();
}

// The loop function is called in an endless loop
void loop()
{
	imu->Update();
}
