#include <Arduino.h>
//#include "CI2C.h"
#include "CLSD.h"
//#include "CBNO.h"

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

	delay( 1000 );

	Initialize();

}

// The loop function is called in an endless loop
void loop()
{
	Update();
}
