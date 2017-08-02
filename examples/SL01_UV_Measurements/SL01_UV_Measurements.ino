/*************************************************************
	This is an examples for the SW01
	Ambient, UVA and UVB light sensor
	
	You can buy one on our store!
	-----> https://xinabox.cc/SL01/
	
	This example request a UV meausrement to be 
	made by the SL01 and display the data over the Serial
	bus.
	
	The sensor communicates over the I2C Bus.

	------------------------TIPS--------------------------
	Change this line to ----->Wire.begin(2,14); for CW01
	
*************************************************************/
#include <xCore.h>
#include <xSL01.h>

const int DELAY_TIME = 1000;

void setup() {
	// Start the Serial Monitor
	Serial.begin(115200);
	
	// Start the I2C Comunication
	Wire.begin();
	
	// Start the  SL01 Sensor
	SL01.begin();

	// Delayy for Sensor to normalise
	delay(1000);
	
	Serial.println("================================");
	Serial.println(" XINABOX SL01 LUX Measurements  ");
	Serial.println("================================"); 
}

void loop() {
	// Create a variable to store the incoming measurements
	// from SL01 sensor
	float uv;
	uv = 0;

	// Poll Sensor for collect data
	SL01.poll();

	// Request SL01 to return calculated UVB intensity
	uv = SL01.getUVA();
	// Display Data on the Serial monitor
	Serial.print("UVA Intersity: ");
	Serial.print(uv);
	Serial.println(" uW/cm^2");
	
	// Request SL01 to return calculated UVB intensity
	uv = SL01.getUVB();
	// Display Data on the Serial monitor
	Serial.print("UVB Intensity: ");
	Serial.print(uv);
	Serial.println(" uW/cm^2");

	// Request SL01 to return calculated UVB intensity
	uv = SL01.getUVIndex();
	// Display Data on the Serial monitor
	Serial.print("UVB Index: ");
	Serial.println(uv);

	Serial.println();

	delay(DELAY_TIME);
}