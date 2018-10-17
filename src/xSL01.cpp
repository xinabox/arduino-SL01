/*
	This is a library for the SL01 Digital Light Sensor
	breakout board.
	The board uses I2C for communication.
	
	The board communicates with two I2C devices:
	* 	VEML6075  
	*	TSL4531
	
	Data Sheets:
	VEML6075	- http://www.vishay.com/docs/84304/veml6075.pdf
	VEML6075 Note	- https://www.vishay.com/docs/84339/designingveml6075.pdf
	TSL4531		- http://wisense.in/datasheets/TSL4531.pdf
*/

#include <SL01.h>

/*---Public Function---*/
/********************************************************
 	Constructor
*********************************************************/
xSL01::xSL01(void) {
	VEML6075_I2C_ADDRESS = 0x10;	
	TSL4531_I2C_ADDRESS = 0x29;
	UVAintensity = 0.0;
	UVBintensity = 0.0; 
	UVindex = 0.0;
	LUX = 0.0;
	rawUVA = 0;
	rawUVB = 0;
	UVcomp1 = 0;
	UVcomp2 = 0;
}

/********************************************************
 	Configure Sensor
*********************************************************/
bool xSL01::begin(void){
	writeVEML(VEML6075_REG_CONF, VEML6075_CONF_IT_100, 0x00);
	xCore.write8(TSL4531_I2C_ADDRESS , (TSL4531_WRITE_CMD | TSL4531_REG_CONTROL), TSL4531_CONF_START);
	xCore.write8(TSL4531_I2C_ADDRESS , (TSL4531_WRITE_CMD | TSL4531_REG_CONF), TSL4531_CONF_IT_100);
	poll();
}

/********************************************************
 	Reads Sensor Data
*********************************************************/
void xSL01::poll(void) {
	GET_TSL();
	GET_VEML();
	calculateIndex();
}

/********************************************************
 	Request UVA data in micoWatts/cm^2
*********************************************************/
float xSL01::getUVA(void){
	return UVAintensity;
}

/********************************************************
 	Request UVB data in micoWatts/cm^2
*********************************************************/
float xSL01::getUVB(void){
	return UVBintensity;
}

/********************************************************
 	Request ambeint light in LUX
*********************************************************/
float xSL01::getLUX(void){
	return LUX;
}

/********************************************************
 	Request ambeint light in LUX
*********************************************************/
float xSL01::getUVIndex(void){
	return UVindex;
}

/*---Private Function---*/

/********************************************************
 	Reads Data from TSL4531
*********************************************************/
void xSL01::GET_TSL(void) {
	int multi = 4;
	raw_LUX_H = xCore.read8(TSL4531_I2C_ADDRESS, (TSL4531_WRITE_CMD | TSL4531_REG_DATA_HIGH));
	raw_LUX_L = xCore.read8(TSL4531_I2C_ADDRESS, (TSL4531_WRITE_CMD | TSL4531_REG_DATA_LOW));
	uint16_t data = ((raw_LUX_H <<8)|(raw_LUX_L));
	LUX = multi*((float)data);
}

/********************************************************
 	Reads data from VEML6075
*********************************************************/
void xSL01::GET_VEML(void){	
	readUVdata();	
	
	UVAintensity = (float)rawUVA;
	UVBintensity = (float)rawUVB;
	
	UVAintensity -= (VEML6075_UVA_VIS_COEFF * UVcomp1) - (VEML6075_UVA_IR_COEFF * UVcomp2);
	UVBintensity -= (VEML6075_UVB_VIS_COEFF * UVcomp1) - (VEML6075_UVB_IR_COEFF * UVcomp2);
}

/********************************************************
 	Reads data from VEML6075
*********************************************************/
void xSL01::readUVdata(void){
	rawUVA = readVEML(VEML6075_REG_UVA);
	rawUVB = readVEML(VEML6075_REG_UVB);
	UVcomp1 = readVEML(VEML6075_REG_UVCOMP1);
	UVcomp2 = readVEML(VEML6075_REG_UVCOMP2);
}

/********************************************************
 	Calculation of UV Index
*********************************************************/
void xSL01::calculateIndex(void){
	float UVAComp, UVBComp = 0;
	UVAComp = (UVAintensity * VEML6075_UVA_RESP);
	UVBComp = (UVBintensity * VEML6075_UVB_RESP);
	UVindex = (UVAComp + UVBComp)/2.0;
}

/********************************************************
 	Write Config Data to VEML
*********************************************************/
void xSL01::writeVEML(byte reg, byte lowbyte, byte highbyte){
	Wire.beginTransmission(VEML6075_I2C_ADDRESS);
	Wire.write((uint8_t)reg);
	Wire.write((uint8_t)lowbyte);
	Wire.write((uint8_t)highbyte);
	Wire.endTransmission();
}

/********************************************************
 	Read Data from VEML
*********************************************************/
uint16_t xSL01::readVEML(byte reg){
	uint16_t value = 0;
	uint8_t lowByte = 0;
	uint8_t highByte = 0;

	Wire.beginTransmission(VEML6075_I2C_ADDRESS);
	Wire.write((uint8_t)reg);
	Wire.endTransmission(false);
	Wire.requestFrom(VEML6075_I2C_ADDRESS, 2);    // Read two bytes from slave register address
	if(Wire.available()){
		lowByte = Wire.read();
		highByte = Wire.read();
	}
 	value = (highByte<<8 | lowByte);
	
  return value;
}



