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
#ifndef xSL01_h
#define xSL01_h

// Include File Headers
#include "xCore.h"

// Defines VEML6075 Registers
#define VEML6075_REG_CONF 0x00	  // Configuration register
#define VEML6075_REG_UVA 0x07	  // UVA register
#define VEML6075_REG_UVB 0x09	  // UVB register
#define VEML6075_REG_UVCOMP1 0x0A // Visible compensation register
#define VEML6075_REG_UVCOMP2 0x0B // IR compensation register

// Defines VEML6075
#define VEML6075_CONF_HD_NORM 0x00		// Normal Dynamic Setting
#define VEML6075_CONF_HD_HIGH 0x80		// High Dynamic Setting
#define VEML6075_CONF_UV_TRIG_ONCE 0x04 // Triggers UV Measurement Once
#define VEML6075_CONF_UV_TRIG_NORM 0x00 // Normal Mode Operation
#define VEML6075_CONF_AF_FORCE 0x00		// Normal Mode Enabled
#define VEML6075_CONF_AF_AUTO 0x02		// Active Force Mode Disabled
#define VEML6075_CONF_SD_OFF 0x00		// Power ON
#define VEML6075_CONF_SD_ON 0x01		// Power OFF

#define VEML6075_CONF_IT_50 0x00  // 50ms
#define VEML6075_CONF_IT_100 0x10 // 100ms
#define VEML6075_CONF_IT_200 0x20 // 200ms
#define VEML6075_CONF_IT_400 0x30 // 400ms
#define VEML6075_CONF_IT_800 0x40 // 800ms

// Page 15/22 VEML6075 AppNote 84339
// No teflon (open air)
#define VEML6075_UVA_VIS_COEFF 2.22
#define VEML6075_UVA_IR_COEFF 1.33
#define VEML6075_UVB_VIS_COEFF 2.95
#define VEML6075_UVB_IR_COEFF 1.74
#define VEML6075_UVA_RESP (1.0 / 684.46)
#define VEML6075_UVB_RESP (1.0 / 385.95)

// Defines TSL4531 Registers
#define TSL4531_REG_CONTROL 0x00   // Control Register Address
#define TSL4531_REG_CONF 0x01	   // Configuration Register Address
#define TSL4531_REG_DATA_LOW 0x04  // ADC low byte
#define TSL4531_REG_DATA_HIGH 0x05 // ADC high byte

// Defines TSL4531
#define TSL4531_WRITE_CMD 0x80	   // Command Register. Must write as 1.
#define TSL4531_CONF_PWR_DOWN 0x00 // Power OFF
#define TSL4531_CONF_ONE_RUN 0x02  // Run ONCE then Power OFF
#define TSL4531_CONF_START 0x03	   // Power ON

#define TSL4531_CONF_IT_100 0x02 // 100ms
#define TSL4531_CONF_IT_200 0x01 // 200ms
#define TSL4531_CONF_IT_400 0x00 // 400ms

#define TSL4531_CONF_PSAVE 0x08

class SL01v1 : public xCoreClass
{
public:
	/**
		* Constructor
		* Creates a new instance of Sensor class.
		*/
	SL01v1();

	/*
		* Runs the setup of the sensor.
		* Call this in setup(), before reading any sensor data.
		*
		* @return true if setup was successful.
		*/
	bool begin(void);

	/*
		* Used read all raw sensors data and convert to usefull results.
		* Call this in loop().
		*
		* @return none
		*/
	void poll(void);

	/*
		* Used to get UVA data.
		* Call this in loop().
		*
		* @return UVA. Returns float value of UVA.
		*/
	float getUVA(void);

	/*
		* Used to get UVB data.
		* Call this in loop().
		*
		* @return UVB. Returns float value of UVB.
		*/
	float getUVB(void);

	/*
		* Used to get LUX.
		* Call this in loop().
		*
		* @return LUX. Returns float value of LUX.
		*/
	float getLUX(void);

	/*
		* Used to get UV index.
		* Call this in loop().
		*
		* @return UVindex. Returns float value of UV Index.
		*/
	float uvi(void);

private:
	/*
		* Reads RAW lUX data.
		*
		* @return none
		*/
	void GET_TSL(void);

	/*
		* Reads Raw UV data over I2C.
		*
		* @return none
		*/
	void GET_VEML(void);

	/*
		* Reads UVA and UVB data over I2C.
		*
		* @return none
		*/
	void readUVdata(void);

	/*
		* Reads Raw LUX data over I2C.
		*
		* @return none
		*/
	void readRawLux(void);

	/*
		* Calculates UV Index
		*
		* @return none
		*/
	void calculateIndex(void);

	/*
		* Write Config Data For VEML
		*
		* @return none
		*/
	void writeVEML(byte reg, byte lowbyte, byte highbyte);

	/*
		* read Data from VEML
		*
		* @param reg, register to read from
		* @return reg value.
		*/
	uint16_t readVEML(byte reg);

	// Used to store UV Data
	float UVAintensity;
	float UVBintensity;
	float UVindex;
	uint16_t rawUVA;
	uint16_t rawUVB;
	uint16_t UVcomp1;
	uint16_t UVcomp2;
	uint16_t Dummy;

	// Used to store LUX Data
	float LUX;
	uint8_t raw_LUX_L;
	uint8_t raw_LUX_H;

	// I2C Chip Addresses
	uint8_t VEML6075_I2C_ADDRESS = 0x10;
	uint8_t TSL4531_I2C_ADDRESS = 0x29;
};

#include "si1133_config.h"
/***************************************************************************/
#define X_ORDER_MASK 0x0070
#define Y_ORDER_MASK 0x0007
#define SIGN_MASK 0x0080
#define get_x_order(m) ((m & X_ORDER_MASK) >> 4)
#define get_y_order(m) ((m & Y_ORDER_MASK))
#define get_sign(m) ((m & SIGN_MASK) >> 7)
#define UV_INPUT_FRACTION 15
#define UV_OUTPUT_FRACTION 12
#define UV_NUMCOEFF 2
#define ADC_THRESHOLD 16000
#define INPUT_FRACTION_HIGH 7
#define INPUT_FRACTION_LOW 15
#define LUX_OUTPUT_FRACTION 12
#define NUMCOEFF_LOW 9
#define NUMCOEFF_HIGH 4
/**************************************************************************/
#ifndef SI1133_CONFIG_I2C_DEVICE
#define SI1133_CONFIG_I2C_DEVICE (I2C0)
#endif
#ifndef SI1133_CONFIG_I2C_BUS_ADDRESS
#define SI1133_CONFIG_I2C_BUS_ADDRESS (0xAA)
#endif
#define SI1133_I2C_DEVICE (SI1133_CONFIG_I2C_DEVICE)
#define SI1133_I2C_DEVICE_BUS_ADDRESS (SI1133_CONFIG_I2C_BUS_ADDRESS)
/***************************************************************************/
#define SI1133_OK 0x0000
#define SI1133_ERROR_I2C_TRANSACTION_FAILED 0x0001
#define SI1133_ERROR_SLEEP_FAILED 0x0002
/***************************************************************************/
/***************************************************************************/
typedef struct
{
	uint8_t irq_status;
	int32_t ch0;
	int32_t ch1;
	int32_t ch2;
	int32_t ch3;
} SI1133_Samples_TypeDef;
/***************************************************************************/
typedef struct
{
	int16_t info;
	uint16_t mag;
} SI1133_Coeff_TypeDef;
/***************************************************************************/
typedef struct
{
	SI1133_Coeff_TypeDef coeff_high[4];
	SI1133_Coeff_TypeDef coeff_low[9];
} SI1133_LuxCoeff_TypeDef;
/***************************************************************************/
#define SI1133_REG_PART_ID 0x00
#define SI1133_REG_HW_ID 0x01
#define SI1133_REG_REV_ID 0x02
#define SI1133_REG_HOSTIN0 0x0A
#define SI1133_REG_COMMAND 0x0B
#define SI1133_REG_IRQ_ENABLE 0x0F
#define SI1133_REG_RESPONSE1 0x10
#define SI1133_REG_RESPONSE0 0x11
#define SI1133_REG_IRQ_STATUS 0x12
#define SI1133_REG_HOSTOUT0 0x13
#define SI1133_REG_HOSTOUT1 0x14
#define SI1133_REG_HOSTOUT2 0x15
#define SI1133_REG_HOSTOUT3 0x16
#define SI1133_REG_HOSTOUT4 0x17
#define SI1133_REG_HOSTOUT5 0x18
#define SI1133_REG_HOSTOUT6 0x19
#define SI1133_REG_HOSTOUT7 0x1A
#define SI1133_REG_HOSTOUT8 0x1B
#define SI1133_REG_HOSTOUT9 0x1C
#define SI1133_REG_HOSTOUT10 0x1D
#define SI1133_REG_HOSTOUT11 0x1E
#define SI1133_REG_HOSTOUT12 0x1F
#define SI1133_REG_HOSTOUT13 0x20
#define SI1133_REG_HOSTOUT14 0x21
#define SI1133_REG_HOSTOUT15 0x22
#define SI1133_REG_HOSTOUT16 0x23
#define SI1133_REG_HOSTOUT17 0x24
#define SI1133_REG_HOSTOUT18 0x25
#define SI1133_REG_HOSTOUT19 0x26
#define SI1133_REG_HOSTOUT20 0x27
#define SI1133_REG_HOSTOUT21 0x28
#define SI1133_REG_HOSTOUT22 0x29
#define SI1133_REG_HOSTOUT23 0x2A
#define SI1133_REG_HOSTOUT24 0x2B
#define SI1133_REG_HOSTOUT25 0x2C
/***************************************************************************/
#define SI1133_PARAM_I2C_ADDR 0x00
#define SI1133_PARAM_CH_LIST 0x01
#define SI1133_PARAM_ADCCONFIG0 0x02
#define SI1133_PARAM_ADCSENS0 0x03
#define SI1133_PARAM_ADCPOST0 0x04
#define SI1133_PARAM_MEASCONFIG0 0x05
#define SI1133_PARAM_ADCCONFIG1 0x06
#define SI1133_PARAM_ADCSENS1 0x07
#define SI1133_PARAM_ADCPOST1 0x08
#define SI1133_PARAM_MEASCONFIG1 0x09
#define SI1133_PARAM_ADCCONFIG2 0x0A
#define SI1133_PARAM_ADCSENS2 0x0B
#define SI1133_PARAM_ADCPOST2 0x0C
#define SI1133_PARAM_MEASCONFIG2 0x0D
#define SI1133_PARAM_ADCCONFIG3 0x0E
#define SI1133_PARAM_ADCSENS3 0x0F
#define SI1133_PARAM_ADCPOST3 0x10
#define SI1133_PARAM_MEASCONFIG3 0x11
#define SI1133_PARAM_ADCCONFIG4 0x12
#define SI1133_PARAM_ADCSENS4 0x13
#define SI1133_PARAM_ADCPOST4 0x14
#define SI1133_PARAM_MEASCONFIG4 0x15
#define SI1133_PARAM_ADCCONFIG5 0x16
#define SI1133_PARAM_ADCSENS5 0x17
#define SI1133_PARAM_ADCPOST5 0x18
#define SI1133_PARAM_MEASCONFIG5 0x19
#define SI1133_PARAM_MEASRATE_H 0x1A
#define SI1133_PARAM_MEASRATE_L 0x1B
#define SI1133_PARAM_MEASCOUNT0 0x1C
#define SI1133_PARAM_MEASCOUNT1 0x1D
#define SI1133_PARAM_MEASCOUNT2 0x1E
#define SI1133_PARAM_THRESHOLD0_H 0x25
#define SI1133_PARAM_THRESHOLD0_L 0x26
#define SI1133_PARAM_THRESHOLD1_H 0x27
#define SI1133_PARAM_THRESHOLD1_L 0x28
#define SI1133_PARAM_THRESHOLD2_H 0x29
#define SI1133_PARAM_THRESHOLD2_L 0x2A
#define SI1133_PARAM_BURST 0x2B
/***************************************************************************/
#define SI1133_CMD_RESET_CMD_CTR 0x00
#define SI1133_CMD_RESET 0x01
#define SI1133_CMD_NEW_ADDR 0x02
#define SI1133_CMD_FORCE_CH 0x11
#define SI1133_CMD_PAUSE_CH 0x12
#define SI1133_CMD_START 0x13
#define SI1133_CMD_PARAM_SET 0x80
#define SI1133_CMD_PARAM_QUERY 0x40
/***************************************************************************/
#define SI1133_RSP0_CHIPSTAT_MASK 0xE0
#define SI1133_RSP0_COUNTER_MASK 0x1F
#define SI1133_RSP0_SLEEP 0x20
/***************************************************************************/

class SL01v2
{
public:
	SL01v2();
	uint32_t SI1133_reset(void);
	uint32_t SI1133_resetCmdCtr(void);
	uint32_t SI1133_measurementForce(void);
	uint32_t SI1133_measurementPause(void);
	uint32_t SI1133_measurementStart(void);
	uint32_t SI1133_paramSet(uint8_t address, uint8_t value);
	uint32_t SI1133_paramRead(uint8_t address);
	uint32_t SI1133_init(void);
	uint32_t SI1133_deInit(void);
	uint32_t SI1133_measurementGet(SI1133_Samples_TypeDef *samples);
	int32_t SI1133_getUv(int32_t uv, SI1133_Coeff_TypeDef *uk);
	int32_t SI1133_getLux(int32_t vis_high, int32_t vis_low, int32_t ir, SI1133_LuxCoeff_TypeDef *lk);
	uint32_t SI1133_measureLuxUvi(float *lux, float *uvi);
	uint32_t SI1133_getHardwareID(uint8_t *hardwareID);
	uint32_t SI1133_getMeasurement(float *lux, float *uvi);
	uint32_t SI1133_getIrqStatus(uint8_t *irqStatus);

private:
	uint32_t SI1133_registerRead(uint8_t reg, uint8_t *data);
	uint32_t SI1133_registerWrite(uint8_t reg, uint8_t data);
	uint32_t SI1133_registerBlockRead(uint8_t reg, uint8_t length, uint8_t *data);
	uint32_t SI1133_registerBlockWrite(uint8_t reg, uint8_t length, uint8_t *data);
	uint32_t SI1133_waitUntilSleep(void);
	uint32_t SI1133_sendCmd(uint8_t command);
	int32_t SI1133_calcPolyInner(int32_t input, int8_t fraction, uint16_t mag, int8_t shift);
	int32_t SI1133_calcEvalPoly(int32_t, int32_t, uint8_t, uint8_t, uint8_t, SI1133_Coeff_TypeDef *);
};

class xSL01
{
public:
	xSL01();
	void begin();
	float getUVIndex();
	float getLUX();
	void poll();

private:
	uint8_t version;
	uint8_t checkVersion();
};

#endif
