#include "Types.h"
#define	NULL	0
// Use "Switch" for parachute output 
// attach On/Off switch to /MCLR input 
// #define Switch		PORTAbits.RA4
#define LISL_CS		PORTCbits.RC5
#define LISL_SDA	PORTCbits.RC4
#define LISL_SCL	PORTCbits.RC3
#define LISL_IO		TRISCbits.TRISC4


// the sensor bus lines
#define I2C_SDA		PORTBbits.RB6
#define I2C_DIO		TRISBbits.TRISB6
#define I2C_SCL		PORTBbits.RB7
#define I2C_CIO		TRISBbits.TRISB7

#define	I2C_ACK		0
#define	I2C_NACK	1

// The LEDs and the beeper
#define ON	1
#define OFF	0


#define ADC_CH_UBAT  0b00000011 // select AN0 Roll leaving ADC power on
#define ADC_CH_ROLL  0b00000111 // select AN1 Roll leaving ADC power on
#define ADC_CH_PITCH 0b00001011 // select AN2 Pitch, leaving ADC power on
#define ADC_CH_SPARE 0b00001111 // AN3 TP2 - not used other then diagnostics
#define ADC_CH_YAW   0b00010011 // select AN4 Yaw, , leaving ADC power on


#define LedYellow	LED6
#define LedGreen	LED4
#define	LedBlue		LED2
#define LedRed		LED3
#define LedAUX1		LED5
#define LedAUX2		LED1
#define LedAUX3		LED7

#define LED1	0x01	/* Aux2 */
#define LED2	0x02	/* blue */
#define LED3	0x04	/* red */ 
#define LED4	0x08	/* green */
#define LED5	0x10	/* Aux1 */
#define LED6	0x20	/* yellow */
#define LED7	0x40	/* Aux3 */
#define Beeper	0x80

#define ALL_LEDS_ON		SwitchLedsOn(LedBlue|LedRed|LedGreen|LedYellow)
#define AUX_LEDS_ON		SwitchLedsOn(LedAUX1|LedAUX2|LedAUX3)

#define AUX_LEDS_OFF	SwitchLedsOff(LedAUX1|LedAUX2|LedAUX3)

#define LedRed_ON		SwitchLedsOn(LedRed);
#define LedBlue_ON		SwitchLedsOn(LedBlue);
#define LedGreen_ON		SwitchLedsOn(LedGreen+LedAUX2);
#define LedYellow_ON	SwitchLedsOn(LedYellow);
#define LedAUX1_ON		SwitchLedsOn(LedAUX1);
#define LedAUX2_ON		SwitchLedsOn(LedAUX2);
#define LedAUX3_ON		SwitchLedsOn(LedAUX3);
#define LedRed_OFF		SwitchLedsOff(LedRed);
#define LedBlue_OFF		SwitchLedsOff(LedBlue);
#define LedGreen_OFF	SwitchLedsOff(LedGreen + LedAUX2);
#define LedYellow_OFF	SwitchLedsOff(LedYellow);
#define LedRed_TOG		ToggleLeds(LedRed);
#define LedGreen_TOG	ToggleLeds(LedGreen+LedAUX2);
#define LedYellow_TOG	ToggleLeds(LedYellow);
#define LedBlue_TOG		ToggleLeds(LedBlue);
#define Beeper_OFF		SwitchLedsOff(Beeper);
#define Beeper_ON		SwitchLedsOn(Beeper);
#define Beeper_TOG		ToggleLeds(Beeper);

// compass sensor
#define COMPASS_ADDR	0x42	/* I2C slave address */
#define COMPASS_MAXDEV	30	/* maximum yaw compensation of compass heading */
#define COMPASS_MAX		240	/* means 360 degrees */
#define COMPASS_INVAL	(COMPASS_MAX+15)	/* 15*4 cycles to settle */
#define COMPASS_MIDDLE	10	/* yaw stick neutral dead zone */

// baro (altimeter) sensor
// baro (altimeter) sensor
#define BARO_I2C_ID			0xee
#define BARO_TEMP_BMP085	0x2e
#define BARO_TEMP_SMD500	0x6e
#define BARO_PRESS			0xf4
#define BARO_CTL			0xf4
#define BARO_ADC_MSB		0xf6
#define BARO_ADC_LSB		0xf7
#define BARO_TYPE			0xd0
//#define BARO_ID_SMD500		??
#define BARO_ID_BMP085		0x55

#define THR_DOWNCOUNT	255	// 128 PID-cycles (=3 sec) until current throttle is fixed
#define THR_MIDDLE		10  /* throttle stick dead zone for baro */
#define THR_HOVER		75	/* min throttle stick for alti lock */

	


//extern volatile T_Flags Flags;

#define _NoSignal		Flags.bit0 	// if no valid signal is received
#define _Unused0		Flags.bit2 	//  
#define _Unused1		Flags.bit3 	//
#define _NegIn			Flags.bit4 	// negative signed input (serial.c)
#define _LowBatt		Flags.bit5 	// if Batt voltage is low
#define	_UseLISL		Flags.bit6 	// 1 if LISL Sensor is used
#define	_UseCompass		Flags.bit7	// 1 if compass sensor is enabled
#define _UseBaro		Flags.bit8 	// 1 if baro sensor active
#define _BaroTempRun	Flags.bit9	// 1 if baro temp a/d conversion is running
#define _UseParam1		Flags.bit10	



// LISL-Register mapping
#define	LISL_WHOAMI		(0x0f)
#define	LISL_OFFSET_X	(0x16)
#define	LISL_OFFSET_Y	(0x17)
#define	LISL_OFFSET_Z	(0x18)
#define	LISL_GAIN_X		(0x19)
#define	LISL_GAIN_Y		(0x1A)
#define	LISL_GAIN_Z		(0x1B)
#define	LISL_CTRLREG_1	(0x20)
#define	LISL_CTRLREG_2	(0x21)
#define	LISL_CTRLREG_3	(0x22)
#define	LISL_STATUS		(0x27)
#define LISL_OUTX_L		(0x28)
#define LISL_OUTX_H		(0x29)
#define LISL_OUTY_L		(0x2A)
#define LISL_OUTY_H		(0x2B)
#define LISL_OUTZ_L		(0x2C)
#define LISL_OUTZ_H		(0x2D)
#define LISL_FF_CFG		(0x30)
#define LISL_FF_SRC		(0x31)
#define LISL_FF_ACK		(0x32)
#define LISL_FF_THS_L	(0x34)
#define LISL_FF_THS_H	(0x35)
#define LISL_FF_DUR		(0x36)
#define LISL_DD_CFG		(0x38)
#define LISL_INCR_ADDR	(0x40)
#define LISL_READ		(0x80)
