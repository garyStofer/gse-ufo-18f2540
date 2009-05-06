#include <p18cxxx.h>		// includes the processor specific file according to -pxxxx option on the cmd lin
#include <math.h>
#include <delays.h>
#include <timers.h>
#include <usart.h>
#include <adc.h>

#include "types.h"

#ifndef BATCHMODE

// uncomment this to enable Tri-Copter Mixing.
// connector K4 = front motor
//           K2 = rear left motor 
//           K3 = rear right motor 
//           K1 = yaw servo output
// Camera controlling can be used!
//#define TRICOPTER

// Define the desired oscilator frequency , i.e. when 8mhz Xtal  FOSC = 32 , when 16 Mhz Xtal FOSC = 16
//#define FOSC_32
// else
// #define FOSC_16

// To initialixe the flight paramters (both sets) to something reasonamle 
//#define INIT_PARAMS

// for added RX drop out protection define the actual channel number of the transmitter  i.e. : 
//#define MY_RX_CH_NUM 9

// for heavy Gyro Rate filtering -- number of averages has to be validated with Scope, 
// so that there is still some loop idle time in PID_Deleay()
// #define HEAVY_GYRO_AVG 25

// Define this when full PPM train is present at TP3 -- Undefine for wired OR mixing of odd channels  
#define RX_PPM
#endif // !BATCHMODE

// Enable this to use the Accelerator sensors
#define USE_ACCSENS
#define Version	"+.01"

#define Set(S,b) 		((uint8)(S|=(1<<b)))
#define Clear(S,b) 		((uint8)(S&=(~(1<<b))))
#define IsSet(S,b) 		((uint8)((S>>b)&1))
#define IsClear(S,b) 	((uint8)(!(S>>b)&1))
#define Invert(S,b) 	((uint8)(S^=(1<<b)))

// #define NO_SENSORS 1   // for a board with absolutely no Sensors, does mixing of CH only 
// ==============================================
// == External variables
// ==============================================

extern 		uns8 	Flying;
extern		uns8	IGas;
extern		int8 	IRoll,IPitch,IYaw;
extern		uns8	IK5,IK6,IK7;

extern		int8	Roll_Rate_8, Pitch_Rate_8, Yaw_Rate_8;
extern		int8	Roll_Rate_8_prev,Pitch_Rate_8_prev,Yaw_Rate_8_prev;
extern		int16	Pitch_Angle, Roll_Angle,Yaw_Angle;
extern		int16	RollRate10, PitchRate10, YawRate10;
extern		int16	UpDown_Velocity;
extern		int8	LRIntKorr, FBIntKorr;
extern		int8	NeutralLR, NeutralFB, NeutralUD;

extern		int8	NegFact; // general purpose
extern		int16	niltemp1;
extern		int16	niltemp;

extern		int8	Rw,Nw;	// angles
extern  	int8	BatteryVolts; // added by Greg Egan
		
// Variables for barometric sensor PD-controller
extern		uns16	BasePressure;
extern 		uns16   BaseTemp;
extern		uns16	TempCorr;
extern		int8	VBaroComp;
extern  	int16   BaroCompSum;
extern 		uns8	BaroType;

// These variables MUST keep their order!!!
#define rollKp 0
#define rollKi 1
#define rollKd 2
#define rollLim1 3
#define rollLim2 4
#define pitchKp 5
#define pitchKi 6
#define pitchKd 7

extern		int8	RollPropFactor; 	// 01
extern		int8	RollIntFactor;		// 02
extern		int8	RollDiffFactor;		// 03
extern		int8 	RollLimit;			// 04
extern		int8	RollIntLimit;		// 05
extern		int8	PitchPropFactor;	// 06
extern		int8	PitchIntFactor;		// 07
extern		int8	PitchDiffFactor;	// 08
extern		int8 	PitchLimit;			// 09
extern		int8	PitchIntLimit;		// 10
extern		int8	YawPropFactor; 		// 11
extern		int8	YawIntFactor;		// 12
extern		int8	YawDiffFactor;		// 13
extern		int8	YawLimit;			// 14
extern		int8 	YawIntLimit;		// 15
extern		int8	ConfigParam;		// 16
extern		int8 	Interval_time;		// 17    
extern		int8	LowVoltThres;		// 18
extern		int8	CamRollFactor;		// 19  
extern		int8	Unused1;			// 20
extern		int8	LinUDIntFactor;		// 21
extern		int8 	MiddleUD;			// 22
extern		int8	MotorLowRun;		// 23
extern		int8	MiddleLR;			// 24
extern		int8	MiddleFB;			// 25
extern		int8	CamPitchFactor;		// 26
extern		int8	CompassFactor;		// 27
extern		int8	BaroThrottleDiff;	// 28


#define FirstProgReg RollPropFactor
#define	LastProgReg BaroThrottleDiff
// end of "order-block"

// Mask Bits of ConfigParam
#define FlyCrossMode 	(ConfigParam & 0b00000001)
#define FutabaMode		(ConfigParam & 0b00000010)
#define IntegralTest	(ConfigParam & 0b00000100)
#define DoubleRate		(ConfigParam & 0b00001000)
#define NegativePPM		(ConfigParam & 0b00010000)
#define CompassTest		(ConfigParam & 0b00100000)


extern		uns8	M_front,M_left,M_right,M_rear;	// output channels
extern		uns8	MCamRoll,MCamPitch;
extern		int16	Ml, Mr, Mv, Mh;
extern		int16	Rl,Nl,Tl;	// PID output values
extern		char_int Rp,Np,Tp;
extern		int16	Vud;
extern		uns8	BiasZeroCount;
extern 		volatile uns8    Vtmp;

// measured neutral gyro values
// current stick neutral values
extern		int8	RollNeutral, PitchNeutral, YawNeutral;
extern		uns8	ThrNeutral;
extern		uns8	ThrDownCount;
extern		uns16	RollBias, PitchBias, YawBias;
extern		uns16	AbsDirection;	// wanted heading (240 = 360 deg)
extern		int8	CurDeviation;	// deviation from correct heading

// From irq.c 
#define MAX_RX_CH 9
#define MIN_RX_SYNC_PAUSE 1250  // 1250 *4us = 5ms  
#define MAX_RX_SYNC_PAUSE 4000  //  4000 *4us = 16  ms
extern volatile uns8 PWM_Val[6];	// These vars should be volatile but ccx5 compiler doesn't know that keyword
extern volatile uns8 PWM_Pause;
extern volatile uns8 TimeTick1ms;
extern volatile uns8 PID_Delay;
extern volatile uns8 RXCh_Val[MAX_RX_CH];
extern uns8 RX_Num_Channels;		// This stores the number of channels decoded in a frame -- might be useful to outside code 
extern int8 volatile RxSignalOK;	// count number of bad RX frames or loss of signal 	


// Based on: TMR0 at 4us and terminal count of: 0xff == 256 * 4 us = 1.024ms 
// These limits are calculated so that pulses are exactly symmetrical around 1.5ms 
// The Maximum limit can be raised to a upper limit of ~248 for bigger PWM range (input & output)

// NOTE: Raising _Maximum too high will result in TMR0 (+ISR processing time) to overrun TMR2 on channels set to _Maximum which 
//  in effect will leave that channels output signal Permanently high. 

#define	_Minimum	0 	// -100%  == 1.024ms  
#define _Maximum	238 // +100% ==   238 x 4us + 1024 us = 1976 us 
// For asymetrical  PWM around neutral  _Maximum can be raised up to 248.
// Higer than that and the  PWM ISR routines can not handle it!
#define _Neutral_Out    (_Maximum/2)  // 0%  ==  119 x 4us + 1024 = 1500us


#define _Neutral_In 	119  // this is (1500 ms -1024ms )  /4 us  for input channels  = 119 . This is always the case for RX 

// The various defines below should be expressed in terms of % Netral instead 
#define _ThresStop	18
#define _ThresStart	(_ThresStop +2)


// number of samples to average to get the Gyros bias voltage
#define BiasZeroAvgCount 64

// fixed  dt in PID loop
// chaning this causes the deck angles to change in magnitude because dt is not take into account 
// in the math 
#define PID_LOOP_DELAY 10


// check the PPM RX and motor values
#if _Minimum >= _Maximum
#error _Minimum < _Maximum!
#endif
#if _ThresStart <= _ThresStop
#error _ThresStart <= _ThresStop!
#endif
#if (_Maximum < _Neutral_Out)
#error Maximum < _Neutral_Out!
#endif

// ADC Channels
#define ADCPORTCONFIG 0b00001010 // AAAAA
#define ADCBattVoltsChan 	0 
#define ADCRollChan 		1
#define ADCPitchChan 		2
#define ADCVRefChan 		3 
#define ADCYawChan			4
extern int16 ADCquick(uint8);
extern int16 ADC(uint8);
extern void InitADC(void);
// Parameters for UART port

#ifdef FOSC_16
#define _ClkOut		(160/4)		// 16.0 MHz quartz 
#elif defined FOSC_32
#define _ClkOut		(320/4)		
#endif

// only 38Kbd is used because the bootloader is hardwired at 38Kbd 
#define _B38400		(_ClkOut*100000/(4*38400) - 1) 


// EEPROM parameter set addresses
#define _EESet1	0		// first set starts at address 0x00
#define _EESet2	0x20	// second set starts at address 0x20

// Gyro Erection and drift compensation 
#define CompDeadBand 	40		// band within Gyro erection is active 
#define CompDecay	4			// number of counts the angle is driver to 0 within the deadband. i.e speed of correction


// Prototypes
extern		void OutSignals(void);
extern		void GetGyroValues(void);
extern		void CalcGyroValues(void);
extern		void GetVbattValue(void);
extern		void SendComValH(uns8);
extern		void SendComChar(char);
extern		void ShowSetup(uns8);
extern		void ProcessComCommand(void);
extern		void SendComValU(uns8);
extern		void SendComValS(uns8);
extern		void GetEvenValues(void);
extern		void DoProgMode(void);
extern		void InitGlobals(void);
extern  	void PID(void);
extern		void AddUpLRArr(uns8);
extern		void AddUpFBArr(uns8);
extern		void MixAndLimit(void);
extern		void MixAndLimitCam(void);
extern      void Delay_ms( uns8);
extern      void Delay_us( uns16);
extern 	 	void PID_Sleep( void );

extern		void ReadParametersEE(void);
extern		void WriteParametersEE(uns8);
extern		void WriteEE(uns8, int8);
extern		int8 ReadEE(uns8);

extern		void SendLeds(void);
extern		void SwitchLedsOn(uns8);
extern		void SwitchLedsOff(uns8);
extern		void All_Leds_Off( void );
extern 		void ToggleLeds( uns8);

extern		void ReadAccel(void);
extern		uns8 IsLISLactive(void);
extern 		uns8 ReadLISL(uns8);
extern 		uns8 ReadLISLNext(void);
extern		void InitDirection(void);
extern		void GetDirection(void);
extern		void InitAltimeter(void);
extern		void ComputeBaroComp(void);
extern 		void SetMotorsOff( void );
extern		void GetInputCH( void );
extern		void SetDefaultRX_PWM(void);
//extern		uns8 StartBaroADC(uns8);
extern	void BootStart(void);
extern 		t_flags Flags;

