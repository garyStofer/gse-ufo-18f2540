// The globals
#include "types.h"
uns8 	Flying;		// Flag indicating craft is up 
uns8	IGas;		// actual input channel, can only be positive!
int8 	IRoll,IPitch,IYaw;	// actual input channels, 0 = neutral
uns8	IK5;		// actual channel 5 input
uns8	IK6;		// actual channel 6 input
uns8	IK7;		// actual channel 7 input

// PID Vars
int8	Roll_Rate_8,Pitch_Rate_8,Yaw_Rate_8;				// the proportional error
int8	Roll_Rate_8_prev,Pitch_Rate_8_prev,Yaw_Rate_8_prev;	// the derivative error 
int16	Roll_Angle, Pitch_Angle,Yaw_Angle;					// the integral error
int16	RollRate10, PitchRate10,YawRate10;
int16	UpDown_Velocity;
int8	LRIntKorr, FBIntKorr;
int8	NeutralLR, NeutralFB, NeutralUD;


int8	NegFact; // general purpose

int16	niltemp1;
int16	niltemp;
int8	BatteryVolts; 
int8	Rw,Nw;

uns16	BasePressure, BaseTemp, TempCorr;
uns8	BaroType;
int8	VBaroComp;
int16 	BaroCompSum;

#pragma idata params
// Principal quadrocopter parameters - MUST remain in this order
// for block read/write to EEPROM
int8	RollPropFactor		=-12;
int8	RollIntFactor		=-10;
int8	RollDiffFactor		=60;
int8	BaroTempCoeff		=0;
int8	RollIntLimit		=1;
int8	PitchPropFactor		=-12;
int8	PitchIntFactor		=-10;	
int8	PitchDiffFactor		=60;	 
int8	BaroThrottleProp	=0;
int8	PitchIntLimit		=1;
int8	YawPropFactor		=25;
int8	YawIntFactor		=25;
int8	YawDiffFactor		=0;
int8	YawLimit			=30;
int8	YawIntLimit			=1;
int8	ConfigParam			=0b00001000; //  Enable half rate  on yaw, pitch and roll
int8	Interval_time		=4;	// control PWM update interval
int8	LowVoltThres		=43;
int8	CamRollFactor		=10;	 
int8	Unused1		=0;
int8	LinUDIntFactor		=0;
int8	MiddleUD			=0;
int8	MotorLowRun			=30;
int8	MiddleLR			=0;
int8	MiddleFB			=0;
int8	CamPitchFactor		=10;
int8	CompassFactor		=0;
int8	BaroThrottleDiff	=0;
#pragma idata


// End of order requirment

uns16	RollBias, PitchBias, YawBias;


uns16	AbsDirection;	// wanted heading (240 = 360 deg)
int8	CurDeviation;	// deviation from correct heading


uns8	M_front,M_left,M_right,M_rear;	// output channels
uns8	MCamRoll,MCamPitch;
int16	Ml, Mr, Mv, Mh;
int16	Rl,Nl,Tl;	// PID output values
char_int	Rp,Np,Tp;
int16	Vud;

t_flags Flags;

uns8	BiasZeroCount;
int8	RollNeutral, PitchNeutral, YawNeutral;

uns8	ThrNeutral;
uns8	ThrDownCount;
volatile uns8    Vtmp;