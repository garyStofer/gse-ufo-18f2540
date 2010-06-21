// Utilities and subroutines

#include "main.h"
#include "bits.h"

void SetMotorsOff( void )
{
	// all motors off -- Cam netral 
	PWM_Val[0] = 0;  
	PWM_Val[1] = 0; 	
	PWM_Val[2] = 0;
	PWM_Val[3] = 0;
	
	PWM_Val[4] = _Neutral_Out;
	PWM_Val[5] = _Neutral_Out;
	PWM_Val[6] = _Neutral_Out;
	PWM_Val[7] = _Neutral_Out;
}
// assignes and converts the eaw RX channel data to logical channel vars
// applies dual rate to yaw & pitch ( 100% or 50%)
void GetInputCH( void )
{	// do channel  assignenment in here 
	
	if (_NoSignal)
		return;
		
	if ( !FutabaMode )
	{
		IGas  = RXCh_Val[0];
		IRoll = RXCh_Val[1]  - _Neutral_In;
		IPitch = RXCh_Val[2] - _Neutral_In;
		IYaw  = RXCh_Val[3]  - _Neutral_In;
	}
	else
	{
		IRoll = RXCh_Val[0]  - _Neutral_In;
		IPitch = RXCh_Val[1] - _Neutral_In;
		IGas  = RXCh_Val[2];
		IYaw  = RXCh_Val[3]  - _Neutral_In;
	}
	
	IK5   = RXCh_Val[4];		
	IK6   = RXCh_Val[5];		
	IK7   = RXCh_Val[6];
	
	if ( DoubleRate )
	{
		IRoll = IRoll/2;
		IPitch = IPitch/2;
	}		
}

// Copies the Motor and servo PWM number to the array for the ISR PWM generation
void OutSignals(void)
{	
	PWM_Val[0] = M_rear;
	PWM_Val[1] = M_left;
	PWM_Val[2] = M_right;
	PWM_Val[3] = M_front;
	PWM_Val[4] = MCamRoll;
	PWM_Val[5] = MCamPitch;
	PWM_Val[6] = RXCh_Val[7];	// Pass through Ch 8 of Radio to RB6 for Camera ON/OFF/trigger servo
	PWM_Val[7] = RXCh_Val[8];	// Pass through Ch 9 of Radio to RB7 for push button trigger

// Also output CH9 trigger button in digital form -- wired to Parachute trigger Q4, i.e. trigger for Cannon USB trigger
#ifndef Scope_PID_idle_time
	if (RXCh_Val[8] > 0x80 )
		PORTCbits.RC0 =1;
	else
		PORTCbits.RC0 =0;	
#endif
	
}

void Delay_ms( uns8 ms_dur)	//Note: can only do delay up to 255 ms 
{
	uns8 t_end;
	t_end = TimeTick1ms + ms_dur; 	 
	while (TimeTick1ms != t_end)
	;
}


void PID_Sleep( void )		// 1.024 ms interrupt counts PID_Delay down 
{
#ifdef Scope_PID_idle_time	
	PORTCbits.RC0 =1;  			// Scope probe to measure the loop idle time
#endif	
	while (PID_Delay  )  	// wait here until delay time is up 
	{ /* wait */ }

	PID_Delay = PID_LOOP_DELAY; 	// reset Delay ( in ms )
#ifdef Scope_PID_idle_time
	PORTCbits.RC0 =0;
#endif	
} 

// Read battery voltage 
// Bit _LowBatt is set if voltage is below threshold
void GetVbattValue(void)
{
	uns16 NewBatteryVolts, Temp;

	NewBatteryVolts = ADC( ADC_CH_UBAT );
	NewBatteryVolts >>= 3;

	Temp = BatteryVolts+NewBatteryVolts+1;
	BatteryVolts = Temp>>1;
	_LowBatt = (BatteryVolts < LowVoltThres) ? 1 :0; 
}


// convert Roll and Pitch gyro values
// using 10-bit A/D conversion.
// Values are ADDED into RollSamples and PitchSamples
void GetGyroValues(void)
{
	static uns16 RollSamples;
	static uns16 PitchSamples ;
	static uns16 YawSamples;
	
#ifdef NO_SENSORS
	return;
#endif


#ifdef HEAVY_GYRO_AVG
{
// 50 itterations leaves just 1ms of idle time in the PID loop
// Heavy averaging of Gyro values to reduce jitter due to vibration peaks
// Possibly remove the highest and lowes samples from the bunch for added performance
	
		Vtmp =HEAVY_GYRO_AVG;
		RollSamples = PitchSamples = YawSamples = 0;
		for (; Vtmp; Vtmp-- )
		{
			RollSamples +=ADC(ADC_CH_ROLL);
			PitchSamples += ADC(ADC_CH_PITCH);
			YawSamples += ADC(ADC_CH_YAW);
		}	
		// average 
		RollSamples  += HEAVY_GYRO_AVG/2;
		RollSamples  /= HEAVY_GYRO_AVG;
		PitchSamples += HEAVY_GYRO_AVG/2;
		PitchSamples /= HEAVY_GYRO_AVG;
		YawSamples   += HEAVY_GYRO_AVG/2;
		YawSamples   /= HEAVY_GYRO_AVG;
}
#else
	RollSamples =ADC(ADC_CH_ROLL);
	PitchSamples = ADC(ADC_CH_PITCH);
	YawSamples = ADC(ADC_CH_YAW);
#endif
	// pre-flight auto-zero mode	
	if( BiasZeroCount > 0 )
	{
		RollBias += RollSamples;	// adding up 10 bit quantities 
		PitchBias += PitchSamples;
		YawBias += YawSamples;

		// hold all angles at 0 until we have  the bias values collected
		Roll_Angle = 0;
		Pitch_Angle = 0;
		Yaw_Angle = 0;
		LRIntKorr = 0;
		FBIntKorr = 0;

		if( BiasZeroCount == 1 )
		{
			RollBias += (BiasZeroAvgCount/2);	// round 
			RollBias /=  BiasZeroAvgCount;
			PitchBias += (BiasZeroAvgCount/2);
			PitchBias /= BiasZeroAvgCount;
			YawBias += (BiasZeroAvgCount/2);
			YawBias /=  BiasZeroAvgCount;
		}

		BiasZeroCount--;
	}
	else
	{
		RollRate10  = RollSamples - RollBias;			// Subtract Gyro Bias, makes +/- Delta Theta  in 10 bit resolution 
		PitchRate10 = PitchSamples -PitchBias;		
		YawRate10   = YawSamples - YawBias;				
	}	
}

// Integrates Gyro rates into angles and applies gyro drift correction from the accel readings
// Angles for 90deg rotation = ~12000 counts in  RollAngle or PitchAngle vars
void CalcGyroValues(void)
{
#ifdef NO_SENSORS
	return;
#endif

	if( BiasZeroCount > 0 )   // No gyro Bias value yet,  Can't fly yet
			return;

// Roll
	Roll_Angle += RollRate10;		// Integrate rate of roll into angle
	Roll_Rate_8 = (char) (RollRate10/4);	// Roll_Rate_8  has Delta Theta of roll axis in 8 bit representation  
 
	
	
	Roll_Angle += LRIntKorr;
	
// Pitch
	Pitch_Angle +=  PitchRate10;		// Integrate rate of roll into angle
	Pitch_Rate_8 = (char)  (PitchRate10/4);	 // Use 8 bit for PD

	// Application of error correction. see Read_Accel() for discussion
	Pitch_Angle += FBIntKorr;

// Yaw
	// 10 bit Samples added up + 8 bit IYaw input from stick 
#ifdef GYRO_150			
	YawRate10 /=2;  	// for 150deg gyro /2,so that YawRate10 for 150 and 300 deg gyros is at the same scale 
#endif
	YawRate10 += IYaw;  // Add in Stick input here so that the Yaw Angle can be driven back to 0 by the yaw control to the motors
	
	Yaw_Angle += YawRate10; // the +- 32K allows for a +- 270 degree variation of the Yaw angle. Since Yaw angle is used to hold heading and the control loop drives it back to 0 this is not a problem 
	Yaw_Rate_8 = (char) (YawRate10/4);	 // use 8 bit for PD

}
static uns8	LedShadow;	
void SendLeds(void)
{
	uns8 tmp;
	/* send LedShadow byte to TPIC */
	LISL_CS = 1;	// CS to 1
	LISL_IO = 0;	// SDA is output
	LISL_SCL = 0;	// because shift is on positive edge
	
	for(tmp=0x80; tmp; tmp >>=1 )	// from MSB to LSB
	{
		
		if( LedShadow & tmp )
			LISL_SDA = 1;
		else
			LISL_SDA = 0;
		
		LISL_SCL = 1;		// clock in
		LISL_SCL = 0;
		
	}

	PORTCbits.RC1 = 1;
	PORTCbits.RC1 = 0;	// latch into drivers
}

void SwitchLedsOn(uns8 x)
{
	LedShadow |= x;
	SendLeds();
}

void SwitchLedsOff(uns8 x)
{
	LedShadow &= ~x;
	SendLeds();
}

void ToggleLeds( uns8 x )
{
	{
	LedShadow ^= x;
	SendLeds();
}	
	
}	

void All_Leds_Off( void )
{
	LedShadow = 0;
	SendLeds();
	
}	


void InitGlobals(void)
{

	M_front =_Minimum;	// stop all motors
	M_left = _Minimum;
	M_right = _Minimum;
	M_rear = _Minimum;

	MCamPitch = _Neutral_Out;
	MCamRoll = _Neutral_Out;


	Roll_Rate_8_prev = 0;
	Pitch_Rate_8_prev = 0;
	Yaw_Rate_8_prev = 0;
	
	Rp.I = 0;
	Np.I = 0;
	Vud = 0;
	VBaroComp = 0;
	BaroCompSum = 0;

	LRIntKorr = 0;
	FBIntKorr = 0;
	Yaw_Angle = 0;
    Roll_Angle = 0;
    Pitch_Angle = 0;
	UpDown_Velocity = 0;
	Flying = 0;
}
