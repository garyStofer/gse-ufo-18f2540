// The PID controller algorithm
#include "main.h"
#include "bits.h"
#include "math.h"

//  limit to valid  PWM widths
static uns8 
LimitPWM(int16 l)
{
	if ( l > _Maximum )
		l =  _Maximum ;
		
	if ( l < _Minimum ) 
		l = _Minimum;

	return((uns8)l);
}


// compute the correction adders for the motors
// using the gyro values (PID controller)
// for the axes Roll and Pitch
void PID(void)
{

	static long t;
	
#ifdef NO_SENSORS
return;
#endif

	if(	BiasZeroCount > 0 )  // NO PID while averaging the gyro bias values
		return;
		

// PID controller
// E0 = current gyro error
// E1 = previous gyro error
// Sum(Ex) = integrated gyro error, sinst start of ufo!
// A0 = current correction value
// fx = programmable controller factors
//
// for Roll and Pitch:
//       E0*fP + E1*fD     Sum(Ex)*fI
// A0 = --------------- + ------------
//            16               256


//  ###############  ROLL  #####################
	// The Derivative term  uses the delta of  the rates of roll  between itterations  
	Rl = (Roll_Rate_8_prev - Roll_Rate_8) * (int16) RollDiffFactor;

	// The proportional term 
	Rl += Roll_Rate_8 * (int16) RollPropFactor;
	Rl /= 16;
 
	// Integral part for Roll
	t = (((Roll_Angle/4) * (long) RollIntFactor) / 256);
	t = (t>127) ? 127 : ((t < -127) ? -127 :t);		// limit to 8 bits
	Rl +=  t;
	
#ifdef GYRO_150	// For 150 deg/sec gyros divide by 2 to have the same gain 
	Rl /= 2;
#endif	

	// Add stick signal
	Rl += IRoll;


// ###############  PITCH  #####################
	// Derivative  
	Nl = Pitch_Rate_8_prev - Pitch_Rate_8;
	Nl *=  PitchDiffFactor;

	// Proportional 
	Nl += Pitch_Rate_8   * (int16) PitchPropFactor;
	Nl /= 16;	

	// Integral 
	t = (((Pitch_Angle/4) * (long) PitchIntFactor) / 256);
	t = (t>127) ? 127 : ((t < -127) ? -127 :t);		// limit to 8 bits
	Nl +=  t ;	
	
#ifdef GYRO_150	// For 150 deg/sec gyros divide by 2 to have the same gain 
	Nl /= 2;
#endif	
	// Add  stick signal
	Nl += IPitch;
	
// PID controller for Yaw (Heading Lock)
//       E0*fp + E1*fD     Sum(Ex)*fI
// A0 = --------------- + ------------
//             16              256

// ##############  YAW  ######################
// Apply Integral factor 
// stick input IYaw is added to Gyro YawSamples first  and then integrated into Yaw_Angle 
// so that the angle in the end stays at 0 during a turn  

//Apply  Derivative factor 
	Tl  = (Yaw_Rate_8_prev - Yaw_Rate_8 ) * (int16) YawDiffFactor;

// Apply Proportional factor	
	Tl += (Yaw_Rate_8  * (int16) YawPropFactor);
	Tl /= 16;

	t = ((( Yaw_Angle/4) * (long) YawIntFactor) / 256 );
	t = (t>127) ? 127 : ((t < -127) ? -127 :t);		// limit to 8 bits 
	Tl += t; 
	
// limit Maximum Yaw rate to something slow enough for the human 	
	if( Tl < -YawLimit ) Tl = -YawLimit;
	if( Tl > YawLimit ) Tl = YawLimit;
	
	

}




// mix the PID-results (Rl, Nl and Tl) and the throttle
// on the motors and check for numerical overrun
void MixAndLimit(void)
{
	int16 min;
	int16 A,B;
	
	if(	BiasZeroCount > 0 ) // wait until the gyro Bias is measured
		return;


#ifdef NO_SENSORS
Vud = VBaroComp = 0;
Rl = IRoll;
Nl = IPitch;
Tl = IYaw;
#endif


#ifndef TRICOPTER
	if( FlyCrossMode )
	{	// "Cross" Mode
// mix pitch and roll inputs together at 50% each for a 45deg "X" orientation of flight
// motor names don't make sense anymore as they are front-left, front-righ, rear-left, rear-right.
// PID outputs need to be limited so that they don't fall below the Igas parameter in the subtration 
// and cause an imbalance in the thrust betwen the forward and reverse running motors in term causing 
// an unintened yaw 

		A = (Nl - Rl )/2;  // for CW rotating nmotors 
		B = (Nl + Rl )/2;  // for CCW ro

		if ( A > IGas )
			A = IGas;
		else if ( A < -IGas )
			A = -IGas; 
					
		if (B > IGas )
			B = IGas;
		else if ( B <  -IGas )
			B =  -IGas;

//	PWM_Val[0] = M_rear;  // X-mode Rear-Right
//	PWM_Val[1] = M_left;  // X-mode Rear-Left
//	PWM_Val[2] = M_right; // x_mode Front-Right
//	PWM_Val[3] = M_front; // X-mode Front-Left

		Mh = IGas + B;		//cw Rear-Right
		Ml = IGas + A; 		//ccw Rear-Left
		Mr = IGas - A;		//ccw Front-Right
		Mv = IGas - B;		//cw Front-Left
	
		
		

	}
	else
	{	// "Plus" Mode
		//The normal way 
		// limit so that substration of  Rl / Nl from Igas can't go below 0
		// otherwise we incur an unintened yaw
		
		if ( Rl > IGas )
			Rl = IGas;
		else if ( Rl < -IGas )
			Rl = -IGas; 
					
		if ( Nl > IGas )
			Nl =IGas;
		else if ( Nl <  -IGas )
			Nl =  -IGas;
	
		Ml = IGas - Rl;	// motor Left
		Mr = IGas + Rl; // Motor right
		Mv = IGas - Nl; // Motor front
		Mh = IGas + Nl; // motor rear 
	}

	// Find lowest running motor 
	min = (Mv < Mh)  ? Mv: Mh;
	min = (Ml < min) ? Ml :min;
	min = (Mr < min) ? Mr :min;

		
	// limit Yaw input so that no motor goes below 0 when added/subtracted	
	if ( Tl > min )
		Tl = min;
	
	if (Tl < -min )
		Tl = -min;

	// Mix in the differential power to the motors for yawing 	
	Mv += Tl;
	Mh += Tl;
	Ml -= Tl;
	Mr -= Tl;
	


// Altitude stabilization factor
// This doesn't work because the Z accel alone can not be used to get up down Velocity whe the craft banks and pitches \
// As the Z axes sees less of the 1G whenever it's not perfectly level. This leads to a negative Velocity  in the end which sends
// the craft Skyward ... 
//	Mv += Vud;
//	Mh += Vud;
//	Ml += Vud;
//	Mr += Vud;
//if (BlinkCount	& 0x4 ) 			
//	SendComValS( (char)Vud);
/*
	Mv += VBaroComp;
	Mh += VBaroComp;
	Ml += VBaroComp;
	Mr += VBaroComp;
*/

#else	// TRICOPTER
// TODO  Same  low gas protection needed as above for + plus mode

	Mv = IGas + Nl;	// front motor
	Ml = IGas + Rl;
	Mr = IGas - Rl;
	Rl >>= 1;
	Ml -= Rl;	// rear left
    Mr -= Nl;	// rear right
	Mh = Tl + _Neutral_Out;	// yaw servo

	if( IGas > MotorLowRun )
	{
		if( (Ml > Mr) && (Mr < MotorLowRun) )
		{
			// Mv += Mh - MotorLowRun
			Ml += Mr;
			Ml -= MotorLowRun;
		}
		if( (Mr > Ml) && (Ml < MotorLowRun) )
		{
			// Mh += Mv - MotorLowRun
			Mr += Ml;
			Mr -= MotorLowRun;
		}
	}
#endif

// Add in Idle and limit to valid PWM's
//NOTE:
// if the PWM limit is readed in the following check then an unbalance is created in the trust
// This is only a last chance check & limit, the code up to here has to make sure that we don't get into this situation	
	M_front = 	LimitPWM(Mv + MotorLowRun); 
	M_left = 	LimitPWM(Ml + MotorLowRun);
	M_right = 	LimitPWM(Mr + MotorLowRun);
	M_rear = 	LimitPWM(Mh + MotorLowRun);
	

}

// Mix the Camera tilt channel (Ch6) and the
// ufo air angles (roll and nick) to the 
// camera servos. 
void MixAndLimitCam(void)
{
	int16	tmp =0;

// Calculate Cam Servos use  integral part (direct angle)

	if( BiasZeroCount > 0 ) // while detecting The Zero Bias voltages  do not use the gyros values to correct
	{
		MCamRoll = IK7;
		MCamPitch = IK6;
		return;
	}
	
	// scale and apply the camera compensation factors -- factors used as divisors to avoid overflow problems   
	if (CamRollFactor)
	{
		tmp= Roll_Angle/2; // Make 9 bits first 
		tmp /= CamRollFactor;
	}
	else
		tmp=0;
		
	MCamRoll = LimitPWM( tmp + IK7 );	// add RX input and limit 
	
	if (CamPitchFactor)
	{
		tmp= Pitch_Angle /2; // make 9 bit first 
		tmp  /= CamPitchFactor;	
	}
	else
		tmp =0;
	
	MCamPitch = LimitPWM(tmp + IK6);	// add RX input and limit 
	
}