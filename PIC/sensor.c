// The "sensor bus" routines (barometric and compass)

#include "main.h"
#include "bits.h"

void I2CDelay(void)
{
	Delay10TCY();
	Delay10TCY();
}

// put SCL to high-z and wait until line is really hi
// returns != 0 if ok
uns8 I2CWaitClkHi(void)
{
	uns8 nii=1;

	I2CDelay();
	I2C_CIO=1;	// set SCL to input, output a high
	while( I2C_SCL == 0 )	// wait for line to come hi
	{
		nii++;
		if( nii == 0 )
			break;
	}	
	I2CDelay();
	return(nii);
}

// send a start condition
void I2CStart(void)
{
	I2C_DIO=1;	// set SDA to input, output a high
	I2CWaitClkHi();
	I2C_SDA = 0;	// start condition
	I2C_DIO = 0;	// output a low
	I2CDelay();
	I2C_SCL = 0;
	I2C_CIO = 0;	// set SCL to output, output a low
}

// send a stop condition
void I2CStop(void)
{
	I2C_DIO=0;	// set SDA to output
	I2C_SDA = 0;	// output a low
	I2CWaitClkHi();

	I2C_DIO=1;	// set SDA to input, output a high, STOP condition
	I2CDelay();		// leave clock high
}


// send a byte to I2C slave and return ACK status
// 0 = ACK
// 1 = NACK
uns8 SendI2CByte(uns8 nidata)
{
	uns8 nii;
	
	for(nii=0; nii<8; nii++)
	{
		if( nidata & 0x80 )
		{
			I2C_DIO = 1;	// switch to input, line is high
		}
		else
		{
			I2C_SDA = 0;			
			I2C_DIO = 0;	// switch to output, line is low
		}
	
		if(!I2CWaitClkHi()) 
			return(I2C_NACK);
		I2C_SCL = 0;
		I2C_CIO = 0;	// set SCL to output, output a low
		nidata <<= 1;
	}
	I2C_DIO = 1;	// set SDA to input
	if(!I2CWaitClkHi())
		return(I2C_NACK);
	nii = I2C_SDA;	

	I2C_SCL = 0;
	I2C_CIO = 0;	// set SCL to output, output a low
//	I2CDelay();
//	I2C_IO = 0;		// set SDA to output
//	I2C_SDA = 0;	// leave output low
	return(nii);
}


// read a byte from I2C slave and set ACK status
// 0 = ACK
// 1 = NACK
// returns read byte
uns8 RecvI2CByte(uns8 niack)
{
	uns8 nidata=0;
	uns8 nii;

	I2C_DIO=1;	// set SDA to input, output a high

	for(nii=0; nii<8; nii++)
	{
		if( !I2CWaitClkHi() ) 
			return(0);
		nidata <<= 1;
		if( I2C_SDA )
			nidata |= 1;
		I2C_SCL = 0;
		I2C_CIO = 0;	// set SCL to output, output a low
	}
	I2C_SDA = niack;
	I2C_DIO = 0;	// set SDA to output
	if( !I2CWaitClkHi() )
		return(0);

	I2C_SCL = 0;
	I2C_CIO = 0;	// set SCL to output, output a low
//	I2CDelay();
//	I2C_IO = 0;	// set SDA to output
	return(nidata);
}


// initialize compass sensor
void InitDirection(void)
{
#ifdef NOT_PORTED_FOR18F  // use gregs code when ready
// set Compass device to Compass mode 
	I2CStart();
	if( SendI2CByte(COMPASS_ADDR) != I2C_ACK ) goto IDerror;
// this initialization is no longer needed, is done by testsoftware!
#if 0
	if( SendI2CByte('G')  != I2C_ACK ) goto IDerror;
	if( SendI2CByte(0x74) != I2C_ACK ) goto IDerror;
	// select operation mode, continuous mode, 20 Hz
	if( SendI2CByte(0b0.11.1.00.10) != I2C_ACK ) goto IDerror;
	I2CStop();

	I2CStart();
	if( SendI2CByte(COMPASS_ADDR) != I2C_ACK ) goto IDerror;
	if( SendI2CByte('r')  != I2C_ACK ) goto IDerror;
	if( SendI2CByte(0x06) != I2C_ACK ) goto IDerror;
	I2CStop();

// read multiple read count to avoid wear-and-tear on EEPROM
	I2CStart();
	if( SendI2CByte(COMPASS_ADDR+1) != I2C_ACK ) goto IDerror;
	if( RecvI2CByte(!I2C_ACK) != 16 )
	{	// not correctly set, set it up.
		I2CStop();

		I2CStart();
		if( SendI2CByte(COMPASS_ADDR) != I2C_ACK ) goto IDerror;
		if( SendI2CByte('w')  != I2C_ACK ) goto IDerror;
		if( SendI2CByte(0x06) != I2C_ACK ) goto IDerror;
		if( SendI2CByte(16)   != I2C_ACK ) goto IDerror;
	}
#endif
	_UseCompass = 1;
IDerror:
	I2CStop();
#endif	
_UseCompass = 0;
}

// Read direction, convert it to 2 degrees unit
// and store result in variable AbsDirection.
// The current heading correction is stored in CurDeviation
void GetDirection(void)
{
#ifdef NOT_PORTED_FOR18F  // use gregs code when ready
	 int16 DirVal, temp;

// set Compass device to Compass mode 
	I2CStart();
	if( SendI2CByte(COMPASS_ADDR+1) != I2C_ACK ) 
	{
		I2CStop();
		CurDeviation = 0;	// no sensor present, deviation = 0
		return;
	}
	DirVal.high8 = RecvI2CByte(I2C_ACK);
	DirVal.low8  = RecvI2CByte(!I2C_ACK);
	I2CStop();

// DirVal has 1/10th degrees
// convert to set 360.0 deg = 240 units
	DirVal /= 15;

// must use pre-decrement, because of dumb compiler
	if( AbsDirection > COMPASS_MAX )
	{
		CurDeviation = 0;
		AbsDirection--;
	}
	else
	{
// setup desired heading (AbsDirection)
		if( AbsDirection == COMPASS_MAX )	// no heading stored yet
		{
			AbsDirection = DirVal;	// store current heading
			CurDeviation = 0;
		}
// calc deviation and direction of deviation
		DirVal = AbsDirection - DirVal;
// handle wraparound
		if( DirVal <= -240/2 )
			DirVal +=  240;
		if( DirVal >   240/2 )
			DirVal -=  240;

// positive means ufo is left off-heading
// negative means ufo is right off-heading

		if( DirVal > 20 )	// limit to give soft reaction
			DirVal = 20;
		if( DirVal < -20 )
			DirVal = -20;

// Empirical found :-)
// New_CurDev = ((3*Old_CurDev)+DirVal) / 4
//
		temp = (long)CurDeviation;	// the previous value!
		temp *= 3;
		temp += DirVal;			// add the new value
		temp <<= 2;		// = 16* NewCurDev
		temp *= CompassFactor;
		CurDeviation = temp.high8;
	}
#endif	
}


// read temp & pressure values from baro sensor
// return value= niltemp;
// returns 1 if value is available
uns8 ReadValueFromBaro(void)
{
#ifdef NOT_PORTED_FOR18F  // use gregs code when ready
// test if conversion is ready
	I2CStart();
	if( SendI2CByte(BARO_ADDR) != I2C_ACK ) goto RVerror;
// access control register
	if( SendI2CByte(0xf4) != I2C_ACK ) goto RVerror;

	I2CStart();		// restart
	if( SendI2CByte(BARO_ADDR+1) != I2C_ACK ) goto RVerror;
// read control register
	niltemp.low8 = RecvI2CByte(I2C_NACK);
	I2CStop();

	if( (niltemp.low8 & 0b0010.0000) == 0 )
	{	// conversion is ready, read it!

		I2CStart();
		if( SendI2CByte(BARO_ADDR) != I2C_ACK ) goto RVerror;
// access A/D registers
		if( SendI2CByte(0xf6) != I2C_ACK ) goto RVerror;
		I2CStart();		// restart
		if( SendI2CByte(BARO_ADDR+1) != I2C_ACK ) goto RVerror;
		niltemp.high8 = RecvI2CByte(I2C_ACK);
		niltemp.low8 = RecvI2CByte(!I2C_NACK);
		I2CStop();
		return(1);
	}
	return(0);
RVerror:
	I2CStop();
#endif
	_UseBaro = 0;	// read error, disable baro
	return(0);
}

// start A/D conversion on altimeter sensor
// niaddr = 0xee to convert temperature
//          0xf4 to convert pressure
// returns 1 if successful, else 0
uns8 StartBaroADC(uns8 niaddr)
{
#ifdef NOT_PORTED_FOR18F  // use gregs code when ready
	I2CStart();
	if( SendI2CByte(BARO_ADDR) != I2C_ACK ) goto SBerror;

// access control register, start measurement
	if( SendI2CByte(0xf4) != I2C_ACK ) goto SBerror;
// select 32kHz input, measure temperature
	if( SendI2CByte(niaddr) != I2C_ACK ) goto SBerror;
	I2CStop();
// set or clear bit to signal what A/D is currently running
	_BaroTempRun = (niaddr == 0xee);
		
	return(1);
SBerror:
	I2CStop();
#endif
	return(0);
}

// initialize compass sensor
void InitAltimeter(void)
{
#ifdef NOT_PORTED_FOR18F  // use gregs code when ready
// read temperature once to get base value
// set SMD500 device to start temperature conversion
	if( !StartBaroADC(0xee) ) goto BAerror;
// wait 40ms
	for( Vtmp=40; Vtmp!=0; Vtmp--)
	{
		T0IF=0;
		while(T0IF == 0);
	}
	ReadValueFromBaro();

	BaseTemp = niltemp;	// save start value

// read pressure once to get base value
// set SMD500 device to start pressure conversion
	if( !StartBaroADC(0xf4) ) goto BAerror;

// wait 40ms
	for( Vtmp=40; Vtmp!=0; Vtmp--)
	{
		T0IF=0;
		while(T0IF == 0);
	}
	ReadValueFromBaro();
 	
	BasePressure = niltemp;

	_UseBaro = 1;
// prepare for next run
//	if( !StartBaroADC(0xf4) ) goto BAerror;

	return;
BAerror:
	I2CStop();
#endif
	_UseBaro = 0;

}


void ComputeBaroComp(void)
{
#ifdef NOT_PORTED_FOR18F  // use gregs code when ready

	if( ReadValueFromBaro() )	// returns niltemp as value
	{	// successful
		if( !_BaroTempRun )
		{	// current measurement was "pressure"
			if( ThrDownCount )	// while moving throttle stick
			{
				BasePressure = niltemp;	// current read value is the new level
				BaroCompSum = 0;
			}
			else
			{	// while holding altitude
				niltemp -= BasePressure;
//SendComChar('B');
// the uncorrected relative height
//				SendComValH(niltemp.high8);
//				SendComValH(niltemp.low8);
//				SendComValH(TempCorr.high8);
//				SendComValH(TempCorr.low8);
// niltemp1 has -400..+400 approx
				niltemp1 = TempCorr * BaroTempCoeff;
				niltemp1 += 16;
				niltemp1 /= 32;
				niltemp += niltemp1;	// compensating temp
// the corrected relative height, the higher alti, the lesser value

				// New Baro = (3*BaroSum + New_Baro)/4
				niltemp1 = niltemp;	// because of bank bits
				niltemp = BaroCompSum;	// remember old value for delta
				BaroCompSum *= 3;
				BaroCompSum += niltemp1;
				BaroCompSum += 2;	// rounding
				BaroCompSum >>= 2;	// div by 4
				niltemp1 = BaroCompSum - niltemp;	// subtract new height to get delta
					
#ifdef INTTEST
		SendComChar('a');
		SendComValH(BaroCompSum.high8);
		SendComValH(BaroCompSum.low8);	// current height
		SendComChar(';');
		SendComValH(TempCorr.high8);
		SendComValH(TempCorr.low8);	// current temp
		SendComChar(';');
		SendComValH(niltemp1.low8);		// delta height
		SendComChar(';');
		
#endif
// was: +10 and -5
				if( BaroCompSum > 8 ) // zu tief: ordentlich Gas geben
					BaroCompSum = 8;
				if( BaroCompSum < -3 ) // zu hoch: nur leicht nachlassen
					BaroCompSum = -3;
			// weiche Regelung (proportional)
			// nitemp kann nicht überlaufen (-3..+8 * PropFact)
				nitemp = (int)BaroCompSum.low8 * BaroThrottleProp;
				if( VBaroComp > nitemp )
					VBaroComp--;
				else
				if( VBaroComp < nitemp )
					VBaroComp++;

				if( VBaroComp > nitemp )
					VBaroComp--;
				else
				if( VBaroComp < nitemp )
					VBaroComp++;
			// Differentialanteil
				if( niltemp1 > 8 )
					niltemp1.low8 = 8;
				else
				if( niltemp1 < -8 )
					niltemp1.low8 = -8;
					
				nitemp = (int)niltemp1.low8 * BaroThrottleDiff;
				VBaroComp += nitemp;
				if( VBaroComp > 15 )
					VBaroComp = 15;
				if( VBaroComp < -5 )
					VBaroComp = -5;
					
#ifdef INTTEST
		SendComValH(VBaroComp);
		SendComChar(0x0d);
		SendComChar(0x0a);
#endif

			}
			StartBaroADC(0xee);	// next is temp
		}
		else
		{
			if( ThrDownCount )
				BaseTemp = niltemp; // current read value
			else // TempCorr: The warmer, the higher
			{
				TempCorr = niltemp - BaseTemp;
//				TempCorr += 4;	// compensate rounding error later /8
			}
			StartBaroADC(0xf4);	// next is pressure
		}
	}
	// eliminate static drift of baro sensor
#ifdef NADA
	if( BlinkCount == 1 )
	{
		if( BaroCompSum > 0 )
			BasePressure++;
		if( BaroCompSum < 0 )
			BasePressure--;	
	}
#endif
#endif
}	


