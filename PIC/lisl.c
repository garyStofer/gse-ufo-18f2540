// The LISL controller routines

#include "main.h"
#include "bits.h"


void WriteLISLByte(uns8 d)
{
	uns8	s;

	LISL_SCL = 0;						// to give a little more low time
	for( s = 8; s ; s-- )
	{
		Delay10TCY();Delay10TCY();

		LISL_SCL = 0;
		if (d & 0x80) 
			LISL_SDA = 1;
		else
			LISL_SDA = 0;
		d <<= 1;
		LISL_SCL = 1;
	}
} // WriteLISLByte

uns8 ReadLISLNext(void)
{
	uns8	s, d;
	
	LISL_SCL = 0;					
	d = 0;
	for( s = 8; s ; s-- )
	{

		Delay10TCY();Delay10TCY();

		LISL_SCL = 0;
		d = (d << 1) & 0xfe;
		if ( LISL_SDA )
			d |= 1;	
		LISL_SCL = 1;
	}	

	return(d);
} // ReadLISLNext

uns8 ReadLISL(uns8 addr)
{
	uns8	d;

	LISL_SDA = 1;						// very important!! really!! LIS3L likes it

	LISL_IO = 0;						// write
	LISL_CS = 0;
	WriteLISLByte(addr);

	LISL_IO = 1;						// read	
	d=ReadLISLNext();
	
	if( (addr & LISL_INCR_ADDR) == 0 )	// is this a single byte Read?
		LISL_CS = 1;
				
	return(d);
} // ReadLISL

void WriteLISL(uns8 d, uns8 addr)
{
	LISL_IO = 0;						// write
	LISL_CS = 0;

	WriteLISLByte(addr);
	WriteLISLByte(d);

	LISL_CS = 1;
	LISL_IO = 1;						// read
} // WriteLISL


// put the base setup to linear sensor
// enable all axes, setup resolution
// setup parachute options
uns8  IsLISLactive(void)
{
	Delay_ms(100);	// wait 1/10 sec until LISL is ready to talk after power up
	LISL_CS = 1;
	WriteLISL(0b01011010, LISL_CTRLREG_2); // enable 3-wire, INTerrupt=1 , BDU=1, +/-2g , BOOT
	Delay_ms(50);   // wait until Boot command is completed and Factory calibration values are loaded.

	Vtmp = ReadLISL(LISL_WHOAMI + LISL_READ);
	if( Vtmp == 0x3A )	// a LIS03L sensor is there!
	{
	//	WriteLISL(0b.1101.0111, LISL_CTRLREG_1); // startup, enable all axis 160hz data-rate 
		WriteLISL(0b11000111, LISL_CTRLREG_1); // startup, enable all axis , 40hz data-rate
		WriteLISL(0b00000000, LISL_CTRLREG_3);
		
	//	 We are not using the interanal Calibration registers nor the FreeFall interrupt.  

		return  true;
	}
	return false;
}

// this routine is called ONLY ONCE while booting
// read 16 time all 3 axis of linear sensor.
// Puts values in Neutralxxx registers.
void GetEvenValues(void)
{	
	uns8 i;

	Delay_ms(100);	// wait  until LISL is ready to talk

	Rl = 0;
	Nl = 0;
	Tl  = 0;
	
	while( (ReadLISL(LISL_STATUS + LISL_READ) & 0x08) == 0 );
	
	for( i=0; i < 16; i++)
	{	
		// wait for new set of data
		while( (ReadLISL(LISL_STATUS + LISL_READ) & 0x08) == 0 );
		
		Rp.low8  = ReadLISL(LISL_OUTX_L + LISL_INCR_ADDR + LISL_READ);
		Rp.high8 = ReadLISLNext();
		Tp.low8  = ReadLISLNext();
		Tp.high8 = ReadLISLNext();
		Np.low8  = ReadLISLNext();
		Np.high8 = ReadLISLNext();
		LISL_CS = 1;	// end transmission
		
		Rl += Rp.I;
		Nl += Np.I;
		Tp.I -= -0x400; // minus 1 G
		Tl += Tp.I;
		
	}
	Rl += 8;Rl >>= 4;	// divide with rounding
	Nl += 8;Nl >>= 4; 	// divide with rounding
	Tl += 8;Tl >>= 4;	// divide with rounding
	
	NeutralLR = (char) Rl;
	NeutralUD = (char) Tl;
	NeutralFB = (char) Nl;
}

/* Roll and Pitch error compensation discussion:
   ---------------------------------------------
  The roll and pitch angles, integrated from the gyro rates, suffer from 3 different erros:
  
  A.) Bias drift in the gyro itself. This is a thermal unstability inherent of the rate sensor
      devices, basically the voltage that the device outputs while not in motion varies with 
      temperature. This causes a slow drift in the integrated angle. Devices such a the Melexis
      part have an active temp compensation built in and the error is calibrated out at the 
      factory to a great extent. 
  
  B.) Quantization errors in the ADC. Basically this means not enough bits in the ADC. Erros 
 	  arrive because for example the ADC can not measure the gyros bias voltage (0 motion) 
 	  absolutely preciisely and looses 1/2 lbs when the output change. This can be helped by 
 	  a heavy handed oversampling of the rate readings and averaging the readings. This is 
 	  is probably the smallest error of the 3.
 	  
  C.) Frame of reference translation error. This error arises from the fact that this SW 
      implementation doesn't translate the 3 gyros angles into an earth frame of reference
      first, but instead directly uses the body angles (body frame of referecne). This works
      as long as there are no compound motions where all 3 axes change at the same time. 
      To understand the nature of this considering the following exaturated case:
      
       1.) Pitch up 90 deg,  	body pitch angle is now +90 deg
       2.) Roll right 90 deg, 	body roll angle is now +90 deg 
       3.) Pitch down 90 deg,  
       
       The bodys pitch angle is now 0 again, BUT the bodys roll angle is still at +90 deg !!
       What should have happeened was a translation of the roll change into a Yaw change when
       the craft rolled while pointing straight up. The above is the extreme case, but the effect
       is present whenever rolling and pitching at the same time, i.e. in the case of a QC almost
       always. Out of the three this is the biggest error when the craft is manouvering or even 
       when simply beeing buffeted by wind. 
       
       This math involved to translate between frames of reference is not trivial and involves 
       trigonometry and/or vector match in the form of quaternions. Needless to say that a lowly
       8 bit PIC would be overwhelmed by it.  
       
   To counteract the above errors a simple erection scheme is used to bring the roll and pitch angles
   back to the earth, i.e. gravity frame of reference. This is done my driving the body angle toward zero
   whenever the associated accelerometer reads within a narrow band around level. This of course has it's 
   own problems, in that the accel can be fooled into reading level because of centrifugal forces as they 
   would occur in a "coordinated" turn around a point. This beeing not a winged craft one is less likely to
   actually perform a coordinated turn, and thus this solution seems adeqate.  Other solutions where tried 
   where the angle and the accel reading had to coincide within a deadband in order for the compensation to
   take effect. This however had the problem that once coincedence was lost there was no way for the 
   compensation to regain it again, sort of a catch 22. 
*/   
  
  float Acc_Z, Acc_X, Acc_Y;
       
// Read Accel values and formulate gyro error compensation
void ReadAccel(void)
{
	int16 tmp;
	
	if( ! _UseLISL)
	{	
		Rp.I = Np.I = 0;
		return;	
	}
		// GS: what's reading the status good for ??
			
	ReadLISL(LISL_STATUS + LISL_READ);
	// the LISL registers are in order here!!
	// 1 unit is 1/4096 of 2g = 1/2048g
	Rp.low8  = ReadLISL(LISL_OUTX_L + LISL_INCR_ADDR + LISL_READ);  // left right 
	Rp.high8 = ReadLISLNext();

	Tp.low8  = ReadLISLNext(); // up/down 
	Tp.high8 = ReadLISLNext();

	Np.low8  = ReadLISLNext();	// front/back
	Np.high8 = ReadLISLNext();
	
	LISL_CS = 1;	// end transmission


	// The MiddleYX values are from the UAVP Setup -- stored in the config EEPROM section 
	Rp.I -= MiddleLR;
	Tp.I -= MiddleUD;
	Np.I -= MiddleFB;
	Acc_X = (float)  Np.I /1024.0; 
	Acc_Y = (float) Rp.I / 1024.0;
	Acc_Z=  (float) Tp.I /1024.0;
	
// This is flawed -- -1G is only correct if perfectly level 
	Tp.I -= 1024;	// subtract 1g

// =====================================
// Roll-angle erection / error compensation 
// =====================================
	tmp = Rp.I;// - ((Roll_Angle * 11) / 128);	// sacle to accel 

	if ( tmp > -CompDeadBand && tmp < CompDeadBand )
	{
		if (Roll_Angle > 0 )		// drive angle towards 0 
			LRIntKorr = -CompDecay;
		else
			LRIntKorr = CompDecay;
	}
	else 
		LRIntKorr = 0;

// =====================================
// Pitch-Angle  erection / error compensation 
// =====================================
	tmp = Np.I ; // - ((Pitch_Angle * 11) / 128);
	
	if ( tmp> -CompDeadBand && tmp < CompDeadBand )
	{
		if (Pitch_Angle > 0 )		// drive angle towards 0 
			FBIntKorr = -CompDecay;
		else
			FBIntKorr = CompDecay;
	}
	else 
		FBIntKorr = 0;	
		
UpDown_Velocity = 0;

// this is all flawed because all three axes need to ve considered to extract the Z acceleration from 

// UpDownVelocity integrates  up down acceleration  == up down velocity 
// The Up down stuf doesn't work  See comments in MixANdLimit

//	UpDown_Velocity += Tp.I;
// this is missimg the correction for pitch and yaw tilt -- velocity will be wrong unless perfectly level all the time 
//	Tp.I = UpDown_Velocity;
//	Tp.I += 64;
//	Tp.I >>= 7;  // div by 64 
	//Tp .I*= LinUDIntFactor;
	
	// Vud = UpDown_Velocity/64;	
	// Vud = Tp.high8;

/*
	if( (BlinkCount & 0x03) == 0 )	
	{
		if( (int)Tp.high8 > Vud )
			Vud++;
			
		if( (int)Tp.high8 < Vud )
			Vud--;
		
		if( Vud >  20 ) Vud =  20;
		if( Vud < -20 ) Vud = -20;
	}
	
	if( UpDownVelocity >  10 ) UpDownVelocity -= 10;
	if( UpDownVelocity < -10 ) UpDownVelocity += 10;
	
*/
							
}


