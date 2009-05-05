#include "main.h"
#include "bits.h"


int8 ReadEE(uns8 addr)
{
	int8 b;

	EEADR = addr;
	EECON1bits.EEPGD = false;
	EECON1bits.RD = true;
	b=EEDATA;
	EECON1 = 0;
	return(b);	
} // ReadEE

void WriteEE(uns8 addr, int8 d)
{
	int8 rd;
	
	rd = ReadEE(addr);
	if ( rd != d )						// avoid redundant writes
	{
		EEDATA = d;				
		EEADR = addr;
		EECON1bits.EEPGD = false;
		EECON1bits.WREN = true;

//		DisableInterrupts; ?? Ask Greg why he has this
		EECON2 = 0x55;
		EECON2 = 0xaa;
		EECON1bits.WR = true;
		while(EECON1bits.WR);
//		EnableInterrupts;

		EECON1bits.WREN = false;
	}

} // WriteEE

void WriteParametersEE(uns8 s)
{
	int8 *p;
	uns8 b;
	uns16 addr;
	
	if( s == 1 )
		addr = _EESet1;	
	else
		addr = _EESet2;

	p = &FirstProgReg; 
	while ( p <= &LastProgReg)
		WriteEE(addr++, *p++);
} 
void ReadParametersEE(void)
{
	int8 *p, c; 
	uns16 addr;
	
	if( IK5 > _Neutral_In )
		addr = _EESet2;	
	else
		addr = _EESet1;
	
	for(p = &FirstProgReg; p <= &LastProgReg; p++)
		*p = ReadEE(addr++);

	BatteryVolts = LowVoltThres;

	// Sanity check
	//if timing value is lower than 1, set it to 10ms!
	if( Interval_time < 1 )
		Interval_time = 1;
	else
	if ( Interval_time > 10 )
		Interval_time = 10;
		

} // ReadParametersEE



