
// Serial support (RS232 option)

#include "main.h"
#include "bits.h"

// data strings

#pragma idata menu1
const char SerHello[] = "\r\nQudracopter UFO \r\n";
const char SerSetup[] = "\r\nFirmware Version: " Version "\r\n"
						"Gyros: 3x MLX90609 or ADXL300\r\n"
						"Linear sensors ";

const char  SerLSavail[]="ONLINE\r\n";
const char  SerLSnone[]= "not available\r\n";
const char  SerBaro[]=   "Baro ";
const char  SerBaroBMP085[]=   "BMP085\r\n";
const char  SerBaroSMD500[]=   "SMD500\r\n";
const char  SerChannel[]="Throttle Ch";
const char  SerFM_Fut[]= "3";
const char  SerFM_Grp[]= "1";

#pragma idata
#pragma idata menu2

const char  SerCompass[]="Compass ";
const char  SerReg1[]  = "\r\nRegister ";
const char  SerReg2[]  = " = ";
const char  SerPrompt[]= "\r\n>";
const char  SerHelp[]  = "\r\nCommands:\r\n"
					 		  "L...List param\r\n"
							  "M...Modify param\r\n"
							  "S...Show setup\r\n"
							  "N...Neutral values\r\n"
							  "R...Show receiver channels\r\n"
							  "B...start Boot-Loader\r\n";

// THE FOLLOWING LINE NOT TO BE CHANGED, it is important for UAVPset
const char  SerList[]  = "\r\nParameter list for set #";
const char  SerSelSet[]= "\r\nSelected parameter set: ";

const char  SerNeutralR[]="\r\nNeutral Roll:";
const char  SerNeutralN[]=" Ptch:";
const char  SerNeutralY[]=" Yaw:";

const char  SerRecvCh[]=  "\r\nT:";
#pragma idata

// send a character to the serial port
void TxChar(char ch)
{
	while( PIR1bits.TXIF == 0 ) ;	// wait for transmit ready
	TXREG = ch;		// put new char
}

// transmit a fix text from a table
void TxText(const char *pch)
{
	while( *pch != '\0' )
	{
		TxChar(*pch);
		pch++;
	}
}

void ShowPrompt(void)
{
	TxText(SerPrompt);
}



// converts an uintigned byte to decimal and send it
void TxValU(uns8 v)
{	
	uns8 nival;

	nival = v;

	v = nival / 100;
	TxChar(v+'0');
	nival %= 100;		// Einsparpotential: Modulo als Mathlib

	v = nival / 10;
	TxChar(v+'0');
	nival %= 10;

	TxChar(nival+'0');
}

// converts a nibble to HEX and sends it
void TxNibble(uns8 v)
{
	uns8 nival;

	nival = v + '0';
	if( nival > '9' )
		nival += 7;		// A to F
	TxChar(nival);
}

// converts an uintigned byte to HEX and sends it
void TxValH(uns8 v)
{
	TxNibble(v >> 4);
	TxNibble(v & 0x0f);
}

// converts an uintigned double byte to HEX and sends it
void TxValH16(uns16 v)
{
	TxValH(v >> 8);
	TxValH(v & 0xff);
} // TxValH16

// converts a signed byte to decimal and send it
// because of dumb compiler nival must be declared as uintigned :-(
void TxValS(int8 v)
{
	if( v < 0 )
	{
		TxChar('-');	// send sign
		v = -v;
	}
	else
		TxChar('+');	// send sign

	TxValU(v);
}

void TxNextLine(void)
{
	TxChar(0x0d);
	TxChar(0x0a);
}

// if a character is in the buffer
// return it. Else return the NUL character
char RxChar(void)
{
	uns8 ch;
	
	if( PIR1bits.RCIF )	// a character is waiting in the buffer
	{
		if( RCSTAbits.OERR || RCSTAbits.FERR )	// overrun or framing error?
		{
			RCSTAbits.CREN = 0;	// diable, then re-enable port to
			RCSTAbits.CREN = 1;	// reset OERR and FERR bit
			ch = RCREG;	// dummy read
		}
		else
		{
			ch = RCREG;	// get the character
			TxChar(ch);	// echo it
			return(ch);		// and return it
		}
	}
	return( '\0' );	// nothing in buffer
}


// enter an uintigned number 00 to 99
uns8 RxNumU(void)
{
	char ch;
	uns8 nival;

	nival = 0;
	do
	{
		ch = RxChar();
	}
	while( (ch < '0') || (ch > '9') );
	nival = ch - '0';
	nival *= 10;
	do
	{
		ch = RxChar();
	}
	while( (ch < '0') || (ch > '9') );
	nival += ch - '0';
	return(nival);
}


// enter a signed number -99 to 99 (always 2 digits)!
int8 RxNumS(void)
{
	char ch;
	int8 nival;

	nival = 0;

	_NegIn = 0;
	do
	{
		ch = RxChar();
	}
	while( ((ch < '0') || (ch > '9')) &&
           (ch != '-') );
	if( ch == '-' )
	{
		_NegIn = 1;
		do
		{
			ch = RxChar();
		}
		while( (ch < '0') || (ch > '9') );
	}
	nival = ch - '0';
	nival *= 10;

	do
	{
		ch = RxChar();
	}
	while( (ch < '0') || (ch > '9') );
	nival += ch - '0';
	if( _NegIn )
		nival = -nival;
	return(nival);
}

// send the current configuration setup to serial port
void ShowSetup(uns8 h)
{
	if( h )
	{
		TxText(SerHello);
		IK5 = _Minimum;	
	}

	TxText(SerSetup);	// send hello message
	if( _UseLISL )
		TxText(SerLSavail);
	else
		TxText(SerLSnone);

	TxText(SerCompass);
	if( _UseCompass )
		TxText(SerLSavail);
	else
		TxText(SerLSnone);

	TxText(SerBaro);
	if( _UseBaro )
		if ( BaroType == BARO_ID_BMP085 )
			TxText(SerBaroBMP085);
		else
			TxText(SerBaroSMD500);
	else
		TxText(SerLSnone);

	ReadParametersEE();
	TxText(SerChannel);
	if( FutabaMode )
		TxText(SerFM_Fut);
	else
		TxText(SerFM_Grp);

	TxText(SerSelSet);
	if( IK5 > _Neutral_In )
		TxChar('2');
	else
		TxChar('1');
	
	ShowPrompt();
}

void ProgRegister(void)
{
	uns8 IRQStat;
	
	EECON1bits.EEPGD = 0;
	IRQStat = INTCON;
	INTCON &= 0x3F;					// Disable both High and low IRQ's 

	EECON1bits.WREN = 1;			// enable eeprom writes
	EECON2 = 0x55;					// Set programming key sequence
	EECON2 = 0xAA;
	EECON1bits.WR = 1;				// start write cycle

	INTCON = IRQStat; 				// Re-enable IRQ's again 

	while( EECON1bits.WR == 1 );	// wait to complete
	EECON1bits.WREN = 0;			// disable EEPROM write
}

// if a command is waiting, read and process it.
// Do NOT call this routine while in flight!
void ProcessComCommand(void)
{
	int8  *p;
	uns8 ch;
	uns8 addr;
	uns8 ee_addr;
	int8 d;
	
	ch = RxChar();
	if( islower(ch))							// check lower case
		ch=toupper(ch);

	switch( ch )
	{
		case '\0' : break;
		case 'L'  :	// List parameters
			TxText(SerList);	// must send it (UAVPset!)
			if( IK5 > _Neutral_In )
				TxChar('2');
			else
				TxChar('1');
			ReadParametersEE();
			addr = 1;
			for(p = &FirstProgReg; p <= &LastProgReg; p++)
			{
				TxText(SerReg1);
				TxValU(addr++);
				TxText(SerReg2);
				d = *p;
				TxValS(d);
			}
			ShowPrompt();
			break;
		case 'M'  : // modify parameters
			LedBlue_ON;
		
			TxText(SerReg1);
			addr = RxNumU()-1;
			TxText(SerReg2);
			d = RxNumS();

// The Modify command should indicate which param set it's programming and not rely on the 
// IK5 switch on the transmitter. This causes a lot of confucsions when modifying params 
// in that it's very easy to have one thing displayed on the PC and on other selected with IK5

			

			if( IK5 > _Neutral_In )
				ee_addr = _EESet2;
			else
				ee_addr =_EESet1;
			WriteEE(ee_addr + (uns16)addr, d);

#ifdef PITCH_ROLL_TIED_KLUDGE  
			if ( addr == pitchKp)
				WriteEE(ee_addr + (uns16)rollKp, d);
			if ( addr == pitchKi)
				WriteEE(ee_addr + (uns16)rollKi, d);
			if ( addr == pitchKd)
				WriteEE(ee_addr + (uns16)rollKd, d);
#endif				
		
// TODO: this needs to be cleaned up, but for now just cross the parameters that should be the same 
// in both setups over to the other set

			if( addr > 14 ) // all from ConfigParam on up
			{									
				if( IK5 > _Neutral_In )
					WriteEE(_EESet1 + (uns16)addr, d);			
				else
					WriteEE(_EESet2 + (uns16)addr, d);

			}
		
			LedBlue_OFF;
  			ShowPrompt();
			break;
		case 'S' :	// show status
			ShowSetup(0);
			break;
		case 'N' :	// neutral values
			TxText(SerNeutralR);
			TxValS(NeutralLR);

			TxText(SerNeutralN);
			TxValS(NeutralFB);

			TxText(SerNeutralY);	
			TxValS(NeutralUD);
			ShowPrompt();
			break;
		case 'R':	// receiver values
			TxText(SerRecvCh);
			TxValU(IGas);
			TxChar(',');
			TxChar('R');
			TxChar(':');
			TxValS(IRoll);
			TxChar(',');
			TxChar('N');
			TxChar(':');
			TxValS(IPitch);
			TxChar(',');
			TxChar('Y');
			TxChar(':');
			TxValS(IYaw);
			TxChar(',');
			TxChar('5');
			TxChar(':');
			TxValU(IK5);
			TxChar(',');
			TxChar('6');
			TxChar(':');
			TxValU(IK6);
			TxChar(',');
			TxChar('7');
			TxChar(':');
			TxValU(IK7);
			ShowPrompt();
			break;

		case 'B':	// call bootloader
//		BootStart();							// never comes back!
		
#ifndef TESTOUT	
		case 'T':
// What does this do ?		
			Roll_Rate_8 = 10;
			Pitch_Rate_8 = 20;
			Rw = 30;
			Nw = 40;
			ShowPrompt();
			break;
#endif

		case '?'  : // help
			TxText(SerHelp);
			ShowPrompt();
	}
}

