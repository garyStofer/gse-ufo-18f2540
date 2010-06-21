/*********************************************************************
*	Interrupt service routine for UAVP Project running on PIC 18F2520
*	File :			ISR.c
*	Compiler:		c18
* 	Initial version:Jan, 31, 2009
*	Description:	See below
*	Author:			Gary Stofer,  Gary@Stofer.name
*
*         Links: 	http://code.google.com/p/uavp-mods/
*					http://uavp.ch/moin/StartSeite
*					http://Stofer.name
***********************************************************************/

//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//

 

// Discussion: Output PWM creation with TMR0 and TMR2
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// The following ISR creates 6 output signals that are pulse width modulated between 1 and 2 ms each. The
// generation of the pulses is governed by the hardware timers TMR0 and TMR2 arranged in a cascading way. 
// The resulting output pulses start at increments of 1ms  and therefore the max PWM repeat rate is 6ms. 
// TMR0  is used to  time the initial 1ms of each pulse, TMR2 is then used to time the variable portion of each pulse, 
// therefore TMR0 starts the pulse output for a particular channel  while TMR2 terminates it. 
// For example:   Upon TMR0 interrupt , which always happens every 1ms,  the ISR enables the port output for Channel N and
// sets up TMR2/PR2 with the variable time component of Channel N-1, then enables TMR2 interrupts before exiting out of the ISR.  
// The next thing that happens is that TMR2 reaches the value in the Preset register (and interrupts.  TMR2 ISR simply clears the N-1 output 
// and disables itself until TMR0 ISR re-enables it again for the next channel.  
// 
// Since TMR0 always interrupts at 256*4us  == 1024us , The resulting PWM can never reach all the way down to 1ms exactly.  
// To make the PW symmetrical around 1.5ms the max variable time should be reduced 238 counts. 
// AND MORE IMPORTANTLY:  This setup makes it so that TMR0 always runs longer than TMR2and guarantees that the pulse termination interrupt from TMR2 
// is always seen before TMR0 move to the next channel. Higher than 248counts and the ISR fails.  
//
// Potential for jitter in timing:
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  Since TMR2 is always guaranteed to interrupt before  TMR0 comes around again there is no jitter introduced from the interaction between these toe timers. 
// However, Timer1, used for the RX signal decoding could be currently executing when TMR2 comes around and needs to shut off the PWM 
// output. This will introduce jitter in the affected PWM signal since it has to wait for the TMR1 processing to be completed. Having the TMR2 IF flag evaluated 
// inside TMR1 processing  and keeping TMR1 processing to a minimum reduces this potential for jitter. 

// Data Flow
//~~~~~~~~~~
// The timing values for the 6 output channels are being conveyed to the ISR via an array indexed by channel number.
// This array stores the variable time for each channel in 4us increments, since TMR2 counts up from that value and  interrupts when 
// reaching 0xff.   The array can be updated at anytime by the outside code, 
// The variable PWM_Pause is used to reduce the PWM repeat rate (Frequency)  in increments of 1ms down from 6ms max rate. 

// Timer 0 and Timer2  input conditions:
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Timer 0 timebase set to 4us per tick OPTION_REG = 0b.10000.011 or 0b.10000.111 -- ( depends on actual FOSC 16 or 32 Mhz )
// T0IF = 0;   Timer0 interrupt Flag cleared 
// T0IE = 1; Timer0 interrupt enable on continuously , except see below
// Timer 2 timebase set to Fosc/4/16 == 4us per tick via T2CON =  0b.0000111 or  0b.0001111 ( depends on actual FOSC 16 or 32 Mhz )
// TMR2IF = 0; Timer2 interrupt flag initially cleared
// TMR2IE = 0; Timer2 interrupt enable initially off, nobody except ISR to modify it
// PEIE   = 1;  Peripheral interrupt enable permanently on 
// GIE    = 1; Global interrupt enable  never turned off unless to purposely trying to stop PWM generation, Timekeeping and RX decoding
// Variable PWM_Pause to be set to some reasonable value 
// Array PWM_Val[] set to the value of PWM time.

// Discussion RX signal Capture
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// The PWM signals from the transmitter are captured with the combined use of Timer1 (16bit) and its associated capture feature.
// Timer 1 is setup so that it counts up from 0 in 1 or 2 us increments (depends on FOSC 16 or 32 mhz).  Capture1 is setup so that it triggers on a high to low transition for composite RX signal or
// on every edge  on wired-OR signals when feeding the device with all odd RX channels.  Upon Capture interrupt the value in the capture register is divided to increment in 4us steps and
//  and compared to the MIN_SYNC_PAUSE value to detect the transmitters frame sync at >5ms. Detecting a frame sync resets the index counter into 
// the RXch_Val[] so that the next channel received is stored as CH1.  
// Captures that are less than 5ms are treated  as RX channels. Then a check is made for basic validity of the signal by  making sure that the captured
//  values is between 1.024 and 2.048 ms. So validated channels are then stored in the appropriate slot of the RXCh_Val array. Channels with pulse widths outside this range are 
// not stored to the array.  Only the low 8 bits of the incoming signal are stored,this in essence stores only the "variable" part of the PWM signal. 
// All other RX related issues, such as channel mapping , channel swapping etc need to be done by the non ISR code to keep time in the ISR to a minimum 

// A  "No-Signal" situation from the receiver is indicated when Timer1 elapses and interrupts when reaching terminal count at about 130 or 64ms.
// The No-Signal state is reset upon receiving the first valid Frame sync from the receiver.  

// Note on accessing data by outside code:
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Since the two arrays are 8bit based  and accessed to 8 bit variables is atomic the data can be accessed non ISR code at any time without
// any special precautions.
// Further note that the Pause width is not stored  because its length would not fit into an 8bit variable and therefore it would be difficult 
// to read from outside code. ( Would have to disable/enable CCP1IE around the read) 


/**************************************************************************************************************************/

#include "main.h"
#include "bits.h"


static uns8 Ch_On;	// Contains the bit for the CH that is beeing turned on next
static uns8 Ch_Off; // Contains the bit postiotion for the CH that is beeing tured off next 
static uns8 Pause;  // Contains the number of 1ms ticks to pause  between pulses.
static uns8 Ch_Ndx; 	// An index into the PVM_Val[] 
static uns8 RXCh_Ndx;	// An index into the RXCh_Val[] 


uns8 volatile TimeTick1ms;  	// maintain a 1 ms tick counter for delay / wait functions outside of ISR 
uns8 volatile PWM_Val[10]; 		// array must be same or bigger than MAX_OUT_CH 
uns8 volatile PWM_Pause;	 	// Time added in between PWM to slow down the repeat rate of the Output signals 
uns8 volatile RXCh_Val[MAX_RX_CH]; 	// 9 channels + pause
static uns8   tmpRXCh_Val[MAX_RX_CH];
uns8 RX_Num_Channels ;			// This stores the number of channels decoded in a frame -- might be useful to outside code 
								// knowing how many channels we are receiving 

uns8 volatile PID_Delay;		// Count down var for PID timing.   ISR counts down to 0 at 1ms intervals -- 
								// outside code sets var and sleeps until counted down	
uns8 volatile RxFrameErr;		// count number of bad RX frames or loss of signal 																

#pragma interrupt high_isr_handler
void high_isr_handler(void)
{

	// Timer 2 only runs when kicked off by timer0's ISR  , it terminates the PWM output pulse 
	// TMR2 has to have the highest priority 
	if( PIR1bits.TMR2IF )	// Timer 2 Interrupt -- This happens when the PW time of a channel has elapsed 
	{	
		LATB &= ~(Ch_Off);		// Using LATB instead of PORTB - This guards against a wrong setup ADCON1 which makes port b not readable via PORTB
		
		PIE1bits.TMR2IE = 0;  // Stop timer from generating further interrupts until re-eanabled by TMR0 ISR again 
		PIR1bits.TMR2IF = 0;  // Clear, so we can check it again further down in the ISR if need be....
	}
	
	
	// Periodic Timer0 interrupt -- every 1.024ms continuously , used to keep time in system and  it
	//  starts the PW output signal then kicks on Timer 2 to finished the pulse after the initial 1ms has elapsed
	if( INTCONbits.T0IF )  //  
	{
		// First keep time and setup for the next interrupt again
		TimeTick1ms++;	// This is the timetick that seves as timebase throughout 
	
		if (PID_Delay)  // outside PID sleeps until PID_Delay becomes 0, then sets new delay and executes PID loop
			PID_Delay--;
	
		
		// Next deal with the output (PWM) signal generation for all 6 channels 
		Ch_Off = Ch_On; 	// next Ch to turn off is the previous one we turned on
		
		if (Pause)
			Pause--;
		else 
		{
			if (Ch_On == 0)		 // out of Pause
			{
				Ch_On = 1;		 // Start at CH "One" again
				Ch_Ndx = 0;
			}	
			else
				Ch_On <<= 1; 	 // advance to the next channel to be turned on 
			
			
			// All channels have been generated -- enter Pause 
			if (Ch_Ndx >= MAX_OUT_CH-1)
			{
				Pause = PWM_Pause;
				Ch_On = 0;
			}

			LATB |= Ch_On;		// Using LATB instead of PORTB - This guards against a wrong setup ADCON1 which makes port b not readable via PORTB

		}


		if (Ch_Off )			// If there is something to be turned off , i.e. not in Pause, then setup TMR2 
		{
			TMR2 =0; 			 //Important this Resets the postscalar !! 
			PR2=PWM_Val[Ch_Ndx]; // Setup timeout AFTER resetting TMR2
	
			// Interrupt is generated when TMR2 counts up to PR2 		 
			PIR1bits.TMR2IF = 0; 	// Clear potentially pending interrupt request  -- Should never be the case 
			PIE1bits.TMR2IE = 1;     // and enable   
			
			Ch_Ndx++;
		}
		INTCONbits.T0IF = 0;       // and erase our Timer0 interrup flag for next interrupt	
	}

}



// NOTE !! This is Pragma'd as a low interrupt, therefore not using the FAST IRET  
// This is neeed so the STATUS register doesn't become corrupted 
#pragma interruptlow low_isr_handler
void low_isr_handler (void)
{
	char_int 	temp;

  	// A captured RX  edge has been detected 
	if( PIR1bits.CCP1IF )
	{	
	  	// Disable the high IRQ momentarily to get reset the captures timer immediatly 
	  	// Work around missing Hardware reset o0f Timer1 upon capture event.
	  	// Since the capture can happen when the high ISR is running we need to take into account the time 
	  	// it took to get to this ISR when resetting timer1, This reduces jitter on Captured value down to +- 4us 

		INTCONbits.GIEH  = 0; 		// High priority interrupts off while read modify write of TMR1

		temp.low8 =TMR1L;			// observe the sequence, so that High byte is latched first 
		temp.high8 = TMR1H;
		temp.I = temp.I - CCPR1;
		TMR1H = temp.high8;			// observe proper sequence 
		TMR1L =	temp.low8;
 		INTCONbits.GIEH  = 1; 		// Re-enable

		temp.I = CCPR1;	 // read the captured time value 
		

		
// divide so that 1 count is 4us 		
#ifdef FOSC_16		
		temp.I >>= 1;	
#elif defined FOSC_32
		temp.I >>= 2;		
#endif

		if ( temp.I > MIN_RX_SYNC_PAUSE ) 	// A pause , > 5ms 
		{
			RX_Num_Channels = RXCh_Ndx;
			
			// Check that CH 5 is  either near minimum or maximum  to  "validate" the  RX frame
			// check that there are at least 5 channels. 
			// or Check for axact number of CH the transmitter sends ... 
			// Check that the Sync Pause is less than 16 ms
	

			if (( tmpRXCh_Val[4] < 50 ||  tmpRXCh_Val[4] > 190 )  && temp.I < MAX_RX_SYNC_PAUSE)
			{  
#ifdef MY_RX_CH_NUM 				
				if ( RX_Num_Channels == MY_RX_CH_NUM )
#else 		
				if ( RX_Num_Channels >= 5 )	
#endif	
				{
					RXCh_Val[0] = tmpRXCh_Val[0];
					RXCh_Val[1] = tmpRXCh_Val[1];
					RXCh_Val[2] = tmpRXCh_Val[2];
					RXCh_Val[3] = tmpRXCh_Val[3];
					RXCh_Val[4] = tmpRXCh_Val[4];
					RXCh_Val[5] = tmpRXCh_Val[5];
					RXCh_Val[6] = tmpRXCh_Val[6];
					RXCh_Val[7] = tmpRXCh_Val[7];
					RXCh_Val[8] = tmpRXCh_Val[8];
					_NoSignal = 0;
				}
				else
				{	
					_NoSignal = 1; 	
					if ( RxFrameErr < 20 ) // so it doesn't wrapp around
						RxFrameErr++;	
				}	
			}else
			{
				_NoSignal = 1; 		
				if ( RxFrameErr < 20 ) // so it doesn't wrapp around
					RxFrameErr++;
			}	

			
			RXCh_Ndx =0;					// Sync pulse detected - next CH is CH1 
			
		}
		else				// An actual channel -- Record the variable part of the PWM time 
		{
			if (RXCh_Ndx < MAX_RX_CH )
			{	
// Note: Since we chop off the first 1.024ms by only using the lower 8 bits  and therefore cutting off  24us  from the bottom
// of the RX pulses range we should also limit the top end of the RX pulse to 0xff - 3*6 or = 238 to deliver a pulse width 
// value that is symmetrical around the transmitters midpoint of 1.5ms and so that _Neutral then truly represents the midpoint				

				if (temp.high8 == 0x1 )				// only take value when between 1 .024 and 2.048 ms pulse 
				{
					if (temp.low8 > _Maximum)
						tmpRXCh_Val[RXCh_Ndx] = _Maximum;
					else
						tmpRXCh_Val[RXCh_Ndx] = temp.low8; // chop off first 1.024 ms -- store 1024 - 2048 us only  
				}
				else
					tmpRXCh_Val[RXCh_Ndx] = _Minimum;	// if under 1.024 ms -- This can happen on the throttle and 
					
				RXCh_Ndx++;							
			}
		}
		
	

#ifndef RX_PPM			// When NOT a single PPM pulse train from receiver. i.e. the diaode wired or from the Odd channels 
		CCP1CONbits.CCP1M0 ^= 1;	// toggle edge bit to detect a channel on very edge transition
#endif
		PIR1bits.CCP1IF = 0;		// Clear this interrupt flag for next time around 
	}	

	
	 // timer 1 interrupted after  ~130 or ~64 ms of no activity on the capture pin 
	if ( PIR1bits.TMR1IF )
	{
		RX_Num_Channels = 0;

		_NoSignal = 1; 	// 130 ms of silence from the RX signal indicates that we have no signal 
		PIR1bits.TMR1IF=0;
	}


} // low_isr
#pragma code high_isr = 0x08
void high_isr (void)
{
  _asm goto high_isr_handler _endasm
} // high_isr
#pragma code

#pragma code low_isr = 0x18
void low_isr (void)
{
  
  _asm goto low_isr_handler _endasm
} // low_isr
#pragma code

