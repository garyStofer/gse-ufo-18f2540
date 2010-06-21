#include "config.h" 	// Chip Config pragmas must be executed before any other includes take place because of redefinition of pragma attribute keywords
#include "main.h"
#include "bits.h"
#include "math.h"

void main(void)
{  
	uns8	DropCount;		// to lower gas slowy upon latched TX signal loss 
	uns8	LowGasCount;
	uns8	LedCount;
	uns8 	RxSignalLost;
	

	// general ports setup
	TRISA = 0b00111111;	// all inputs
	
	InitADC();			// Setup ADC clock and mode etc

// The setup below would be used if Magnetometer I2C is needed  	
//	PORTB = 0b11000000;	// all outputs to low, except RB6 & 7 (I2C)!
//	TRISB = 0b01000000;	// all servo and LED outputs

	// This setup is used to gain two more Servo PWM signals on RB6 and RB7 
	// Note: Signals are not present when programmed via debug mode 
	PORTB = 0b00000000;	// All Low initially 
	TRISB = 0b00000000;	// All outputs -- Servo PWMs

	PORTC = 0b01100000;	// all outputs to low, except TxD and CS
	TRISC = 0b10000100;	// RC7, RC2 are inputs

	SSPSTATbits.CKE = 1;// default I2C - enable SMBus thresholds for 3.3V LIS

   // setup serial port for 8N1
	TXSTA = 0b00100100;	// async mode, BRGH = 1
	RCSTA = 0b10010000;	// receive mode
	SPBRG = _B38400;
	Vtmp = RCREG;			// be sure to empty FIFO


	
// Timer0 setup     ~~~~~~~~~~~~~~ Used for 1ms Timetick and PWM generation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#ifdef FOSC_16		
	T0CON =  0b00000011;	// TMR0 w/ int clock, 1:16 presc (see _PreScale0),  Ticks at 4us, interrupts every 1024us , for Xtal 16Mhz, no PLL, FOSC = 16Mhz 
#elif defined FOSC_32
	T0CON =  0b00000100;	// TMR0 w/ int clock, 1:32 presc (see _PreScale0),  Ticks at 4us, interrupts every 1024us , for Xtal 8Mhz, PLLx4, Fosc =32Mhz	
#endif
	
	T0CONbits.T08BIT = 1; 	// function in 8 bit mode 
	T0CONbits.TMR0ON = 1;   // Enable the timer
	
	INTCONbits.T0IF = 0;
	INTCONbits.T0IE = 1;
	INTCON2bits.RBPU = 1;				// enable weak pullups  -- 
	
// Timer1 setup       ~~~~~~~~~~ Used to capture RX signal ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	TMR1H = 0;
	TMR1L = 0;
	
	T1CON = 0b10110001;	// enable TMR1, 1:8 prescaler at Foac 16Mhz ==  run with 2us clock  == maxrun of 0xffff x 2us = 130ms
						// Fosc 32 Mhz = run at  1us clock ==  maxrun 65us
						// See ifdefs in IRQ.c  for dealing with differen Fosc rates
	PIR1bits.TMR1IF = 0;
	PIE1bits.TMR1IE = 1;

	// Capture 
	CCP1CON = 0b00000100;	// capture mode: every falling edge
	PIE1bits.CCP1IE = 1;
	
// Timer2 setup   ~~~~~~~~~~~~~~~ Used for PWM generation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#if FOSC_16		
	T2CON = 0b00000111; // enable the timer at 4us per tick, i.e Fosc/4/16 for Xtal 16Mhz No PLL , Fosc = 16Mhz
#elif defined FOSC_32	
	T2CON = 0b00001111; // enable the timer at 4us per tick, i.e Fosc/4/32for Xtal 8Mhz PLL 4x , Fosc = 32Mhz
#endif

	PR2 = 0xFF; 
	PIR1bits.TMR2IF = 0;
	PIE1bits.TMR2IE = 0;		// is beeing started by Tmr0 ISR

	// Setting up interrupt priorities 
	RCONbits.IPEN =1;			// Priority schema 
	INTCON2bits.TMR0IP  = 1;	// timer 0 == high interrup priority , used for 1ms tick used inPWM 
	IPR1bits.CCP1IP 	= 0;    // Capture 1 interrup = low Priority	used for RX signal capture 
	IPR1bits.TMR1IP		= 0;	// and timer 1 overflow = low priority 	used for RX signal timeout
	IPR1bits.TMR2IP     = 1;	// High Priortity -- PWM variable part 	
	
	SetMotorsOff();
	
	// Enable interrrupts 
	INTCONbits.GIEL   = 1;		// low priority  interrupts enable
	INTCONbits.GIEH   = 1; 		// High priority  interrupts enable
	
	// Intit some globals
	Flags.all = 0;
	PWM_Pause = 5;		// initially until the eeprom is read
	
 	All_Leds_Off();
 	
#ifdef INIT_PARAMS
	LedBlue_ON;	
	WriteParametersEE(1);					// copy RAM initial values to EE
	WriteParametersEE(2);
	LedBlue_OFF;
#endif 

#ifdef USE_ACCSENS
	
	if( (_UseLISL = IsLISLactive()) == true)
	{
		LedYellow_ON;	
		GetEvenValues();	// into Rp, Np, Tp
	}
	else
	{
		NeutralLR = 0;
		NeutralFB = 0;
		NeutralUD = 0;
	}
#endif

    InitDirection();	// init compass sensor
  	InitAltimeter();
	ShowSetup(1);


	while(1)
	{
		char temp;
		
		IGas =IK5 = _Minimum;	// assume parameter set #1
		
		All_Leds_Off( );

		InitGlobals();
		SetMotorsOff(); 			// stop motors 
		ThrNeutral = 0xFF;

		temp=0;
		do 	// Wait for RX signal and throttle closed
		{			
			ReadParametersEE(); 	//needed here becuase the ProcessComCommand needs to know IK5 --  
			_NoSignal = 1;			//Reset so that we are sure that in the last 30ms a valid frame arrived
			Delay_ms(30);

			LedGreen_TOG;    // quick feedback to indication SW is waiting for the right starting conditions
			
			GetInputCH();   // copy the input channels into the IK vars -- needed for IGas only in here 		
			
			if (_NoSignal )
			{
				Beeper_ON;
				temp = 0;
				LedRed_TOG;
			}
			else
			{ 
				LedRed_OFF;
				Beeper_OFF;
				if( IGas < _ThresStop) // No starting if Throttle was not at low pos
				{
					LedYellow_OFF;
					temp++;
				}
				else	
				{
					LedYellow_TOG;	// toggle red Led  indicating  Throttle not off
					temp=0;
				}	
			}
			ProcessComCommand(); 
		}
		while(temp < 20);	// no signal  or throttle is not closed  for n itterations
		


// ###############
// ## MAIN LOOP ##
// ###############
		Beeper_OFF;
		_UseParam1 = (IK5 < _Neutral_In) ? 1 : 0;
		PWM_Pause = Interval_time;  		
		BiasZeroCount = BiasZeroAvgCount;	// do n cycles to find integral zero point
		ThrDownCount = THR_DOWNCOUNT;
		RxSignalLost = RxFrameErr = PID_Delay = 0;

			
		while(2)	// until  forever
		{
			
			PID_Sleep();		// Sleeps until the PID delay is expired..  ISR managed counter based on Tmr0 -- 
								// Jitter free, not affected by execution  time of code in PID loop
		
			if ( _NoSignal || RxFrameErr >= 4)
					RxSignalLost = 1;

			if ( RxSignalLost ) //  comulative missed frames
			{           						// Auto let down mode
				Beeper_ON;
				LedBlue_ON;
				if ( DropCount++ > 16 )	 	// speed of let down
				{	
					DropCount=0;
					// powerdown slowly 			
					if (IGas > _ThresStart)
						IGas--;	
				}
				
				// there is no return to the outer loop if signal is lost once.
				// The craft reduces power and idles at threshstart until power is removed or reset is pressed!
								
				IRoll = RollNeutral;	// these are the values that where recorded upon takeoff and represent trimmed stick values
				IPitch = PitchNeutral;
				IYaw = YawNeutral;
			}
			else
			{
				GetInputCH();   // copy the input channels into the IK vars 
			}
				

			GetGyroValues();	// first thing after sleep so that there is not jitter in 'dt'.
			ReadAccel(); 		// Read the accel  and  compute the correction factors 
			CalcGyroValues();	// makes 8 bit  Gyro ADC values  integrates samples into Roll_Angle, Pitch_Angle.Yaw_Angle applies correction factors to angles 

// 			Roll_Rate_8 , Pitch_Rate_8 now have Delta Theta of Roll and Pitch in 8 bit representation
// 			Roll_Rate_8_prev , Pitch_Rate_8_prev now have the PREVIOUS Delta Theta 
// 			RollRate10, PitchRat10 now have Delta Theta in 10 bit representation  -- used to integrate to Angle. 
//          Roll_Angle, Pitch_Angle  now have Current roll an pitch angles 			

 			

			if( _UseCompass  )	
				GetDirection();	// read compass sensor

			if( _UseBaro )
				ComputeBaroComp();

// allow motors to run on low throttle 
// even if stick is at minimum for a short time
			if( Flying && (IGas <= _ThresStop) )
			{
				if( --LowGasCount > 0 )
					goto DoPID;
			}

			if( (  Flying && (IGas <= _ThresStop)) ||  // ufo is landed  -- hopefully
			     (!Flying && (IGas <= _ThresStart)) )  // ufo has not taken off yet
			{	// stop all motors
				SetMotorsOff();
				ThrDownCount = THR_DOWNCOUNT;
				InitGlobals();	// resets _Flying flag!  All  angles are set to 0 in here   !!  ASSUMING LEVEL !! ground of UAVP 
				
				BiasZeroCount = BiasZeroAvgCount;	// do "n" cycles to find integral zero point
				RollBias = PitchBias = YawBias = 0;	// gyro neutral points

				All_Leds_Off();
				LedGreen_ON;	 // This steady Green indicates that we have transitioned into ready to fly mode 
			}
			else
			{	// UFO is  about to be flying!
				if( !Flying )	// about to start
				{	// set current stick values as midpoints
					RollNeutral = IRoll;
					PitchNeutral = IPitch;
					YawNeutral  = IYaw;
					AbsDirection = COMPASS_INVAL;
					LedCount = 1;
					UpDown_Velocity = 0;
				}
				// ufo is flying
				Flying = 1;
				
				LowGasCount = 100;
				--LedCount;

DoPID:
// this block checks if throttle stick has moved for Barometer altitude lock , supposed to be 3 sec until lock takes effect 
// after last stick movement 
				if( ThrDownCount > 0 )
				{
					if( (LedCount & 0x1) == 0 )
						ThrDownCount--;
					if( ThrDownCount == 0 )
						ThrNeutral = IGas;	// remember current Throttle level
				}
				else
				{
					if( ThrNeutral < THR_MIDDLE )
						NegFact = 0;
					else
						NegFact = ThrNeutral - THR_MIDDLE;
					if( IGas < THR_HOVER )
						ThrDownCount = THR_DOWNCOUNT;	// left dead area
					if( IGas < NegFact )
						ThrDownCount = THR_DOWNCOUNT;	// left dead area
					if( IGas > ThrNeutral + THR_MIDDLE )
						ThrDownCount = THR_DOWNCOUNT;	// left dead area
				}

				PID();
				MixAndLimit();
				// remember old gyro values for next itteration , used with Derivative factor
				Roll_Rate_8_prev = Roll_Rate_8;      
				Pitch_Rate_8_prev = Pitch_Rate_8;
				Yaw_Rate_8_prev = Yaw_Rate_8;
			}
	
			MixAndLimitCam();	// Always mix the Camera Servos, even when not flying.
			OutSignals();		// Output the results to the speed controllers

// indicate problems 
			GetVbattValue();		// Get Battery status 

			if( _LowBatt )
				Beeper_ON;			// Latch beeper for now  conflicts with beeper usage for RxSignalLost
				
				
// Check K5 and reload Param set 
			// Only read params from the EEPROM when switch on CH5 has changed - this saves about 192us in the loop 
			if( (IK5 > _Neutral_In &&  _UseParam1   )|| (IK5 < _Neutral_In  && !_UseParam1  )   )
			{ 	 
				ReadParametersEE();	// re-sets Interval_time
				PWM_Pause = Interval_time;
				_UseParam1 = (IK5 < _Neutral_In) ? 1 : 0;
			}
			
			if( !Flying )
				ProcessComCommand();
				
		}	// End  while (2)
	}  // End  while (1)
}

