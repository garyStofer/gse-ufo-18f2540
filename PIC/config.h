
// All config Pragmas before  include files because of the redefinition of certain words like ON and OFF
// FOSC_16 is running with a 16 Mhz xtal  giving 16Mhz 
// FOSC_32 is running with a 8Mhz Xtal and 4x PLL for 32 mhz 

#ifdef FOSC_16
#pragma config OSC = HS 
#elif defined FOSC_32 
#pragma config OSC = HSPLL
#else 
#error Neither FOSC_16 nor FOSC_32 is defined in the Build Settings of the project
#endif



#pragma	config MCLRE=ON    		//MCLR pin enabled; RE3 input pin disabled	
#pragma	config STVREN = ON  	//Stack Full/Underflow Reset Enable 
#pragma	config LVP=OFF			//Single-Supply ICSP Enable
#pragma	config CCP2MX = PORTC	// CCP2 input/output is multiplexed with RC1
#pragma	config FCMEN = OFF		// Fail-Safe Clock Monitor disabled
#pragma	config IESO = OFF		//Oscillator Switchover mode disabled
#pragma	config PWRT = OFF		//Power-up Timer Enable
#pragma	config BOREN = OFF		//Brown-out Reset disabled in hardware and software
#pragma	config WDT=OFF			//Watchdog Timer Enable
#pragma	config LPT1OSC = OFF	// Low-Power Timer1 Oscillator Enable
#pragma	config PBADEN = OFF		//PORTB<4:0> pins are configured as digital I/O on Reset
#pragma	config XINST = OFF		//Extended Instruction Set Enable
#pragma	config WRTD = OFF      	// Data EEPROM Write Protection 
#pragma	config WRTC = OFF  		// Configuration Register Write Protection
#pragma	config WRTB = OFF 		// Boot Block Write Protection bit:
#pragma config DEBUG = OFF  	// Must be off otherwise won't run whe programmed with standalone programmer
								// The IDE automatically turns this on when running under the debugger
								
								