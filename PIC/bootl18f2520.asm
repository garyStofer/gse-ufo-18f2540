        title   "BootLoader.asm"
;*************************************************************************
; BootLoader used by UAVP Quadrocopter
;
; Rewritten by G.K. Egan 2008 for 18F2520 PIC
;
;*************************************************************************
; Version
; 1.0 .... Erste Version
; 2.0 .... G.K. Egan 2008

; Assumptions:
;	loader records do not cross 32 byte boundaries
;	16 bit addressing of program memory

#define __18F2520
;#define BOOT_ONLY

        LIST C=200,R=dec

; config is already defined in main routine!
;        config=3F72h
        
        include "p18f2520.inc"
        include "general.asm"

		_B38400			equ		25
		_RestoreVec		equ		0
		_MaxRxBuffer	equ		80	;normal max 64 hex chars + tags

		; RAM variables all in Bank 0
		cblock 16
		Flags
		AddressL
		AddressH
		AddressB
		BlockOffset
		BufferPoint
		NoOfBytes
		Ch
		ChkSum
		cc
		wb
		LEDs
		BootReset:4					;4 byte goto BootStart
		PICBuffer:64
		RxBuffer:_MaxRxBuffer
 		
		endc
	
		code

;		goto	BootStart

		org		07d00h				

		global	BootStart

BootStart
		; absolutely no C registers or variables are preserved
	;	movf	PCL,f					;for message table address
		movlw	high Messages
		movwf	PCLATH

		bra	Init

Messages								
		;FAILS if a 256 byte boundary is crossed!
		;moved to base of boot which always starts at the base
		;of a 256 byte segment
		addwf	PCL,f
		_Hello		dt	"Boot 2.0\0"
		_Err		dt	"ERR\0"
		_CSum		dt	"CSUM\0"
		_OK			dt	"OK\0"
		_Done		dt	"SUCCESS!\0"
;#define LOADREC
#ifdef LOADREC
		_hex		db	":020000040000FA",0x0a,0x0d ;17
		;_hex		db	":1000500003011423030177515FE18BC01EF00001FF",0x0a,0x0d ;45
		;_hex		db	":00000001FF",0x0a,0x0d ;13
#endif
		Txt_Hello	equ	_Hello-Messages-1
		Txt_Err		equ	_Err-Messages-1
		Txt_CSum	equ	_CSum-Messages-1
		Txt_OK		equ	_OK-Messages-1
		Txt_Done	equ	_Done-Messages-1

Init		
		movlb	0
		DII							;disable interrupts
		clrf	EECON1				;disable all program memory writes

		clrf	T0CON
		movlw	10100100b			;set RC6 as output (TxD), LEDs
		movwf	TRISC
		clrf	TRISB				;all output
		movlw	_B38400
		movwf	SPBRG
		movlw	00100100b			;async mode, BRGH = 1
		movwf	TXSTA
		clrf	STATUS				;clears RP0 & IRP also!

		movlw	10010000b			;receive async on
		movwf	RCSTA
 		clrf	PORTB				;SERVOS OFF!

#ifdef LOADREC
		lfsr	FSR0,RxBuffer		
		clrf	TBLPTRU
		movlw	0x7c
		movwf	TBLPTRH
		movlw	_hex
		movwf	TBLPTR

		movlw 	17					
		movwf 	cc
xx
		tblrd*+
		movf	TABLAT,w
		movwf 	POSTINC0
		decfsz 	cc
		bra 	xx
	
		bra		VerifyRec
#endif
		
		movlw	Txt_Hello
		call	SendString

		clrf	Flags				;clear reset vector seen flag
					
MainLoop
		; Read a complete string line
		clrf	STATUS
	
		lfsr	FSR0,RxBuffer

		movlw	00100000b			;yellow LED on, green off
		call	BootLeds		
RxPoll
		btfss	PIR1,RCIF			;wait for a char
		bra		RxPoll
		
		movf	RCREG,w				;read char

		btfsc	RCSTA,OERR			;check for USART errors
		bra		RxError
		btfsc	RCSTA,FERR
		bra		RxError

		movwf	INDF0				;store char in buffer
		sublw	':'
		beq		RXCharOK			;colon is ok

		swapf	INDF0,w
		andlw	0x0f				;check for 0-9 A-F (0x30..0x4F)
		addlw	-3
		beq		RXCharOK
		addlw	-1
		beq		RXCharOK

		movlw	0x0a
		subwf	INDF0,w				;check for LF
		beq		VerifyRec			;ready!
		
		addlw	0x0a-0x0d			;check for CR
		bne		RxError

RXCharOK
		incf	FSR0L,f
		bra		RxPoll				;receive next char
; ____________________________________________________________________________
;
; Check and Translate Record
; ____________________________________________________________________________

VerifyRec
		lfsr	FSR0,RxBuffer		;the complete line inclusive of LF is in RxBuffer	

		movf	INDF0,w
		sublw	':'					;first char must be a colon
		bne		RxError

		incf	FSR0L,f
		clrf	ChkSum				;initialise checksum

		call	ASCII2Bin
		movwf	NoOfBytes			;number of bytes to prog

		sublw	64
		bcc		RxError				;too many bytes!

		call	ASCII2Bin
		movwf	AddressH			;address high byte
		call	ASCII2Bin
		movwf	AddressL			;address low byte
		movlw	11000000b
		andwf	AddressL,w
		movwf	AddressB			;block base address low byte
		movlw	00111111b
		andwf	AddressL,w
		movwf	BlockOffset			;offset within the block

		call	ASCII2Bin			;get record type
		addlw	0
		beq		CopyRec				;data record
		addlw	-1
		beq		AckOK				;last record
		; otherwise unsupported record type - ignore
		bra		NextRec

CopyRec								;copy balance of record and check Checksum	
		lfsr	FSR1,RxBuffer			
		movff 	NoOfBytes,cc
NextByte
		call	ASCII2Bin
		movwf	POSTINC1
		decfsz	cc,f
		bra		NextByte
		
		call	ASCII2Bin			;check record Checksum
		movf	ChkSum,w
		beq		ValidRec

		movlw	Txt_CSum
		call	SendString
		bra		MainLoop

RxError		
		bcf		RCSTA,CREN			; disable, then re-enable serial port to clear errors
		movlw	Txt_Err
		call	SendString
		bsf		RCSTA,CREN

		bra		MainLoop
; ____________________________________________________________________________
;
; Process Record
; ____________________________________________________________________________	
ValidRec
		movlw	00101000b			;yellow LED on, green on
		call	BootLeds			;exit is always C=0, Z=1 due to loop end
		
CheckReset
		; check if address is 0
		movf	AddressH,w
		bne		CheckBoot
		movf	AddressL,w
		bne		CheckBoot

		call	SaveResetVec
		call	WriteRec ;???
		bra		NextRec

CheckBoot				
		; do not program words with addresses >= BootStart
		movlw	high BootStart		
		subwf	AddressH,w
		bcs		NextRec				;ignore this record	

		; Only PROGRAM memory records can make it here!
		call	WriteRec
		bra		NextRec

NextRec
		movlw	Txt_OK			
		call	SendString
		bra		MainLoop

WriteRec		
		call	ReadPICBlock
	
		lfsr	FSR0,RxBuffer
		lfsr	FSR1,PICBuffer

		movf	BlockOffset,w		;cannot work out how to offset FSR1 directly!!!
		addlw	0
		beq		uglybypass
		movwf	cc
uglyoffset
		movf	POSTINC1
		decfsz	cc,f
		bra		uglyoffset
uglybypass

		movff 	NoOfBytes,cc
NextBytePIC
		movf	POSTINC0,w
		movwf	POSTINC1		
		decfsz	cc,f
		bra		NextBytePIC
			
		call	WritePICBlock
		return
; ____________________________________________________________________________
;
; Save Reset Vector etc.
; ____________________________________________________________________________			

SaveResetVec
		;16 bit addressing only
		movff	RxBuffer,BootReset
		movff	RxBuffer+1,BootReset+1 
		movff	RxBuffer+2,BootReset+2
		movff	RxBuffer+3,BootReset+3		

		movlw	0xef				;there's probably a neater way?
		movwf	RxBuffer+1

		bcf     STATUS,C
		movlw	high BootStart
		movwf	wb
		rrcf	wb
		movff	wb,RxBuffer+2	

		movlw	low BootStart		;should really be zero!
		movwf	wb
		rrcf	wb
		movff	wb,RxBuffer		

		movlw	0xf0
		movwf	RxBuffer+3
	
		bsf		Flags,_RestoreVec	;reset vector needs to be restored
		return
; ____________________________________________________________________________
;
; Restore Reset Vector
; ____________________________________________________________________________			

AckOK
		call	SendString_OK

		btfss	Flags,_RestoreVec	;check reset vector needs to be restored?
		bra		Finished

		clrf	AddressL
		clrf	AddressH
		clrf	AddressB

		call	ReadPICBlock

		movff	BootReset,PICBuffer  		
		movff	BootReset+1,PICBuffer+1
		movff	BootReset+2,PICBuffer+2
		movff	BootReset+3,PICBuffer+3 

		call	WritePICBlock

Finished
		movlw	Txt_Done
		call	SendString
Forever
	;	clrwdt
		bra		Forever					;wait for reboot
; ____________________________________________________________________________
;
; PIC Read/Write
; ____________________________________________________________________________
		;Stupid hardware allows reads of 64 bytes but after mandatory erase of
		;64 bytes requires 2x32 byte writes to restore the block. Refer manual
		;DS39631D page 73. Previous manuals in error stating all transactions 
		;64 bytes.

WritePICBlock
		call	LoadBlockTBLPTR
		movlw	10010100b			;setup erase
		call 	WritePIC					
		tblrd*-

		lfsr	FSR0, PICBuffer							
		movlw	2
		movwf	wb

NextWriteBlock						;must write in 2x32 byte blocks
		movlw 	32					
		movwf 	cc

WritePICByte
		movf 	POSTINC0,w
		movwf 	TABLAT
		tblwt+*
		decfsz 	cc
		bra 	WritePICByte
	
		movlw	10000100b			;setup write	
		call 	WritePIC
		decfsz 	wb
		bra		NextWriteBlock
	
		return

WritePIC
		movwf 	EECON1
		movlw 	055h
		movwf 	EECON2
		movlw 	0AAh
		movwf 	EECON2
		bsf 	EECON1,WR			;write

		return

ReadPICBlock
		call 	LoadBlockTBLPTR	
		lfsr	FSR0,PICBuffer

		movlw 	64					
		movwf 	cc
ReadPICByte
		tblrd*+
		movf	TABLAT,w
		movwf 	POSTINC0
		decfsz 	cc
		bra 	ReadPICByte
		return

LoadBlockTBLPTR
		clrf	TBLPTRU
		movff	AddressH,TBLPTRH	
		movff	AddressB,TBLPTRL
		return
; ____________________________________________________________________________
;
; HEX conversion
; ____________________________________________________________________________
		; converts a character [0..9,A..F] at INDF to a nibble 0x00-0x0F
		; advances FSR by 1 char!
		; does not disturb bank selection
ASCII2Nibble
		movlw	'0'
		subwf	INDF0,w				;0x00..ox16
		btfsc	INDF0,6
		addlw	-7
		incf	FSR0L,f
		return

		; convert 2 ascii chars (FSR) to a binary byte
		; advances FSR by 2 chars!
		; does not disturb bank selection
ASCII2Bin
		call	ASCII2Nibble
		movwf	Ch
		swapf	Ch,f

		call	ASCII2Nibble
		iorwf	Ch,w
		addwf	ChkSum,f
		return	
; ____________________________________________________________________________
;
; LEDs
; ____________________________________________________________________________
BootLeds
		movwf	LEDs
		movlw	8
LEDScan
		rlf	LEDs,f
		bcf	PORTC,4				;clear SDA
		skipnc
		bsf	PORTC,4				;set SDA
		bsf	PORTC,3				;set SCLK
		bcf	PORTC,3				;clr SCLK
		addlw	-1
		bne	LEDScan

		bsf	PORTC,1				;RCLK on
		bcf	PORTC,1				;RCLK off
		return			
; ____________________________________________________________________________
;
; Messages
; ____________________________________________________________________________
SendString_OK
		movlw	Txt_OK

SendString
		movwf	cc					;string selected
SS_1
		movf	cc, w
		call	Messages
		addlw	0
		bne		SS_2				;end of text?

		movlw	0x0d				;send CR, LF
		call	TxChar
		movlw	0x0a
		call	TxChar				;=call and return!
		return
SS_2
		call	TxChar
		incf	cc,f
		incf	cc,f				;word index
		bra		SS_1
TxChar
		btfss	PIR1,TXIF			;wait until shift register is empty
		bra		TxChar
		movwf	TXREG				;send char

		return
		
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop

		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop

		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop

		;nop
		nop

		end
