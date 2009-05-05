typedef unsigned char uns8 ;
typedef unsigned char uint8;
typedef signed char int8;
typedef unsigned int uns16;
typedef unsigned int uint16;
typedef signed int int16;
typedef signed long int32;
typedef unsigned long uns32;

#define true 1
#define false 0

typedef union {
	int16 I;
	uns16 uI;
	struct {
	unsigned char low8;
	unsigned char high8;
	};
}char_int;

typedef union {
	int32 L;
	uns32 uL;
	int16 I;
	uns16 uI;
	struct {
	unsigned char low8;
	unsigned char high8;
	};
}char_int_long;

typedef union {
	unsigned int  all;
	struct
	{
		unsigned bit0: 1;
		unsigned bit1: 1;
		unsigned bit2: 1;
		unsigned bit3: 1;
		unsigned bit4: 1;
		unsigned bit5: 1;
		unsigned bit6: 1;
		unsigned bit7: 1;
		unsigned bit8: 1;
		unsigned bit9: 1;
		unsigned bit10: 1;
		unsigned bit11: 1;
		unsigned bit12: 1;
		unsigned bit13: 1;
		unsigned bit14: 1;
		unsigned bit15: 1;
	} ;
}t_flags;



