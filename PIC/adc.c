// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =      Rewritten and ported to 18F2xxx 2008 by Prof. Greg Egan        =
// =                          http://www.uavp.org                        =
// =======================================================================
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

#include "main.h"
#include "bits.h"

int16 ADC( uns8  ch) // 
{	
	ADCON0 = ch ; 
	while( ADCON0bits.GO ) 
	{ /* wait */}     // wait to complete
	
	return ADRES;
}	

void InitADC()
{
 OpenADC(
#ifdef FOSC_32	
		  ADC_FOSC_32 & 
#else
		  ADC_FOSC_16 &
#endif
          ADC_RIGHT_JUST &
          ADC_12_TAD,				// this is Fosc/4/ADC_FOSC_16*ADC_12_TAD = 48us
		  ADC_CH0 &
		  ADC_INT_OFF &
          ADC_VREFPLUS_VDD &
          ADC_VREFMINUS_VSS,	  
          ADCPORTCONFIG);
} // InitADC


