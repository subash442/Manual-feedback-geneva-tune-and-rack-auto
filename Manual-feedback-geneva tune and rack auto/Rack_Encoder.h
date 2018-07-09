/* 
* Rack_Encoder.h
*
* Created: 5/29/2018 1:55:29 PM
* Author: Subash Timilsina
*/


#ifndef __RACK_ENCODER_H__
#define __RACK_ENCODER_H__

#include <avr/io.h>
#include <avr/interrupt.h>
#include "headers.h"

#define PPR_CYTRON		180

//encoder 5

#define  ENCODER5_CHANNELA	E,4
#define	 ENCODER5_CHANNELB	C,5
#define	 ENCODER5_INT		INT4

#define	 ENCODER5_CHAPORTPIN	 PINC
#define	 ENCODER5_CHBPIN		 PC5
#define	 INT5_ISC1			ISC41
#define	 INT5_INTF			INTF4

#define  INT_VECT5			INT4_vect

//encoder 6
#define  ENCODER6_CHANNELA	E,5
#define	 ENCODER6_CHANNELB	C,7
#define	 ENCODER6_INT		INT5

#define	 ENCODER6_CHAPORTPIN	 PINC
#define	 ENCODER6_CHBPIN		 PC7
#define	 INT6_ISC1			ISC51
#define	 INT6_INTF			INTF5

#define  INT_VECT6			INT5_vect


class Rack_Encoder
{
	private:
	uint8_t no;
	long int speed;
	volatile long int pprcount;
	
	
	public:
	
	long int angle;
	
	Rack_Encoder():speed(0),pprcount(0){};
	void Initialise(uint8_t x);
	void Init_Interrupts();
	inline void Calc_Speed();
	inline long int Get_Speed(){return speed;};
	inline void Set_Speed(int val){speed = val;};
	inline void incCount(){pprcount++;};
	inline void dcrCount(){pprcount--;};
	inline int Get_count(){return pprcount;};
	inline void Set_count(int val){pprcount = val;};
};

#endif //__RACK_ENCODER_H__
