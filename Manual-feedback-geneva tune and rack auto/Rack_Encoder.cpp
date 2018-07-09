/* 
* Rack_Encoder.cpp
*
* Created: 5/29/2018 1:55:29 PM
* Author: Subash Timilsina
*/


#include "Rack_Encoder.h"

void Rack_Encoder::Initialise(uint8_t x)
{
	no = x;
	if(no==1)
	{
		INPUT(ENCODER5_CHANNELA);
		INPUT(ENCODER5_CHANNELB);
		SET(ENCODER5_CHANNELA);
		SET(ENCODER5_CHANNELB);
	}
	else if (no==2)
	{
		INPUT(ENCODER6_CHANNELA);
		INPUT(ENCODER6_CHANNELB);
		SET(ENCODER6_CHANNELA);
		SET(ENCODER6_CHANNELB);
	}
	Init_Interrupts();
}

void Rack_Encoder::Init_Interrupts()
{
	sei();
	if (no==1)
	{
		EIMSK &= ~(1<<ENCODER5_INT);
		EICRB |= (1<<INT5_ISC1);	//falling edge
		EIMSK |= (1<<ENCODER5_INT);		//setting INT pin
		EIFR |= (1<<INT5_INTF);	    //clear int flag
	}
	else if (no==2)
	{
		EIMSK &= ~(1<<ENCODER6_INT);
		EICRB |= (1<<INT6_ISC1);	//falling edge
		EIMSK |= (1<<ENCODER6_INT);		//setting INT pin
		EIFR |= (1<<INT6_INTF);	    //clear int flag
	}
}



inline void Rack_Encoder::Calc_Speed()				//keep in timer compare match
{
	speed = pprcount;
	pprcount = 0;
}