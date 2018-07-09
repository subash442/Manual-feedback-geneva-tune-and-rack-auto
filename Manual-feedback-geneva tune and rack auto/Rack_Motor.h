/* 
* Rack_Motor.h
*
* Created: 5/25/2018 5:03:48 PM
* Author: Subash Timilsina
*/


#ifndef __RACK_MOTOR_H__
#define __RACK_MOTOR_H__


#include "headers.h"

#define RACK_ICR_TOP   249
#define RACK_MAX_VALUE 249
#define RACK_MIN_VALUE -249




//For motor 5


#define DD_F5 B,4
#define DD_B5 H,6

#define DD_PWM5			B,5
#define PWM_TCCRA5		TCCR1A
#define PWM_TCCRB5		TCCR1B
#define PWM_ICR5			ICR1
#define PWM_OCR5			OCR1A

#define PWM_5COM0		COM1A0
#define PWM_5COM1		COM1A1

#define PWM_5WGM0		WGM10
#define PWM_5WGM1		WGM11
#define PWM_5WGM2		WGM12
#define PWM_5WGM3		WGM13
#define PWM_5CS0			CS10
#define PWM_5CS1			CS11
#define PWM_5CS2			CS12

//For motor 6

#define DD_F6		B,7
#define DD_B6		H,5

#define DD_PWM6			B,6
#define PWM_TCCRA6		TCCR1A
#define PWM_TCCRB6		TCCR1B
#define PWM_ICR6			ICR1
#define PWM_OCR6			OCR1B

#define PWM_6COM0		COM1B0
#define PWM_6COM1		COM1B1

#define PWM_6WGM0		WGM10
#define PWM_6WGM1		WGM11
#define PWM_6WGM2		WGM12
#define PWM_6WGM3		WGM13
#define PWM_6CS0			CS10
#define PWM_6CS1			CS11
#define PWM_6CS2			CS12


class Rack_Motor
{

	private:
	uint8_t num;
	public:
	void Initialise(uint8_t no);

	void InitPWM();

	void SetForwardDirection();
	void SetReverseDirection();
	void StopMotor();
	void SetOcrValue(int x);
};

#endif //__RACK_MOTOR_H__
