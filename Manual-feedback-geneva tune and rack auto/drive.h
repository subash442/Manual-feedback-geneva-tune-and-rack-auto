/*
 * drive.h
 *
 * Created: 5/29/2018 12:57:52 PM
 *  Author: Subash Timilsina
 */ 


#ifndef DRIVE_H_
#define DRIVE_H_

#define MAX_RPM		470.0   //for 12V supply in the base 235.0 RPM
#define RPM_COUNT	40		//for 12V supply in the base 20 count     //in 30 ms
#define MAX_RPM_XY	400		//RPM of robot on xy velocity
#define MAX_RPM_YAW 100		//MAX YAW of the robot



#include "Motor.h"
#include "Encoder.h"
#include "uart.h"
#include "PID.h"


 extern volatile int8_t timer_count;
 extern bool rampupflag_start;
 
 extern bool pidflag;
 
 //extern bool back_drive;
 //extern bool front_drive;
 
 extern PID p[4];
 extern Encoder e[4];	
 extern bool pidflag;
 
 extern uint16_t robot_rpm;
 extern int velocity_robot[4];

 void drive_init();
 void init_timer_ramping();
 void calculate_wheel_velocity();
 void update_wheel_velocity();


#endif /* DRIVE_H_ */