/*
 * Base_Omni.cpp
 *
 * Created: 9/17/2017 11:52:27 AM
 * Author : Subash Timilsina
 */ 

#include "RobotDriver.h"


int main(void)
{    	
	initUART0();
	initUART2();
	drive_init();
	rack_init();
	sei();
	while(1)
	{
		operate();
		rack_limit_check();
		calculate_wheel_velocity();
		update_wheel_velocity();
		rack_limit_check();
		
	}

	return 0;
}


 
 
 /*********************************************************PID computation timer*****************************************************/

 ISR(TIMER4_COMPA_vect)
 {
	 e[0].Calc_Speed();
	 e[1].Calc_Speed();
	 e[2].Calc_Speed();
	 e[3].Calc_Speed();
	 pidflag = true;
	 angle_pid_compute = true;
 }
