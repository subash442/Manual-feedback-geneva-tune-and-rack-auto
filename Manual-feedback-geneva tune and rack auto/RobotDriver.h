/*
 * RobotDriver.h
 *
 * Created: 6/25/2018 2:31:46 PM
 *  Author: Subash Timilsina
 */ 


#ifndef ROBOTDRIVER_H_
#define ROBOTDRIVER_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "drive.h"
#include "Rack.h"
#include "uart.h"
#include "JOYSTICK.h"
#include "Pneumatic.h"



 void operate()
 {
	 /**********************************************GAMEBUTTONB_SECTION*********************************/
	 
	 
	 
	 if (GAMEBUTTONB == RIGHT)
	 {
		 velocity_robot[0] = 0;
		 velocity_robot[1] = robot_rpm;
		 velocity_robot[2] = 0;
	 }
	 else if (GAMEBUTTONB == LEFT)
	 {
		 velocity_robot[0] = 0;
		 velocity_robot[1] = -robot_rpm;
		 velocity_robot[2] = 0;
	 }
	 else if (GAMEBUTTONB == UP)
	 {
		 velocity_robot[0] = -robot_rpm;
		 velocity_robot[1] = 0;
		 velocity_robot[2] = 0;
		 //front_drive = true;
		 
	 }
	 else if (GAMEBUTTONB == DOWN)
	 {
		 velocity_robot[0] = robot_rpm;
		 velocity_robot[1] = 0;
		 velocity_robot[2] = 0;
		 //back_drive = true;
	 }
	 
	 /**********************************************************************GAMEBUTTONA_SECTION****************************/
	 if (GAMEBUTTONA == BUTTON_A)
	 {
		 RACK_GRIP_TOGGLE();
		 GAMEBUTTONA = 0;
	 }
	 else if (GAMEBUTTONA == BUTTON_B && !Rack_home_position)
	 {
		 RACK_LIFT_TOGGLE();
		 GAMEBUTTONA = 0;
	 }
	 else if (GAMEBUTTONA == BUTTON_X)
	 {
		 Buttonx_pressed = true;
		 if(Rack_home_position)
		 {
			 RackMotor.SetOcrValue(-RACK_SPEED_MOTOR);
		 }
		 else
		 {
			 RackMotor.SetOcrValue(RACK_SPEED_MOTOR);
		 }
		 GAMEBUTTONA = 0;
	 }
	 else if (GAMEBUTTONA == BUTTON_Y)
	 {
		 angle_pid.Set_SP(360);
		 Geneva_Start = true;
		 GAMEBUTTONA = 0;
	 }
	 else if (GAMEBUTTONA == RIGHT_STICK_CLICK && !Rack_home_position)
	 {
		 Buttonx_pressed = true;
		 rack_throw_auto = true;
		 SHUTTCOCK_PASS_OPEN();
		 RACK_LIFT_OPEN();
		 RackMotor.SetOcrValue(RACK_SPEED_MOTOR);
		 GAMEBUTTONA = 0;
	 }
	 else if (!pass_the_shuttcock && !rack_throw_auto && !pneumatic_geneva_start && GAMEBUTTONA == RIGHT_BUTTON)	//until the shuttlecock passing completes and until rack auto rack throw completes and until the geneva completes it's rotation
	 {
		 if(inside_robot)
		 {
			 pneumatic_geneva_start = true;
			 pneumatic_geneva_time = millis();
			 SHUTTCOCK_GRIP_TOGGLE();
		 }
		 else
		 {
			 SHUTTCOCK_GRIP_CLOSE();
		 }
		 
		 pass_the_shuttcock = true;
		 passing_time = millis();
		 inside_robot = (1^inside_robot);
		 GAMEBUTTONA = 0;
	 }

	 
	 

	 /**************************************************Rack Operation******************************************************/
	 if (!Buttonx_pressed)
	 {
		 if (LEFTTRIGGER > 20 && READ(LIMIT_SWITCH_PIN))
		 {
			 RackMotor.SetOcrValue(RACK_SPEED_MOTOR);
		 }
		 else if (RIGHTTRIGGER > 20 )
		 {
			 RackMotor.SetOcrValue(-RACK_SPEED_MOTOR);
		 }
		 else
		 RackMotor.StopMotor();
	 }
	 

	 if (RackEncoder.Get_count() <= RACK_POSITION_COUNT && Rack_home_position) // if reached at mid-where somewhere
	 {
		 Rack_home_position = false;
		 donotstop = true;
	 }
	 
	 if(RackEncoder.Get_count() <= RACK_COUNT && Buttonx_pressed && donotstop  )		//reached final position
	 {
		 RackMotor.StopMotor();
		 Buttonx_pressed = false;
		 donotstop = false;
		 if (rack_throw_auto)
		 {
			 RACK_LIFT_CLOSE();
			 angle_pid.Set_SP(360);
			 Geneva_Start = true;
		 }
	 }
	 /*********************************************************************Move using joystick analog stick********************************************/
	 	 if ((abs(LEFTSTICKY-50) > 5) || (abs(LEFTSTICKX-50) > 5) || abs(RIGHTSTICKX-50)>5)
	 	 {
				rampupflag_start = true;
				velocity_robot[0] = (-(LEFTSTICKY-50)*timer_count/3000.0)*MAX_RPM_XY;
			    velocity_robot[1] = ((LEFTSTICKX-50)*timer_count/3000.0)*MAX_RPM_XY;
				velocity_robot[2] = -((RIGHTSTICKX-50)*timer_count/3000.0)*MAX_RPM_YAW;  
	 	 }
	 	 else
	 	 rampupflag_start = false;
	 

	 
	 /*******************************************Geneva operation*********************************************/
	 
	 if (Geneva_Start && GenevaEncoder.angle >= 360 && angle_pid.Get_Error() <= 3)
	 {
		 if(rack_throw_auto)
		 {
			 SHUTTCOCK_PASS_CLOSE();
			 SHUTTCOCK_GRIP_CLOSE();
			 inside_robot = true;
			 rack_throw_auto = false;
		 }
		 GenevaEncoder.angle = 0;
		 angle_pid.Set_SP(0);
		 angle_pid.reset_iterm();
		 angle_pid.reset_output();
		 Geneva_Start = false;
	 }
	 
	 if (Geneva_Start && GenevaEncoder.angle >= 100)
	 {
		 //stop geneva
		 pneumatic_geneva_start = false;
	 }
	 
	 if (angle_pid_compute)
	 {
		 GenevaMotor.SetOcrValue(angle_pid.angle_Compute(GenevaEncoder.angle));
		 angle_pid_compute = false;
	 }
	 
	 /*************************************************Delay operation*****************************************/
	 
	 if (rack_throw_auto && Rack_home_position && (millis()-previous_time)>700)
	 {
		 RackMotor.SetOcrValue(-RACK_SPEED_MOTOR);
		 Buttonx_pressed = true;
	 }
	 
	 
	 if(pneumatic_geneva_start && (millis()-pneumatic_geneva_time) > 700)
	 {
		 angle_pid.Set_SP(360);
		 Geneva_Start = true;
	 }
	 
	 if (pass_the_shuttcock && (millis()-passing_time) > 300)
	 {
		 SHUTTCOCK_PASS_TOGGLE();
		 pass_the_shuttcock = false;
	 }
	 
	 if(throw_rack && (millis()-previous_time) >= 700)
	 {
		 RACK_GRIP_OPEN();
		 throw_rack = false;
	 }
	 
	 //if (robot_rpm>MAX_RPM) robot_rpm = MAX_RPM;
	 //else if (robot_rpm<20)	robot_rpm = 20;

 }

#endif /* ROBOTDRIVER_H_ */