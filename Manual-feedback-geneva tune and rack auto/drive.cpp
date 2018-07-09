/*
 * drive.cpp
 *
 * Created: 5/29/2018 12:58:09 PM
 *  Author: Subash Timilsina
 */ 

#include "drive.h"

int8_t coupling_matrix[4][3] = {{-1,1,1},{1,1,1},{-1,1,-1},{1,1,-1}};	//just for the direction so kept 1
volatile int8_t timer_count;													//for joystick ramping
bool rampupflag_start;
bool pidflag;													//For ramping_the robot using joystick 

//bool back_drive;
//bool front_drive;

 int id,jd;					//global loop counter 
 uint16_t robot_rpm;		       //manual robot rpm set
 int velocity_motor[4];		//individual motor velocity
 int velocity_robot[4];		//robot velocity
 int ocr_motor[4];			//motor ocr 
 Motor m[4];				//base motors
 Encoder e[4];				// base motors encoders
 PID p[4];					//pid for the base motors
 
 /*					16 bit Timers used 
 ***************	timer 5:(5A 5B 5C for 3 motors) and 3:(3A)motor == base motors
 ***************	timer 1:(1A,1B) == Rack motors
 ***************	timer 4:(4A == pid_computation , 0A == Joystick manual ramping )
*/
 
 /*					base motors and encoders
 **************		motors:
 **************		motor1 -- m[0]	motor2 -- m[1]	motor3 -- m[2]	motor4 -- m[3]
 **************		encoder1 -- e[0] encoder2 -- e[1] encoder3 -- e[2] encoder4 -- e[3]
 **************		velocity_robot[0] -- Vx  velocity_robot[1] -- Vy  velocity_robot[2] -- W
 **************
 */ 
 
 
 /**************************Initialise the drive components************************/
 
 void drive_init()
 {
	 pidflag = false;
	 rampupflag_start = false;
	 
	 //back_drive = false;
	 //front_drive = false;
	 
	 timer_count = 12;
	 robot_rpm = 80;
	 for(id=0;id<4;id++)
	 {
		 velocity_motor[id] = 0;
		 velocity_robot[id] = 0;
		 m[id].Initialise(id+1);
		 e[id].Initialise(id+1);
		 p[id].Set_Pid(2.415,0,0.82);	//2.415 , 0 , 0.82
	 }
	 init_timer_ramping();
 }
 
 
 //Timer for ramping interrupt in 10ms
 //Global timer
 void init_timer_ramping()
 {
	 
	TCCR0A |= (1<<WGM01);
	TCCR0B |= (1<<CS00)|(1<<CS02);	//1024 prescaler CTC mode
	TIMSK0 |= (1<<OCIE0A);
	TCNT0 = 0; 
	OCR0A = 155;
}

 
 /***********************Calculation of inverse kinematics****************************************/

  void calculate_wheel_velocity()
  {
	  for(id=0;id<4;id++)
	  {
		  velocity_motor[id] = 0;
		  for(jd=0;jd<3;jd++)
		  {
			  velocity_motor[id] += velocity_robot[jd] * coupling_matrix[id][jd];
		  }
		  
	  }
	  //if (front_drive)
	  //{
		  //velocity_motor[2] = 0;
		  //velocity_motor[3] = 0;
	  //}
	  //
	  //if (back_drive)
	  //{
		  //velocity_motor[0] = 0;
		  //velocity_motor[1] = 0;
	  //}
	  
	  for(id=0;id<4;id++)
	  {
		  ocr_motor[id] = (velocity_motor[id]/MAX_RPM)*RPM_COUNT;
		  p[id].Set_SP(ocr_motor[id]);
		  velocity_robot[id] = 0;	//reset the robot velocities
	  }
	
	  
	  
  }
	/*******************************************Updating the calculated velocity in motors********************************************/
  void update_wheel_velocity()
  {  
		if(pidflag)
		{
			for(id = 0; id<4 ; id++)
			{
				m[id].SetOcrValue(p[id].Compute(e[id].Get_Speed()));
			}
			//if (back_drive)
			//{
				 //m[0].SetOcrValue(0);
				 //p[0].reset_error();
				 //p[0].reset_output();
				 //
				 //m[1].SetOcrValue(0);
				 //p[1].reset_error();
				 //p[1].reset_output();
//
				 //back_drive = false;
			//}
			//if (front_drive)
			//{
				 //m[2].SetOcrValue(0);
				 //p[2].reset_error();
				 //p[2].reset_output();
				 //
				 //m[3].SetOcrValue(0);
				 //p[3].reset_error();
				 //p[3].reset_output();
				 				 	//
				//front_drive = false;
			//}
			
			pidflag = false;
		}
		
  }


  //Global timer interrupt
  
  /***********************************Ramping from the joystick analog button*************************************/

  ISR(TIMER0_COMPA_vect)
  {
	  if (rampupflag_start)
	  timer_count++;
	  else
	  timer_count = 0;
	  
	  if (timer_count >60)
	  timer_count = 60;
	  

  }



/*********************************************************************Motors encoders interrupts*****************************************************************/

// motor orientation is alternate so channel a and channel b in two sides are different.

ISR(INT_VECT1)
{
	if(bit_is_clear(ENCODER1_CHAPORTPIN,ENCODER1_CHBPIN))		
	{
		e[0].incCount();
	}
	else
		e[0].dcrCount();
	
}



ISR(INT_VECT2)
{
	if(bit_is_clear(ENCODER2_CHAPORTPIN,ENCODER2_CHBPIN))		
	{
		e[1].incCount();
	}
	else
		e[1].dcrCount();
	
}
ISR(INT_VECT3)
{
	if(bit_is_set(ENCODER3_CHAPORTPIN,ENCODER3_CHBPIN))		
	{
		e[2].incCount();
	}
	else
		e[2].dcrCount();
}

ISR(INT_VECT4)
{
	if(bit_is_set(ENCODER4_CHAPORTPIN,ENCODER4_CHBPIN))		
	{
		e[3].incCount();
	}
	else
		e[3].dcrCount();
}

/******************************************************************************************************************************************************************/


