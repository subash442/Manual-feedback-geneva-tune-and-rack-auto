/*
 * Definations.cpp
 *
 * Created: 5/28/2018 12:11:13 AM
 *  Author: Subash Timilsina
 */ 
#include "Rack.h"



Rack_Motor RackMotor,GenevaMotor;
Rack_Encoder RackEncoder,GenevaEncoder;

unsigned long previous_time;
unsigned long pneumatic_geneva_time;
unsigned long passing_time;
PID angle_pid;
float ramp_speed;

volatile unsigned long timer2_millis = 0;
volatile unsigned long timer2_fract = 0;

bool Geneva_Start;
bool Rack_home_position;
bool Buttonx_pressed;
bool donotstop;
bool throw_rack;
bool rack_throw_auto;
bool pneumatic_geneva_start;
bool inside_robot;
bool pass_the_shuttcock;
bool angle_pid_compute;



//initialize global timer

void initialise_timeperiod()
{
	TCCR2B |= (1<<CS22);
	TIMSK2 |= (1<<TOIE2);
	TCNT2 = 0;
}


void rack_init()
{
	
	Geneva_Start = false;
	Buttonx_pressed = false;
	donotstop = false;
	throw_rack = false;
	rack_throw_auto = false;
	pneumatic_geneva_start = false;
	inside_robot = true;
    pass_the_shuttcock = false;
	angle_pid_compute = false;
	
	pneumatic_geneva_time = 0;
	ramp_speed = RACK_SPEED_MOTOR;
	
	RackMotor.Initialise(1);
	GenevaMotor.Initialise(2);
	
	RackEncoder.Initialise(1);
	GenevaEncoder.Initialise(2);
	
	
	INPUT(LIMIT_SWITCH_PIN);
	SET(LIMIT_SWITCH_PIN);
	initialize_pneumatics();
	
	//home position of rack rotate
	while(READ(LIMIT_SWITCH_PIN))
	{
		RackMotor.SetOcrValue(RACK_SPEED_MOTOR);
	}
	
	previous_time = millis();
	RackMotor.StopMotor();
	RackEncoder.Set_count(0);
	Rack_home_position = true;		// true rack home-position -- initial position and false rack position -- final position
	angle_pid.Set_Pid(47.29,0.139,29.30);
	initialise_timeperiod();
}


void rack_limit_check()
{
	
	if(!READ(LIMIT_SWITCH_PIN) && !Rack_home_position )
	{
		RackMotor.StopMotor();
		Rack_home_position = true;
		RackEncoder.Set_count(0);
		Buttonx_pressed = false;
		throw_rack = true;	
		previous_time = millis();
		ramp_speed = RACK_SPEED_MOTOR;
	}
	if(READ(LIMIT_SWITCH_PIN) && throw_rack && rack_throw_auto)
	{
		RackMotor.SetOcrValue(RACK_SPEED_MOTOR);
	}
}

void close_all_pneumatics()
{
	RACK_GRIP_OPEN();
	RACK_LIFT_CLOSE();
	SHUTTCOCK_GRIP_CLOSE();
	SHUTTCOCK_PASS_CLOSE();
}

void initialize_pneumatics()
{
	OUTPUT(RACK_GRAB);
	OUTPUT(LIFT_RACK);
	OUTPUT(SC_PASS);
	OUTPUT(SC_GRIPPER);
	close_all_pneumatics();
}

//calculate the time from beggining of robot start 

unsigned long millis()
{
	unsigned long m;
	uint8_t oldSREG = SREG;
	
	// disable interrupts while we read timer2_millis or we might get an
	// inconsistent value 
	cli();
	m = timer2_millis;
	SREG = oldSREG;
	
	return m;
}




//Global timer2 interrupt

ISR(TIMER2_OVF_vect) {
	timer2_millis += 1;
	timer2_fract += 3;
	if (timer2_fract >= 125) {
		timer2_fract -= 125;
		timer2_millis += 1;
	}
}

ISR(INT_VECT5)
{
	if(bit_is_clear(ENCODER5_CHAPORTPIN,ENCODER5_CHBPIN))		//ENCODER_CHAPORTPIN,ENCODER_CHBPIN
	{
		RackEncoder.incCount();
	}
	else
	RackEncoder.dcrCount();
	
}

ISR(INT_VECT6)
{
	if(bit_is_clear(ENCODER6_CHAPORTPIN,ENCODER6_CHBPIN))		//ENCODER_CHAPORTPIN,ENCODER_CHBPIN
	{
		GenevaEncoder.angle++;
		
	}
	else
	{
		GenevaEncoder.angle--;
	}
	
	
	
}

