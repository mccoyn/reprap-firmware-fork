
#include "parameters.h"
#include "pins.h"
#include "ThermistorTable.h"
#include "extruder.h" 

// Keep all extruders up to temperature etc.

void manage_all_extruders()
{
    for(byte i = 0; i < EXTRUDER_COUNT; i++)
       ex[i]->manage();
}

// Select a new extruder

void new_extruder(byte e)
{
  if(e < 0)
    e = 0;
  if(e >= EXTRUDER_COUNT)
    e = EXTRUDER_COUNT - 1;
  
  if(e != extruder_in_use)
  {  
    extruder_in_use = e;
    //setExtruder();
  }
}

/***************************************************************************************************************************

If we have a new motherboard (V 1.x, x >= 1), the extruder is entirely controlled via the RS485, and all  the functions to do
it are simple inlines in extruder.h

Otherwise, we have to do the work ourselves...
*/

#if USE_EXTRUDER_CONTROLLER == false
extruder::extruder(byte md_pin, byte ms_pin, byte h_pin, byte f_pin, byte t_pin, byte vd_pin, byte ve_pin, signed int se_pin)
{
         motor_dir_pin = md_pin;
         motor_speed_pin = ms_pin;
         heater_pin = h_pin;
         fan_pin = f_pin;
         temp_pin = t_pin;
         valve_dir_pin = vd_pin;
         valve_en_pin = ve_pin;
         step_en_pin = se_pin;
         
	//setup our pins
	pinMode(motor_dir_pin, OUTPUT);
	pinMode(motor_speed_pin, OUTPUT);
	pinMode(heater_pin, OUTPUT);

	pinMode(temp_pin, INPUT);
	pinMode(valve_dir_pin, OUTPUT); 
        pinMode(valve_en_pin, OUTPUT);

	//initialize values
	digitalWrite(motor_dir_pin, EXTRUDER_FORWARD);
	
	analogWrite(heater_pin, 0);
	analogWrite(motor_speed_pin, 0);
	digitalWrite(valve_dir_pin, false);
	digitalWrite(valve_en_pin, 0);

// The step enable pin and the fan pin are the same...
// We can have one, or the other, but not both

        if(step_en_pin >= 0)
        {
          pinMode(step_en_pin, OUTPUT);
	  disableStep();
        } else
        {
	  pinMode(fan_pin, OUTPUT);
          analogWrite(fan_pin, 0);
        }

        //these our the default values for the extruder.
        e_speed = 0;
        target_celsius = 0;
        max_celsius = 0;
        heater_low = 64;
        heater_high = 255;
        heater_fast = 255;
        heater_current = 0;
        valve_open = false;
        last_celsius = -1;
        last_celsius_time = 0;
        waiting_for_heater = false;
        
//this is for doing encoder based extruder control
//        rpm = 0;
//        e_delay = 0;
//        error = 0;
//        last_extruder_error = 0;
//        error_delta = 0;
        
        //default to cool
        set_target_temperature(target_celsius);
}


byte extruder::wait_till_hot()
{
  waiting_for_heater = true;
  while (get_temperature() < target_celsius - HALF_DEAD_ZONE)
  {
	manage_all_extruders();
        send_status();
	delay(1000);
  }
  waiting_for_heater = false;
  return 0;
}

byte extruder::wait_till_cool()
{  
  waiting_for_heater = true;
  while (get_temperature() > target_celsius + HALF_DEAD_ZONE)
  {
	manage_all_extruders();
        send_status();
	delay(1000);
  }
  waiting_for_heater = false;
  return 0;
}

void extruder::valve_set(bool open, int dTime)
{
        wait_for_temperature();
	valve_open = open;
	digitalWrite(valve_dir_pin, open);
        digitalWrite(valve_en_pin, 1);
        delay(dTime);
        digitalWrite(valve_en_pin, 0);
}

void extruder::set_target_temperature_and_wait(int temp)
{
  set_target_temperature(temp);
  
  if (temp < get_temperature())
    wait_till_cool();
  else
    wait_till_hot();
}


void extruder::set_target_temperature(int temp)
{
	target_celsius = temp;
	max_celsius = temp + 5;

        // If we've turned the heat off, we might as well disable the extrude stepper
       // if(target_celsius < 1)
        //  disableStep(); 
}

int extruder::get_target_temperature()
{
        return target_celsius;
}

/**
*  Samples the temperature and converts it to degrees celsius.
*  Returns degrees celsius.
*/
int extruder::get_temperature()
{
#ifdef USE_THERMISTOR
	int raw = sample_temperature();

	int celsius = 0;
	byte i;

	for (i=0; i<NUMTEMPS; i++)
	{
		if (temptable[i][0] > raw)
			break;
	}

        // Overflow: Set to fail safe value
        if (  (i >= NUMTEMPS) || (i <= 0)  )
            celsius = 511;
        else
            celsius  = temptable[i-1][1] + 
                (raw - temptable[i-1][0]) * 
                (temptable[i][1] - temptable[i-1][1]) /
                (temptable[i][0] - temptable[i-1][0]);

        // Clamp
        if (celsius > 511) celsius = 511;
        else if (celsius < 0) celsius = 0; 
  
	last_celsius = celsius;
#else
  last_celsius = ( 5.0 * sample_temperature() * 100.0) / 1024.0;
#endif

  last_celsius_time = millis();
  return last_celsius;
}



/*
* This function gives us an averaged sample of the analog temperature pin.
*/
int extruder::sample_temperature()
{
        int raw;
	int sum = 0;
        int largest = 0;
        int smallest = 1024;
	
	//read in a certain number of samples
	for (byte i=0; i<TEMPERATURE_SAMPLES; i++)
        {
                raw = analogRead(temp_pin);
                sum += raw;
                if (raw > largest)
                    largest = raw;
                if (raw < smallest)
                    smallest = raw;
        }
                    

        //drop the largest and smallest and get the average
        if (smallest < largest)
        {
                sum = sum - smallest - largest;
                raw = sum / (TEMPERATURE_SAMPLES - 2);
        }
        else
        {
                raw = sum / TEMPERATURE_SAMPLES;
        }
		
	//send it back.
	return raw;
}

/*!
  Manages extruder functions to keep temps, speeds etc
  at the set levels.  Should be called only by manage_all_extruders(),
  which should be called in all non-trivial loops.
  o If temp is too low, don't start the motor
  o Adjust the heater power to keep the temperature at the target
 */
void extruder::manage()
{
        byte newheat = 0;

	//make sure we know what our temp is.
	last_celsius = get_temperature();
        
        //put the heater into high mode if we're not at our target.
        if (last_celsius < target_celsius)
        {
                if (waiting_for_heater)
                      newheat = heater_fast;
                else
                      newheat = heater_high;
        }
        //put the heater on low if we're at our target.
        else if (last_celsius < max_celsius)
        {
                if (waiting_for_heater)
                      newheat = 0;
                else
                      newheat = heater_low;
        }
        //shut the heater off if things are getting out of hand
        else
                newheat = 0;
        
        // Only update heat if it changed
        if (heater_current != newheat)
          heater_current = newheat;
        analogWrite(heater_pin, heater_current);
}


#if 0
void extruder::set_speed(float sp)
{
  // DC motor?
    if(step_en_pin < 0)
    {
      e_speed = (byte)sp;
      if(e_speed > 0)
          wait_for_temperature();
      analogWrite(motor_speed_pin, e_speed);
      return;
    }
      
    // No - stepper
  disableTimerInterrupt();
  
  if(sp <= 1.0e-4)
  {
    disableStep();
    e_speed = 0; // Just use this as a flag
    return;
  } else
  {
    wait_for_temperature();
    enableStep();
    e_speed = 1;
  }
    
  extrude_step_count = 0;
  
  float milliseconds_per_step = 60000.0/(E_STEPS_PER_MM*sp);
  long thousand_ticks_per_step = 4*(long)(milliseconds_per_step);
  setupTimerInterrupt();
  setTimer(thousand_ticks_per_step);
  enableTimerInterrupt();
}


void extruder::interrupt()
{
    if(!e_speed)
      return;
    extrude_step_count++;
    if(extrude_step_count > 1000)
    {
      step();
      extrude_step_count = 0;
    }
}
#endif
#endif
