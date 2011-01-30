
// Class for controlling each extruder
//
// Adrian Bowyer 14 May 2009

#ifndef EXTRUDER_H
#define EXTRUDER_H

#define EXTRUDER_COUNT 1

void manage_all_extruders();

void new_extruder(byte e);

/**********************************************************************************************

* Sanguino/RepRap Motherboard v 1.0

*/

#if USE_EXTRUDER_CONTROLLER == false

#define EXTRUDER_FORWARD true
#define EXTRUDER_REVERSE false

class extruder
{
private:

//these our the default values for the extruder.
    byte e_speed;
    int target_celsius;
    int max_celsius;
    byte heater_low;
    byte heater_high;
    byte heater_fast;
    byte heater_current;
    int extrude_step_count;

// These are used for temperature control    
    int last_celsius;
    int last_celsius_time;
    bool waiting_for_heater;
    
//this is for doing encoder based extruder control
//    int rpm;
//    long e_delay;
//    int error;
//    int last_extruder_error;
//    int error_delta;
    bool valve_open;

// The pins we control
    byte motor_dir_pin, motor_speed_pin, heater_pin, fan_pin, temp_pin, valve_dir_pin, valve_en_pin;
    signed int step_en_pin;
    
     byte wait_till_hot();
     byte wait_till_cool();
     void temperature_error(); 
     int sample_temperature();
     
public:

   extruder(byte md_pin, byte ms_pin, byte h_pin, byte f_pin, byte t_pin, byte vd_pin, byte ve_pin, signed int se_pin);
   void wait_for_temperature();
   void valve_set(bool open, int dTime);

   void set_direction(bool direction);
   //void set_speed(float es);
   void set_cooler(byte e_speed);
   void set_target_temperature(int temp);
   void set_target_temperature_and_wait(int temp);
   int get_target_temperature();
   int get_temperature();
   inline int get_last_temperature(long maxAge_ms = -1);
   void manage();
// Interrupt setup and handling functions for stepper-driven extruders
   
   //void interrupt();
   void step();

   void enableStep();
   void disableStep();
   
};

inline void extruder::enableStep()
{
  if(step_en_pin < 0)
    return;
  digitalWrite(step_en_pin, ENABLE_ON); 
}

inline void extruder::disableStep()
{
  if(step_en_pin < 0)
    return;
  digitalWrite(step_en_pin, !ENABLE_ON);
}

inline void extruder::step()
{
   digitalWrite(motor_speed_pin, HIGH);
   delayMicroseconds(5);
   digitalWrite(motor_speed_pin, LOW);
}

inline void extruder::temperature_error()
{
      Serial.print("E: ");
      Serial.println(get_temperature());  
}

//warmup if we're too cold; cool down if we're too hot
inline void extruder::wait_for_temperature()
{
/*
  if(wait_till_cool())
   {
      temperature_error();
      return;
   }
*/
   if(wait_till_hot())
     temperature_error();
}

inline int extruder::get_last_temperature(long maxAge_ms)
{
  if (  (maxAge_ms > 0) && (millis() -last_celsius_time > maxAge_ms )  )
    return get_temperature();
  else
    return last_celsius;
}

inline void extruder::set_direction(bool dir)
{
    digitalWrite(motor_dir_pin, dir);
}

inline void extruder::set_cooler(byte sp)
{
  if(step_en_pin >= 0) // Step enable conflicts with the fan
    return;
  analogWrite(fan_pin, sp);
}

/**********************************************************************************************

* RepRap Motherboard v 1.x (x >= 1)
* Extruder is now on RS485

*/

#else

class extruder
{
private:

  byte address;
 
public:
   extruder(byte a);
   void wait_for_temperature();
   void valve_set(bool open, int dTime);
   void set_direction(bool direction);
   void set_cooler(byte e_speed);
   void set_temperature(int temp);
   int get_temperature();
   void manage();
   void step();

   void enableStep();
   void disableStep();
   
};

inline extruder::extruder(byte a)
{
  address = a;
}

inline  void extruder::wait_for_temperature()
{
  
}

inline  void extruder::valve_set(bool open, int dTime)
{
   if(open)
     talker.sendPacket(address, "V1");
   else
     talker.sendPacket(address, "V0");
   delay(dTime);
}

inline void extruder::set_direction(bool direction)
{
  
}

inline  void extruder::set_cooler(byte e_speed)
{
  
}

inline  void extruder::set_temperature(int temp)
{
  
}

inline  int extruder::get_temperature()
{
  return 1;  
}

inline  void extruder::manage()
{
  
}

inline  void extruder::step()
{
  
}

inline  void extruder::enableStep()
{
  
}

inline  void extruder::disableStep()
{
  
}

#endif

extern extruder* ex[];
extern byte extruder_in_use;

#endif
