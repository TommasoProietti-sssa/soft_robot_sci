#include <math.h>
#include <String.h>
#include "Definitions.h"
#include "MathFunctions.h"  
#include "IO.h"
#include "Sensors.h"
#include "Controls.h"


unsigned long previous_time;
unsigned long current_time;
unsigned long delta;
const unsigned long period = 1000/FREQ; 


void SetupPins(){
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(PUMP1, OUTPUT);
  digitalWrite(PUMP1, LOW);
  pinMode(PUMP2, OUTPUT);
  digitalWrite(PUMP2, LOW);
  pinMode(VALVE1_IN, OUTPUT);
  digitalWrite(VALVE1_IN, LOW);
  pinMode(VALVE1_EX, OUTPUT);
  digitalWrite(VALVE1_EX, LOW);
  pinMode(VALVE2_IN, OUTPUT);
  digitalWrite(VALVE2_IN, LOW);
  pinMode(VALVE2_EX, OUTPUT);
  digitalWrite(VALVE2_EX, LOW);
  pinMode(VALVE_DISCHARGE, OUTPUT);
  digitalWrite(VALVE_DISCHARGE, LOW);
  pinMode(BUT_INF, INPUT);
  pinMode(BUT_VEN, INPUT);
  pinMode(SWITCH, INPUT);
  pinMode(TRIGGER_OTB,OUTPUT);
  pinMode(TRIGGER_OTB,LOW);
}

// ----------------- SETUP ------------------------- //

void setup(){
  Serial.setTimeout(5);
  Serial.begin(115200);
  while(!Serial);
  SetupPins();
  previous_time = millis();
}

// -------------------- LOOP ----------------------- //
void loop(){
  current_time = millis();
  if (current_time - previous_time >= period) 
  {
    UserInput();
    GetPressure(); // pressure sensors 
    
    // controls
    if(control_mode==0){IdleControl();}
    else if(control_mode==4){ThresholdControl();}
    else if(control_mode==5){GravityCompensation();}
    else if(control_mode==7){CalibrateGC();}
    else if(control_mode==9){EnduranceTest();}
    if(flag_control && control_mode!=1 && control_mode!=6 && control_mode!=8){AutomaticPressureControl();}

    HandleReservoir();
    Actuate();

    // pumps discharging after each inflation for a short time
    if((old_system_state[0] == INFLATING && system_state[0] == SEALING)||(old_system_state[1] == INFLATING && system_state[1] == SEALING)||(old_reservoir_state == INFLATING && reservoir_state == SEALING))
    {discharging = true; counter_discharge = millis();}
    if(discharging){DischargePumps(counter_discharge);}
    
    // update reservoir state history
    setReservoirState();

    // trigger OTB
    if (trigger_otb){
      triggerOtb();
    }
  
    // serial communication
    Print2Serial_binary();

    // update timing variables
    delta = current_time - previous_time;
    previous_time = current_time; 
  }
}