#include "Sensors.h"

// Pressure sensor
// --- actuator 1
float pmeas1 = 0;
float pfilt_previous1 = 0;
float pfilt1 = 0;
float praw1 = 0;
// --- actuator 2
float pmeas2 = 0;
float pfilt_previous2 = 0;
float pfilt2 = 0;
float praw2 = 0;

// --- reservoir
float pmeas_reservoir = 0; 
float pfilt_reservoir_previous = 0;
float pfilt_reservoir = 0;
float praw_reservoir = 0;

// trigger otb
bool trigger_otb = false;
uint32_t trigger_begin = 0;

void GetPressure(){
  praw1 = analogRead(PRESSURE_SENSOR1);
  praw2 = analogRead(PRESSURE_SENSOR2);
  praw_reservoir = analogRead(PRESSURE_SENSOR_RESERVOIR);

  praw1 = (praw1-OFFSET_P1)*GAIN_P1;
  if(praw1<0){praw1=0;}
  praw2 = (praw2-OFFSET_P2)*GAIN_P2;
  if(praw2<0){praw2=0;}
  praw_reservoir = (praw_reservoir-OFFSET_P3)*GAIN_P3;
  if(praw_reservoir<0){praw_reservoir=0;}

  pfilt1 = ALPHA_PRESSURE*praw1 + (1-ALPHA_PRESSURE)*pfilt_previous1;
  pfilt2 = ALPHA_PRESSURE*praw2 + (1-ALPHA_PRESSURE)*pfilt_previous2;
  pfilt_reservoir = ALPHA_PRESSURE*praw_reservoir + (1-ALPHA_PRESSURE)*pfilt_reservoir_previous;

  pfilt_previous1 = pfilt1;
  pmeas1 = pfilt1;
  pfilt_previous2 = pfilt2;
  pmeas2 = pfilt2;
  pfilt_reservoir_previous = pfilt_reservoir;
  pmeas_reservoir = pfilt_reservoir;
}

void triggerOtb(){
  if (millis() - trigger_begin >= 50) {
    trigger_otb = false;
    digitalWrite(TRIGGER_OTB,LOW);
  }
}
