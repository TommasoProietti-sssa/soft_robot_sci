#ifndef Sensors_header
#define Sensors_header

#include "Definitions.h"
#include "MathFunctions.h"
#include "Controls.h"

// -------------------- Pressure -------------------- //
extern float pmeas1;
extern float pfilt_previous1;
extern float pfilt1;
extern float praw1;
extern float pmeas2;
extern float pfilt_previous2;
extern float pfilt2;
extern float praw2;
extern float pmeas_reservoir;// reservoir
extern float pfilt_reservoir_previous;
extern float pfilt_reservoir;
extern float praw_reservoir;

// trigger otb
extern bool trigger_otb;
extern uint32_t trigger_begin;

// -------------------- Functions -------------------- //

void GetPressure();
void triggerOtb();

#endif