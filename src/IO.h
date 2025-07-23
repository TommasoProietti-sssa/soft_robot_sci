#ifndef IO_header
#define IO_header

#include "Definitions.h"
#include "Sensors.h"
#include "Controls.h"

// SW Safety 
extern float max_pressure;

// user input vars
extern String user_input;
extern int tmp, tmp1, tmp2, tmp3, tmp4, tmpR;
extern unsigned long current_time, delta;

// counter for received GC calbration coefficients
extern int cnt_GC_parameters;
extern float GC_coeffs[6];
extern int gc_min_elev;
extern float gt_thr_dir;
extern float max_press_calib;

// elbow control parameters
extern float elbow_thr_dir;

// elbow rom off/on tests
extern bool perform_rom_repetition;


void Print2Serial_binary();
void UserInput();

#endif