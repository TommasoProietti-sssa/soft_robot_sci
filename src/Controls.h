#ifndef Controls_header
#define Controls_header

#include "Definitions.h"
#include "Sensors.h"
#include "IO.h"

extern int control_mode;
extern bool flag_control;
extern uint32_t counter_discharge;
extern bool discharging;
// automatic control
extern float pref_automatic_control1;
extern float pref_automatic_control2;
extern float pref;
extern float pref1;
extern float pref2;
extern float pmeas;
extern float perr_th;
extern float perr1;
extern float perr2;
extern bool flag_hysteresis1;
extern bool flag_hysteresis2;
// reservoir
extern bool flag_hysteresis_reservoir;
extern float reservoir_pref;
extern int old_reservoir_state;
extern int reservoir_state;
extern uint32_t timer_reservoir;
extern float perr_reservoir;
// fatigue control
extern int ft_current_cycle;
extern int ft_needed_cycles;
extern int ft_duration;
extern float ft_pref;
extern uint32_t ft_counter;
// threshold control
extern float angle_thcontrol;
extern float pref_thcontrol;
// idle control
extern int counter_idle;
extern bool flag_idle_start;
// system states
extern int old_system_state[2];
extern int system_state[2];
// gravity compensation
extern float gc_a;
extern float gc_b;
extern float gc_c;
extern float gc_dir;
extern float gc_prev_dir;
extern float gc_alpha_dir;
extern int gc_counter;
extern int gc_counter_thr;
extern bool gc_switch;
// gravity compensation calibration
extern float gc_calibration_max_angle;
extern uint32_t gc_calibrate_t0;
extern uint32_t gc_calibration_timer;
extern int gc_calibration_duration;
extern bool first_gc_calibration_cycle;
extern bool gc_calibrating;
extern bool ascending_phase;

// elbow flex-extension direction detection
extern float elbow_dir;
extern float elbow_prev_dir;
extern float elbow_alpha_dir;
extern int elbow_counter;
extern int elbow_counter_thr;
extern bool elbow_switch;

// endurance test
extern float shoulder_angle_endurance;
extern float angle_err;
extern float angle_err_th;
extern float pref1_endurance;

// automatic controls
extern bool sinergy_out_mode;
extern bool sinergy_in_mode;
extern float max_pressure_elbow;

// Angles
extern float elev;
extern float prev_elev;
extern float elbow_flex;
extern float prev_elbow_flex;

void AutomaticPressureControl();
void ThresholdControl();
void FatigueTest();
void CalibrateGC();
void GravityCompensation();
void HandleReservoir();
void setReservoirState();
void EnduranceTest();
void Actuate();
void Inflate(int nb);
void Vent(int nb);
void Seal(int nb);
void IdleControl();
void DischargePumps(uint32_t Tstart);

#endif