#include "Controls.h"

int control_mode = 0; 
bool flag_control = false;

// low-level control
int old_system_state[2] = {SEALING, SEALING};
int system_state[2] = {SEALING, SEALING};

// discharging valve
uint32_t counter_discharge = 0;
bool discharging = false;

// pressure setpoint mode
float pref_automatic_control1 = 0;
float pref_automatic_control2 = 0;
float pref = 0;
float pref1 = 0;
float pref2 = 0;
float pmeas = 0;
float perr_th = 0.1;
float perr1 = 0;
float perr2 = 0;
bool flag_hysteresis1 = false;
bool flag_hysteresis2 = false;

// reservoir automatic control
bool flag_hysteresis_reservoir = false;
float reservoir_pref = 10;
int old_reservoir_state = SEALING;
int reservoir_state = SEALING;
float perr_reservoir = 0;

// fatigue mode
int ft_current_cycle = 0;
int ft_needed_cycles = 2;
int ft_duration = 10*FREQ; // [s]
uint32_t ft_counter = 0;
float ft_pref = 10;

// idle control mode
int counter_idle = 0;
bool flag_idle_start = false;

// threshold control mode
float angle_thcontrol = 50;
float pref_thcontrol = 10;

// shoulder (gravity compensation)
float gc_dir = 0;
float gc_prev_dir = 0;
float gc_alpha_dir = 0.1;
int gc_counter = 0;
int gc_counter_thr = 15;
bool gc_switch = false;

// gravity compensation calibration
float gc_calibration_max_angle = 90;
uint32_t gc_calibrate_t0 = 0;           
uint32_t gc_calibration_counter = 0;    
int gc_calibration_duration = 30*FREQ;  
bool first_gc_calibration_cycle = true;
bool gc_calibrating = false;
bool ascending_phase = true;

// elbow flex-extension direction detection
float elbow_dir = 0;
float elbow_prev_dir = 0;
float elbow_alpha_dir = 0.1;
int elbow_counter = 0;
int elbow_counter_thr = 15;
bool elbow_switch = false;

// endurance test
float shoulder_angle_endurance = 0;
float angle_err = 0;
float angle_err_th = 0.8;
float pref1_endurance = 0;

// automatic controls (synergistic) 
bool sinergy_out_mode = false; // shoulder+extensor
bool sinergy_in_mode = false;  // shoulder+flexor
float max_pressure_elbow = 10;

// Angles
float elev = 0;
float prev_elev = 0;
float elbow_flex = 0;
float prev_elbow_flex = 0;


// --------- HIGH-LEVEL CONTROLS --------- //

void AutomaticPressureControl(){
  // first actuator
  if(pref1 == 0){system_state[0] = VENTING;}
  else{  
    if(pref1>max_pressure){pref1=max_pressure;}
    perr1=pmeas1-pref1;
    if(abs(perr1)>perr_th){
      if(perr1>0){system_state[0] = VENTING; flag_hysteresis1 = false;}
      else{system_state[0] = INFLATING; flag_hysteresis1 = true;}
    }
    else{
      if(flag_hysteresis1){system_state[0] = INFLATING;}
      else{system_state[0] = SEALING;}
    }
  }

  // second actuator
  if(pref2 == 0){system_state[1] = VENTING;}
  else{
    if(pref2>max_pressure){pref2=max_pressure;}
    perr2=pmeas2-pref2;
    if(abs(perr2)>perr_th){
      if(perr2>0){system_state[1] = VENTING; flag_hysteresis2= false;}
      else{system_state[1] = INFLATING; flag_hysteresis2 = true;}
    }
    else{
      if(flag_hysteresis2){system_state[1] = INFLATING;}
      else{system_state[1] = SEALING;}
    }
  }

}

void ThresholdControl(){
  if(elev>angle_thcontrol){pref1 = pref_thcontrol;}
  else{pref1 = 0;}
}

void FatigueTest(){
  if(ft_current_cycle<ft_needed_cycles){
    if((millis()-ft_counter) < 4*ft_duration){
      if((millis()-ft_counter)<ft_duration){pref1=0;} // pause
      else if((millis()-ft_counter)<(2*ft_duration)){pref1+=(ft_pref/ft_duration*5);} // inflate
      else if((millis()-ft_counter)<(3*ft_duration)){pref1=ft_pref;} // hold
      else{pref1-=(ft_pref/ft_duration*5);} // deflate
    }
    else{
      ft_counter = millis();
      ft_current_cycle = ft_current_cycle+1;
    }
  }
  else{control_mode = 0;flag_control=false;counter_idle = 0;} // go in idle control
}


void CalibrateGC(){
  if(first_gc_calibration_cycle){ // first cycle, set pref1 = 0
        pref1 = 0;
        first_gc_calibration_cycle = false;
  }
  else{
    if(ascending_phase){
      pref1 = max_press_calib;
      if(pmeas1>=pref1-3*perr_th){
        ascending_phase=false;
        gc_calibration_counter = 0;
      }
    }
    else{
      if(gc_calibration_counter<=3*FREQ){  // keep pref1 = 0 for 3 s
        gc_calibration_counter++;
        pref1 = max_press_calib;
      }
      else{
        pref1 = 0;
        if(pmeas1<=0.2){
          gc_calibrating = false;
          first_gc_calibration_cycle = true;
          ascending_phase = true;
          control_mode = 0;
          flag_control=false;
          counter_idle = 0;}
        }
      }
  }
}

void GravityCompensation(){
  // --- detect direction based on angle ---
	gc_dir = (elev - prev_elev);
	gc_dir = (gc_alpha_dir * gc_dir) + (1.0 - gc_alpha_dir) * gc_prev_dir;
	if(abs(gc_dir)>gt_thr_dir){ // change direction of FF based on threshold + hysteresis
		if(gc_counter<gc_counter_thr){gc_counter++;}
		else{
			if(gc_dir>0){gc_switch = true;}
			else{gc_switch = false;}
		}
	}
	else{gc_counter = 0;}
  gc_prev_dir = gc_dir;
  prev_elev = elev;
  // compute FF
  if(elev>gc_min_elev){
    if(gc_switch){
      pref1 = pow(elev,2)*GC_coeffs[0] + elev*GC_coeffs[1] + GC_coeffs[2];
      }
    else{pref1 = 0;}
  }
  else{pref1 = 0;}
  // ------------------- elbow threshold detection --------------------------------
  // ------------- elbow extension(sinergy_out)/elbow flexion(sinergy_in) ---------------
  if (sinergy_out_mode || sinergy_in_mode){
    elbow_dir = (elbow_flex - prev_elbow_flex);
    elbow_dir = (elbow_alpha_dir * elbow_dir) + (1.0 - elbow_alpha_dir) * elbow_prev_dir;
    if (!elbow_switch){
      if(abs(elbow_dir)>elbow_thr_dir){
        if(elbow_counter<elbow_counter_thr){elbow_counter++;}
        else{
          if(sinergy_out_mode){
            if(elbow_dir < 0){elbow_switch = true; pref2 = max_pressure_elbow;}  // extend elbow
          }
          else if (sinergy_in_mode){
            if(elbow_dir > 0){elbow_switch = true; pref2 = max_pressure_elbow;}  // flex elbow
          }
        }
      }
      else{elbow_counter = 0;}
    }
    elbow_prev_dir = elbow_dir;
    prev_elbow_flex = elbow_flex;
  }
}


void HandleReservoir(){
  perr_reservoir=pmeas_reservoir-reservoir_pref;
  if(abs(perr_reservoir)>perr_th){
    if(perr_reservoir>0){
      reservoir_state = SEALING;
      digitalWrite(PUMP1, LOW); digitalWrite(PUMP2, LOW);
      flag_hysteresis_reservoir = false;
      }
    else{
      reservoir_state = INFLATING;
      digitalWrite(PUMP1, HIGH);  digitalWrite(PUMP2, HIGH);
      flag_hysteresis_reservoir = true;
      }
  }
  else{
    if(flag_hysteresis_reservoir){
      reservoir_state = INFLATING;
      digitalWrite(PUMP1, HIGH);  digitalWrite(PUMP2, HIGH);
    }
    else{
      reservoir_state = SEALING;
      digitalWrite(PUMP1, LOW); digitalWrite(PUMP2, LOW);
    }
  }
}

void setReservoirState(){
  old_reservoir_state = reservoir_state;
}


void EnduranceTest(){
  if(shoulder_angle_endurance == 0){pref1 = 0;}
  else{  
    if(shoulder_angle_endurance>90){shoulder_angle_endurance=90;}
    angle_err=elev-shoulder_angle_endurance;
    if(abs(angle_err) > angle_err_th){
      if(angle_err<0){pref1_endurance = pref1_endurance + 0.1; pref1 = pref1_endurance;}
      else{pref1_endurance = pref1_endurance-0.1; pref1 = pref1_endurance;}
    }
    else{
      if(angle_err>=0){pref1_endurance = pmeas1; pref1 = pref1_endurance;}
    }
  }
}

void Actuate(){
  for(int i=0; i<2; i++){
    if(system_state[i]!=old_system_state[i]){
      if(system_state[i] == INFLATING){Inflate(i);}
      else if(system_state[i] == VENTING){Vent(i);}
      else{Seal(i);}
    }
    // update the state history
    old_system_state[i] = system_state[i];
    }
}

void Inflate(int nb){
  if(nb==0){digitalWrite(VALVE1_IN, HIGH); digitalWrite(VALVE1_EX, LOW);}
  else {digitalWrite(VALVE2_IN, HIGH); digitalWrite(VALVE2_EX, LOW);}
}

void Vent(int nb){
  if(nb==0){digitalWrite(VALVE1_IN, LOW); digitalWrite(VALVE1_EX, HIGH);}
  else {digitalWrite(VALVE2_IN, LOW); digitalWrite(VALVE2_EX, HIGH);}
}

void Seal(int nb){
  if(nb==0){digitalWrite(VALVE1_IN, LOW); digitalWrite(VALVE1_EX, LOW);}
  else {digitalWrite(VALVE2_IN, LOW); digitalWrite(VALVE2_EX, LOW);}
}

// Vent pump line for a short time to avoid pump stucks at high pressure 
void DischargePumps(uint32_t Tstart){
  if((millis()-Tstart)<DISCHARGE_COUNTER_TH){digitalWrite(VALVE_DISCHARGE, HIGH);}
  else{digitalWrite(VALVE_DISCHARGE, LOW); discharging = false;}
}

void IdleControl(){
  if(pmeas1>IDLE_PMAX || pmeas2>IDLE_PMAX || flag_idle_start){
    flag_idle_start = true;
    counter_idle=counter_idle+1;
    if(counter_idle<COUNTER_IDLE_LIM){system_state[0] = VENTING; system_state[1] = VENTING;}
    else{system_state[0] = SEALING; system_state[1] = SEALING; flag_idle_start = false; counter_idle = 0;}
  }
}

}