#include "IO.h"

// user input vars
String user_input;
int tmp, tmp1, tmp2, tmp3, tmp4, tmpR;
// SW safety variables
float max_pressure = 12;
// shoulder control parameters (GC controller)
int cnt_GC_parameters = 0;  // counter of received parameters
float GC_coeffs[6] = {0.0139, -0.2185, 2.878, 0, 0, 0};
int gc_min_elev = 8;
float gt_thr_dir = 0.08;
float max_press_calib = 12;

// elbow control parameters
float elbow_thr_dir = 0.1;  // threshold for detection of elbow flex-extension direction
bool perform_rom_repetition = true;


// data packet structure
// --------------------- //
// 0: 4 bytes  uint32_t  time
// 1: 4 bytes  uint32_t  delta-time
// 2: 4 bytes float      torso
// 3: 4 bytes float      elev
// 4: 4 bytes float     elbow_flex
// 5: 4 bytes float     praw1
// 6: 4 bytes float     praw2
// 7: 4 bytes float     pmeas2
// 8: 4 bytes float     pmeas1
// 9: 4 bytes float     pref1
// 10: 4 bytes float     perr_th
// 11: 4 byte  int       digitalRead(PUMP1)
// 12: 4 byte  int       digitalRead(PUMP2)
// 13: 4 byte  int       digitalRead(VALVE1_IN)
// 14: 4 byte  int       current_cycle
// 15: 4 byte  int       needed_cycles
// 16: 4 byte  int       system_state[0]
// 17: 4 byte  int       control_mode
// 18: 4 bytes float     angle_thcontrol
// 19: 4 bytes float     pref_thcontrol
// 20: 4 bytes float     ft_pref
// 21: 4 bytes float     ft_pref
// 22: 4 byte  int       digitalRead(VALVE1_EX)
// 23: 4 byte  int       digitalRead(VALVE2_IN)
// 24: 4 byte  int       digitalRead(VALVE2_EX)
// 25: 4 byte  int       system_state[1]
// 26: 1 byte  bool      gc_calibrating
// 27: 4 bytes float     pmeas_reservoir
// 28: 1 byte  bool      trigger_otb
// --------------------- //

void Print2Serial_binary(){
  Serial.write(SERIAL_HEADER);
  Serial.write((byte *)&current_time, sizeof(current_time)); 
  Serial.write((byte *)&delta, sizeof(delta));
  Serial.write((byte *)&torso, sizeof(torso));
  Serial.write((byte *)&elev, sizeof(elev));
  Serial.write((byte *)&elbow_flex, sizeof(elbow_flex));
  Serial.write((byte *)&praw1, sizeof(praw1));
  Serial.write((byte *)&praw2, sizeof(praw2));
  Serial.write((byte *)&pmeas2, sizeof(pmeas2));
  Serial.write((byte *)&pmeas1, sizeof(pmeas1));
  Serial.write((byte *)&pref1, sizeof(pref1));
  Serial.write((byte *)&perr_th, sizeof(perr_th));
  int tmp1 = digitalRead(PUMP1);
  int tmp2 = digitalRead(PUMP2);
  int tmp3 = digitalRead(VALVE1_IN);
  Serial.write((byte *)&tmp1, sizeof(tmp1));
  Serial.write((byte *)&tmp2, sizeof(tmp2));
  Serial.write((byte *)&tmp3, sizeof(tmp3));
  Serial.write((byte *)&ft_current_cycle, sizeof(ft_current_cycle));
  Serial.write((byte *)&ft_needed_cycles, sizeof(ft_needed_cycles));
  Serial.write((byte *)&system_state[0], sizeof(system_state[0]));
  Serial.write((byte *)&control_mode, sizeof(control_mode));
  Serial.write((byte *)&angle_thcontrol, sizeof(angle_thcontrol));
  Serial.write((byte *)&pref_thcontrol, sizeof(pref_thcontrol));
  Serial.write((byte *)&ft_pref, sizeof(ft_pref));
  Serial.write((byte *)&ft_duration, sizeof(ft_duration));
  int tmp4 = digitalRead(VALVE1_EX);
  int tmp5 = digitalRead(VALVE2_IN);
  int tmp6 = digitalRead(VALVE2_EX);
  Serial.write((byte *)&tmp4, sizeof(tmp4));
  Serial.write((byte *)&tmp5, sizeof(tmp5));
  Serial.write((byte *)&tmp6, sizeof(tmp6));
  Serial.write((byte *)&system_state[1], sizeof(system_state[1]));
  Serial.write((byte *)&gc_calibrating, sizeof(gc_calibrating));
  Serial.write((byte *)&pmeas_reservoir, sizeof(pmeas_reservoir));
  Serial.write((byte *)&trigger_otb, sizeof(trigger_otb));
  Serial.write(SERIAL_FOOTER);
}

void UserInput(){
  if(Serial.available()){
    // read user input and make integer out of it
    user_input = Serial.readStringUntil(*(int32_t*)"\r\n");
    tmp = user_input.toInt();
    tmp1 = user_input.charAt(0) - '0'; // first digit
    tmp2 = user_input.charAt(1) - '0'; // second digit
    tmp3 = user_input.charAt(2) - '0'; // third digit
    tmp4 = user_input.charAt(3) - '0'; // forth digit
    tmpR = tmp - 1000*tmp1; // rest of number

    switch (tmp1) {
      case 0: // SW reset
        flag_control = false;
        counter_idle = 0;
        control_mode = 0;
        // reset GC calibration variables
        gc_calibrating = false;   
        first_gc_calibration_cycle = true;
        // reset automatic controls flags
        sinergy_out_mode = false;
        sinergy_in_mode = false;
        break;
      case 1: // manual control
        counter_idle = 0;
        control_mode = 1;
        switch (tmpR) {
          case 1:
            system_state[0] = INFLATING;
            break;
          case 2:
            system_state[0] = VENTING;
            break;
          case 3:
            system_state[0] = SEALING;
            break;
          case 4:
            system_state[1] = INFLATING;
            break;
          case 5:
            system_state[1] = VENTING;
            break;
          case 6:
            system_state[1] = SEALING;
            break;
        }
        break;
      case 2: // automatic controls
        counter_idle = 0;
        flag_control = !flag_control;
        switch (tmp4) {
          case 0: // endurance test mode
            control_mode = 9*(int)flag_control;
            if(control_mode==9){pref1_endurance = max_press_calib; pref2=max_pressure_elbow;}
            else{pref2 = 0; shoulder_angle_endurance = 0;}  // turn off mode
            break;
          case 1: // pressure setpoint
            control_mode = 2*(int)flag_control;
            if(control_mode==2){pref1=pref_automatic_control1; pref2=pref_automatic_control2;}
            break;
          case 2: // fatigue test
            control_mode = 3*(int)flag_control; 
            if(control_mode==3){ft_current_cycle = 0; ft_counter = millis();}
            break;
          case 3: // threshold control
            control_mode = 4*(int)flag_control;
            break;
          case 4: // gravity compensation control
            control_mode = 5*(int)flag_control;
            sinergy_out_mode = true;
            sinergy_in_mode = false;
            break;
          case 5: // GC calibration
            control_mode = 7*(int)flag_control;
            gc_calibrating = true;
            break;
          case 6: // sinergy out (shoulder+extension)
            switch(tmp3) {
              case 0:
                control_mode = 5;
                sinergy_out_mode = true;
                sinergy_in_mode = false;
                elbow_switch = false;
                break;
              case 1:
                flag_control = true;
                // toggle finish movement for the ROM task
                if (control_mode == 9){
                  perform_rom_repetition = false;
                  pref2=max_pressure_elbow*(int)perform_rom_repetition;
                }
                else{
                  control_mode = 5;
                  elbow_switch = true;
                  system_state[1] = VENTING;
                  pref2 = 0;
                  elbow_dir = 0;
                }
                break;
              case 2:
                flag_control = true;
                // toggle start movement for the ROM task
                if (control_mode == 9){
                  perform_rom_repetition = true;
                  pref2=max_pressure_elbow*(int)perform_rom_repetition;
                }
                else{
                  elbow_switch = false;
                  system_state[1] = VENTING;
                  elbow_dir = 0;
                  elbow_prev_dir = 0; 
                  elbow_counter = 0;
                }
                break;
            }
            break;
          case 7: // sinergy in (shoulder+flexion)
            control_mode = 5;
            sinergy_out_mode = false;
            sinergy_in_mode = true;
            elbow_switch = false;
            break;
        }
        break;
      case 3: // set reference pressure
        switch (tmp2){
          case 0:
            pref_automatic_control1 = tmpR;
            break;
          case 1:
            pref_thcontrol = tmpR-100;
            break;
          case 2:
            ft_pref = tmpR-200;
            break;
          case 3:
            reservoir_pref = tmpR-300;
            break;
          case 4:
            max_pressure_elbow = tmpR-400;
            break;
          case 5:
            max_pressure_ps = tmpR - 500;
            break;
          case 6:
            pref_automatic_control2 = tmpR - 600;
            break;
        }
        break;
      case 4: // set error/angle threshold
        switch (tmp2){
          case 0:
            perr_th = tmpR/10.0;
            break;
          case 1:
            angle_thcontrol = tmpR-100;
            break;
          case 2:
            ft_needed_cycles = tmpR-200;
            break;
          case 3:
            shoulder_angle_endurance = tmpR-300;
            break;
        }
        break;
      case 5: // various parameters
        switch (tmp2) {
          case 1: // offset IMUs
            imu_calibrated = false;
            break;
          case 3: // minimum elevation for GC controller
            gc_min_elev = tmpR-300;
            break;
          case 4: // maximum pressure allowed during GC calibration
            max_press_calib = tmpR-400;
            break;
          case 5: // threshold for movement direction detection in GC controller
            gt_thr_dir = (tmpR-500)/100.0; 
            break;
          case 6: // overall maximum pressure allowed
            max_pressure = tmpR-600;
            break;
          case 8: // threshold for movement direction detection in elbow (flex-extension)
            elbow_thr_dir = (tmpR-800)/100.0;
            break;
        }
        break;
      case 6: // elbow flexion angle: '6xxx'
          prev_elbow_flex = elbow_flex;
          elbow_flex = tmpR;
        break;
      case 7: // receive calibration parameters for GC controller (pt.1, module)
        GC_coeffs[cnt_GC_parameters] = tmpR;
        cnt_GC_parameters += 1;
        break;
      case 8: // receive calibration parameters for GC controller (pt.1, sign, exponent)
        switch (tmp2){
          case 1: // negative sign
            GC_coeffs[cnt_GC_parameters-1] = -GC_coeffs[cnt_GC_parameters-1];
            break;
            }
        GC_coeffs[cnt_GC_parameters-1] = GC_coeffs[cnt_GC_parameters-1]/pow(10,tmp4);
        if (cnt_GC_parameters == 6) {cnt_GC_parameters=0;}  // reset counter
        break;
      case 9:
        switch(tmp2):
          case 0: // shoulder elevation angle '90xx'
            prev_elev = elev;
            elev = tmpR;
            break;
          case 9: // '99xx'
            trigger_otb = true;
            digitalWrite(TRIGGER_OTB,HIGH);
            trigger_begin = millis();
            break;
        }
        break;
      default:
        break;
    }
  }
  else{
    user_input = "";
  }
}