#ifndef Definitions_header
#define Definitions_header

// --------- PINOUT --------- //

// sensors pins
#define PRESSURE_SENSOR1 A3
#define PRESSURE_SENSOR2 A4
#define PRESSURE_SENSOR_RESERVOIR A2

// pinout
#define PUMP1 6
#define PUMP2 5
#define VALVE_DISCHARGE A0
#define TRIGGER_OTB 9
#define VALVE1_IN 10
#define VALVE1_EX 11
#define VALVE2_IN 12
#define VALVE2_EX 13
#define BUT_INF 0
#define BUT_VEN 1
#define SWITCH 4

// --------- SAMPLING FREQUENCY --------- //
#define FREQ 200 // sampling frequency in Hz

// pressure sensor
#define ALPHA_PRESSURE 0.2 // exponential filtering on pressure signal
#define OFFSET_P1 155.0 
#define GAIN_P1 25.0/(770-OFFSET_P1)
#define OFFSET_P2 155.0
#define GAIN_P2 25.0/(770-OFFSET_P2)
#define OFFSET_P3 155.0 // reservoir
#define GAIN_P3 25.0/(770-OFFSET_P3)

// idle control
#define COUNTER_IDLE_LIM 300 // counter to vent when turning in idle mode
#define IDLE_PMAX 1 // [psi] threshold above which the system vents when turning in idle mode

// others
#define DISCHARGE_COUNTER_TH 75 // number of samples to keep on discharging valves after startup

// system states 
#define INFLATING 1
#define SEALING 0
#define VENTING -1

// binary communication
#define SERIAL_HEADER 0x21 // '!'
#define SERIAL_FOOTER 0x40 // '@'

#endif