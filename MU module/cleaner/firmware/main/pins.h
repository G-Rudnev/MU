#ifndef PINS_H
#define PINS_H

#include "modbus_regs.h"
#include <Arduino.h>

//FET control
#define FET_1             3
#define FET_2             5
#define FET_3             7
#define FET_4             9
#define FET_5             11
#define FET_6             13

#define FET_7             2
#define FET_8             4
#define FET_9             6
#define FET_10            39 

//power-on button
#define BTN               23

//PC control
#define PC_ON             25
#define PC_ST             27

//DC-DC control pins
#define DC_DC_1_CNTRL     22
#define DC_DC_2_CNTRL     24
#define DC_DC_3_CNTRL     26

#define DC_DC_1_V         A7
#define DC_DC_2_V         A9
#define DC_DC_3_V         A11

#define DC_DC_1_CUR       A1
#define DC_DC_2_CUR       A3
#define DC_DC_3_CUR       A5

//input voltage
#define INPUT_V           A13

//motor control pins
#define MOTOR_1_PWM       10 
#define MOTOR_1_DIR       14 
#define MOTOR_1_CUR       A6

#define MOTOR_2_PWM       12 
#define MOTOR_2_DIR       16 
#define MOTOR_2_CUR       A8

#define MOTOR_3_PWM       8 
#define MOTOR_3_DIR       32 
#define MOTOR_3_CUR       A10

#define MOTOR_4_PWM       45 
#define MOTOR_4_DIR       34 
#define MOTOR_4_CUR       A12

#define MOTOR_5_PWM       44 
#define MOTOR_5_DIR       36 
#define MOTOR_5_CUR       A14

#define MOTOR_6_PWM       46 
#define MOTOR_6_DIR       42 
#define MOTOR_6_CUR       A15

//discrete inputs
#define INPUT_1           40 
#define INPUT_2           33
#define INPUT_3           35
#define INPUT_4           37
#define INPUT_5           41
#define INPUT_6           43

//discrete outputs
#define OUTPUT_1          29
#define OUTPUT_2          31
#define OUTPUT_3          28
#define OUTPUT_4          30

void init_pins(void);
void disable_pins(void);
void poll_analog_pins(float (& vals) [13]);
float read_analog(int pin, int coef);

#endif
