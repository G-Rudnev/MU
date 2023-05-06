#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "modbus_regs.h"
#include "pins.h"
#include <Arduino.h>
#include <ArduinoModbus.h>
#include <ArduinoRS485.h>

void subMotor1(int16_t val);
void subMotor2(int16_t val);

void test_motors(void); 
void poll_motors(void);

#endif
