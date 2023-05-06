#ifndef FET_CONTROL_H
#define FET_CONTROL_H

#include "modbus_regs.h"
#include "pins.h"
#include <Arduino.h>

#include <ArduinoModbus.h>
#include <ArduinoRS485.h>

void poll_fet(void);

#endif
