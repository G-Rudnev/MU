#ifndef MODBUS_REGS_H
#define MODBUS_REGS_H

#include <Arduino.h>

#define REG_INPUT_VOLTAGE     5
#define REG_SENSORS_STATE     6
#define REG_RESET_TIMEOUT     7
#define REG_NO_RESET_MODE     8
#define REG_TICKS             9

#define REG_FRONT_ACTUATOR    10
#define REG_REAR_ACTUATOR     11
#define REG_WATER_PUMP        14
#define REG_VACUUM_CLEAN      16

#define REG_FRESH_FULL        20
#define REG_FRESH_EMPTY       21

#define REG_DIRTY_FULL        22
#define REG_DIRTY_EMPTY       23

#define REG_FLOW_METER        24

#define REG_LED_1             30
#define REG_LED_2             31
#define REG_LED_3             32
#define REG_LED_4             33

#define REG_US_SENSOR_1       40
#define REG_US_SENSOR_2       41
#define REG_US_SENSOR_3       42
#define REG_US_SENSOR_4       43
#define REG_US_SENSOR_5       44

#define REG_IR_SENSOR_1       45
#define REG_IR_SENSOR_2       46

#define REG_US_THRESH_1       47
#define REG_US_THRESH_2       48
#define REG_US_THRESH_3       49
#define REG_US_THRESH_4       50

#endif
