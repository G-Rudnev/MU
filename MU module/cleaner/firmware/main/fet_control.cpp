#include "pins.h"
#include "fet_control.h"


void poll_fet(void) {

  analogWrite( FET_9, ModbusRTUServer.holdingRegisterRead(REG_WATER_PUMP));
  analogWrite( FET_3, ModbusRTUServer.holdingRegisterRead(REG_VACUUM_CLEAN));
  //analogWrite( FET_3, ModbusRTUServer.holdingRegisterRead(REG_FET_3));
  //analogWrite( FET_4, ModbusRTUServer.holdingRegisterRead(REG_FET_4));
  //analogWrite( FET_5, ModbusRTUServer.holdingRegisterRead(REG_FET_5));
  //analogWrite( FET_6, ModbusRTUServer.holdingRegisterRead(REG_FET_6));
  //analogWrite( FET_7, ModbusRTUServer.holdingRegisterRead(REG_FET_7));
  //analogWrite( FET_8, ModbusRTUServer.holdingRegisterRead(REG_FET_8));
  //analogWrite( FET_9, ModbusRTUServer.holdingRegisterRead(REG_FET_9));
  //analogWrite( FET_10, ModbusRTUServer.holdingRegisterRead(REG_FET_10));

}
