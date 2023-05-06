#include <ArduinoModbus.h>
#include <ArduinoRS485.h>

#include "pins.h"
#include "motor_control.h"
#include "fet_control.h"
#include "SonarI2C.h"


#define NO_POLL_TIMEOUT 100

uint8_t numsonars = 5;
SonarI2C sonars[] = {
  SonarI2C (0x3F, 0, 2500),
  SonarI2C (0x3F, 1, 2500),
  SonarI2C (0x3F, 2, 2500),
  SonarI2C (0x3F, 3, 2500),
  SonarI2C (0x3F, 4, 2500),
};

float analog_vals[13];
volatile uint8_t pulseCount;  
unsigned long oldTime;

uint32_t no_poll_time = 0;

void setup()
{
  SonarI2C::begin(19, 15);
  SonarI2C::inverse = true;
  for (uint8_t i = 0; i < numsonars; i++) {
    sonars[i].init();
  }
     
  init_pins();
  //attachInterrupt(4, pulseCounter, FALLING);

  Serial.begin(115200);
  if (!ModbusRTUServer.begin(1, 115200)) {
    while (1);
  }

  ModbusRTUServer.configureHoldingRegisters(0x00, 100);

  //test_motors();

  digitalWrite( FET_1, HIGH ); 
  digitalWrite( FET_2, HIGH ); 
  //analogWrite( FET_1, 100 ); 
  //analogWrite( FET_2, 100 );
  //analogWrite( FET_3, 100 ); 
  //analogWrite( FET_4, 100 );  
  //analogWrite( FET_5, 100 );  
  //analogWrite( FET_6, 100 ); 
  //analogWrite( FET_7, 100 ); 
  //analogWrite( FET_8, 100 );   
  //analogWrite( FET_9, 200 );
  //digitalWrite( FET_10, true );

  delay(2000);

} 

void loop()
{
  ModbusRTUServer.poll();
  no_poll_time = no_poll_time + 1;
  if (no_poll_time > NO_POLL_TIMEOUT) {
    
   digitalWrite( FET_9, LOW ); 
   digitalWrite( FET_3, LOW );
   digitalWrite( MOTOR_1_PWM, LOW );
   digitalWrite( MOTOR_2_PWM, LOW );
   ModbusRTUServer.holdingRegisterWrite(REG_FRONT_ACTUATOR, 0);
   ModbusRTUServer.holdingRegisterWrite(REG_REAR_ACTUATOR, 0);
   ModbusRTUServer.holdingRegisterWrite(REG_WATER_PUMP, 0);
   ModbusRTUServer.holdingRegisterWrite(REG_VACUUM_CLEAN, 0); 
  }

  if (ModbusRTUServer.holdingRegisterRead(REG_RESET_TIMEOUT) == 1) {
    no_poll_time = 0;
    ModbusRTUServer.holdingRegisterWrite(REG_RESET_TIMEOUT, 0);
  }
  ModbusRTUServer.holdingRegisterWrite(REG_TICKS, no_poll_time);

  SonarI2C::doSonar();
  poll_analog_pins(analog_vals);
  ModbusRTUServer.holdingRegisterWrite(REG_INPUT_VOLTAGE, analog_vals[6]);

  for (uint8_t i = 0; i < numsonars; i++) {

    int val = sonars[i].mm();
    if (val == 0) val = 2500;
    ModbusRTUServer.holdingRegisterWrite(REG_US_SENSOR_1 + i, val);
    }

  if (digitalRead(INPUT_1) == HIGH) {
    ModbusRTUServer.holdingRegisterWrite(REG_DIRTY_EMPTY, 1);
  }
  else{
    ModbusRTUServer.holdingRegisterWrite(REG_DIRTY_EMPTY, 0);
  }


  if (digitalRead(INPUT_2) == HIGH) {
    ModbusRTUServer.holdingRegisterWrite(REG_DIRTY_FULL, 1);
  }
  else{
    ModbusRTUServer.holdingRegisterWrite(REG_DIRTY_FULL, 0);
  }


  if (digitalRead(INPUT_3) == HIGH) {
    ModbusRTUServer.holdingRegisterWrite(REG_FRESH_EMPTY, 1);
  }
  else{
    ModbusRTUServer.holdingRegisterWrite(REG_FRESH_EMPTY, 0);
  }    


  if (digitalRead(INPUT_4) == HIGH) {
    ModbusRTUServer.holdingRegisterWrite(REG_FRESH_FULL, 1);
  }
  else{
    ModbusRTUServer.holdingRegisterWrite(REG_FRESH_FULL, 0);
  }


  if (digitalRead(INPUT_5) == HIGH) {
    ModbusRTUServer.holdingRegisterWrite(REG_IR_SENSOR_1, 1);
  }
  else{
    ModbusRTUServer.holdingRegisterWrite(REG_IR_SENSOR_1, 0);
  }

  if (digitalRead(INPUT_6) == HIGH) {
    ModbusRTUServer.holdingRegisterWrite(REG_IR_SENSOR_2, 1);
  }
  else{
    ModbusRTUServer.holdingRegisterWrite(REG_IR_SENSOR_2, 0);
  }

  digitalWrite( OUTPUT_1, ModbusRTUServer.holdingRegisterRead(REG_LED_1));
  digitalWrite( OUTPUT_2, ModbusRTUServer.holdingRegisterRead(REG_LED_2));   

  if((millis() - oldTime) > 1000) { 
      detachInterrupt(4);
      oldTime = millis();
      ModbusRTUServer.holdingRegisterWrite(REG_FLOW_METER, pulseCount);
      pulseCount = 0;
      attachInterrupt(4, pulseCounter, FALLING);
  }

  poll_motors();
  poll_fet();
  poll_analog_pins(analog_vals);

  delay(10);
}


void pulseCounter()
{
  pulseCount++;
}
