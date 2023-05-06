#include "pins.h"
#include "motor_control.h"


void subMotor1( int16_t val ) {

  if ( val != 0 )
  {
    if ( val > 0 )
    {
      digitalWrite( MOTOR_1_DIR, HIGH );
    }
    else
    {
      digitalWrite( MOTOR_1_DIR, LOW );
    }
    analogWrite( MOTOR_1_PWM, abs( val ) );
  }
  else
  {
    digitalWrite( MOTOR_1_PWM, LOW );
  }  
}



void subMotor2( int16_t val ) {

  if ( val != 0 )
  {
    if ( val > 0 )
    {
      digitalWrite( MOTOR_2_DIR, HIGH );
    }
    else
    {
      digitalWrite( MOTOR_2_DIR, LOW );
    }
    analogWrite( MOTOR_2_PWM, abs( val ) );
  }
  else
  {
    digitalWrite( MOTOR_2_PWM, LOW );
  }  
}


void poll_motors(void) {

  subMotor1(ModbusRTUServer.holdingRegisterRead(REG_FRONT_ACTUATOR));
  subMotor2(ModbusRTUServer.holdingRegisterRead(REG_REAR_ACTUATOR));
  //subMotor3(ModbusRTUServer.holdingRegisterRead(REG_MOTOR_3));
  //subMotor4(ModbusRTUServer.holdingRegisterRead(REG_MOTOR_4));
  //subMotor5(ModbusRTUServer.holdingRegisterRead(REG_MOTOR_5));
  //subMotor6(ModbusRTUServer.holdingRegisterRead(REG_MOTOR_6));

}


void test_motors(void) {

  digitalWrite( MOTOR_1_DIR, LOW );
  digitalWrite( MOTOR_2_DIR, LOW );
  digitalWrite( MOTOR_3_DIR, LOW );
  digitalWrite( MOTOR_4_DIR, LOW );
  digitalWrite( MOTOR_5_DIR, LOW );
  digitalWrite( MOTOR_6_DIR, LOW );

  analogWrite( MOTOR_1_PWM, 100);
  analogWrite( MOTOR_2_PWM, 100);
  analogWrite( MOTOR_3_PWM, 100);
  analogWrite( MOTOR_4_PWM, 100);
  analogWrite( MOTOR_5_PWM, 100);
  analogWrite( MOTOR_6_PWM, 100);

}
