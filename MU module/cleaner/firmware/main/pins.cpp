#include "pins.h"
#include <stdint.h>

void init_pins(void) {

 //TCCR1B = TCCR1B & B11111000 | B00000001; 
 //TCCR2B = TCCR2B & B11111000 | B00000001;
 //TCCR4B = TCCR4B & B11111000 | B00000001; 
 //TCCR5B = TCCR5B & B11111000 | B00000001;
 //TCCR3B = TCCR3B & B11111000 | B00000001;   

  pinMode( FET_1, OUTPUT );
  pinMode( FET_2, OUTPUT );
  pinMode( FET_3, OUTPUT );
  pinMode( FET_4, OUTPUT );
  pinMode( FET_5, OUTPUT );
  pinMode( FET_6, OUTPUT );
  pinMode( FET_7, OUTPUT );
  pinMode( FET_8, OUTPUT );
  pinMode( FET_9, OUTPUT );
  pinMode( FET_10, OUTPUT );    

  pinMode( BTN, INPUT );

  pinMode( PC_ON, OUTPUT );
  pinMode( PC_ST, INPUT );

  pinMode( DC_DC_1_CNTRL, OUTPUT );
  pinMode( DC_DC_2_CNTRL, OUTPUT );
  pinMode( DC_DC_3_CNTRL, OUTPUT );

  pinMode( MOTOR_1_PWM, OUTPUT );
  pinMode( MOTOR_1_DIR, OUTPUT );
  pinMode( MOTOR_2_PWM, OUTPUT );
  pinMode( MOTOR_2_DIR, OUTPUT );
  pinMode( MOTOR_3_PWM, OUTPUT );
  pinMode( MOTOR_3_DIR, OUTPUT );
  pinMode( MOTOR_4_PWM, OUTPUT );
  pinMode( MOTOR_4_DIR, OUTPUT );
  pinMode( MOTOR_5_PWM, OUTPUT );
  pinMode( MOTOR_5_DIR, OUTPUT );
  pinMode( MOTOR_6_PWM, OUTPUT );
  pinMode( MOTOR_6_DIR, OUTPUT );

  pinMode( OUTPUT_1, OUTPUT );
  pinMode( OUTPUT_2, OUTPUT );
  pinMode( OUTPUT_3, OUTPUT );
  pinMode( OUTPUT_4, OUTPUT );

  pinMode( INPUT_1, INPUT );
  pinMode( INPUT_2, INPUT );
  pinMode( INPUT_3, INPUT );
  pinMode( INPUT_4, INPUT );
  pinMode( INPUT_5, INPUT );
  pinMode( INPUT_6, INPUT );

  pinMode( MOTOR_5_CUR, INPUT );

  pinMode(19, INPUT);
  digitalWrite(19, HIGH);

}

void disable_pins(void) {

  digitalWrite( MOTOR_1_PWM, LOW);
  digitalWrite( MOTOR_2_PWM, LOW);
  digitalWrite( MOTOR_3_PWM, LOW);
  digitalWrite( MOTOR_4_PWM, LOW);
  digitalWrite( MOTOR_5_PWM, LOW);
  digitalWrite( MOTOR_6_PWM, LOW);

  digitalWrite( FET_1, LOW);
  //digitalWrite( FET_2, LOW); 
  digitalWrite( FET_3, LOW); 
  digitalWrite( FET_4, LOW); 
  digitalWrite( FET_5, LOW); 
  //digitalWrite( FET_6, LOW); 
  digitalWrite( FET_7, LOW);         
  digitalWrite( FET_8, LOW); 
  digitalWrite( FET_9, LOW); 
  digitalWrite( FET_10, LOW);

} 


float read_analog(int pin, int coef) {

  return (analogRead(pin) * (5.0 / 1023.0) * coef);
}


void poll_analog_pins(float (& vals) [13]) {

  vals[0] = read_analog( DC_DC_1_V, 11) - 0.2;
  vals[1] = read_analog( DC_DC_2_V, 11) - 0.2;
  vals[2] = read_analog( DC_DC_3_V, 11) - 0.2;

  vals[3] = read_analog( DC_DC_1_CUR, 0.2);
  vals[4] = read_analog( DC_DC_1_CUR, 0.2);
  vals[5] = read_analog( DC_DC_1_CUR, 0.2);

  vals[6] = read_analog( INPUT_V, 11) - 0.2;

  vals[7] = read_analog( MOTOR_1_CUR, 1.5);
  vals[8] = read_analog( MOTOR_2_CUR, 1.5);
  vals[9] = read_analog( MOTOR_3_CUR, 1.5);
  vals[10] = read_analog( MOTOR_4_CUR, 1.5);
  vals[11] = read_analog( MOTOR_5_CUR, 1.5);
  vals[12] = read_analog( MOTOR_6_CUR, 1.5);

} 
