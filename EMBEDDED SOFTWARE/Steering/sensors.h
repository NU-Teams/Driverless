#ifndef sensors_h
#define sensors_h

#include "Arduino.h"

void setup_sensors(void);
void read_magnetic_encoder_PWM(void);
float read_pot(void);
float get_duty_cylce(void);
void reset_duty_cylce(void);

#endif