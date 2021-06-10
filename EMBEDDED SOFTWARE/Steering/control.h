#ifndef control_h
#define control_h

#include "Arduino.h"

void controlAction(int);
float get_feedback(void);
bool get_motor_direction(void);
float get_error(void);
float get_reference(void);
void set_motor_speed(int);

#endif