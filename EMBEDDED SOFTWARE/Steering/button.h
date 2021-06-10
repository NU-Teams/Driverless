#ifndef button_h
#define button_h

#include "Arduino.h"

void button_setup(void);
void button_isr(void);
bool is_pressed(void);

#endif