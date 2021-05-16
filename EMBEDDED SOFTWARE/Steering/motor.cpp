#include "motor.h"
#include "control.h"

#define motor_pin 6
#define motor_dir 7

void motor_setup(void) {
    pinMode(motor_pin, OUTPUT);
    pinMode(motor_dir, OUTPUT);

    analogWrite(motor_pin, 0);
    digitalWrite(motor_dir, HIGH);
}

void set_motor(void) {
    digitalWrite(motor_dir, get_motor_direction());
    analogWrite(motor_pin, int(get_feedback()));
}

void reset_motor(void) {
    digitalWrite(motor_dir, HIGH);
    analogWrite(motor_pin, 0);
}