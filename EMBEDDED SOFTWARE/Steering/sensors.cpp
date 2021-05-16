#include "sensors.h"
#include "functions.h"

#define mag_enocder A0
#define pot_pin A1

#define min_pot 75.0
#define max_pot 315.0

// Variables for PWM reading of magnetic encoder
float dutyCycle = 0.0;
float averageDutyCylce = 0.0;
float counter = 0.0;

void setup_sensors(void) {
    pinMode(pot_pin, INPUT);
    pinMode(mag_enocder, INPUT);
}

// Function to calculate the PWM duty cylce of an input
void read_magnetic_encoder_PWM(void) {
    unsigned long highTime = pulseIn(mag_enocder, HIGH);
    unsigned long lowTime = pulseIn(mag_enocder, LOW);
    unsigned long cycleTime = highTime + lowTime;
    dutyCycle = (float)highTime / float(cycleTime);
    //counter ++;
}

// Return mapped potentiometer value
float read_pot(void) {
    return (mapValue(float(analogRead(pot_pin)), 1024.0, 0.0, min_pot, max_pot));
}

// Return duty cylce of PWM input
float get_duty_cylce(void) {
    averageDutyCylce = dutyCycle * 360.0; // dutyCycle / counter * 360.0;
    return averageDutyCylce;
}

void reset_duty_cylce(void) {
    dutyCycle = 0.0;
    counter = 0.0;
}