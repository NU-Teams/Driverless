#include "control.h"
#include "button.h"
#include "sensors.h"
#include "functions.h"
#include "steerData.h"

#define feedback_limit 255

float feedback = 0.0;
float reference = 0.0;
float motor_angle = 0.0;
float error = 0.0;
bool direction = LOW;
float P_feedback = 8.0;

unsigned long prev_millis2 = 0;
#define delay_millis 50
int data_increment = 0;

void controlAction(int reference_mode) {

    // Retrieve sensor data
    if (reference_mode == 1) {
        reference = float(steerData[data_increment]) / 10000.0 * 180.0 / PI * 4.0 + 210.0;
        // Increment reference angle if delay has been reached.
        if (millis() - prev_millis2 > delay_millis) {
            prev_millis2 = millis();
            data_increment += 1;
        }
    } else {
        reference = read_pot();
    }
    
    // Get measured angle from Magnetic Encoder
    motor_angle = get_duty_cylce();
    //reset_duty_cylce();
    
    // Control law
    error = reference - motor_angle;

    // If increment is outside range of data file, output feedback of 0 and display message
    if (reference_mode == 1) {
        if (data_increment > 1000) {
            feedback = 0;
            Serial.println("FINISHED DATA READ");
        } else {
            feedback = P_feedback * error;
        }
    } else {
        feedback = P_feedback * error;
    }

    // Sort direction and output
    if (feedback < 0) {
        direction = HIGH;
    } else {
        direction = LOW;
    }
    // Set feedback value
    feedback = absoluteValue(feedback);

    // Check the feedback doesn't excced the analogWrite max value
    if (feedback > feedback_limit) {
        feedback = feedback_limit;
    }
}

// Return feedback value
float get_feedback(void) {
    return feedback;
}

// Return motor direction
bool get_motor_direction(void) {
    return direction;
}

// Return error
float get_error(void) {
    return error;
}

// Return reference
float get_reference(void) {
    return reference;
}

void set_motor_speed(int speed) {
    int feedback_limit2 = speed;
}