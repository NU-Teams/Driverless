/* TODO:
 * - Read battery voltage and have safe limits
 * - Implement software hard limits
 * - Set limits on change in PWM signal from magnetic encoder
 */


#include "button.h"
#include "sensors.h"
#include "motor.h"
#include "control.h"
#include "Arduino.h"

// Control modes:
//  = 0, use pot as reference
//  = 1, use path planning data as reference
#define control_mode 0
#define delay_millis 10

unsigned long prev_millis = 0;

void setup() {
    Serial.begin(115200);
    setup_sensors();
    motor_setup();
    
    delay(100);
    // set_motor_speed(100);
    // while (read_pot() > get_duty_cylce() * 0.95 && read_pot() < get_duty_cylce() * 1.05) {
    //     for (int i = 0; i < 10; i++) {
    //         read_magnetic_encoder_PWM();
    //     }
    //     Serial.println("Centering Program Running");
    //     controlAction(control_mode);
    //     set_motor();
    // }
    // Serial.println("Centering Program Finished");
    delay(100);
    //set_motor_speed(255);
}


// Main loop
void loop() {
    // If delay is met, execute control action
    if (millis() - prev_millis > delay_millis) {
        // Continually update control action
        read_magnetic_encoder_PWM();
        prev_millis = millis();
        printSerialData();
        controlAction(control_mode);
        set_motor();
    }
}

void printSerialData() {
    // Print to serial monitor the motor angle and reference value from the pot
    Serial.print(get_duty_cylce());
    Serial.print("   ");
    if (control_mode == 1) {
        Serial.print(get_reference());
    } else {
        Serial.print(read_pot());
    }
    Serial.print("   ");
    Serial.print(get_error());
    Serial.print("   ");
    Serial.println(get_feedback());
}

