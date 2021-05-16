#include "button.h"

#define on_off_but 3
#define on_off_led 13

bool _pressed = false;

void button_setup(void) {
    pinMode(on_off_led, OUTPUT);
    pinMode(on_off_but, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(on_off_but), button_isr, CHANGE);
}

// Button debouncer and interrupt
void button_isr(void) {
    static unsigned long last_interrupt_time2 = 0;
    unsigned long interrupt_time2 = millis();
    if (interrupt_time2 - last_interrupt_time2 > 200) {
        _pressed = !_pressed;
        digitalWrite(on_off_led, _pressed);
    }
    last_interrupt_time2 = interrupt_time2;
}

bool is_pressed(void) {
    return _pressed;
}