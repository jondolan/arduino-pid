/* Jon Dolan - 2019 - Arduino PID Implementation */
/* Motor.h - Implementation of an interrupt based quadrature encoder motor class (used on Pololu motors)
 * 
 * Motor::Motor()            - Initalize the class with the pwm pin, direction pin, encoder_a and _b pins, and desired PID controller constants
 * Motor::stop()             - Stop the motor movement
 * Motor::update()           - Implemented by inherited classes to update the controller and output
 * Motor::backwards()        - Specify if the motor is backwards from the default config
 * Motor::set_target()       - Command the PID controller with a new target
 * Motor::update_target()    - Update the PID controller traget without resetting the error terms
 * 
 * Internal methods:
 * Motor::set_output()    - Command a new output value to the pwm pin, 0-255
 * Motor::encoder_isr()   - function for handling quadrature encoder interrupts
*/

#pragma once
#include "PID.h"
#include "Arduino.h"

const int8_t encoder_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
class Motor {
    protected:

        int current_direction = 1; // start forward
        int direction_coef = 1; // 0 if backwards
        
        uint32_t last_micros = 0;
        int32_t last_encoder = 0;

        PID * pid;
        float target = 0.0f;

        static volatile int32_t encoder_count;
        static uint8_t enc_val;

        void set_output(int);           // 0-255
        static void encoder_isr(void);
        void forward(void);
        void backward(void);

    public:
        // constructor
        Motor(float p, float i, float d);

        void backwards(void);

        void set_target(float);
        void update_target(float);
        void stop(void);

        virtual int update(void) = 0;
};


// Initalizer
template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
Motor<pwm_pin, dir_pin, enca_pin, encb_pin>::Motor(float p, float i, float d) {

    // attach the interrupt pins, which should be passed in using digitalPinToInterrupt
    attachInterrupt(enca_pin, encoder_isr, CHANGE);
    attachInterrupt(encb_pin, encoder_isr, CHANGE);

    // set the direction pin as an output
    pinMode(dir_pin, OUTPUT);

    // initalize the PID
    pid = new PID(p, i, d, 255);
    pid->reset();

}

// Protected
template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
volatile int32_t Motor<pwm_pin, dir_pin, enca_pin, encb_pin>::encoder_count = 0;

template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
uint8_t Motor<pwm_pin, dir_pin, enca_pin, encb_pin>::enc_val = 0;

template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
void Motor<pwm_pin, dir_pin, enca_pin, encb_pin>::encoder_isr() {
    enc_val = (enc_val << 2) | (digitalRead(enca_pin) << 1) | digitalRead(encb_pin);

    encoder_count += encoder_table[enc_val & 0b1111];
}

template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
void Motor<pwm_pin, dir_pin, enca_pin, encb_pin>::set_output(int new_speed) {
    // analogWrite(pwm_pin, new_speed);
}

template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
void Motor<pwm_pin, dir_pin, enca_pin, encb_pin>::forward() {
    digitalWrite(dir_pin, 1);
}

template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
void Motor<pwm_pin, dir_pin, enca_pin, encb_pin>::backward() {
    digitalWrite(dir_pin, 0);
}


// Public
template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
void Motor<pwm_pin, dir_pin, enca_pin, encb_pin>::backwards() {
    direction_coef = 0;
}

template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
void Motor<pwm_pin, dir_pin, enca_pin, encb_pin>::stop() {
    // analogWrite(pwm_pin, 0);
}

template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
void Motor<pwm_pin, dir_pin, enca_pin, encb_pin>::set_target(float new_target) {
    this->update_target(new_target);
    this->stop();
    pid->reset();
    this->update();
}

template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
void Motor<pwm_pin, dir_pin, enca_pin, encb_pin>::update_target(float new_target) {
    this->target = (direction_coef) ? new_target : -new_target;
}

