/* Jon Dolan - 2019 - Arduino PID Implementation */
/* MotorPositon.h - Inherit Motor.h to implement position control
 * 
 * Motor::Motor()         - Inherited initalizer, see Motor.h
 * Motor::update()        - Provide position control capabilities
*/

#pragma once
#include "Motor.h"
#include "Arduino.h"

template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
class MotorPosition : public Motor<pwm_pin, dir_pin, enca_pin, encb_pin> {
    public:
        MotorPosition(float p, float i, float d) : Motor<pwm_pin, dir_pin, enca_pin, encb_pin>(p,  i,  d) {};
        int update(void);
};

template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
int MotorPosition<pwm_pin, dir_pin, enca_pin, encb_pin>::update() {

    // disable interrupts to take the measurement without potentially changing the encoder_count
    noInterrupts();
    float error = this->target - this->encoder_count;
    interrupts();

    // get the updated output
    float output = this->pid->update(error);

    // set the direction according to the sign of the output
    if (output > 0) {
        this->forward();
    } else {
        this->backward();
    }

    #ifdef MOTOR_DEBUG
        Serial.println(this->encoder_count);
    #endif

    // constrain to our range of ADC putput
    int cmd = constrain(fabsf(output), 0, 255);

    // set the output value for the motor
    this->set_output(cmd);

    // return it to the caller as well
    return cmd;
}