/* Jon Dolan - 2019 - Arduino PID Implementation */
/* MotorVelocity.h - Inherit Motor.h to implement velocity control
 * 
 * Motor::Motor()         - Inherited initalizer, see Motor.h
 * Motor::update()        - Provide velocity control capabilities
*/

#pragma once
#include "Motor.h"
#include "Arduino.h"

template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
class MotorVelocity : public Motor<pwm_pin, dir_pin, enca_pin, encb_pin> {
    public:
        MotorVelocity(float p, float i, float d) : Motor<pwm_pin, dir_pin, enca_pin, encb_pin>(p,  i,  d) {};
        int update(void);
};

template<int pwm_pin, int dir_pin, int enca_pin, int encb_pin>
int MotorVelocity<pwm_pin, dir_pin, enca_pin, encb_pin>::update() {

    // disable interrupts to take the measurement without potentially changing the encoder_count
    noInterrupts();
    float dx = this->last_encoder - this->encoder_count;
    // update the last encoder value
    this->last_encoder = this->encoder_count;
    // record now time
    uint32_t now = millis();
    interrupts();
    
    // calculate the velocity
    float velocity = dx / (now - this->last_micros);

    // update the last time
    this->last_micros = now;

    // calculate the error
    float error = this->target - velocity;

    // get the result from PID
    float output = this->pid->update(error);

    // Serial.print("left rear error: "); Serial.println(error);

    // set the output according to sign of the output
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