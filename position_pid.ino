/* Jon Dolan - 2019 - Arduino PID Implementation */
/* position_pid.ino - Example position PID implementation */

// enable motor debug serial print statements, comment to disable
#define MOTOR_DEBUG (1)

// motor pid libraries
#include "Arduino.h"
#include "MotorVelocity.h"

/* motor hookups */
#define PWM1    (9)
#define DIR1    (48)

/* encoders */
#define ENC1_A  (42)
#define ENC1_B  (44)

/* PIDs */
#define P1 (6.0f)
#define I1 (0.00001f)
#define D1 (0.0f)

/* ticks per rev, can be found in motor/encoder datasheet */
#define MOTOR1_TICKS_REV (4741.44)  // 1 motor revolution is 4741.44 encoder counts

/* control flow */
#define PID_PERIOD  (10000)    // rate at which PID updates in microseconds

uint32_t last_run = 0,      // time of last PID run in microseconds
         curr_run = 0;

// setup the motor class
MotorVelocity<PWM1, DIR1, ENC1_A, ENC1_B> motor1(P1, I1, D1);

void setup() {

    #ifdef MOTOR_DEBUG
        Serial.begin(115200);
    #endif

    motor1.set_target(0);

    // uncomment if the motor does not start moving "forwards"
    // motor1.backwards();

}

// for demonstration, vary the position from 0 to 1 rev to -1 rev back to 1 rev
int32_t pos = 0;
uint32_t last_position_change = 0;
bool up = true;

void loop() {

    // increment the speed every .001 seconds
    if (micros() - last_position_change > 1000) {
        if (up) {
            pos++;
            if (pos >= MOTOR1_TICKS_REV) {
                up = false;
            }
        } else {
            pos--;
            if (pos <= MOTOR1_TICKS_REV) {
                up = true;
            }
        }

        motor1.update_target(pos);
        last_position_change = micros();
    }

    // determine if it's time to run the PID calculations again
    // this controller operates on a fixed dt
    if (micros() - last_run > PID_PERIOD) {

        motor1.update();

        last_run = micros();
    }
}