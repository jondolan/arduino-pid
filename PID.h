/* Jon Dolan - 2019 - Arduino PID Implementation */
/* PID.h - Implementation of a simple PID class 
 * 
 * PID::PID()    - Initalize PID class with p, i, d constants and a saturation threshold value above which the integral term will not accumulate, to avoid windup
 * PID::reset()  - Reset the current error terms
 * PID::update() - Update the PID with a new error term and return the controller's output value
*/

#include <stdint.h>

class PID {
    private:
        float kp, ki, kd,   // PID coefficients
              last_error, error_sum;    // state variables
            
        float threshold;    // saturation threshold for integral windup protection

        uint32_t last_micros;   // last time PID calculated an update

    public:
        // constructor
        PID(float p, float i, float d, float thres) : kp(p), ki(i), kd(d), threshold(thres) {};

        // commands
        void reset(void);       // automatically resets terms
        float update(float error);

};
void PID::reset() {
    error_sum = last_error = 0;
    last_micros = micros();
}
float PID::update(float error) {

    // get the latest microsecond time
    uint16_t now = micros();

    // determine the change in time
    uint16_t dt = now - last_micros;

    // calculate the p, i, and d error terms
    float output = kp*error + ki*error_sum*dt + kd*((error - last_error)/dt);
    
    // check against goal threshold value
    if (fabsf(output) <= threshold) {
        // if less than the saturation threshold, add the error to the running sum
        error_sum += error;
        // and update the output for the additional current error term, neglected earlier
        output += ki*error*dt;
    }

    // keep track of last error and last run time
    last_error = error;
    last_micros = now;

    // return controller output value
    return output;
}