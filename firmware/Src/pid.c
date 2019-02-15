#include "pid.h"


int pid_init(pidctl_t *obj, float kp, float ki, float kd) {
    obj->setpoint = 0;
    obj->in_last = 0;
    obj->iterm = 0;
    obj->d_out = 0;
    obj->kp = kp;
    obj->ki = ki;
    obj->kd = kd;
    return 0;
}

void pid_setpoint_set(pidctl_t *obj, float x) {
    obj->setpoint = x;
}

float pid_compute(pidctl_t *obj, float in)
{
    float err = obj->setpoint - in;

    obj->iterm += (obj->ki * err);

    if (obj->iterm > PID_OUT_MAX) {
        obj->iterm = PID_OUT_MAX;
    } else if (obj->iterm < PID_OUT_MIN) {
        obj->iterm = PID_OUT_MIN;
    }

    float d_in = (in - obj->in_last);
    obj->d_out = obj->kp * err + obj->iterm - obj->kd * d_in;

    if (obj->d_out > PID_OUT_MAX) {
        obj->d_out = PID_OUT_MAX;
    } else if (obj->d_out < PID_OUT_MIN) {
        obj->d_out = PID_OUT_MIN;
    }

    obj->in_last = in;

    return obj->d_out;
}

float pid_out(pidctl_t *obj) {   
    return obj->d_out;
}
