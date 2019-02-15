#ifndef PID_H
#define PID_H

#include <stdint.h>

#define PID_OUT_MIN 0.0f
#define PID_OUT_MAX 100.0f

/* #define PID_KP 5.28 */
/* #define PID_KI 0.055f */
/* #define PID_KD 0.54f */

typedef struct {
    float setpoint;
    float in_last;
    float iterm;
    float d_out;
    float kp;
    float ki;
    float kd;
} pidctl_t;

int pid_init(pidctl_t *obj, float kp, float ki, float kd);
void pid_setpoint_set(pidctl_t *obj, float x);
float pid_compute(pidctl_t *obj, float in);
float pid_out(pidctl_t *obj);

#endif // PID_H
