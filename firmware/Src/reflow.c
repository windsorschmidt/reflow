#include <stm32f0xx_hal.h>
#include <cmsis_os.h>
#include <SEGGER_RTT.h>
#include "app.h"
#include "reflow.h"


int reflow_init(reflow_t *obj, profile_t *profile, triac_t *triac1,
                triac_t *triac2, pidctl_t *pid) {
    obj->profile = profile;
    obj->triac1 = triac1;
    obj->triac2 = triac2;
    obj->temp = 0;
    obj->temp_1 = 0;
    obj->temp_2 = 0;
    obj->temp_3 = 0;
    obj->pid = pid;
    obj->runtime = 0;
    obj->running = 0;
    obj->stage = 0;
    obj->stage_last = 0;
    return 0;
}

void reflow_start(reflow_t *obj) {
    obj->running = 1;
}

void reflow_stop(reflow_t *obj) {
    obj->runtime = 0;
    obj->running = 0;
    obj->stage = 0;
    obj->stage_last = 0;
}

uint16_t reflow_running(reflow_t *obj) {
    return obj->running;
}

uint16_t reflow_runtime(reflow_t *obj) {
    return obj->runtime;
}

uint16_t reflow_stage(reflow_t *obj) {
    return obj->stage + obj->running;
}

uint16_t reflow_profile_temp(reflow_t *obj) {
    return profile_temp(obj->profile, obj->runtime);
}

uint8_t reflow_stage_changed(reflow_t *obj) {
    if (obj->stage != obj->stage_last) {
        return 1;
    }
    return 0;
}

uint8_t reflow_complete(reflow_t *obj) {
    if (obj->runtime == profile_duration(obj->profile)) {
        return 1;
    }
    return 0;
}

uint16_t reflow_power(reflow_t *obj, triac_t *triac)
{
    uint16_t p = pid_out(obj->pid);
    uint16_t d;

    d = triac_derating(triac);

    return (d <= p) ? p - d : 0;
}

void reflow_temp_set(reflow_t *obj, uint16_t t) {   
    obj->temp_3 = obj->temp_2;
    obj->temp_2 = obj->temp_1;
    obj->temp_1 = obj->temp;
    obj->temp = (obj->temp_1 + obj->temp_2 + obj->temp_3 + t) / 4;
}

void reflow_step(reflow_t *obj) {
    uint16_t setpoint;

    setpoint = profile_temp(obj->profile, obj->runtime) * 100;
    pid_setpoint_set(obj->pid, (float) setpoint);
    pid_compute(obj->pid, (float) obj->temp);

    if (obj->running) {
        obj->runtime++;
        obj->stage_last = obj->stage;
        obj->stage = profile_stage(obj->profile, obj->runtime);
    }
}
