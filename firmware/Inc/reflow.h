#ifndef REFLOW_H
#define REFLOW_H

#include <stdint.h>
#include "profile.h"
#include "triac.h"
#include "temp.h"
#include "pid.h"

#define REFLOW_STAGE_IDLE    0
#define REFLOW_STAGE_PREHEAT 1
#define REFLOW_STAGE_SOAK    2
#define REFLOW_STAGE_REFLOW  3
#define REFLOW_STAGE_HOLD    4
#define REFLOW_STAGE_COOL    5
#define REFLOW_STAGE_DONE    6

typedef struct {
    profile_t *profile;
    triac_t *triac1;
    triac_t *triac2;
    pidctl_t *pid;
    uint16_t temp;
    uint16_t temp_1;
    uint16_t temp_2;
    uint16_t temp_3;
    uint16_t runtime;
    uint16_t running;
    uint16_t stage;
    uint16_t stage_last;
} reflow_t;

int reflow_init(reflow_t *obj, profile_t *profile, triac_t *triac1,
                triac_t *triac2, pidctl_t *pid);
void reflow_start(reflow_t *obj);
void reflow_stop(reflow_t *obj);
uint16_t reflow_running(reflow_t *obj);
uint16_t reflow_runtime(reflow_t *obj);
uint16_t reflow_stage(reflow_t *obj);
uint16_t reflow_profile_temp(reflow_t *obj);
uint8_t reflow_stage_changed(reflow_t *obj);
uint8_t reflow_complete(reflow_t *obj);
uint16_t reflow_power(reflow_t *obj, triac_t *triac);
void reflow_temp_set(reflow_t *obj, uint16_t temp);
void reflow_step(reflow_t *obj);

#endif // REFLOW_H
