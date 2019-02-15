#include <stdint.h>

#include "profile.h"

/* Leaded (Sn63 Pb37) reflow profile with heating/cooling rates
 * achievable using my modified Breville BOV450XL oven:
 *
 * - heating a small board with aluminum tray : ~2.2 degC/second
 * - cooling with the oven door fully open    : ~1.0 degC/second
 *
 * Reference:
 * https://www.compuphase.com/electronics/reflowsolderprofiles.htm
 */
static setpoint_t default_setpoints[PROFILE_NUM_SETPOINTS] = {
    { 0,    30 }, /* ramp to soak */
    { 70,  150 }, /* soak */
    { 190, 160 }, /* ramp to peak */
    { 235, 230 }, /* reflow */
    { 250, 230 }, /* hold */
    { 390,  30 }  /* cool down */
};

/* Looks like a model reflow profile, but just for show */
/* static setpoint_t default_setpoints[PROFILE_NUM_SETPOINTS] = { */
/*     { 0,  20}, /\* ramp to soak *\/ */
/*     {12, 150}, /\* soak *\/ */
/*     {36, 165}, /\* ramp to peak *\/ */
/*     {46, 235}, /\* reflow *\/ */
/*     {48, 235}, /\* hold *\/ */
/*     {70,  20} /\* cool down *\/ */
/* }; */

int profile_init(profile_t *obj)
{
    int i;
    for (i = 0; i < PROFILE_NUM_SETPOINTS; i++) {
        obj->setpoints[i].time = default_setpoints[i].time;
        obj->setpoints[i].temp = default_setpoints[i].temp;
    }
    return 0;
}

uint8_t profile_num_setpoints(profile_t *obj) { return PROFILE_NUM_SETPOINTS; }

uint16_t profile_setpoint_temp(profile_t *obj, int n)
{
    return obj->setpoints[n].temp;
}

uint16_t profile_setpoint_time(profile_t *obj, int n)
{
    return obj->setpoints[n].time;
}

void profile_setpoint_temp_set(profile_t *obj, int n, uint16_t temp)
{
    obj->setpoints[n].temp = temp;
}

void profile_setpoint_time_set(profile_t *obj, int n, uint16_t time)
{
    obj->setpoints[n].time = time;
}

uint16_t profile_duration(profile_t *obj)
{
    return obj->setpoints[PROFILE_NUM_SETPOINTS - 1].time;
}

uint16_t profile_max_temp(profile_t *obj)
{
    uint16_t t = 0;
    int i;

    for (i = 0; i < PROFILE_NUM_SETPOINTS; i++) {
        if (obj->setpoints[i].temp > t) {
            t = obj->setpoints[i].temp;
        }
    }

    return t;
}

uint16_t profile_stage(profile_t *obj, uint16_t t)
{
    setpoint_t *sp;
    int8_t i;
    uint16_t stage = 0;

    for (i = 0; i < PROFILE_NUM_SETPOINTS; i++) {
        sp = &obj->setpoints[i];
        if (t < sp->time) {
            break;
        };
        stage = i;
    }
    return stage;
}

uint16_t profile_temp(profile_t *obj, uint16_t t)
{
    setpoint_t *sp;
    int16_t x1, y1, x2, y2;
    int8_t i;

    for (i = 0; i < PROFILE_NUM_SETPOINTS; i++) {
        sp = &obj->setpoints[i];
        x1 = sp->time;
        y1 = sp->temp;
        if (i < PROFILE_NUM_SETPOINTS - 1) {
            sp = &obj->setpoints[i + 1];
            x2 = sp->time;
            y2 = sp->temp;
        } else {
            x2 = x1;
            y2 = y1;
        }
        if (t < x2) {
            break;
        };
    }

    int32_t tsp = (t - x1); /* time since last setpoint */
    int32_t dx = (x2 - x1); /* time between setpoints */
    int32_t dy = (y2 - y1); /* temperature change */
    int32_t m;

    m = (dx == 0) ? 0 : (dy * 100) / dx;

    return (m * tsp) / 100 + y1;
}
