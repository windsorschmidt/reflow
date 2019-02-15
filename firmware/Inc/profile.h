#ifndef PROFILE_H
#define PROFILE_H

#define PROFILE_NUM_SETPOINTS 6
#define PROFILE_MAX_TEMP 250
#define PROFILE_MAX_TIME 300

typedef struct {
    uint16_t time;
    uint16_t temp;
} setpoint_t;

typedef struct {
    uint8_t num_setpoints;    
    setpoint_t setpoints[PROFILE_NUM_SETPOINTS];
} profile_t;

int profile_init(profile_t *obj);
uint8_t profile_num_setpoints(profile_t *obj);
uint16_t profile_setpoint_temp(profile_t *obj, int n);
uint16_t profile_setpoint_time(profile_t *obj, int n);
void profile_setpoint_temp_set(profile_t *obj, int n, uint16_t temp);
void profile_setpoint_time_set(profile_t *obj, int n, uint16_t time);
uint16_t profile_duration(profile_t *obj);
uint16_t profile_max_temp(profile_t *obj);
uint16_t profile_stage(profile_t *obj, uint16_t t);
uint16_t profile_temp(profile_t *obj, uint16_t t);   

#endif // PROFILE_H
