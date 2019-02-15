#ifndef TRIAC_H
#define TRIAC_H

#include "stm32f0xx_hal.h"

#define TRIAC_DUTY_MAX 100

typedef struct {
    TIM_HandleTypeDef *htim;
    uint16_t channel;
    uint16_t derating;
    uint16_t duty;    
} triac_t;

int triac_init(triac_t *obj, TIM_HandleTypeDef *htim,
               uint16_t channel, uint16_t derating);
uint16_t triac_duty(triac_t *obj);
uint16_t triac_derating(triac_t *obj);
void triac_duty_set(triac_t *obj, uint16_t duty);

#endif // TRIAC_H
