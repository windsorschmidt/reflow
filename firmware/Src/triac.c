#include "triac.h"

int triac_init(triac_t *obj, TIM_HandleTypeDef *htim,
               uint16_t channel, uint16_t derating) {
    obj->htim = htim;
    obj->channel = channel;
    obj->duty = 0;
    obj->derating = derating;
    triac_duty_set(obj, 0);
    HAL_TIM_PWM_Start(htim, channel);
    return 0;
}

uint16_t triac_duty(triac_t *obj) {
    return obj->duty;
}

uint16_t triac_derating(triac_t *obj) {
    return obj->derating;
}

void triac_duty_set(triac_t *obj, uint16_t duty) {
    if (duty <= TRIAC_DUTY_MAX) {
        obj->duty = duty;
        __HAL_TIM_SET_COMPARE(obj->htim, obj->channel, 1000 - (duty * 10));
    }
}
