#include <stdint.h>
#include <string.h>

#include "SEGGER_RTT.h"
#include "app.h"
#include "cmsis_os.h"
#include "main.h"
#include "speaker.h"

extern TIM_HandleTypeDef htim15;

void speaker_init(speaker_t *obj) {
    obj->alert_ticks = 0;
}

void speaker_test(speaker_t *obj)
{
}

void speaker_tick(speaker_t *obj)
{
    if (obj->alert_ticks) {
        HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
        obj->alert_ticks--;
    }
}

void speaker_beep(speaker_t *obj) {
#ifndef NOBEEP
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
#endif
}

void speaker_alert(speaker_t *obj)
{
    obj->alert_ticks = 5;
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
}
