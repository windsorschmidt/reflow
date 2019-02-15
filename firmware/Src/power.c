#include <stdint.h>
#include <string.h>
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "SEGGER_RTT.h"
#include "main.h"
#include "app.h"
#include "power.h"

extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

extern QueueHandle_t oled_ipc_q;
static uint32_t ipc_msg;
static uint8_t duty;

void power_init(void) {
    duty = 0;
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
}

void power_more_power(void) {
    if (duty < 100) {
        duty += 10;
        __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 1000 - (duty * 10));
    }
    SEGGER_RTT_printf(0, "power duty: %d\n", duty);
    ipc_msg = MSG_POWER_DUTY_1;
    ipc_msg |= duty << 16;
    xQueueSend(oled_ipc_q, &ipc_msg, (TickType_t) 0);
    ipc_msg = MSG_POWER_DUTY_2;
    ipc_msg |= duty << 16;
    xQueueSend(oled_ipc_q, &ipc_msg, (TickType_t) 0);
}

void power_less_power(void) {
    if (duty >= 10) {
        duty -= 10;
        __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 1000 - (duty * 10));
    }
    SEGGER_RTT_printf(0, "power duty: %d\n", duty);
    ipc_msg = MSG_POWER_DUTY_1;
    ipc_msg |= duty << 16;
    xQueueSend(oled_ipc_q, &ipc_msg, (TickType_t) 0);
    ipc_msg = MSG_POWER_DUTY_2;
    ipc_msg |= duty << 16;
    xQueueSend(oled_ipc_q, &ipc_msg, (TickType_t) 0);
}
