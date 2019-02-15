#include <stdint.h>
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "SEGGER_RTT.h"
#include "main.h"
#include "app.h"

extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

extern QueueHandle_t button_isr_q;

static uint8_t zeroes;
static const uint8_t cycles = 10;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{    
    if (GPIO_Pin == ZERO_CROSS_Pin) {
        zeroes++;
        if (zeroes >= (2 * cycles)) {
            __HAL_TIM_SET_COUNTER(&htim16, 0);
            zeroes = 0;
            HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
        }
    } else {
        xQueueSendFromISR(button_isr_q, (void *) &GPIO_Pin, (TickType_t) 0);
    }
}
