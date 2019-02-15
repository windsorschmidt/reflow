#include <stdint.h>
#include <string.h>
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "SEGGER_RTT.h"
#include "main.h"
#include "app.h"
#include "timer.h"

extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim3;
extern QueueHandle_t app_ipc_q;

QueueHandle_t timer_isr_q;
QueueHandle_t timer_ipc_q;

void timer_init(void) {
    timer_isr_q = xQueueCreate(TIMER_ISR_Q_SIZE, sizeof(uint8_t));
    timer_ipc_q = xQueueCreate(TIMER_IPC_Q_SIZE, sizeof(uint32_t));
}

void timer_tick_1hz(void) {
    uint8_t isr_msg = 0;
    xQueueSendFromISR(timer_isr_q, &isr_msg, (TickType_t) 0);
}      

void timer_tick_speaker(void) {
    uint8_t isr_msg = 1;
    xQueueSendFromISR(timer_isr_q, &isr_msg, (TickType_t) 0);
}

void timer_task(void *params) {
  (void)params;

  uint8_t isr_msg;
  uint32_t ipc_msg;

  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_Base_Start_IT(&htim3);

  while (1) {
    if (xQueueReceive(timer_isr_q, &isr_msg, 0)) {
      switch (isr_msg) {
      case 0:
        ipc_msg = MSG_TIMER_1HZ_TICK;
        break;
      case 1:
        ipc_msg = MSG_TIMER_SPEAKER_TICK;
        break;
      }
      xQueueSend(app_ipc_q, &ipc_msg, (TickType_t)0);
    }
  }
}

