#include <stdint.h>
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "SEGGER_RTT.h"
#include "main.h"
#include "app.h"

/* QueueHandle_t button_isr_q; */
/* QueueHandle_t button_ipc_q; */

/* extern QueueHandle_t app_ipc_q; */
/* extern TIM_HandleTypeDef htim15; */


/* void button_init(void) { */
/*     button_isr_q = xQueueCreate(BUTTON_ISR_Q_SIZE, sizeof(uint16_t)); */
/*     button_ipc_q = xQueueCreate(BUTTON_IPC_Q_SIZE, sizeof(uint32_t)); */
/* } */

/* void button_task(void *params) */
/* { */
/*     uint16_t isr_msg; */
/*     uint32_t ipc_msg; */
    
/*     while(1) { */
/*         if(xQueueReceive(button_isr_q, &isr_msg, 0)) { */
/*             switch(isr_msg) { */
/*             case BUTTON_PREV_Pin: */
/*                 HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2); */
/*                 ipc_msg = MSG_BUTTON_PREV; */
/*                 break; */
/*             case BUTTON_SET_Pin: */
/*                 HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2); */
/*                 ipc_msg = MSG_BUTTON_SET; */
/*                 break; */
/*             case BUTTON_NEXT_Pin: */
/*                 HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2); */
/*                 ipc_msg = MSG_BUTTON_NEXT; */
/*                 break; */
/*             case BUTTON_PANIC_Pin: */
/*                 HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2); */
/*                 ipc_msg = MSG_BUTTON_PANIC; */
/*                 break; */
/*             case BT_STATE_Pin: */
/*                 if (HAL_GPIO_ReadPin(BT_STATE_GPIO_Port, BT_STATE_Pin)) { */
/*                     ipc_msg = MSG_BT_CONNECT; */
/*                 } else { */
/*                     ipc_msg = MSG_BT_DISCONNECT; */
/*                 } */
/*                 break; */
/*             } */
/*             xQueueSend(app_ipc_q, &ipc_msg, (TickType_t) 0);             */
/*         } */
/*     } */
/* } */
