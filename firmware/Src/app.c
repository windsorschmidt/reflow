#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <string.h>
#include <usbd_cdc_if.h>

#include "SEGGER_RTT.h"
#include "app.h"
#include "button.h"
#include "cmsis_os.h"
#include "main.h"
#include "oled.h"
#include "pid.h"
#include "profile.h"
#include "reflow.h"
#include "speaker.h"
#include "task.h"
#include "temp.h"
#include "timer.h"
#include "triac.h"
#include "ui.h"

QueueHandle_t app_ipc_q;
QueueHandle_t button_isr_q;
QueueHandle_t button_ipc_q;
QueueHandle_t oled_ipc_q;
QueueHandle_t speaker_ipc_q;
QueueHandle_t temp_ipc_q;
QueueHandle_t log_ipc_q;

temp_t temp;
triac_t triac1;
triac_t triac2;
profile_t profile;
reflow_t reflow;
u8g2_t u8g2;
oled_t oled;
speaker_t speaker;
ui_t ui;
pidctl_t pid;

extern SPI_HandleTypeDef hspi1;   /* temp */
extern SPI_HandleTypeDef hspi2;   /* OLED */
extern UART_HandleTypeDef huart4; /* bluetooth */
extern TIM_HandleTypeDef htim2;   /* servo */
extern TIM_HandleTypeDef htim14;  /* reflow timebase */
extern TIM_HandleTypeDef htim15;  /* tone transducer */
extern TIM_HandleTypeDef htim16;  /* triac 1 */
extern TIM_HandleTypeDef htim17;  /* triac 2 */

void button_task(void *params)
{
    (void)params;

    uint16_t isr_msg;
    uint32_t ipc_msg;

    while (1) {
        if (xQueueReceive(button_isr_q, &isr_msg, 0)) {
            switch (isr_msg) {
            case BUTTON_PREV_Pin:
                ipc_msg = MSG_BUTTON_PREV;
                break;
            case BUTTON_SET_Pin:
                ipc_msg = MSG_BUTTON_SET;
                break;
            case BUTTON_NEXT_Pin:
                ipc_msg = MSG_BUTTON_NEXT;
                break;
            case BUTTON_PANIC_Pin:
                ipc_msg = MSG_BUTTON_PANIC;
                break;
            case BT_STATE_Pin:
                if (HAL_GPIO_ReadPin(BT_STATE_GPIO_Port, BT_STATE_Pin)) {
                    ipc_msg = MSG_BT_CONNECT;
                } else {
                    ipc_msg = MSG_BT_DISCONNECT;
                }
                break;
            }
            xQueueSend(app_ipc_q, &ipc_msg, (TickType_t)0);
        }
    }
}

void temp_task(void *params)
{
    (void)params;

    temp_sample_t t;
    uint32_t ipc_msg;

    while (1) {
        temp_sample(&temp, &t);

        if (!t.fault) {
            ipc_msg = MSG_TEMP_TEMP;
            ipc_msg |=
                ((t.degrees_c * 100 + t.half * 50 + t.quarter * 25) << 16);
            xQueueSend(app_ipc_q, &ipc_msg, (TickType_t)0);
        } else {
            ipc_msg = MSG_TEMP_FAULT;
            xQueueSend(app_ipc_q, &ipc_msg, (TickType_t)0);
        }

        vTaskDelay(1000);
    }
}

void oled_task(void *params)
{
    (void)params;

    uint32_t msg;
    uint16_t mode = oled_mode(&oled);

    while (1) {
        while (xQueueReceive(oled_ipc_q, &msg, 0)) {
            switch (mode) {
            case OLED_MODE_UI:
                ui_handle_msg(&ui, msg);
                if (ui_dirty(&ui)) {
                    ui_update(&ui, &profile);
                }
                break;
            case OLED_MODE_SPLASH:
                oled_splash_handle_msg(&oled, msg);
                break;
            }
        }
    }
}

void speaker_task(void *params)
{
    (void)params;

    uint32_t ipc_msg;

    while (1) {
        if (xQueueReceive(speaker_ipc_q, &ipc_msg, 0)) {
            switch (ipc_msg) {
            case MSG_TIMER_SPEAKER_TICK:
                speaker_tick(&speaker);
                break;
            case MSG_SPEAKER_BEEP:
                speaker_beep(&speaker);
                break;
            case MSG_SPEAKER_ALERT:
                speaker_alert(&speaker);
                break;
            case MSG_SPEAKER_TEST:
                speaker_test(&speaker);
                break;
            }
        }
    }
}

void log_task(void *params)
{
    (void)params;

    char s[48];
    uint32_t msg;
    uint32_t t;

    while (1) {
        if (xQueueReceive(log_ipc_q, &msg, 0)) {
            uint16_t cmd = (msg & 0xffff);
            uint16_t arg = (msg >> 16);
            (void)arg;

            switch (cmd) {
            case MSG_TIMER_1HZ_TICK:
                break;
            case MSG_TIMER_SPEAKER_TICK:
                break;
            default:
                t = xTaskGetTickCount();
                sprintf(s, "{\"t\":%lu,\"cmd\":%d,\"arg\":%d}\r\n", t, cmd,
                        arg);
                HAL_UART_Transmit(&huart4, (uint8_t *)s, strlen(s),
                                  HAL_MAX_DELAY);
                CDC_Transmit_FS((uint8_t *)s, strlen(s));
                break;
            }
        }
    }
}

void app_task(void *params)
{
    (void)params;

    uint32_t msg;

    while (1) {
        if (xQueueReceive(app_ipc_q, &msg, 0)) {
            uint16_t cmd = (msg & 0xffff);
            uint16_t arg = (msg >> 16);

            xQueueSend(log_ipc_q, &msg, (TickType_t)0);

            switch (cmd) {
            case MSG_TIMER_SPEAKER_TICK:
                xQueueSend(speaker_ipc_q, &msg, (TickType_t)0);
                break;
            case MSG_TIMER_1HZ_TICK:
                if (reflow_running(&reflow)) {
                    reflow_step(&reflow);

                    if (reflow_stage_changed(&reflow)) {
                        if (reflow_stage(&reflow) == REFLOW_STAGE_COOL) {
                            msg = MSG_SPEAKER_ALERT;
                            xQueueSend(speaker_ipc_q, &msg, (TickType_t)0);
                        }
                        if (reflow_complete(&reflow)) {
                            reflow_stop(&reflow);
                            HAL_GPIO_WritePin(GPIOA, SERVO_EN_Pin, RESET);

                            msg = MSG_REFLOW_RUNTIME |
                                  (reflow_runtime(&reflow) << 16);
                            xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                            xQueueSend(log_ipc_q, &msg, (TickType_t)0);

                            triac_duty_set(&triac1, 0);
                            msg = MSG_TRIAC1_DUTY | (triac_duty(&triac1) << 16);
                            xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                            xQueueSend(log_ipc_q, &msg, (TickType_t)0);

                            triac_duty_set(&triac2, 0);
                            msg = MSG_TRIAC2_DUTY | (triac_duty(&triac2) << 16);
                            xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                            xQueueSend(log_ipc_q, &msg, (TickType_t)0);

                            msg = MSG_SPEAKER_ALERT;
                            xQueueSend(speaker_ipc_q, &msg, (TickType_t)0);
                        }
                        msg = MSG_REFLOW_STAGE | (reflow_stage(&reflow) << 16);
                        xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                        xQueueSend(log_ipc_q, &msg, (TickType_t)0);
                    }

                    if (reflow_running(&reflow)) {
                        msg = MSG_REFLOW_RUNTIME |
                              (reflow_runtime(&reflow) << 16);
                        xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                        xQueueSend(log_ipc_q, &msg, (TickType_t)0);
                    }

                    if (reflow_running(&reflow)) {
                        uint16_t p1 = reflow_power(&reflow, &triac1);
                        triac_duty_set(&triac1, p1);
                        msg = MSG_TRIAC1_DUTY | (p1 << 16);
                        xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                        xQueueSend(log_ipc_q, &msg, (TickType_t)0);

                        uint16_t p2 = reflow_power(&reflow, &triac2);
                        triac_duty_set(&triac2, p2);
                        msg = MSG_TRIAC2_DUTY | (p2 << 16);
                        xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                        xQueueSend(log_ipc_q, &msg, (TickType_t)0);
                    }
                }

                break;
            case MSG_TEMP_TEMP:
                reflow_temp_set(&reflow, arg);
                xQueueSend(oled_ipc_q, &msg, (TickType_t)0);

                /* send reflow profile temp at same rate as temp sensor */
                msg = MSG_PROFILE_TEMP | (reflow_profile_temp(&reflow) << 16);
                xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                xQueueSend(log_ipc_q, &msg, (TickType_t)0);
                break;
            case MSG_TEMP_FAULT:
                SEGGER_RTT_printf(0, "temp: fault\n");
                xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                break;
            case MSG_BUTTON_PREV:
                xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                msg = MSG_SPEAKER_BEEP;
                xQueueSend(speaker_ipc_q, &msg, (TickType_t)0);
                break;
            case MSG_BUTTON_SET:
                xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                msg = MSG_SPEAKER_BEEP;
                xQueueSend(speaker_ipc_q, &msg, (TickType_t)0);
                break;
            case MSG_BUTTON_NEXT:
                xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                msg = MSG_SPEAKER_BEEP;
                xQueueSend(speaker_ipc_q, &msg, (TickType_t)0);
                break;
            case MSG_BUTTON_PANIC:
                xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                msg = MSG_SPEAKER_BEEP;
                xQueueSend(speaker_ipc_q, &msg, (TickType_t)0);
                break;
            case MSG_BT_CONNECT:
                xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                break;
            case MSG_BT_DISCONNECT:
                xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                break;
            case MSG_REFLOW_START:
                if (!reflow_running(&reflow)) {
                    reflow_start(&reflow);
                    HAL_GPIO_WritePin(GPIOA, SERVO_EN_Pin, GPIO_PIN_SET);

                    msg = MSG_REFLOW_STAGE | (reflow_stage(&reflow) << 16);
                    xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                    xQueueSend(log_ipc_q, &msg, (TickType_t)0);
                }
                break;
            case MSG_REFLOW_STOP:
                if (reflow_running(&reflow)) {
                    reflow_stop(&reflow);
                    HAL_GPIO_WritePin(GPIOA, SERVO_EN_Pin, RESET);

                    msg = MSG_REFLOW_RUNTIME | (reflow_runtime(&reflow) << 16);
                    xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                    xQueueSend(log_ipc_q, &msg, (TickType_t)0);

                    msg = MSG_REFLOW_STAGE | (reflow_stage(&reflow) << 16);
                    xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                    xQueueSend(log_ipc_q, &msg, (TickType_t)0);

                    triac_duty_set(&triac1, 0);
                    msg = MSG_TRIAC1_DUTY | (triac_duty(&triac1) << 16);
                    xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                    xQueueSend(log_ipc_q, &msg, (TickType_t)0);

                    triac_duty_set(&triac2, 0);
                    msg = MSG_TRIAC2_DUTY | (triac_duty(&triac2) << 16);
                    xQueueSend(oled_ipc_q, &msg, (TickType_t)0);
                    xQueueSend(log_ipc_q, &msg, (TickType_t)0);
                }
                break;
            case MSG_TRIAC_LESS_POWER:
                triac_duty_set(&triac1, triac_duty(&triac1) - 10);
                msg = MSG_TRIAC1_DUTY | (triac_duty(&triac1) << 16);
                xQueueSend(log_ipc_q, &msg, (TickType_t)0);

                triac_duty_set(&triac2, triac_duty(&triac2) - 10);
                msg = MSG_TRIAC2_DUTY | (triac_duty(&triac2) << 16);
                xQueueSend(log_ipc_q, &msg, (TickType_t)0);
                break;
            case MSG_TRIAC_MORE_POWER:
                xQueueSend(oled_ipc_q, &msg, (TickType_t)0);

                triac_duty_set(&triac1, triac_duty(&triac1) + 10);
                msg = MSG_TRIAC1_DUTY | (triac_duty(&triac1) << 16);
                xQueueSend(log_ipc_q, &msg, (TickType_t)0);

                triac_duty_set(&triac2, triac_duty(&triac2) + 10);
                msg = MSG_TRIAC2_DUTY | (triac_duty(&triac2) << 16);
                xQueueSend(log_ipc_q, &msg, (TickType_t)0);
                break;
            }
        }
    }
}

void app_init(void)
{
    BaseType_t rval;

    triac_init(&triac1, &htim16, TIM_CHANNEL_1, 0);
    triac_init(&triac2, &htim17, TIM_CHANNEL_1, 0);
    profile_init(&profile);

    pid_init(&pid, 0.27f, 0.0f, 2.5f);
    reflow_init(&reflow, &profile, &triac1, &triac2, &pid);

    app_ipc_q = xQueueCreate(APP_IPC_Q_SIZE, sizeof(uint32_t));
    rval = xTaskCreate(app_task, "app", 180, NULL, 0, NULL);
    if (rval != pdPASS) {
        SEGGER_RTT_printf(0, "failed to create app task!\n");
    }

    log_ipc_q = xQueueCreate(LOG_IPC_Q_SIZE, sizeof(uint32_t));
    rval = xTaskCreate(log_task, "log", 140, NULL, 0, NULL);
    if (rval != pdPASS) {
        SEGGER_RTT_printf(0, "failed to create logging task!\n");
    }

    button_isr_q = xQueueCreate(BUTTON_ISR_Q_SIZE, sizeof(uint16_t));
    button_ipc_q = xQueueCreate(BUTTON_IPC_Q_SIZE, sizeof(uint32_t));
    rval = xTaskCreate(button_task, "button", 100, NULL, 0, NULL);
    if (rval != pdPASS) {
        SEGGER_RTT_printf(0, "failed to create button task!\n");
    }

    oled_init(&oled, &u8g2, &hspi2);
    ui_init(&ui, &profile, &u8g2);
    oled_ipc_q = xQueueCreate(OLED_IPC_Q_SIZE, sizeof(uint32_t));
    rval = xTaskCreate(oled_task, "oled", 180, NULL, 0, NULL);
    if (rval != pdPASS) {
        SEGGER_RTT_printf(0, "failed to create oled task!\n");
    }

    speaker_init(&speaker);
    speaker_ipc_q = xQueueCreate(SPEAKER_IPC_Q_SIZE, sizeof(uint32_t));
    rval = xTaskCreate(speaker_task, "speaker", 40, NULL, 0, NULL);
    if (rval != pdPASS) {
        SEGGER_RTT_printf(0, "failed to create speaker task!\n");
    }

    temp_init(&temp, &hspi1);
    temp_ipc_q = xQueueCreate(TEMP_IPC_Q_SIZE, sizeof(uint32_t));
    rval = xTaskCreate(temp_task, "temp", 100, NULL, 0, NULL);
    if (rval != pdPASS) {
        SEGGER_RTT_printf(0, "failed to create temperature task!\n");
    }

    timer_init();
    rval = xTaskCreate(timer_task, "timer", 100, NULL, 0, NULL);
    if (rval != pdPASS) {
        SEGGER_RTT_printf(0, "failed to create timer task!\n");
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
    (void)xTask;
    SEGGER_RTT_printf(0, "stack overflow in %s\n", pcTaskName);
}
