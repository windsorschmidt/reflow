#ifndef APP_INIT
#define APP_INIT

// #define NOBEEP

#define APP_IPC_Q_SIZE 8
#define LOG_IPC_Q_SIZE 8
#define TEMP_IPC_Q_SIZE 8
#define OLED_IPC_Q_SIZE 8
#define SPEAKER_IPC_Q_SIZE 8
#define BUTTON_IPC_Q_SIZE 8
#define TIMER_IPC_Q_SIZE 8

#define BUTTON_ISR_Q_SIZE 4
#define TIMER_ISR_Q_SIZE 4

#define MSG_TIMER_1HZ_TICK     0x0001
#define MSG_TIMER_SPEAKER_TICK 0x0002
#define MSG_TEMP_TEMP          0x0101
#define MSG_TEMP_FAULT         0x0102
#define MSG_BUTTON_PREV        0x0201
#define MSG_BUTTON_SET         0x0202
#define MSG_BUTTON_NEXT        0x0203
#define MSG_BUTTON_PANIC       0x0204
#define MSG_TRIAC1_DUTY        0x0301
#define MSG_TRIAC2_DUTY        0x0302
#define MSG_TRIAC_MORE_POWER   0x0303
#define MSG_TRIAC_LESS_POWER   0x0304
#define MSG_BT_CONNECT         0x0401
#define MSG_BT_DISCONNECT      0x0402
#define MSG_REFLOW_START       0x0501
#define MSG_REFLOW_STOP        0x0502
#define MSG_REFLOW_STAGE       0x0503
#define MSG_REFLOW_RUNTIME     0x0504
#define MSG_PROFILE_TEMP       0x0601
#define MSG_SPEAKER_TEST       0x0701
#define MSG_SPEAKER_BEEP       0x0702
#define MSG_SPEAKER_ALERT      0x0703

void app_init(void);
void app_task(void *params);

#endif // APP_INIT
