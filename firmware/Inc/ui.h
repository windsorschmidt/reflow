#ifndef UI_H
#define UI_H

#include "u8g2.h"
#include "profile.h"

#define OLED_TEMP_FRAME_WIDTH 80
#define TEMP_DATA_SIZE (OLED_TEMP_FRAME_WIDTH - 2)
#define TRIAC_DUTY_BAR_WIDTH 38

#define UI_TEMP_STEP 1
#define UI_TIME_STEP 1

#define UI_MODE_MAIN        0
#define UI_MODE_PROFILE     1
#define UI_MODE_SPLASH      2

typedef struct {
    profile_t *profile;
    u8g2_t *u8g2;    
    uint16_t temp_data[TEMP_DATA_SIZE];
    uint16_t profile_data[TEMP_DATA_SIZE];
    uint16_t profile_temp;
    uint16_t runtime;
    uint8_t mode;
    uint8_t reflow_stage;
    uint8_t button_1;
    uint8_t button_2;
    uint8_t button_3;
    uint8_t temp_pos;
    uint8_t temp_units;
    uint8_t duty_1;
    uint8_t duty_2;
    uint8_t dirty;
    uint8_t bt_connected;
    uint8_t setpoint_cursor;
} ui_t;

void ui_init(ui_t *obj, profile_t *profile, u8g2_t *u8g2);
uint8_t ui_dirty(ui_t *obj);
void ui_dirty_set(ui_t *obj, uint8_t dirty);
void ui_handle_msg(ui_t *obj, uint32_t msg);
void ui_update(ui_t *obj, profile_t *profile);
void ui_mode_set(ui_t *obj, uint8_t n);
void ui_draw(ui_t *obj);
void ui_draw_profile(ui_t *obj, profile_t *profile);
uint8_t ui_setpoint_activate_next(ui_t *obj);
void ui_setpoint_active_param_increase(ui_t *obj);
void ui_setpoint_active_param_decrease(ui_t *obj);

#endif // UI_H
