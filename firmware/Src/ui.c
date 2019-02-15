#include <stdint.h>
#include <stdio.h>
#include <string.h>

#if !UI_MOCKUP
#include "SEGGER_RTT.h"
#include "cmsis_os.h"
#endif
#include "app.h"
#include "u8g2.h"
#include "profile.h"
#include "ui.h"

#if !UI_MOCKUP
extern QueueHandle_t app_ipc_q;
#endif

static const char *button_labels[] = {"stop", "edit", "start", "-", "select", "+"};
static uint8_t button_label_offsets[] = {9, 10, 7, 17, 5, 15};
static const char *temp_units_strings[] = {"C", "F"};
static const char *reflow_stage_strings[] = {
    "idle", "preheat", "soak", "reflow", "hold", "cool", "done"};

static char s[12];

void ui_init(ui_t *obj, profile_t *profile, u8g2_t *u8g2)
{
    obj->profile = profile;
    obj->u8g2 = u8g2;
    obj->temp_pos = TEMP_DATA_SIZE - 1;
    obj->profile_temp = 0;
    obj->runtime = 0;
    obj->mode = UI_MODE_MAIN;
    obj->reflow_stage = 0;
    obj->button_1 = 0;
    obj->button_2 = 1;
    obj->button_3 = 2;
    obj->temp_units = 0;
    obj->duty_1 = 0;
    obj->duty_2 = 0;
    obj->bt_connected = 0;
    obj->setpoint_cursor = 0;
    obj->dirty = 0;

    uint8_t i;
    for (i = 0; i < TEMP_DATA_SIZE; i++) {
        obj->temp_data[i] = 0;
        obj->profile_data[i] = 0;
    }
}

uint8_t ui_dirty(ui_t *obj) { return obj->dirty; }

void ui_dirty_set(ui_t *obj, uint8_t dirty) { obj->dirty = dirty; }

void ui_handle_msg(ui_t *obj, uint32_t msg)
{
    uint16_t cmd = (msg & 0xffff);
    uint16_t arg = (msg >> 16);

    switch (cmd) {
    case MSG_TIMER_1HZ_TICK:
        obj->runtime++;
        obj->dirty = 1;
        break;
    case MSG_TEMP_TEMP:
        obj->temp_pos++;
        if (obj->temp_pos >= TEMP_DATA_SIZE) {
            obj->temp_pos = 0;
        }
        obj->temp_data[obj->temp_pos] = arg;
        obj->profile_data[obj->temp_pos] = obj->profile_temp * 100;
        obj->dirty = 1;
        break;
    case MSG_PROFILE_TEMP:
        obj->profile_temp = arg;
        break;
    case MSG_TRIAC1_DUTY:
        obj->duty_1 = (arg * TRIAC_DUTY_BAR_WIDTH) / 100;
        obj->dirty = 1;
        break;
    case MSG_TRIAC2_DUTY:
        obj->duty_2 = (arg * TRIAC_DUTY_BAR_WIDTH) / 100;
        obj->dirty = 1;
        break;
    case MSG_BUTTON_PREV:
        switch (obj->mode) {
        case UI_MODE_MAIN:
            msg = MSG_REFLOW_STOP;
#if !UI_MOCKUP
            xQueueSend(app_ipc_q, &msg, (TickType_t)0);
#endif
            break;
        case UI_MODE_PROFILE:
            ui_setpoint_active_param_decrease(obj);
            break;
        }
        obj->dirty = 1;
        break;
    case MSG_BUTTON_SET:
        switch (obj->mode) {
        case UI_MODE_MAIN:
            ui_mode_set(obj, UI_MODE_PROFILE);
            break;
        case UI_MODE_PROFILE:
            if (!ui_setpoint_activate_next(obj)) {
                ui_mode_set(obj, UI_MODE_MAIN);
            }
            break;
        }
        obj->dirty = 1;
        break;
    case MSG_BUTTON_NEXT:
        switch (obj->mode) {
        case UI_MODE_MAIN:
            msg = MSG_REFLOW_START;
#if !UI_MOCKUP
            xQueueSend(app_ipc_q, &msg, (TickType_t)0);
#endif
            break;
        case UI_MODE_PROFILE:
            ui_setpoint_active_param_increase(obj);
            break;
        }
        obj->dirty = 1;
        break;
    case MSG_BUTTON_PANIC:
        break;
    case MSG_BT_CONNECT:
        obj->bt_connected = 1;
        obj->dirty = 1;
        break;
    case MSG_BT_DISCONNECT:
        obj->bt_connected = 0;
        obj->dirty = 1;
        break;
    case MSG_REFLOW_RUNTIME:
        obj->runtime = arg;
        break;
    case MSG_REFLOW_STAGE:
        obj->reflow_stage = arg;
        break;
    }
}

void ui_update(ui_t *obj, profile_t *profile)
{
    switch (obj->mode) {
    case UI_MODE_MAIN:
        ui_draw(obj);
        break;
    case UI_MODE_PROFILE:
        ui_draw_profile(obj, profile);
        break;
    }
}

void ui_mode_set(ui_t *obj, uint8_t n)
{
    obj->mode = n;
    obj->button_1 = 3 * n;
    obj->button_2 = 3 * n + 1;
    obj->button_3 = 3 * n + 2;
}

uint8_t ui_setpoint_activate_next(ui_t *obj)
{
    obj->setpoint_cursor++;
    if (obj->setpoint_cursor == 2 * profile_num_setpoints(obj->profile)) {
        obj->setpoint_cursor = 0;
    }
    return obj->setpoint_cursor;    
}


void ui_setpoint_active_param_increase(ui_t *obj)
{
    uint8_t x = obj->setpoint_cursor / 2;
    uint8_t y = obj->setpoint_cursor % 2;
    profile_t *p = obj->profile;    
    uint16_t t;

    if (y) {
        t = profile_setpoint_temp(p, x);
        if (t <= PROFILE_MAX_TEMP - UI_TEMP_STEP) {
          profile_setpoint_temp_set(p, x, t + UI_TEMP_STEP);
        }
    } else {
        t = profile_setpoint_time(p, x);
        if (t <= PROFILE_MAX_TIME - UI_TIME_STEP) {
            profile_setpoint_time_set(p, x, t + UI_TIME_STEP);
        }
    }
}

void ui_setpoint_active_param_decrease(ui_t *obj)
{
    uint8_t x = obj->setpoint_cursor / 2;
    uint8_t y = obj->setpoint_cursor % 2;
    profile_t *p = obj->profile;    
    uint16_t t;

    if (y) {
        t = profile_setpoint_temp(p, x);
        if (t >= UI_TEMP_STEP) {
          profile_setpoint_temp_set(p, x, t - UI_TEMP_STEP);
        }
    } else {
        t = profile_setpoint_time(p, x);
        if (t >= UI_TIME_STEP) {
            profile_setpoint_time_set(p, x, t - UI_TIME_STEP);
        }
    }
}

void ui_draw(ui_t *obj)
{
    u8g2_t *u8g2 = obj->u8g2;

    ui_dirty_set(obj, 0);

    u8g2_FirstPage(u8g2);

    do {
        /* temperature */
        u8g2_SetFont(u8g2, u8g2_font_calibration_gothic_nbp_tn);
        sprintf(s, "%03u", obj->temp_data[obj->temp_pos] / 100);
        u8g2_DrawStr(u8g2, 4, 14, s);

        /* temperature (fractional) */
        u8g2_SetFont(u8g2, u8g2_font_tom_thumb_4x6_mn);
        sprintf(s, ".%02u", obj->temp_data[obj->temp_pos] % 100);
        u8g2_DrawStr(u8g2, 30, 14, s);

        /* temperature units */
        u8g2_SetFont(u8g2, u8g2_font_chikita_tr);
        u8g2_DrawStr(u8g2, 35, 5, temp_units_strings[obj->temp_units]);
        u8g2_DrawCircle(u8g2, 32, 1, 1, U8G2_DRAW_ALL);

        /* timer */
        u8g2_SetFont(u8g2, u8g2_font_mercutio_sc_nbp_tn);
        sprintf(s, "%02d:%02d", obj->runtime / 60, obj->runtime % 60);
        u8g2_DrawStr(u8g2, 45, 10, s);

        /* reflow stage label */
        u8g2_SetFont(u8g2, u8g2_font_glasstown_nbp_tr);
        sprintf(s, "(%s)", reflow_stage_strings[obj->reflow_stage]);
        u8g2_DrawStr(u8g2, 74, 9, s);

        /* plot frame */
        u8g2_DrawFrame(u8g2, 45, 12, OLED_TEMP_FRAME_WIDTH, 35);

        /* temperature and reflow profile plot */
        uint8_t x;
        for (x = 0; x < TEMP_DATA_SIZE; x++) {
            u8g2_DrawPixel(u8g2, 46 + x,
                           45 - (obj->temp_data[(obj->temp_pos + x + 1) %
                                                (TEMP_DATA_SIZE)] /
                                 100) /
                                    8);
            if (x % 2) {
                u8g2_DrawPixel(u8g2, 46 + x,
                               45 - (obj->profile_data[(obj->temp_pos + x + 1) %
                                                       (TEMP_DATA_SIZE)] /
                                     100) /
                                        8);
            }
        }

        /* setpoint tick */
        int setpoint = obj->profile_data[obj->temp_pos];
        u8g2_DrawTriangle(u8g2, 128, 42 - ((setpoint / 100) / 8), 125,
                          45 - ((setpoint / 100) / 8), 128,
                          48 - ((setpoint / 100) / 8));

        /* setpoint tick label */
        u8g2_SetFont(u8g2, u8g2_font_tom_thumb_4x6_mn);
        sprintf(s, "%d", setpoint / 100);
        u8g2_DrawStr(u8g2, 116 - ((setpoint >= 10000) * 4),
                     43 + (setpoint > 18000) * 10 - ((setpoint / 100) / 8), s);

        /* button frames */
        u8g2_DrawRFrame(u8g2, 3, 50, 38, 14, 2);
        u8g2_DrawRFrame(u8g2, 45, 50, 38, 14, 2);
        u8g2_DrawRFrame(u8g2, 87, 50, 38, 14, 2);

        /* button labels */
        u8g2_SetFont(u8g2, u8g2_font_glasstown_nbp_tr);
        u8g2_DrawStr(u8g2, 4 + button_label_offsets[obj->button_1], 61,
                     button_labels[obj->button_1]);
        u8g2_DrawStr(u8g2, 46 + button_label_offsets[obj->button_2], 61,
                     button_labels[obj->button_2]);
        u8g2_DrawStr(u8g2, 88 + button_label_offsets[obj->button_3], 61,
                     button_labels[obj->button_3]);

        /* triac frame labels */
        u8g2_SetFont(u8g2, u8g2_font_chikita_tr);
        u8g2_DrawStr(u8g2, 4, 23, "ch.1 out");
        u8g2_DrawStr(u8g2, 4, 37, "ch.2 out");

        /* triac duty frames */
        u8g2_DrawFrame(u8g2, 4, 24, TRIAC_DUTY_BAR_WIDTH, 6);
        u8g2_DrawFrame(u8g2, 4, 38, TRIAC_DUTY_BAR_WIDTH, 6);

        /* triac duty bar graphs */
        u8g2_DrawBox(u8g2, 4, 24, obj->duty_1, 6);
        u8g2_DrawBox(u8g2, 4, 38, obj->duty_2, 6);

        /* Bluetooth connection indicator */
        if (obj->bt_connected) {
            u8g2_DrawRBox(u8g2, 116, 1, 9, 9, 2);
            u8g2_SetDrawColor(u8g2, 0);
            u8g2_DrawStr(u8g2, 118, 8, "B");
            u8g2_SetDrawColor(u8g2, 1);
        }

    } while (u8g2_NextPage(u8g2));
}

void ui_draw_profile(ui_t *obj, profile_t *profile)
{
    u8g2_t *u8g2 = obj->u8g2;
    int i;

    const uint16_t x = 4;
    const uint16_t y = 19;
    const uint16_t w = 119;
    const uint16_t h = 26;

    const uint8_t n = profile_num_setpoints(profile);
    const uint16_t d = profile_duration(profile);
    const uint16_t t = profile_max_temp(profile) + 10;

    ui_dirty_set(obj, 0);

    u8g2_FirstPage(u8g2);

    do {
        u8g2_DrawFrame(u8g2, x - 1, y - 1, w + 3, h + 3);

        uint8_t cx = (obj->setpoint_cursor / 2);
        uint8_t cy = (obj->setpoint_cursor % 2);

        u8g2_SetFont(u8g2, u8g2_font_glasstown_nbp_tr);
        u8g2_DrawStr(u8g2, x + 6, 15, "Time=");
        u8g2_DrawStr(u8g2, x + 68, 15, "Temp=");

        u8g2_SetFont(u8g2, u8g2_font_mercutio_sc_nbp_tr);
        sprintf(s, "%03ds", profile_setpoint_time(profile, cx));
        u8g2_DrawStr(u8g2, x + 32, 16, s);
        sprintf(s, "%03dC", profile_setpoint_temp(profile, cx));
        u8g2_DrawStr(u8g2, x + 97, 16, s);
        u8g2_DrawTriangle(u8g2,
                          x + 62 * cy, 6,
                          x + 62 * cy + 5, 11,
                          x + 62 * cy, 16);

        for (i = 0; i < n; i++) {
            uint16_t x1 = profile_setpoint_time(profile, i);
            uint16_t y1 = profile_setpoint_temp(profile, i);
            uint16_t x2 = profile_setpoint_time(profile, i + 1);
            uint16_t y2 = profile_setpoint_temp(profile, i + 1);

            uint8_t sx1 = (x1 * (w * 100) / d) / 100;
            uint8_t sy1 = (y1 * (h * 100) / t) / 100;
            uint8_t sx2 = (x2 * (w * 100) / d) / 100;
            uint8_t sy2 = (y2 * (h * 100) / t) / 100;

            if (i < n - 1) {
                u8g2_DrawLine(u8g2, x + sx1, y + h - sy1, sx2 + x, y + h - sy2);
            }
        }

        uint16_t x1 = profile_setpoint_time(profile, cx);
        uint16_t y1 = profile_setpoint_temp(profile, cx);
        uint16_t sx1 = (x1 * (w * 100) / d) / 100;
        uint16_t sy1 = (y1 * (h * 100) / t) / 100;

        /* profile control point target */
        u8g2_DrawFrame(u8g2, x + sx1 - 3, y + h - sy1 - 3, 7, 7);

        /* profile control point cross hairs */
        if (cy) {
            if (sy1 < (h - 3)) {
                u8g2_DrawVLine(u8g2, x + sx1, y, h - sy1 - 4);
            }
            if (sy1 > 3) {
                u8g2_DrawVLine(u8g2, x + sx1, y + h - sy1 + 5, sy1 - 4);
            }
        } else {
            if (sx1 < (w - 3)) {
                u8g2_DrawHLine(u8g2, x + sx1 + 5, y + h - sy1, w - sx1 - 4);
            }
            if (sx1 > 3) {
                u8g2_DrawHLine(u8g2, x, y + h - sy1, sx1 - 4);
            }
        }

        /* button frames */
        u8g2_DrawRFrame(u8g2, 3, 50, 38, 14, 2);
        u8g2_DrawRFrame(u8g2, 45, 50, 38, 14, 2);
        u8g2_DrawRFrame(u8g2, 87, 50, 38, 14, 2);

        /* button labels */
        u8g2_SetFont(u8g2, u8g2_font_glasstown_nbp_tr);
        u8g2_DrawStr(u8g2, 4 + button_label_offsets[obj->button_1], 61,
                     button_labels[obj->button_1]);
        u8g2_DrawStr(u8g2, 46 + button_label_offsets[obj->button_2], 61,
                     button_labels[obj->button_2]);
        u8g2_DrawStr(u8g2, 88 + button_label_offsets[obj->button_3], 61,
                     button_labels[obj->button_3]);

    } while (u8g2_NextPage(u8g2));
}
