#include <stdint.h>
#include <string.h>
#include <stm32f0xx_hal.h>
#include <cmsis_os.h>
#include <SEGGER_RTT.h>
#include <u8g2.h>
#include "main.h"
#include "app.h"
#include "tetris.h"
#include "oled.h"

tetris_game *tg;
tetris_move move;
bool running;

extern SPI_HandleTypeDef hspi2; /* OLED */

/* TODO: how to get u8g2 SPI routines to take a HAL SPI handle as parameter?? */
uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                               void *arg_ptr)
{
    (void) u8x8;
    switch (msg) {
    case U8X8_MSG_BYTE_SEND:
        HAL_SPI_Transmit(&hspi2, (uint8_t *) arg_ptr, arg_int, 10000);
        break;
    case U8X8_MSG_BYTE_INIT:
        break;
    case U8X8_MSG_BYTE_SET_DC:
        HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int);
        break;
    case U8X8_MSG_BYTE_START_TRANSFER:
        break;
    case U8X8_MSG_BYTE_END_TRANSFER:
        break;
    default:
        return 0;
    }
    return 1;
}

uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
    U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
    U8X8_UNUSED void *arg_ptr)
{
    switch (msg) {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        vTaskDelay(1);
        break;
    case U8X8_MSG_DELAY_MILLI:
        vTaskDelay(arg_int);
        break;
    case U8X8_MSG_GPIO_DC:
        HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int);
        break;
    case U8X8_MSG_GPIO_RESET:
        HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, arg_int);
        break;
    }
    return 1;
}

uint8_t oled_mode(oled_t *obj) {
    return obj->mode;
}


void oled_mode_set(oled_t *obj, uint8_t mode) {
    obj->mode = mode;

    switch(mode) {
    case OLED_MODE_TETRIS:
        move = TM_NONE;
        running = true;
        tg_reinit(tg, 20, 10);
        u8g2_Setup_ssd1309_128x64_noname0_2(obj->u8g2, U8G2_R1,
                                            u8x8_byte_4wire_hw_spi,
                                            u8x8_stm32_gpio_and_delay);
        break;
    }
}

void oled_init(oled_t *obj, u8g2_t *u8g2, SPI_HandleTypeDef *hspi) {
    (void) hspi;
    
    obj->mode = OLED_MODE_UI;
    obj->u8g2 = u8g2;
    
    /* tetris stuff */
    move = TM_NONE;
    running = true;
    tg = tg_create(20, 10);

    u8g2_Setup_ssd1309_128x64_noname0_2(obj->u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi,
                                        u8x8_stm32_gpio_and_delay);
    u8g2_InitDisplay(obj->u8g2);
    u8g2_SetPowerSave(obj->u8g2, 0);

    u8g2_FirstPage(obj->u8g2);

    do {
        u8g2_DrawXBM(obj->u8g2, 0, 8, weyland_yutani_width,
                     weyland_yutani_height, weyland_yutani_bits);
    } while(u8g2_NextPage(obj->u8g2));
}

void oled_splash_handle_msg(oled_t *oled, uint32_t msg) {
    (void) oled;
    (void) msg;
    
    /* TODO: what are we even doing here? */
}

void oled_fsm_tetris(uint32_t ipc_msg) {
    uint16_t cmd = (ipc_msg & 0xffff);

    switch (cmd) {
    case MSG_BUTTON_PREV:
        move = TM_LEFT;
        break;
    case MSG_BUTTON_NEXT:
        move = TM_RIGHT;
        break;
    case MSG_BUTTON_SET:
        move = TM_CLOCK;
        break;
    case MSG_BUTTON_PANIC:
        move = TM_DROP;
        break;
    }
}

void tetris_display_score(tetris_game *tg, u8g2_t *u8g2) {
    char s[12];
    u8g2_SetFont(u8g2, u8g2_font_chikita_tr);
    sprintf(s, "%d", tg->points);
    u8g2_DrawStr(u8g2, 1, 7, s);
    sprintf(s, "%d", tg->level);
    u8g2_DrawStr(u8g2, 1, 14, s);
    sprintf(s, "%d", tg->lines_remaining);
    u8g2_DrawStr(u8g2, 1, 21, s);
}

void tetris_display_board(tetris_game *obj, u8g2_t *u8g2)
{
    int i, j;
    for (i = 0; i < obj->rows; i++) {        
        for (j = 0; j < obj->cols; j++) {
            if (TC_IS_FILLED(tg_get(obj, i, j))) {
                u8g2_DrawBox(u8g2, j*5 + 1, i*5 + 25, 5, 5);
            } else {
                u8g2_DrawPixel(u8g2, j*5 + 3, i*5 + 27);
            }
        }
    }
    u8g2_DrawFrame(u8g2, 0, 24, 52, 102);
}

void tetris_display_piece(tetris_block block, u8g2_t *u8g2)
{
    char s[12];
    int b;
    tetris_location c;
    if (block.typ == -1) {
        return;
    }
    for (b = 0; b < TETRIS; b++) {
        c = TETROMINOS[block.typ][block.ori][b];
        u8g2_DrawBox(u8g2, c.col*5 + 30, c.row*5 + 10, 5, 5);
    }

    u8g2_SetFont(u8g2, u8g2_font_chikita_tr);
    sprintf(s, "NEXT");
    u8g2_DrawStr(u8g2, 28, 6, s);

    u8g2_DrawFrame(u8g2, 28, 8, 24, 14);
}

/* void oled_fsm_splash(oled_t *obj, uint32_t ipc_msg) { */
/*     uint16_t cmd = (ipc_msg & 0xffff); */

/*     switch (cmd) { */
/*     case MSG_TIMER_1HZ_TICK: */
/*         obj->splash_timer++; */
/*         if (obj->splash_timer >= UI_SPLASH_TIMEOUT) { */
/*             obj->mode = OLED_MODE_MAIN; */
/*             obj->dirty = 1; */
/*         } */
/*         break; */
/*     case MSG_BUTTON_PREV: */
/*     case MSG_BUTTON_SET: */
/*     case MSG_BUTTON_NEXT: */
/*     case MSG_BUTTON_PANIC: */
/*         obj->mode = OLED_MODE_MAIN; */
/*         obj->dirty = 1; */
/*         break; */
/*     } */
/* } */
