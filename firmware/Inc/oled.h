#ifndef OLED_H
#define OLED_H

#include <u8g2.h>

#define OLED_MODE_SPLASH      0
#define OLED_MODE_UI          1 
#define OLED_MODE_TETRIS      2

#define OLED_SPLASH_TIMEOUT 0

typedef struct {
    u8g2_t *u8g2;
    uint8_t mode;
    uint8_t splash_timer;
} oled_t;

void oled_init(oled_t *obj, u8g2_t *u8g2, SPI_HandleTypeDef *hspi);
uint8_t oled_mode(oled_t *obj);
void oled_splash_handle_msg(oled_t *oled, uint32_t msg);

#define weyland_yutani_width 128
#define weyland_yutani_height 47

static const unsigned char weyland_yutani_bits[] = {
    0xb6, 0xfd, 0x66, 0x03, 0x8f, 0xe7, 0x03, 0xcc, 0x66, 0x3f, 0x8f, 0x67,
    0xe0, 0xf1, 0x7c, 0x3e, 0xb6, 0xfd, 0x66, 0x83, 0xdf, 0xef, 0x07, 0xcc,
    0x66, 0xbf, 0xdf, 0x6f, 0xf0, 0xfb, 0xfd, 0x7e, 0xb6, 0x0d, 0x66, 0x83,
    0xd9, 0x6c, 0x06, 0xcc, 0x66, 0x8c, 0xd9, 0x6c, 0x30, 0x9b, 0xcd, 0x66,
    0xb6, 0x3d, 0x66, 0x83, 0xd9, 0x6c, 0x06, 0xcc, 0x66, 0x8c, 0xd9, 0x6c,
    0x30, 0x98, 0xcd, 0x66, 0xb6, 0x3d, 0x7e, 0x83, 0xdf, 0x6c, 0xf6, 0xfd,
    0x66, 0x8c, 0xdf, 0x6c, 0x30, 0x98, 0x7d, 0x7e, 0xb6, 0x0d, 0x7c, 0x83,
    0xdf, 0x6c, 0xf6, 0xf9, 0x66, 0x8c, 0xdf, 0x6c, 0x30, 0x98, 0xfd, 0x3e,
    0xb6, 0x0d, 0x60, 0x83, 0xd9, 0x6c, 0x06, 0xc0, 0x66, 0x8c, 0xd9, 0x6c,
    0x30, 0x9b, 0xcd, 0x06, 0xfe, 0xfd, 0x7e, 0xbf, 0xd9, 0xec, 0x07, 0xfc,
    0x7e, 0x8c, 0xd9, 0x6c, 0xf0, 0xfb, 0xcd, 0x06, 0xfc, 0xfc, 0x3e, 0xbf,
    0xd9, 0xec, 0x03, 0x7c, 0x3c, 0x8c, 0xd9, 0x6c, 0xe0, 0xf1, 0xcc, 0x06,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x07, 0xf0, 0xff, 0xff, 0x00, 0xfc,
    0x3f, 0x00, 0xff, 0xff, 0x0f, 0xe0, 0xff, 0xff, 0xff, 0xff, 0x0f, 0xf0,
    0xff, 0xff, 0x00, 0xfe, 0x7f, 0x00, 0xff, 0xff, 0x0f, 0xf0, 0xff, 0xff,
    0xff, 0xff, 0x1f, 0xf0, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff,
    0x0f, 0xf8, 0xff, 0xff, 0xff, 0xff, 0x3f, 0xf0, 0xff, 0xff, 0x80, 0xff,
    0xff, 0x01, 0xff, 0xff, 0x0f, 0xfc, 0xff, 0xff, 0xfe, 0xff, 0x7f, 0xe0,
    0xff, 0x7f, 0xc0, 0xff, 0xff, 0x03, 0xfe, 0xff, 0x07, 0xfe, 0xff, 0x7f,
    0xfc, 0xff, 0xff, 0xc0, 0xff, 0x3f, 0xe0, 0xff, 0xff, 0x07, 0xfc, 0xff,
    0x03, 0xff, 0xff, 0x3f, 0xf8, 0xff, 0xff, 0x81, 0xff, 0x1f, 0xf0, 0xff,
    0xff, 0x0f, 0xf8, 0xff, 0x81, 0xff, 0xff, 0x1f, 0xf0, 0xff, 0xff, 0x03,
    0xff, 0x0f, 0xf8, 0xff, 0xff, 0x1f, 0xf0, 0xff, 0xc0, 0xff, 0xff, 0x0f,
    0xe0, 0xff, 0xff, 0x07, 0xfe, 0x07, 0xfc, 0xff, 0xff, 0x3f, 0xe0, 0x7f,
    0xe0, 0xff, 0xff, 0x07, 0xc0, 0xff, 0xff, 0x0f, 0xfc, 0x03, 0xfe, 0xff,
    0xff, 0x7f, 0xc0, 0x3f, 0xf0, 0xff, 0xff, 0x03, 0x80, 0xff, 0xff, 0x1f,
    0xf8, 0x01, 0xff, 0xff, 0xff, 0xff, 0x80, 0x1f, 0xf8, 0xff, 0xff, 0x01,
    0x00, 0xff, 0xff, 0x3f, 0xf0, 0x80, 0xff, 0xff, 0xff, 0xff, 0x01, 0x0f,
    0xfc, 0xff, 0xff, 0x00, 0x00, 0xfe, 0xff, 0x7f, 0x60, 0xc0, 0xff, 0xff,
    0xff, 0xff, 0x03, 0x06, 0xfe, 0xff, 0x7f, 0x00, 0x00, 0xfc, 0xff, 0xff,
    0x00, 0xe0, 0xff, 0xff, 0xff, 0xff, 0x07, 0x00, 0xff, 0xff, 0x3f, 0x00,
    0x00, 0xf8, 0xff, 0xff, 0x01, 0xf0, 0xff, 0x7f, 0xfe, 0xff, 0x0f, 0x80,
    0xff, 0xff, 0x1f, 0x00, 0x00, 0xf0, 0xff, 0xff, 0x03, 0xf8, 0xff, 0x3f,
    0xfc, 0xff, 0x1f, 0xc0, 0xff, 0xff, 0x0f, 0x00, 0x00, 0xe0, 0xff, 0xff,
    0x07, 0xfc, 0xff, 0x1f, 0xf8, 0xff, 0x3f, 0xe0, 0xff, 0xff, 0x07, 0x00,
    0x00, 0xc0, 0xff, 0xff, 0x0f, 0xfe, 0xff, 0x0f, 0xf0, 0xff, 0x7f, 0xf0,
    0xff, 0xff, 0x03, 0x00, 0x00, 0x80, 0xff, 0xff, 0x1f, 0xff, 0xff, 0x07,
    0xe0, 0xff, 0xff, 0xf8, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0xff, 0xff,
    0xbf, 0xff, 0xff, 0x03, 0xc0, 0xff, 0xff, 0xfd, 0xff, 0xff, 0x00, 0x00,
    0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0x01, 0x80, 0xff, 0xff, 0xff,
    0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x80,
    0x01, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff,
    0xff, 0xff, 0x7f, 0xc0, 0x03, 0xfe, 0xff, 0xff, 0xff, 0x1f, 0x00, 0x00,
    0x00, 0x00, 0xf0, 0xff, 0xff, 0xff, 0x3f, 0xe0, 0x07, 0xfc, 0xff, 0xff,
    0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x1f, 0xf0,
    0x0f, 0xf8, 0xff, 0xff, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff,
    0xff, 0xff, 0x0f, 0xf8, 0x1f, 0xf0, 0xff, 0xff, 0xff, 0x03, 0x00, 0x00,
    0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0x07, 0xfc, 0x3f, 0xe0, 0xff, 0xff,
    0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x03, 0xfe,
    0x7f, 0xc0, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe,
    0xff, 0xff, 0x01, 0xff, 0xff, 0x80, 0xff, 0xff, 0x7f, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x80, 0xff, 0xff, 0x01, 0xff, 0xff,
    0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x7f, 0xc0, 0xff,
    0xff, 0x03, 0xfe, 0xff, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0,
    0xff, 0x3f, 0xc0, 0xff, 0xff, 0x03, 0xfc, 0xff, 0x0f, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xe0, 0xff, 0x1f, 0xc0, 0xff, 0xff, 0x03, 0xf8, 0xff,
    0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x0f, 0xc0, 0xff,
    0xff, 0x03, 0xf0, 0xff, 0x03, 0x00, 0x00, 0x00 };

#endif // OLED_H