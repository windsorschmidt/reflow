#ifndef SPEAKER_H
#define SPEAKER_H

#include <stdint.h>

typedef struct {
    uint8_t alert_ticks;
} speaker_t;

void speaker_init(speaker_t *obj);
void speaker_test(speaker_t *obj);
void speaker_tick(speaker_t *obj);
void speaker_beep(speaker_t *obj);
void speaker_alert(speaker_t *obj);

#endif // SPEAKER_H
