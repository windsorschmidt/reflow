#ifndef TIMER_H
#define TIMER_H

void timer_init(void);
void timer_task(void *params);
void timer_tick_1hz(void);
void timer_tick_speaker(void);

#endif // TIMER_H
