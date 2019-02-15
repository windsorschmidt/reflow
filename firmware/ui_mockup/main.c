#include <stdio.h>
#include <SDL.h>

#include "u8g2.h"
#include "app.h"
#include "profile.h"
#include "ui.h"

u8g2_t u8g2;
profile_t profile;
ui_t ui;

extern SDL_Surface *u8g_sdl_screen;
extern SDL_Window *u8g_sdl_window;
extern uint32_t u8g_sdl_color[256];

int main(int argc, char *argv[])
{
    profile_init(&profile);
    ui_init(&ui, &profile, &u8g2);
    
    u8g2_SetupBuffer_SDL_128x64_1(&u8g2, &u8g2_cb_r0);
    u8x8_InitDisplay(u8g2_GetU8x8(&u8g2));
    u8x8_SetPowerSave(u8g2_GetU8x8(&u8g2), 0);  

    u8g_sdl_color[3] = SDL_MapRGB( u8g_sdl_screen->format, 255, 245, 0 );
    u8g_sdl_color[4] = SDL_MapRGB( u8g_sdl_screen->format, 0, 0, 0 );
    SDL_UpdateWindowSurface(u8g_sdl_window);

    ui_handle_msg(&ui, MSG_BT_CONNECT);

    for (int i=0; i<78; i++) {
        ui_handle_msg(&ui, (MSG_PROFILE_TEMP | profile_temp(&profile, i*6-20) << 16));
        ui_handle_msg(&ui, (MSG_TEMP_TEMP | 27000 << 16));
    }

    ui_draw(&ui);
    SDL_SaveBMP(u8g_sdl_screen, "reflow_main.bmp");

    for (int i=0; i<4; i++) {
        ui_handle_msg(&ui, MSG_BUTTON_SET);
    }
    ui_draw_profile(&ui, &profile);
    SDL_SaveBMP(u8g_sdl_screen, "reflow_profile.bmp");
    
    /* while(1) { */

    /*     ui_draw_profile(&ui, &profile); */
    /*     SDL_Delay(10); */

    /*     switch(u8g_sdl_get_key()) { */
    /*     case 's': */
    /*         SDL_SaveBMP(u8g_sdl_screen, "screenshot.bmp"); */
    /*         break; */
    /*     case 'q': */
    /*         return 0; */
    /*     } */
    /* } */
    
    return 0;
}
