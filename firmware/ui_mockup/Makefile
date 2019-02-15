CFLAGS = -g -Wall -DUI_MOCKUP \
         -I../Middlewares/Third_Party/u8g2/. \
         -I../Inc `sdl2-config --cflags`

SRC = $(shell ls ../Middlewares/Third_Party/u8g2/*.c) \
                 u8x8_d_sdl_128x64.c u8x8_sdl_key.c \
                 ../Src/ui.c ../Src/profile.c ../Src/u8g2_fonts_reflow.c \
                 main.c

OBJ = $(SRC:.c=.o)

helloworld: $(OBJ) 
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJ) `sdl2-config --libs` -o ui_mockup 

clean:	
	-rm $(OBJ) ui_mockup
