CFLAGS = -g -Wall -DUI_MOCKUP \
         -I../firmware/Middlewares/Third_Party/u8g2/. \
         -I../firmware/Inc `sdl2-config --cflags`

SRC = $(shell ls ../firmware/Middlewares/Third_Party/u8g2/*.c) \
                 u8x8_d_sdl_128x64.c u8x8_sdl_key.c \
                 ../firmware/Src/ui.c ../firmware/Src/profile.c \
                 main.c

OBJ = $(SRC:.c=.o)

helloworld: $(OBJ) 
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJ) `sdl2-config --libs` -o ui_mockup 

clean:	
	-rm $(OBJ) ui_mockup
