#include <stdint.h>
#include <string.h>
#include "main.h"
#include "app.h"
#include "cmsis_os.h"
#include "SEGGER_RTT.h"
#include "temp.h"

int temp_init(temp_t *obj, SPI_HandleTypeDef *hspi) {
    obj->hspi = hspi;
    return 0;
}

void temp_sample(temp_t *obj, temp_sample_t *t) {
    HAL_GPIO_WritePin(TEMP_NSS_GPIO_Port, TEMP_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Receive(obj->hspi, (uint8_t *) t, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(TEMP_NSS_GPIO_Port, TEMP_NSS_Pin, GPIO_PIN_SET);
    
    uint32_t *p = (uint32_t *) t;
    *p = __builtin_bswap32(*p);
}
