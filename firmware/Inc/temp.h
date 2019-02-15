#ifndef TEMP_H
#define TEMP_H

#include "stm32f0xx_hal.h"

typedef struct {
    SPI_HandleTypeDef *hspi;
} temp_t;

typedef struct {
    uint32_t oc_fault : 1; /* thermocouple open (no connection) */
    uint32_t scg_fault : 1; /* thermocouple shorted to GND */
    uint32_t scv_fault : 1; /* thermocouple shorted to VCC */
    uint32_t reserved_2 : 1; /* always reads 0 */
    uint32_t junction_temp : 12; /* internal reference junction temperature */
    uint32_t fault : 1; /* set when any of SCV, SCG, or OC faults are active */
    uint32_t reserved_1 : 1; /* always reads 0 */
    uint32_t quarter : 1; /* quarter degree Celsius */
    uint32_t half : 1; /* half degree Celsius */
    uint32_t degrees_c : 11; /* thermocouple temperature in degrees Celsius */
    uint32_t sign : 1; /* set when temperature value is negative */
} temp_sample_t;

int temp_init(temp_t *obj, SPI_HandleTypeDef *hspi);
void temp_sample(temp_t *obj, temp_sample_t *t);

#endif // TEMP_H
