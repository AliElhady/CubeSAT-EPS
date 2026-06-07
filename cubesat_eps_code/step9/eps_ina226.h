#ifndef EPS_INA226_H
#define EPS_INA226_H

#include "main.h"

typedef struct
{
    float current_mA;
    float voltage_V;
    float power_mW;
    uint8_t ok;
} INA226_Data;

void INA226_Init (uint8_t addr);
INA226_Data INA226_Read (uint8_t addr);
uint8_t INA226_Check(uint8_t addr);

#endif
