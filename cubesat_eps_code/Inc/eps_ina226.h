#ifndef EPS_INA226_H
#define EPS_INA226_H

#include "main.h"

typedef struct
{
    float   current_mA;
    float   voltage_V;
    float   power_mW;
    uint8_t ok;
} INA226_Data;

/* is_u3: 1 = U3 (VBAT, I2C2),  0 = U4 (Solar, I2C3) */
void        INA226_Init (uint8_t is_u3);
uint8_t     INA226_Check(uint8_t is_u3);
INA226_Data INA226_Read (uint8_t is_u3);

#endif /* EPS_INA226_H */
