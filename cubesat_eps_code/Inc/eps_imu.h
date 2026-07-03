#ifndef EPS_IMU_H
#define EPS_IMU_H
#define IMU1_CS_PORT GPIOB
#define IMU1_CS_PIN GPIO_PIN_12
#define IMU2_CS_PORT GPIOC
#define IMU2_CS_PIN GPIO_PIN_13 
#include "main.h"

typedef struct
{
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    uint8_t ok;
} IMU_Data;

typedef struct
{
    IMU_Data imu1;
    IMU_Data imu2;
    uint8_t  fault;
} IMU_System;

void IMU_Init  (uint8_t imu_num);
uint8_t IMU_Check (uint8_t imu_num);
IMU_Data IMU_Read  (uint8_t imu_num);
IMU_System IMU_Update(void);
uint8_t IMU_ReadWhoAmI(uint8_t imu_num);

#endif
