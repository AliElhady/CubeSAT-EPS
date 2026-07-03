/* ============================================================
 *  CubeSat EPS — Step 9: ICM-42688 IMU via I2C
 *
 *  U2 (imu_num=1) — I2C2 (PA8/PA9), addr 0x69 (SDO=HIGH)
 *  U9 (imu_num=2) — I2C3 (PC8/PC9), addr 0x69 (SDO=HIGH)
 * ============================================================ */

#include "eps_imu.h"
#include <math.h>

#define IMU_FAULT_THRESHOLD  0.5f
#define IMU_I2C_ADDR        (0x69 << 1)  /* SDO=HIGH → 0x69, shifted for HAL */
#define IMU_TIMEOUT          10

/* ICM-42688 registers */
#define REG_WHO_AM_I  0x75
#define REG_PWR_MGMT0 0x4E
#define REG_ACCEL_CFG 0x50
#define REG_GYRO_CFG  0x51
#define REG_ACCEL_X1  0x1F
#define REG_GYRO_X1   0x25

extern I2C_HandleTypeDef hi2c2;   /* U2 — IMU1 */
extern I2C_HandleTypeDef hi2c3;   /* U9 — IMU2 */

static I2C_HandleTypeDef* IMU_GetBus(uint8_t imu_num)
{
    return (imu_num == 1) ? &hi2c2 : &hi2c3;
}

static HAL_StatusTypeDef IMU_WriteReg(uint8_t imu_num, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return HAL_I2C_Master_Transmit(IMU_GetBus(imu_num),
                                   IMU_I2C_ADDR, buf, 2, IMU_TIMEOUT);
}

static uint8_t IMU_ReadReg(uint8_t imu_num, uint8_t reg)
{
    uint8_t val = 0;
    HAL_I2C_Master_Transmit(IMU_GetBus(imu_num),
                            IMU_I2C_ADDR, &reg, 1, IMU_TIMEOUT);
    HAL_I2C_Master_Receive(IMU_GetBus(imu_num),
                           IMU_I2C_ADDR, &val, 1, IMU_TIMEOUT);
    return val;
}

uint8_t IMU_Check(uint8_t imu_num)
{
    uint8_t id = IMU_ReadReg(imu_num, REG_WHO_AM_I);
    return (id == 0x47) ? 1 : 0;
}

uint8_t IMU_ReadWhoAmI(uint8_t imu_num)
{
    return IMU_ReadReg(imu_num, REG_WHO_AM_I);
}

void IMU_Init(uint8_t imu_num)
{
    IMU_WriteReg(imu_num, REG_PWR_MGMT0, 0x0F);  /* accel+gyro Low Noise */
    HAL_Delay(1);
    IMU_WriteReg(imu_num, REG_ACCEL_CFG, 0x23);  /* ±8g, 1kHz */
    IMU_WriteReg(imu_num, REG_GYRO_CFG,  0x23);  /* ±2000°/s, 1kHz */
}

IMU_Data IMU_Read(uint8_t imu_num)
{
    IMU_Data data = {0};
    uint8_t ax_hi = IMU_ReadReg(imu_num, REG_ACCEL_X1);
    uint8_t ax_lo = IMU_ReadReg(imu_num, REG_ACCEL_X1 + 1);
    uint8_t ay_hi = IMU_ReadReg(imu_num, REG_ACCEL_X1 + 2);
    uint8_t ay_lo = IMU_ReadReg(imu_num, REG_ACCEL_X1 + 3);
    uint8_t az_hi = IMU_ReadReg(imu_num, REG_ACCEL_X1 + 4);
    uint8_t az_lo = IMU_ReadReg(imu_num, REG_ACCEL_X1 + 5);
    uint8_t gx_hi = IMU_ReadReg(imu_num, REG_GYRO_X1);
    uint8_t gx_lo = IMU_ReadReg(imu_num, REG_GYRO_X1 + 1);
    uint8_t gy_hi = IMU_ReadReg(imu_num, REG_GYRO_X1 + 2);
    uint8_t gy_lo = IMU_ReadReg(imu_num, REG_GYRO_X1 + 3);
    uint8_t gz_hi = IMU_ReadReg(imu_num, REG_GYRO_X1 + 4);
    uint8_t gz_lo = IMU_ReadReg(imu_num, REG_GYRO_X1 + 5);
    data.accel_x = (int16_t)((ax_hi << 8) | ax_lo) / 4096.0f;
    data.accel_y = (int16_t)((ay_hi << 8) | ay_lo) / 4096.0f;
    data.accel_z = (int16_t)((az_hi << 8) | az_lo) / 4096.0f;
    data.gyro_x  = (int16_t)((gx_hi << 8) | gx_lo) / 16.4f;
    data.gyro_y  = (int16_t)((gy_hi << 8) | gy_lo) / 16.4f;
    data.gyro_z  = (int16_t)((gz_hi << 8) | gz_lo) / 16.4f;
    data.ok = 1;
    return data;
}

IMU_System IMU_Update(void)
{
    IMU_System sys;
    sys.imu1 = IMU_Read(1);
    sys.imu2 = IMU_Read(2);
    float dx = sys.imu1.accel_x - sys.imu2.accel_x;
    float dy = sys.imu1.accel_y - sys.imu2.accel_y;
    float dz = sys.imu1.accel_z - sys.imu2.accel_z;
    sys.fault = (fabsf(dx) > IMU_FAULT_THRESHOLD ||
                 fabsf(dy) > IMU_FAULT_THRESHOLD ||
                 fabsf(dz) > IMU_FAULT_THRESHOLD) ? 1 : 0;
    return sys;
}