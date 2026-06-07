/* ============================================================
 *  CubeSat EPS — Step 9: ICM-42688 IMU via SPI
 *
 *  U2 (imu_num=1) — SPI1, CS on PA3  // GPIO_PIN
 *  U9 (imu_num=2) — SPI1, CS on PC1  // GPIO_PIN
 *
 *  Two sensors for redundancy.
 *  If the discrepancy between accelerometers > IMU_FAULT_THRESHOLD → fault=1
 *
 *  Call order:
 *    IMU_Init(1) + IMU_Init(2) — once in main()
 *    IMU_Update() — in the main loop
 * ============================================================ */

#include "eps_imu.h"
#include <math.h>
/* ----------------------------------------------------------
 *  ICM-42688 registers
 * ---------------------------------------------------------- */
#define REG_WHO_AM_I 0x75
#define REG_PWR_MGMT0 0x4E
#define REG_ACCEL_CFG 0x50
#define REG_GYRO_CFG 0x51
#define REG_ACCEL_X1 0x1F
#define REG_GYRO_X1 0x25
#define IMU1_CS_PORT GPIOA
#define IMU1_CS_PIN GPIO_PIN_3
#define IMU2_CS_PORT GPIOC
#define IMU2_CS_PIN GPIO_PIN_1
#define IMU_FAULT_THRESHOLD  0.5f
extern SPI_HandleTypeDef hspi2;
/* ----------------------------------------------------------
 *  CS control macros
 *  CS=LOW  → the sensor begins listening
 *  CS=HIGH → the sensor stops listening
 * ---------------------------------------------------------- */
#define IMU_CS_LOW(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define IMU_CS_HIGH(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
/* ----------------------------------------------------------
 *  Utility functions (static — only within the module)
 * ---------------------------------------------------------- */
static void IMU_WriteReg(uint8_t imu_num, uint8_t reg, uint8_t value)
{
    GPIO_TypeDef *port = (imu_num == 1) ? IMU1_CS_PORT : IMU2_CS_PORT;
    uint16_t pin = (imu_num == 1) ? IMU1_CS_PIN : IMU2_CS_PIN;
    uint8_t buf[2];
    buf[0] = reg & 0x7F;
    buf[1] = value;
    IMU_CS_LOW(port, pin);
    HAL_SPI_Transmit(&hspi2, buf, 2, 100);
    IMU_CS_HIGH(port, pin);
}

static uint8_t IMU_ReadReg(uint8_t imu_num, uint8_t reg)
{
    GPIO_TypeDef *port = (imu_num == 1) ? IMU1_CS_PORT : IMU2_CS_PORT;
    uint16_t pin = (imu_num == 1) ? IMU1_CS_PIN : IMU2_CS_PIN;
    uint8_t tx[2] = { reg | 0x80, 0x00 };
    uint8_t rx[2] = {0, 0};
    IMU_CS_LOW(port, pin);
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
    IMU_CS_HIGH(port, pin);

    return rx[1];
}
/* ----------------------------------------------------------
 *  Public API
 * ---------------------------------------------------------- */
/* Connection check — WHO_AM_I should return 0x47 */
uint8_t IMU_Check(uint8_t imu_num)
{
    uint8_t id = IMU_ReadReg(imu_num, REG_WHO_AM_I);
    return (id == 0x47) ? 1 : 0;
}

/* Initialization — enabling sensors and configuring ranges */
void IMU_Init(uint8_t imu_num)
{
    /* Set the accelerometer and gyroscope to Low Noise mode */
    IMU_WriteReg(imu_num, REG_PWR_MGMT0, 0x0F);

    /* Accelerometer: range ±8g, 1 kHz */
    IMU_WriteReg(imu_num, REG_ACCEL_CFG, 0x23);

    /* Gyroscope: range ±2000 °/s, 1 kHz */
    IMU_WriteReg(imu_num, REG_GYRO_CFG,  0x23);
}

/* Reading the accelerometer and gyroscope */
IMU_Data IMU_Read(uint8_t imu_num)
{
    IMU_Data data;
    /* Read 6 bytes from the accelerometer (X, Y, Z—2 bytes each) */
    uint8_t ax_hi = IMU_ReadReg(imu_num, REG_ACCEL_X1);
    uint8_t ax_lo = IMU_ReadReg(imu_num, REG_ACCEL_X1 + 1);
    uint8_t ay_hi = IMU_ReadReg(imu_num, REG_ACCEL_X1 + 2);
    uint8_t ay_lo = IMU_ReadReg(imu_num, REG_ACCEL_X1 + 3);
    uint8_t az_hi = IMU_ReadReg(imu_num, REG_ACCEL_X1 + 4);
    uint8_t az_lo = IMU_ReadReg(imu_num, REG_ACCEL_X1 + 5);
    /* Read 6 bytes from the gyroscope */
    uint8_t gx_hi = IMU_ReadReg(imu_num, REG_GYRO_X1);
    uint8_t gx_lo = IMU_ReadReg(imu_num, REG_GYRO_X1 + 1);
    uint8_t gy_hi = IMU_ReadReg(imu_num, REG_GYRO_X1 + 2);
    uint8_t gy_lo = IMU_ReadReg(imu_num, REG_GYRO_X1 + 3);
    uint8_t gz_hi = IMU_ReadReg(imu_num, REG_GYRO_X1 + 4);
    uint8_t gz_lo = IMU_ReadReg(imu_num, REG_GYRO_X1 + 5);
    /* Combine the two bytes into an int16_t and convert the value.
     * int16_t is signed: the sensor can display values ranging from -8g to +8g */
    data.accel_x = (int16_t)((ax_hi << 8) | ax_lo) / 4096.0f;
    data.accel_y = (int16_t)((ay_hi << 8) | ay_lo) / 4096.0f;
    data.accel_z = (int16_t)((az_hi << 8) | az_lo) / 4096.0f;
    data.gyro_x = (int16_t)((gx_hi << 8) | gx_lo) / 16.4f;
    data.gyro_y = (int16_t)((gy_hi << 8) | gy_lo) / 16.4f;
    data.gyro_z = (int16_t)((gz_hi << 8) | gz_lo) / 16.4f;
    data.ok = 1;
    return data;
}
/* Update both sensors + fault detection */
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
