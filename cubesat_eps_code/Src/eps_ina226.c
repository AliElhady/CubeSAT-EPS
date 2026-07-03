/* ============================================================
 *  CubeSat EPS — Step 8/9: INA226 current/voltage sensors
 *
 *  U3 — VBAT sensing,  I2C2 (PA8/PA9), addr 0x40
 *  U4 — Solar sensing, I2C3 (PC8/PC9), addr 0x40
 *  Shunt R23 = 20 mΩ
 * ============================================================ */

#include "eps_ina226.h"

/* Registers */
#define REG_CONFIG       0x00
#define REG_SHUNT_V      0x01
#define REG_BUS_V        0x02
#define REG_CURRENT      0x04
#define REG_CALIBRATION  0x05
#define REG_ID           0xFF

/* Addresses — both 0x40 (A0=GND, A1=GND), different buses */
#define INA226_ADDR_U3   0x40   /* U3 — VBAT,  I2C2 */
#define INA226_ADDR_U4   0x40   /* U4 — Solar, I2C3 */

#define R_SHUNT_MOHM     20
#define CURRENT_LSB_MA   1
#define CAL_VALUE        256

/* Two separate I2C buses */
extern I2C_HandleTypeDef hi2c2;   /* U3 — PA8/PA9  */
extern I2C_HandleTypeDef hi2c3;   /* U4 — PC8/PC9  */

/* Helper: get correct bus by device */
static I2C_HandleTypeDef* INA226_GetBus(uint8_t is_u3)
{
    return is_u3 ? &hi2c2 : &hi2c3;
}

/* ----------------------------------------------------------
 *  Low-level register access — takes I2C handle explicitly
 * ---------------------------------------------------------- */
static HAL_StatusTypeDef INA226_WriteReg(I2C_HandleTypeDef *hi2c,
                                          uint8_t addr,
                                          uint8_t reg,
                                          uint16_t value)
{
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = value & 0xFF;
    return HAL_I2C_Master_Transmit(hi2c, addr << 1, buf, 3, 100);
}

static uint16_t INA226_ReadReg(I2C_HandleTypeDef *hi2c,
                                uint8_t addr,
                                uint8_t reg)
{
    uint8_t buf[2] = {0, 0};
    HAL_I2C_Master_Transmit(hi2c, addr << 1, &reg, 1, 10);
    HAL_I2C_Master_Receive (hi2c, addr << 1, buf,  2, 10);
    return (uint16_t)(buf[0] << 8 | buf[1]);
}

/* ----------------------------------------------------------
 *  Public API — uses imu_num: 1=U3(VBAT), 2=U4(Solar)
 * ---------------------------------------------------------- */
uint8_t INA226_Check(uint8_t is_u3)
{
    I2C_HandleTypeDef *bus = INA226_GetBus(is_u3);
    uint8_t addr = is_u3 ? INA226_ADDR_U3 : INA226_ADDR_U4;
    uint16_t id = INA226_ReadReg(bus, addr, REG_ID);
    return (id == 0x2260) ? 1 : 0;
}

void INA226_Init(uint8_t is_u3)
{
    I2C_HandleTypeDef *bus = INA226_GetBus(is_u3);
    uint8_t addr = is_u3 ? INA226_ADDR_U3 : INA226_ADDR_U4;
    INA226_WriteReg(bus, addr, REG_CALIBRATION, CAL_VALUE);
    INA226_WriteReg(bus, addr, REG_CONFIG,      0x4527);
}

INA226_Data INA226_Read(uint8_t is_u3)
{
    I2C_HandleTypeDef *bus = INA226_GetBus(is_u3);
    uint8_t addr = is_u3 ? INA226_ADDR_U3 : INA226_ADDR_U4;

    INA226_Data data;
    uint16_t raw_current = INA226_ReadReg(bus, addr, REG_CURRENT);
    uint16_t raw_voltage = INA226_ReadReg(bus, addr, REG_BUS_V);

    data.current_mA = (float)raw_current * CURRENT_LSB_MA;
    data.voltage_V  = (float)raw_voltage * 0.00125f;
    data.power_mW   = data.current_mA * data.voltage_V / 1000.0f;
    data.ok         = 1;
    return data;
}
