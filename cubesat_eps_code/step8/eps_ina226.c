/* ============================================================
 *  CubeSat EPS — Step 8: INA226 current/voltage sensors
 *
 *  U3 (INA226_ADDR_U3) — charging current from the solar panel
 *  U4 (INA226_ADDR_U4) — load current
 *  Shunt R23 = 20 mΩ, I2C2 (PB6/PB7)
 *  Call order:
 *    INA226_Init(INA226_ADDR_U3) — once in main()
 *    INA226_Init(INA226_ADDR_U4)
 *    INA226_Read(INA226_ADDR_U3) — in the main loop
 *    INA226_Read(INA226_ADDR_U4)
 * ============================================================ */

#include "eps_ina226.h"
/* ----------------------------------------------------------
 *  INA226 Registers
 * ---------------------------------------------------------- */
#define REG_CONFIG 0x00
#define REG_SHUNT_V 0x01
#define REG_BUS_V 0x02
#define REG_CURRENT 0x04
#define REG_CALIBRATION 0x05
#define REG_ID 0xFF
/* ----------------------------------------------------------
 *  Device addresses and shunt parameters
 * ---------------------------------------------------------- */
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
 #define INA226_ADDR_U3 0x40
#define INA226_ADDR_U4 0x40
#define R_SHUNT_MOHM 20
#define CURRENT_LSB_MA 1
#define CAL_VALUE 256
extern I2C_HandleTypeDef hi2c2;
/* ----------------------------------------------------------
 *  Utility functions (static — only within the module)
 * ---------------------------------------------------------- */
static HAL_StatusTypeDef INA226_WriteReg(uint8_t addr,
                                          uint8_t reg,
                                          uint16_t value)
{
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = value & 0xFF;

    return HAL_I2C_Master_Transmit(&hi2c2,
                                   addr << 1,
                                   buf, 3, 100);
INA226_Data bat = INA226_Read_OnBus(&hi2c2, INA226_ADDR_U3);
INA226_Data sol = INA226_Read_OnBus(&hi2c3, INA226_ADDR_U4);
}

static uint16_t INA226_ReadReg(uint8_t addr, uint8_t reg)
{
    uint8_t buf[2] = {0, 0};

    HAL_I2C_Master_Transmit(&hi2c2, addr << 1, &reg, 1, 100);
    HAL_I2C_Master_Receive (&hi2c2, addr << 1, buf,  2, 100);
    INA226_Data bat = INA226_Read_OnBus(&hi2c2, INA226_ADDR_U3); // VBAT
    INA226_Data sol = INA226_Read_OnBus(&hi2c3, INA226_ADDR_U4);
    return (uint16_t)(buf[0] << 8 | buf[1]);
}
/* ----------------------------------------------------------
 *  Public API
 * ---------------------------------------------------------- */
/* Connection check — reading the register ID; it should return 0x2260 */
uint8_t INA226_Check(uint8_t addr)
{
    uint16_t id = INA226_ReadReg(addr, REG_ID);
    return (id == 0x2260) ? 1 : 0;
}

/* Initialization — calibration and configuration */
void INA226_Init(uint8_t addr)
{
    INA226_WriteReg(addr, REG_CALIBRATION, CAL_VALUE);
    INA226_WriteReg(addr, REG_CONFIG, 0x4527);
}

/* Reading current, voltage, and power */
INA226_Data INA226_Read(uint8_t addr)
{
    INA226_Data data;
    uint16_t raw_current = INA226_ReadReg(addr, REG_CURRENT);
    uint16_t raw_voltage = INA226_ReadReg(addr, REG_BUS_V);
    data.current_mA = (float)raw_current * CURRENT_LSB_MA;
    data.voltage_V = (float)raw_voltage * 0.00125f;
    data.power_mW = data.current_mA * data.voltage_V / 1000.0f;
    data.ok = 1;
    return data;
}
