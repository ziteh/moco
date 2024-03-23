
#include "main.h"
#include "cmsis_compiler.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_gpio.h"
#include "stspin32g4_gate_driver.h"

#define GD_I2C_INST (I2C3)
#define GD_ADDRESS (0x8EU) // 1000111Xb, x:R/W

#define GD_REG_POWMNG (0x01U)
#define GD_REG_LOGIC (0x02U)
#define GD_REG_READY (0x07U)
#define GD_REG_NFAULT (0x08U)
#define GD_REG_CLEAR (0x09U)
#define GD_REG_STBY (0x0AU)
#define GD_REG_LOCK (0x0BU)
#define GD_REG_RESET (0x0CU)
#define GD_REG_STATUS (0x80U)

/**
 * @note Protected
 */
typedef __PACKED_UNION
{
    __PACKED_STRUCT
    {
        uint8_t vcc_val : 2;
        const uint8_t _reserved23 : 2; // Unused, default=0b00
        uint8_t stby_reg_en : 1;
        uint8_t vcc_dis : 1;
        uint8_t reg3v3_dis : 1;
        const uint8_t _reserved7 : 1; // Unused, default=0b0
    }
    bit;
    uint8_t reg;
}
reg_powmng_t;

/**
 * @note Protected
 */
typedef __PACKED_UNION
{
    __PACKED_STRUCT
    {
        uint8_t ilock : 1;
        uint8_t dtmin : 1;
        uint8_t vds_p_deg : 2;
        const uint8_t _reserved47 : 4; // Unused, default=0x0111
    }
    bit;
    uint8_t reg;
}
reg_logic_t;

typedef __PACKED_UNION
{
    __PACKED_STRUCT
    {
        uint8_t vcc_uvlo_rdy : 1;
        uint8_t thsd_rdy : 1;
        const uint8_t _reserved2 : 1; // Unused, default=0b0
        uint8_t stby_rdy : 1;
        const uint8_t _reserved47 : 4; // Unused, default=0b0000
    }
    bit;
    uint8_t reg;
}
reg_ready_t;

/**
 * @note Protected
 */
typedef __PACKED_UNION
{
    __PACKED_STRUCT
    {
        uint8_t vcc_uvlo_flt : 1;
        uint8_t thsd_flt : 1;
        uint8_t vds_p_flt : 1;
        const uint8_t _reserved37 : 5; // Unused, default=0b01111
    }
    bit;
    uint8_t reg;
}
reg_nfault_t;

typedef __PACKED_UNION
{
    __PACKED_STRUCT
    {
        uint8_t fault_clear : 8;
    }
    bit;
    uint8_t reg;
}
reg_clear_t;

/**
 * @note Protected
 */
typedef __PACKED_UNION
{
    __PACKED_STRUCT
    {
        uint8_t stby : 1;
        const uint8_t _reserved27 : 7; // Unused, default=0b0000000
    }
    bit;
    uint8_t reg;
}
reg_stby_t;

typedef __PACKED_UNION
{
    __PACKED_STRUCT
    {
        uint8_t lock : 4;
        uint8_t nlock : 4;
    }
    bit;
    uint8_t reg;
}
reg_lock_t;

/**
 * @note Protected
 */
typedef __PACKED_UNION
{
    __PACKED_STRUCT
    {
        uint8_t reset : 8;
    }
    bit;
    uint8_t reg;
}
reg_reset_t;

/**
 * @note Ready only
 */
typedef __PACKED_UNION
{
    __PACKED_STRUCT
    {
        uint8_t vcc_uvlo : 1;
        uint8_t thsd : 1;
        uint8_t vds_p : 1;
        uint8_t reset : 1;
        const uint8_t _reserved46 : 3; // Unused
        uint8_t lock : 1;
    }
    bit;
    uint8_t reg;
}
reg_status_t;

int8_t i2c_send_blocking(const uint8_t *data, uint8_t len);
uint8_t i2c_read_blocking(uint8_t addr);

reg_powmng_t reg_powmng = {
    .bit.vcc_val = 0,
    .bit._reserved23 = 0,
    .bit.stby_reg_en = 0,
    .bit.vcc_dis = 0,
    .bit.reg3v3_dis = 0,
    .bit._reserved7 = 0,
};

reg_logic_t reg_logic = {
    .bit.ilock = 1,
    .bit.dtmin = 1,
    .bit.vds_p_deg = 0,
    .bit._reserved47 = 0x7,
};

reg_ready_t reg_ready = {
    .bit.vcc_uvlo_rdy = 1,
    .bit.thsd_rdy = 0,
    .bit._reserved2 = 0,
    .bit.stby_rdy = 1,
    .bit._reserved47 = 0,
};

reg_nfault_t reg_nfault = {
    .reg = 0x7F,
};

const reg_stby_t reg_stby = {
    .bit.stby = 1,
    .bit._reserved27 = 0x00,
};

const reg_lock_t reg_unlock = {
    .bit.nlock = 0xF,
    .bit.lock = 0x0,
};

const reg_lock_t reg_lock = {
    .bit.nlock = 0x0,
    .bit.lock = 0x0,
};

const reg_clear_t reg_clear = {
    .bit.fault_clear = 0xFF,
};

const reg_reset_t reg_reset = {
    .bit.reset = 0xFF, // Setting high all the bits resets the regs to the default value.
};

/**
 * @brief Clear all faults
 */
void gd_clear_fault(void)
{
    uint8_t data[] = {GD_REG_CLEAR, reg_clear.reg};
    i2c_send_blocking(data, 2);
}

/**
 * @brief Unlock to write protected registers, and all gate driver output are forced low.
 */
void gd_unlock(void)
{
    uint8_t data[] = {GD_REG_LOCK, reg_unlock.reg}; // LOCK!=NLOCK: protected registers unlock.
    i2c_send_blocking(data, 2);
}

/**
 * @brief Lock protected registers.
 */
void gd_lock(void)
{
    uint8_t data[] = {GD_REG_LOCK, reg_lock.reg}; // LOCK!=NLOCK: protected registers unlock.
    i2c_send_blocking(data, 2);
}

/**
 * @brief Enter low consumption mode. Need to unlock.
 */
void gd_enter_standby(void)
{
    uint8_t data[] = {GD_REG_STBY, reg_stby.reg};
    i2c_send_blocking(data, 2);
}

/**
 * @brief Reset registers to the default value. Need to unlock.
 */
void gd_reset(void)
{
    uint8_t data[] = {GD_REG_RESET, reg_reset.reg};
    i2c_send_blocking(data, 2);
}

/**
 * @brief Wake-up
 */
void gd_wakeup(void)
{
    LL_GPIO_SetOutputPin(GD_WAKE_GPIO_Port, GD_WAKE_Pin);
    HAL_Delay(50);
    LL_GPIO_ResetOutputPin(GD_WAKE_GPIO_Port, GD_WAKE_Pin);
}

uint8_t gd_init(void)
{
    gd_wakeup();
    HAL_Delay(2);

    gd_clear_fault();
    HAL_Delay(2);

    gd_unlock();
    HAL_Delay(2);

    gd_reset();
    HAL_Delay(2);

    // uint8_t data[2] = {0};

    // Setup power manager config
    // reg_powmng.bit.vcc_val = 0;
    // reg_powmng.bit.stby_reg_en = 0;
    // reg_powmng.bit.vcc_dis = 1;
    // reg_powmng.bit.reg3v3_dis = 1;
    // data[0] = GD_REG_POWMNG;
    // data[1] = reg_powmng.reg;
    // i2c_send_blocking(data, 2);

    // Setup driving logic config
    // reg_logic.bit.ilock = 1;
    // reg_logic.bit.dtmin = 1;
    // reg_logic.bit.vds_p_deg = 0;
    // data[0] = GD_REG_LOGIC;
    // data[1] = reg_logic.reg;
    // i2c_send_blocking(data, 2);

    gd_lock();
    HAL_Delay(2);

    return (uint8_t)i2c_read_blocking(GD_REG_STATUS);
}

uint8_t get_status(void)
{
    return i2c_read_blocking(GD_REG_STATUS);
}

int8_t i2c_send_blocking(const uint8_t *data, uint8_t len)
{
    LL_I2C_HandleTransfer(GD_I2C_INST, GD_ADDRESS, LL_I2C_ADDRSLAVE_7BIT, len, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    for (uint8_t i = 0; i < len; i++)
    {
        while (!LL_I2C_IsActiveFlag_TXIS(GD_I2C_INST))
        {
        }
        LL_I2C_TransmitData8(GD_I2C_INST, data[i]);
    }
    return 0;
}

uint8_t i2c_read_blocking(uint8_t addr)
{
    uint8_t data[] = {addr};
    i2c_send_blocking(data, 1);
    LL_I2C_HandleTransfer(GD_I2C_INST, GD_ADDRESS, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
    while (!LL_I2C_IsActiveFlag_RXNE(GD_I2C_INST))
    {
    }
    return LL_I2C_ReceiveData8(GD_I2C_INST);
}
