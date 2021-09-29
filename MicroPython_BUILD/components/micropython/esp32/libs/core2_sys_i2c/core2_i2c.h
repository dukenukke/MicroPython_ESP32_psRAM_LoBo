#ifndef _CORE2_I2C_H
#define _CORE2_I2C_H
#include <stdint.h>
#include <i2c.h>

#define CORE2_SYS_I2C_BUS_ID I2C_NUM_0
#define CORE2_SYS_I2C_SDA    ( 21 )
#define CORE2_SYS_I2C_SCL    ( 22 )
#define CORE2_SYS_I2C_SPEED  ( 400000 )

#define I2C_ACK_CHECK_EN  ( 1 )

typedef struct {
    bool is_init;

} CORE2_SYS_I2C;

esp_err_t core2_sys_i2c_init(void);
bool is_core2_sys_i2c_init(void);
int32_t core2_sys_i2c_write(uint8_t slave_addr, uint8_t memaddr, uint8_t *data, uint16_t len, bool stop);
int32_t core2_sys_i2c_read(uint8_t slave_addr, uint8_t memread, uint32_t memaddr, uint8_t *data, uint16_t len, bool stop);
#endif