#include "core2_i2c.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"

/*
#ifndef portTICK_PERIOD_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
*/

CORE2_SYS_I2C core2_sys_i2c = {
    false
};

/** **********************************************************
 * 
 ********************************************************** */
esp_err_t core2_sys_i2c_init(void){
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = CORE2_SYS_I2C_SDA;
    conf.scl_io_num = CORE2_SYS_I2C_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = CORE2_SYS_I2C_SPEED;
    i2c_param_config(CORE2_SYS_I2C_BUS_ID, &conf);
    return i2c_driver_install(CORE2_SYS_I2C_BUS_ID, I2C_MODE_MASTER, 0, 0, false, ESP_INTR_FLAG_IRAM);
}
/** **********************************************************
 * 
 ********************************************************** */
int32_t core2_sys_i2c_write(uint8_t slave_addr, uint8_t memaddr, uint8_t *data, uint16_t len, bool stop){
    esp_err_t ret = ESP_FAIL;
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    if (ret == ESP_OK) {
        ret = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
        if (ret == ESP_OK) {
           // send memory address
            ret=i2c_master_write_byte(cmd, memaddr, I2C_ACK_CHECK_EN);
            if (ret == ESP_OK){
                // send data
                if(( data ) && ( len ) ){
                    ret=i2c_master_write(cmd, data, len, I2C_ACK_CHECK_EN);
                    if (ret == ESP_OK) {
                        if(stop){
                            ret = i2c_master_stop(cmd);
                        }
                        if (ret == ESP_OK) {
                            ret = i2c_master_cmd_begin(CORE2_SYS_I2C_BUS_ID, cmd, (5000 + (1000 * len)) / portTICK_RATE_MS);
                            if (ret == ESP_OK){
                                i2c_cmd_link_delete(cmd);
                            }
                        }
                    }
                }
            }
        }
    }
    return ret;
}
/** **********************************************************
 * 
 ********************************************************** */
int32_t core2_sys_i2c_read(uint8_t slave_addr, uint8_t memread, uint32_t memaddr, uint8_t *data, uint16_t len, bool stop)
{
	esp_err_t ret = ESP_FAIL;

	memset(data, 0xFF, len);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (i2c_master_start(cmd) != ESP_OK) {ret=1; goto error;};

    if (memread) {
    	// send slave address
        if (i2c_master_write_byte(cmd, ( slave_addr << 1 ) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN) != ESP_OK) {ret=2; goto error;};

        if (memread > 0) {
			// send memory address, MSByte first
			while (memread > 0) {
				// send memory address, MSByte first
				memread--;
				if (i2c_master_write_byte(cmd, (memaddr >> (memread*8)), I2C_ACK_CHECK_EN) != ESP_OK) {ret=3; goto error;};
			}

			if (stop) {
				// Finish the write transaction
			    if (i2c_master_stop(cmd) != ESP_OK) {ret=5; goto error;};
			    if (i2c_master_cmd_begin((i2c_port_t)CORE2_SYS_I2C_BUS_ID, cmd, 100 / portTICK_RATE_MS) != ESP_OK) {ret=6; goto error;};
			    i2c_cmd_link_delete(cmd);
			    // Start the read transaction
			    cmd = i2c_cmd_link_create();
			    if (i2c_master_start(cmd) != ESP_OK) {ret=7; goto error;};
			}
			else {
				// repeated start, generate START signal, slave address will be send next
				if (i2c_master_start(cmd) != ESP_OK) {ret=4; goto error;};
			}
        }
    }

	// READ, send slave address
    if (i2c_master_write_byte(cmd, ( slave_addr << 1 ) | I2C_MASTER_READ, I2C_ACK_CHECK_EN) != ESP_OK) {ret=8; goto error;};

    if (len > 1) {
        if (i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK) != ESP_OK) {ret=9; goto error;};
    }

    if (i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK) != ESP_OK) {ret=10; goto error;};

    if (i2c_master_stop(cmd) != ESP_OK) {ret=11; goto error;};

    ret = i2c_master_cmd_begin((i2c_port_t)CORE2_SYS_I2C_BUS_ID, cmd, (5000 + (1000 * len)) / portTICK_RATE_MS);

error:
    i2c_cmd_link_delete(cmd);

    return ret;
}



