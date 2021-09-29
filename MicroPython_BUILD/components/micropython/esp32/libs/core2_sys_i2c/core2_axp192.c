#include "core2_i2c.h"
#include "core2_axp192.h"
/*
#include "FreeRTOSConfig.h"
#include "portmacro.h"
//#include <esp_err.h>
*/
enum CHGCurrent {
        kCHG_100mA = 0,
        kCHG_190mA,
        kCHG_280mA,
        kCHG_360mA,
        kCHG_450mA,
        kCHG_550mA,
        kCHG_630mA,
        kCHG_700mA,
        kCHG_780mA,
        kCHG_880mA,
        kCHG_960mA,
        kCHG_1000mA,
        kCHG_1080mA,
        kCHG_1160mA,
        kCHG_1240mA,
        kCHG_1320mA,
    };

uint32_t core2_axp192_init(void){
    uint8_t byte;

    //AXP192 30H - VBUS limit off
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x30, &byte, 1, true);
    byte = (byte & 0x40) | 0x02;
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x30, &byte, 1, true);
    printf("axp: vbus limit off\n");

    //AXP192 GPIO1:OD OUTPUT
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x92, &byte, 1, true);
    byte = byte & 0xf8;
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x92, &byte, 1, true);
    
    //AXP192 GPIO2:OD OUTPUT
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x93, &byte, 1, true);
    byte = byte & 0xf8;
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x93, &byte, 1, true);
    printf("axp: gpio2 init\n");

    //AXP192 RTC CHG
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x35, &byte, 1, true);
    byte = (byte & 0x1c) | 0xa2;
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x35, &byte, 1, true);
    printf("axp: rtc battery charging enabled\n");
    
    core2_axp192_set_esp_voltage(3350);
    printf("axp: esp32 power voltage was set to 3.35v\n");

    core2_axp192_set_lcd_voltage(2800);
    printf("axp: lcd backlight voltage was set to 2.80v\n");

    core2_axp192_set_ldo_voltage(2, 3300);
    printf("axp: lcd logic and sdcard voltage preset to 3.3v\n");

    core2_axp192_set_ldo_voltage(3, 2000);
    printf("axp: vibrator voltage preset to 2v\n");

    core2_axp192_set_ldo_enable(2, true);
    core2_axp192_set_dcdc3(true); //backlight on
    core2_axp192_set_led(true);

    core2_axp192_set_chg_current(kCHG_100mA);

    //AXP192 GPIO4
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x95, &byte, 1, true);
    byte = (byte & 0x72) | 0X84;
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x95, &byte, 1, true);

    byte = 0x4c;
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x36, &byte, 1, true);
    
    byte = 0xff;
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x82, &byte, 1, true);

    core2_axp192_set_lcd_reset(0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    core2_axp192_set_lcd_reset(1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    //  bus power mode_output
    core2_axp192_set_bus_power_mode(0);
    return 0;
}

/** *********************************************************
 * 
 ********************************************************* */
uint8_t core2_axp192_get_in_state(void) {
    uint8_t buf;
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x00, &buf, 1, true);
    return buf;
}

/** *********************************************************
 * 
 ********************************************************* */
void core2_axp192_set_chg_current(uint8_t state){
    uint8_t data;
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x33, &data, 1, true);
    data &= 0xf0;
    data = data | ( state & 0x0f );
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x33, &data, 1, true);
}

/** ***************************************************************
 * 
 **************************************************************** */
void core2_axp192_set_dc_voltage(uint8_t number, uint16_t voltage){
    uint8_t addr,
            data;
    if (number > 2)
        return;
    voltage = (voltage < 700) ? 0 : (voltage - 700) / 25;
    switch (number){
    case 0:
        addr = 0x26;
        break;
    case 1:
        addr = 0x25;
        break;
    case 2:
        addr = 0x27;
        break;
    default:
        return;
    }
    data = (uint8_t)(voltage & 0x7f);
    core2_sys_i2c_write(AXP192_I2C_ADDR, addr, &data, 1, true);
}

/** *****************************************************************
 *  @NAME core2_axp192_set_esp_voltage(uint16_t voltage)
 *  @DESCR Set core voltage
 *  @ARG   voltage - voltage in mV
 ***************************************************************** */
void core2_axp192_set_esp_voltage(uint16_t voltage){
    if (voltage >= 3000 && voltage <= 3400) {
        core2_axp192_set_dc_voltage(0, voltage);
    }
}


/** *****************************************************************
 *  @NAME void core2_axp192_set_ldo_voltage(uint8_t number, uint16_t voltage)
 *  @DESCR Set backlight voltage
 *  @ARG   number
 *  @ARG   voltage
 ***************************************************************** */
void core2_axp192_set_lcd_voltage(uint16_t voltage){
    if (voltage >= 2500 && voltage <= 3300) {
        core2_axp192_set_dc_voltage(2, voltage);
    }
}
/** *****************************************************************
 *  @NAME void core2_axp192_set_ldo_voltage(uint8_t number, uint16_t voltage)
 *  @DESCR Set lcd logic and sdcard voltage 
 *  @ARG   number
 *  @ARG   voltage
 ***************************************************************** */
void core2_axp192_set_ldo_voltage(uint8_t number, uint16_t voltage){
    uint8_t byte;
    voltage = (voltage > 3300) ? 15 : (voltage / 100) - 18;
    switch (number) {
    //uint8_t reg, data;
    case 2:
        core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x28, &byte, 1, true);
        byte = (byte & 0x0F) | (voltage << 4);
        core2_sys_i2c_write(AXP192_I2C_ADDR, 0x28, &byte, 1, true);
        break;
    case 3:
        core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x28, &byte, 1, true);
        byte = (byte & 0xF0) | voltage;
        core2_sys_i2c_write(AXP192_I2C_ADDR, 0x28, &byte, 1, true);
        break;
    }
}


/** *****************************************************************
 *  @NAME void core2_axp192_set_ldo_enable(uint8_t number, uint16_t state)
 *  @DESCR Set load channel on/off
 *  @ARG   number - channel #
 *  @ARG   state - false-off, true-on
 ***************************************************************** */
void core2_axp192_set_ldo_enable(uint8_t number, bool state){   
    uint8_t mark = 0x01,
            byte;
    if ((number < 2) || (number > 3))
        return;
    mark <<= number;
    if (state){
        core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x12, &byte, 1, true);
        byte |= mark;
        core2_sys_i2c_write(AXP192_I2C_ADDR, 0x12, &byte, 1, true);
    }
    else {
        core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x12, &byte, 1, true);
        byte &= ~mark;
        core2_sys_i2c_write(AXP192_I2C_ADDR, 0x12, &byte, 1, true);
    }
}

/** ************************************************************
 * 
 ************************************************************ */
void core2_axp192_set_dcdc3(bool state) {
    uint8_t buf;
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x12, &buf, 1, true);
    if (state == true)
        buf = (1 << 1) | buf;
    else
        buf = ~(1 << 1) & buf;
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x12, &buf, 1, true);
}

/** ************************************************************
 * 
 ************************************************************ */
void core2_axp192_set_led(bool state){
    uint8_t data;
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x94, &data, 1, true);
    if(state) {
      data = data & 0xfd;
    } else {
      data |= 0x02;
    }
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x94, &data, 1, true);
}

/** ************************************************************
 * 
 ************************************************************ */
void core2_axp192_set_lcd_reset(bool state){
    uint8_t gpio_bit = 0x02;
    uint8_t data;

    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x96, &data, 1, true);
    if (state) {
        data |= gpio_bit;
    } else {
        data &= ~gpio_bit;
    }
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x96, &data, 1, true);
}

/** ************************************************************
 * 
 ************************************************************ */
void core2_axp192_set_bus_power_mode(uint8_t state) {
    uint8_t data;
    if (state == 0) {
        core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x91, &data, 1, true);
        data = (data & 0X0F) | 0XF0;
        core2_sys_i2c_write(AXP192_I2C_ADDR, 0x91, &data, 1, true);

        
        core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x90, &data, 1, true);
        data = (data & 0XF8) | 0X02;
        core2_sys_i2c_write(AXP192_I2C_ADDR, 0x90, &data, 1, true); //set GPIO0 to LDO OUTPUT , pullup N_VBUSEN to disable supply from BUS_5V

        
        core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x91, &data, 1, true);
        core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x12, &data, 1, true); //read reg 0x12
    
        data |= 0x40;
        core2_sys_i2c_write(AXP192_I2C_ADDR, 0x12, &data, 1, true); //set EXTEN to enable 5v boost
    } else {
        core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x12, &data, 1, true); //read reg 0x12
        data &= 0XBF;
        core2_sys_i2c_write(AXP192_I2C_ADDR, 0x12, &data, 1, true); //set EXTEN to disable 5v boost
    
        //delay(2000);
        core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x90, &data, 1, true);
        data = (data & 0xF8) | 0x01;
        core2_sys_i2c_write(AXP192_I2C_ADDR, 0x90, &data, 1, true); //set GPIO0 to float , using internal pulldown resistor to enable supply from BUS_5VS
    }
}

/** *********************************************************
 * 
 ********************************************************* */
void core2_axp192_set_lcd_backlight(uint8_t brightness){
    uint8_t data;
    if (brightness > 12) {
        brightness = 12;
    }
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x28, &data, 1, true);
    data = (data & 0x0f) | (brightness << 4);
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x28, &data, 1, true);
}

/** *********************************************************
 * 
 ********************************************************* */
bool core2_axp192_get_bat_state(void){
    uint8_t  data;
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x01, &data, 1, true);
    if (data & 0x20)
        return true;
    else
        return false;
}

/** *********************************************************
 * 
 ********************************************************* */
void core2_axp192_set_ldo2(bool state){
    uint8_t buf;
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x12, &buf, 1, true);
    
    if (state == true)
        buf = (1 << 2) | buf;
    else
        buf = ~(1 << 2) & buf;
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x12, &buf, 1, true);
}

/** *********************************************************
 * 
 ********************************************************* */
bool core2_axp192_is_ac_in(void) {
    uint8_t buf;
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x00, &buf, 1, true);
    return ( buf & 0x80 ) ? true : false;
}

/** *********************************************************
 * 
 ********************************************************* */
bool core2_axp192_is_charging(void) {
    uint8_t buf;
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x00, &buf, 1, true);
    return ( buf & 0x04 ) ? true : false;
}

/** *********************************************************
 * 
 ********************************************************* */
bool core2_axp192_is_vbus(void) {
    uint8_t buf;
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x00, &buf, 1, true);
    return ( buf & 0x20 ) ? true : false;
}

/** *********************************************************
 * 
 ********************************************************* */
//set led state(GPIO high active,set 1 to enable amplifier)
void core2_axp192_set_spk_enable(uint8_t state) {
    uint8_t reg_addr=0x94;
    uint8_t gpio_bit=0x04;
    uint8_t data;
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, reg_addr, &data, 1, true);

    if(state) {
      data|=gpio_bit;
    } else {
      data&=~gpio_bit;
    }
    core2_sys_i2c_write(AXP192_I2C_ADDR, reg_addr, &data, 1, true);
}

/** *********************************************************
 * 
 ********************************************************* */
void core2_axp192_set_hg_current (uint8_t state) {
    uint8_t data;
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x33, &data, 1, true);
    data &= 0xf0;
    data = data | ( state & 0x0f );
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x33, &data, 1, true);
}


/** *********************************************************
 * @NAME void core2_axp192_set_power_off(void)
 * @DESCR Cut all power, except for LDO1 (RTC)
 * @RET none
 ********************************************************* */
void core2_axp192_set_power_off(void){
    uint8_t data;
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x32, &data, 1, true);
    data |= 0b10000000;
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x32, &data, 1, true);
}

/** *********************************************************
 * 
 ********************************************************* */
void core2_axp192_set_adc_state(bool state) {
    // Enable / Disable all ADCs
    uint8_t data = state ? 0xff : 0x00;
    core2_sys_i2c_write(AXP192_I2C_ADDR, 0x82, &data, 1, true);
}
/** *********************************************************
 * 
 ********************************************************* */
void core2_axp192_prepare_to_sleep(void) {
    // Disable ADCs
    core2_axp192_set_adc_state(false);
    // Turn LED off
    core2_axp192_set_led(false);
    // Turn LCD backlight off
    core2_axp192_set_dcdc3(false);
}

/** *********************************************************
 * 
 ********************************************************* */
void core2_axp192_restore_from_light_sleep(void){
    // Turn LCD backlight on
    core2_axp192_set_dcdc3(true);

    // Turn LED on
    core2_axp192_set_led(true);

    // Enable ADCs
    core2_axp192_set_adc_state(true);
}

/** *********************************************************
 * 
 ********************************************************* */
/*
void core2_axp192_get_warning_level(void) {
    Wire1.beginTransmission(0x34);
    Wire1.write(0x47);
    Wire1.endTransmission();
    Wire1.requestFrom(0x34, 1);
    uint8_t buf = Wire1.read();
    return (buf & 0x01);
}
*/

uint8_t core2_axp192_get_warning_level(void) {
    uint8_t data;
    core2_sys_i2c_read(AXP192_I2C_ADDR, 1, 0x47, &data, 1, true);
    return data & 0x01;
}

/** *********************************************************
 * @DESCR sleep
 ********************************************************* */
void core2_axp192_deep_sleep(uint64_t time_in_us) {
    core2_axp192_prepare_to_sleep();

    if (time_in_us > 0) {
        esp_sleep_enable_timer_wakeup(time_in_us);
    } else {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    (time_in_us == 0) ? esp_deep_sleep_start() : esp_deep_sleep(time_in_us);

    // Never reached - after deep sleep ESP32 restarts
}

/** *********************************************************
 * @DESCR sleep
 ********************************************************* */
void core2_axp192_light_sleep(uint64_t time_in_us) {
    core2_axp192_prepare_to_sleep();

    if (time_in_us > 0)
    {
        esp_sleep_enable_timer_wakeup(time_in_us);
    }
    else
    {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    esp_light_sleep_start();
    core2_axp192_restore_from_light_sleep();
}
/*
float GetBatVoltage()
{
    float ADCLSB = 1.1 / 1000.0;
    uint16_t ReData = Read12Bit(0x78);
    return ReData * ADCLSB;
}

float GetBatCurrent()
{
    float ADCLSB = 0.5;
    uint16_t CurrentIn = Read13Bit(0x7A);
    uint16_t CurrentOut = Read13Bit(0x7C);
    return (CurrentIn - CurrentOut) * ADCLSB;
}
*/