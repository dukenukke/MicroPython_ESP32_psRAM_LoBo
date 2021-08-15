#ifndef _CORE2_AXP192_H
#define _CORE2_AXP192_H

#define AXP192_I2C_ADDR   ( 0x34 )
uint32_t core2_axp192_init(void);
uint8_t core2_axp192_get_in_state(void);
void core2_axp192_set_chg_current(uint8_t state);
void core2_axp192_set_dc_voltage(uint8_t number, uint16_t voltage);
void core2_axp192_set_esp_voltage(uint16_t voltage);
void core2_axp192_set_lcd_voltage(uint16_t voltage);
void core2_axp192_set_ldo_voltage(uint8_t number, uint16_t voltage);
void core2_axp192_set_ldo_enable(uint8_t number, bool state);
void core2_axp192_set_dcdc3(bool state);
void core2_axp192_set_led(bool state);
void core2_axp192_set_lcd_reset(bool state);
void core2_axp192_set_bus_power_mode(uint8_t state);
void core2_axp192_set_lcd_backlight(uint8_t brightness);
bool core2_axp192_get_bat_state(void);
void core2_axp192_set_ldo2(bool state);
bool core2_axp192_is_ac_in(void);
bool core2_axp192_is_charging(void);
bool core2_axp192_is_vbus(void);
void core2_axp192_set_spk_enable(uint8_t state);
void core2_axp192_set_hg_current (uint8_t state);
void core2_axp192_set_power_off(void);
void core2_axp192_set_adc_state(bool state);
void core2_axp192_prepare_to_sleep(void);
void core2_axp192_restore_from_light_sleep(void);
uint8_t core2_axp192_get_warning_level(void);
void core2_axp192_deep_sleep(uint64_t time_in_us);
void core2_axp192_light_sleep(uint64_t time_in_us);

#endif