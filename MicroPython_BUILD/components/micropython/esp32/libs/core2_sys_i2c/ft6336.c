#include "core2_i2c.h"
#include "ft6336.h"
#include "platform.h"
#include "driver/gpio.h"

FT6336 ft6336 = {
    false,                               //inited
    false,                               //changed
    (uint8_t)FT6336_DEFAULT_INTERVAL_MS, //default read interval
    0,
    {{-1, -1}, {-1, -1}},
    0,
    0
};

int32_t ft6336_init(void){
    uint8_t       data;
    gpio_config_t pin_conf;

    pin_conf.intr_type    = GPIO_PIN_INTR_DISABLE;
    pin_conf.mode         = GPIO_MODE_INPUT;
    pin_conf.pin_bit_mask = (1ULL << FT6336_INT_PIN);
    pin_conf.pull_down_en = 0;
    pin_conf.pull_up_en   = 0;
    gpio_config(&pin_conf);

    //platform_tick_get_ms();

        // By default, the FT6336 will pulse the INT line for every touch event.
        // But because it shares the Wire1 TwoWire/I2C with other devices, we
        // cannot create an interrupt service routine to handle these events.
        // So instead, we set the INT wire to polled mode, so it simply goes low
        // as long as there is at least one valid touch.
    data = 0x00;
    core2_sys_i2c_write((uint8_t)FT6336_DEVICE_ADDR, 0xA4, &data, 1, true);

    ft6336._interval = (uint8_t)FT6336_DEFAULT_INTERVAL_MS;
    ft6336._lastRead = (long long)platform_tick_get_ms();
    core2_sys_i2c_write((uint8_t)FT6336_DEVICE_ADDR, 0x88, &data, 1, true);
    
    return 0;
}
/** ******************************************************************
 * @NAME void ft6336_read(uint16_t x, uint16_t y, uint16_t z)
 * @
 ******************************************************************* */
void ft6336_read(uint16_t x, uint16_t y, uint16_t z){
    
    uint8_t pts = 0;
    uint8_t p0f = 0;
    TOUCH_POINT p[TP_NUM];
    TOUCH_POINT_UNION *tpu_read, 
                      *tpu_stored;
    ft6336.changed = false;
    int sys_tick = platform_tick_get_ms();
    if( sys_tick - ft6336._lastRead < ft6336._interval) return;
    ft6336._lastRead = sys_tick;

    if(0 == gpio_get_level(FT6336_INT_PIN)){
        uint8_t data[11];
        core2_sys_i2c_read((uint8_t)FT6336_DEVICE_ADDR, 1, 0x02, data, 11, true);
        pts = data[0];
        if (pts > 2) return;
        if (pts) {
            // Read the data. Never mind trying to read the "weight" and
            // "size" properties or using the built-in gestures: they
            // are always set to zero.
            p0f = (data[3] >> 4) ? 1 : 0;
            p[0].x = ((data[1] << 8) | data[2]) & 0x0fff;
            p[0].y = ((data[3] << 8) | data[4]) & 0x0fff;
            if (p[0].x > 320 || p[0].y > 280) return;
            if (pts == 2) {
                p[1].x = ((data[7] << 8) | data[8]) & 0x0fff;
                p[1].y = ((data[9] << 8) | data[10]) & 0x0fff;
                if (p[1].x > 320 || p[1].y > 280) return;
            }
        }
    }
    tpu_read = p;
    tpu_stored = ft6336.point;
    if ((tpu_read[0].touch_point_long != tpu_stored[0].touch_point_long) || (tpu_read[1].touch_point_long != tpu_stored[1].touch_point_long)) {
        ft6336.changed = true;
        tpu_stored[0].touch_point_long = tpu_read[0].touch_point_long;
        tpu_stored[1].touch_point_long = tpu_read[1].touch_point_long;
        ft6336.points = pts;
        ft6336._point0finger = p0f;
    }
}
