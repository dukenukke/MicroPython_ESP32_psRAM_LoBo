#ifndef _FT6336_H
#define _FT6336_H
#include <stdint.h>
#include <stdbool.h>

#define FT6336_DEVICE_ADDR           ( 0x38 )
#define FT6336_INT_PIN              ( 39 )
#define FT6336_DEFAULT_INTERVAL_MS  ( 13 )
#define TP_NUM                      2
typedef struct {
    uint16_t x, 
             y;
} TOUCH_POINT;

typedef union {
    uint32_t    touch_point_long;
    TOUCH_POINT touch_point_struct;
} TOUCH_POINT_UNION;

typedef struct {
    bool        inited;
    bool        changed;
    uint8_t    _interval;
    long long  _lastRead;       //
    TOUCH_POINT point[TP_NUM];  //coordinates of points was pressed
    uint8_t     points,         //number of points was pressed
                _point0finger;
} FT6336;

int32_t ft6336_init(void);
bool ft6336_read(void);
int16_t ft6336_get_touch(uint16_t *x, uint16_t *y, uint16_t *z);
#endif