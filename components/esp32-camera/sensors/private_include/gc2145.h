#ifndef __GC2145_H__
#define __GC2145_H__
#include "sensor.h"
#include "sccb.h"

#define  GC2145_SET_PAGE0    sccb_write_reg(0xfe,0x00)
#define  GC2145_SET_PAGE1    sccb_write_reg(0xfe,0x01)
#define  GC2145_SET_PAGE2    sccb_write_reg(0xfe,0x02)
#define  GC2145_SET_PAGE3    sccb_write_reg(0xfe,0x03)

int gc2145_init(sensor_t *sensor);

int initialize_gc2145_registers(void);
#endif