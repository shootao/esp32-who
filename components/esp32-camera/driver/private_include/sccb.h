/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver.
 *
 */
#ifndef __SCCB_H__
#define __SCCB_H__
#include <stdint.h>
void sccb_bus_init(void);

int sccb_read_reg(uint8_t reg, uint8_t* data);

int sccb_write_reg(uint8_t reg, uint8_t data);

int sccb_slave_prob(void);

int sccb_reg_prob();

uint8_t SCCB_Probe();

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg);

uint8_t SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data);

// #define  GC2145_SET_PAGE0    sccb_write_reg(0xfe,0x00)
// #define  GC2145_SET_PAGE1    sccb_write_reg(0xfe,0x01)
// #define  GC2145_SET_PAGE2    sccb_write_reg(0xfe,0x02)
// #define  GC2145_SET_PAGE3    sccb_write_reg(0xfe,0x03)

#endif // __SCCB_H__
