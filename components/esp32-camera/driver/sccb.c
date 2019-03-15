/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver.
 *
 */
#if 1
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sccb.h"
#include "twi.h"
#include <stdio.h>
#include "sdkconfig.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char* TAG = "sccb";
#endif

#define SCCB_FREQ   (100000) // We don't need fast I2C. 100KHz is fine here.
#define TIMEOUT     (1000) /* Can't be sure when I2C routines return. Interrupts
while polling hardware may result in unknown delays. */


int SCCB_Init(int pin_sda, int pin_scl)
{
    // twi_init(pin_sda, pin_scl);
    return 0;
}

uint8_t SCCB_Probe()
{
    uint8_t reg = 0x00;
    uint8_t slv_addr = 0x00;

    // for (uint8_t i=0x01; i<127; i++) {
    //     if (twi_writeTo(i, &reg, 1, true) == 0) {
    //         slv_addr = i;
    //         break;
    //     }

    //     if (i!=126) {
    //         vTaskDelay(1 / portTICK_PERIOD_MS); // Necessary for OV7725 camera (not for OV2640).
    //     }
    // }
    return slv_addr;
}

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    uint8_t data=0;

    // int rc = twi_writeTo(0x78, &reg, 1, true);
    // if (rc != 0) {
    //     printf("%s  line(%d)  rc = %d\n", __func__, __LINE__, rc);
    //     data = 0xff;
    // } else {
    //     rc = twi_readFrom(0x79, &data, 1, true);
    //     if (rc != 0) {
    //         printf("%s  line(%d)\n", __func__, __LINE__);
    //         data=0xFF;
    //     }
    // }
    // if (rc != 0) {
    //     ESP_LOGE(TAG, "SCCB_Read [%02x] failed rc=%d\n", reg, rc);
    // }
    return data;
}

uint8_t SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    uint8_t ret=0;
    // uint8_t buf[] = {reg, data};

    // if(twi_writeTo(0x78, buf, 2, true) != 0) {
    //     ret=0xFF;
    // }
    // if (ret != 0) {
    //     printf("SCCB_Write [%02x]=%02x failed\n", reg, data);
    // }
    return ret;
}
#endif

#include <stdio.h>
#include "driver/i2c.h"


#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

int i2c_master_port = 0;
uint8_t ESP_SLAVE_ADDR = 0x3c;

struct reg_raw {
    uint8_t reg;
    uint8_t cfg;
};

void sccb_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 26;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = 27;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 200000;
    // conf.use_ref_tick = 0;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

int sccb_read_reg(uint8_t reg, uint8_t* data)
{
    esp_err_t ret = ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) return -1;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? 0 : -1;
}

int sccb_write_reg(uint8_t reg, uint8_t data)
{
    esp_err_t ret = ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? 0 : -1;
}

int sccb_slave_prob(void)
{
    uint8_t slave_addr = 0x0;
    sccb_bus_init();
    while(slave_addr < 0x7f) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( slave_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if( ret == ESP_OK) {
            ESP_SLAVE_ADDR = slave_addr;
            printf("slave_addr 0x%x\n", slave_addr);
            return 0;
        }
        slave_addr++;
    }
    return -1;
}
#include "freertos/task.h"
int sccb_reg_prob()
{
    uint8_t data = 0;
    sccb_write_reg(0xfe, 0x80);
    sccb_write_reg(0xfe, 0x80);
   
    vTaskDelay(5 / portTICK_PERIOD_MS);


    sccb_write_reg(0xfa, 0x11);

    vTaskDelay(5 / portTICK_PERIOD_MS);

    sccb_read_reg(0xf0, &data);
    printf("H %02x ", data);
    if( data != 0x21) {
        return -1;
    }
    sccb_read_reg(0xf1, &data);
    printf("L %02x\n", data);
    if( data != 0x45) {
        return -1;
    }
    return 1;
}
