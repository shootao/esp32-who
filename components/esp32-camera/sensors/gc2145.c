/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV2640 driver.
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "sccb.h"
#include "gc2145.h"
#include "gc2145_regs.h"

#include "ov2640.h"
#include "ov2640_regs.h"
#include "ov2640_settings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char* TAG = "gc2145";
#endif

static volatile ov2640_bank_t reg_bank = BANK_MAX;


int read_all_regs_for_gc2145(void)
{
    uint32_t i, array_length;
    uint8_t data = 0;
    array_length = sizeof (gc2145_reg_array) / sizeof (gc2145_reg_array[0]);
    for (i = 0; i < array_length; i++)
    {
        if(gc2145_reg_array[i].register_address == 0xfe) {
            if (sccb_write_reg(gc2145_reg_array[i].register_address, 
            gc2145_reg_array[i].register_value) == -1){
            ESP_LOGW(TAG, "write gc2145 regs is err  line(%d)[%02x:%02x]", __LINE__, gc2145_reg_array[i].register_address
                , gc2145_reg_array[i].register_value);
            return -1;
            }
        } else {
            data = 0;
            if(sccb_read_reg(gc2145_reg_array[i].register_address, &data) == -1) {
                ESP_LOGW(TAG, "read gc2145 regs is err line(%d)[%02x:%02x]", __LINE__, gc2145_reg_array[i].register_address
                , gc2145_reg_array[i].register_value);
            }
            ESP_LOGI(TAG,"%02x is %02x\n", gc2145_reg_array[i].register_address, data);
        }
    }
    return 0;
}

int initialize_gc2145_registers(void)
{
    uint32_t i, array_length;
    uint8_t data = 0;
    array_length = sizeof (gc2145_reg_array) / sizeof (gc2145_reg_array[0]);
    for (i = 0; i < array_length; i++)
    {

        if (sccb_write_reg(gc2145_reg_array[i].register_address, 
            gc2145_reg_array[i].register_value) == -1){
            ESP_LOGW(TAG, "init gc2145 regs is err [%02x:%02x] [%d]", gc2145_reg_array[i].register_address,
            gc2145_reg_array[i].register_value, i);
            // return -1;
        }
    }
    printf("============INIT===========\n");
    printf("===========================\n");
    return 0;
}


static int set_bank(sensor_t *sensor, ov2640_bank_t bank)
{
    return 0;
}

static int write_regs(sensor_t *sensor, const uint8_t (*regs)[2])
{
    return 0;
}

static int write_reg(sensor_t *sensor, ov2640_bank_t bank, uint8_t reg, uint8_t value)
{
    return 0;
}

static int set_reg_bits(sensor_t *sensor, uint8_t bank, uint8_t reg, uint8_t offset, uint8_t mask, uint8_t value)
{
    return 0;
}

static int read_reg(sensor_t *sensor, ov2640_bank_t bank, uint8_t reg)
{
    return 0;
}

static uint8_t get_reg_bits(sensor_t *sensor, uint8_t bank, uint8_t reg, uint8_t offset, uint8_t mask)
{
    return 0;
}

static int write_reg_bits(sensor_t *sensor, uint8_t bank, uint8_t reg, uint8_t mask, int enable)
{
    return 0;
}

#define WRITE_REGS_OR_RETURN(regs) ret = write_regs(sensor, regs); if(ret){return ret;}
#define WRITE_REG_OR_RETURN(bank, reg, val) ret = write_reg(sensor, bank, reg, val); if(ret){return ret;}
#define SET_REG_BITS_OR_RETURN(bank, reg, offset, mask, val) ret = set_reg_bits(sensor, bank, reg, offset, mask, val); if(ret){return ret;}

static int reset(sensor_t *sensor)
{
    return 0;
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    return 0;
}

//Functions are not needed currently
#if 0
//Set the sensor output window
int set_output_window(sensor_t *sensor, uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    int ret = 0;
    uint16_t endx, endy;
    uint8_t com1, reg32;

    endy = y + height / 2;
    com1 = read_reg(sensor, BANK_SENSOR, COM1);
    WRITE_REG_OR_RETURN(BANK_SENSOR, COM1, (com1 & 0XF0) | (((endy & 0X03) << 2) | (y & 0X03)));
    WRITE_REG_OR_RETURN(BANK_SENSOR, VSTART, y >> 2);
    WRITE_REG_OR_RETURN(BANK_SENSOR, VSTOP, endy >> 2);

    endx = x + width / 2;
    reg32 = read_reg(sensor, BANK_SENSOR, REG32);
    WRITE_REG_OR_RETURN(BANK_SENSOR, REG32, (reg32 & 0XC0) | (((endx & 0X07) << 3) | (x & 0X07)));
    WRITE_REG_OR_RETURN(BANK_SENSOR, HSTART, x >> 3);
    WRITE_REG_OR_RETURN(BANK_SENSOR, HSTOP, endx >> 3);

    return ret;
}

// Set the image output size (final output resolution)
int set_output_size(sensor_t *sensor, uint16_t width, uint16_t height)
{
    int ret = 0;
    uint16_t h, w;

    if(width % 4) {
        return -1;
    }
    if(height % 4 ) {
        return -2;
    }

    w = width / 4;
    h = height / 4;
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, RESET_DVP);
    WRITE_REG_OR_RETURN(BANK_DSP, ZMOW, w & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, ZMOH, h & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, ZMHH, ((w >> 8) & 0X03) | ((h >> 6) & 0X04));
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, 0X00);

    return ret;
}

//Set the image window size >= output size
int set_window_size(sensor_t *sensor, uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    int ret = 0;
    uint16_t w, h;

    if(width % 4) {
        return -1;
    }
    if(height % 4) {
        return -2;
    }

    w = width / 4;
    h = height / 4;
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, RESET_DVP);
    WRITE_REG_OR_RETURN(BANK_DSP, HSIZE, w & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, VSIZE, h & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, XOFFL, x & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, YOFFL, y & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, VHYX, ((h >> 1) & 0X80) | ((y >> 4) & 0X70) | ((w >> 5) & 0X08) | ((x >> 8) & 0X07));
    WRITE_REG_OR_RETURN(BANK_DSP, TEST, (w >> 2) & 0X80);
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, 0X00);

    return ret;
}

//Set the sensor resolution (UXGA, SVGA, CIF)
int set_image_size(sensor_t *sensor, uint16_t width, uint16_t height)
{
    int ret = 0;
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, RESET_DVP);
    WRITE_REG_OR_RETURN(BANK_DSP, HSIZE8, (width >> 3) & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, VSIZE8, (height >> 3) & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, SIZEL, ((width & 0X07) << 3) | ((width >> 4) & 0X80) | (height & 0X07));
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, 0X00);

    return ret;
}
#endif

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    return 0;
}

static int set_contrast(sensor_t *sensor, int level)
{
    return 0;
}

static int set_brightness(sensor_t *sensor, int level)
{
    return 0;
}

static int set_saturation(sensor_t *sensor, int level)
{
    return 0;
}

static int set_special_effect(sensor_t *sensor, int effect)
{
    return 0;
}

static int set_wb_mode(sensor_t *sensor, int mode)
{
    return 0;
}

static int set_ae_level(sensor_t *sensor, int level)
{
    return 0;
}

static int set_quality(sensor_t *sensor, int quality)
{
    return 0;
}

static int set_agc_gain(sensor_t *sensor, int gain)
{
    return 0;
}

static int set_gainceiling_sensor(sensor_t *sensor, gainceiling_t gainceiling)
{
    return 0;
}

static int set_aec_value(sensor_t *sensor, int value)
{
    return 0;
}

static int set_aec2(sensor_t *sensor, int enable)
{
    return 0;
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    return 0;
}

static int set_agc_sensor(sensor_t *sensor, int enable)
{
    return 0;
}

static int set_aec_sensor(sensor_t *sensor, int enable)
{
    return 0;
}

static int set_hmirror_sensor(sensor_t *sensor, int enable)
{
    return 0;
}

static int set_vflip_sensor(sensor_t *sensor, int enable)
{
    return 0;
}

static int set_raw_gma_dsp(sensor_t *sensor, int enable)
{
    return 0;
}

static int set_awb_dsp(sensor_t *sensor, int enable)
{
    return 0;
}

static int set_awb_gain_dsp(sensor_t *sensor, int enable)
{
    return 0;
}

static int set_lenc_dsp(sensor_t *sensor, int enable)
{
    return 0;
}

static int set_dcw_dsp(sensor_t *sensor, int enable)
{
    return 0;
}

static int set_bpc_dsp(sensor_t *sensor, int enable)
{
    return 0;
}

static int set_wpc_dsp(sensor_t *sensor, int enable)
{
    return 0;
}

static int init_status(sensor_t *sensor){

    return 0;
}
#include "freertos/task.h"
int gc2145_init(sensor_t *sensor)
{

    initialize_gc2145_registers();
    // red_value();
    // gc2145_reg_test();
    // GC2145_Sensor_Init();
    // read_all_regs();
    printf("init finish\n");
    vTaskDelay(100);
    read_all_regs_for_gc2145();
    sensor->reset = reset;
    sensor->init_status = init_status;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_contrast  = set_contrast;
    sensor->set_brightness= set_brightness;
    sensor->set_saturation= set_saturation;

    sensor->set_quality = set_quality;
    sensor->set_colorbar = set_colorbar;

    sensor->set_gainceiling = set_gainceiling_sensor;
    sensor->set_gain_ctrl = set_agc_sensor;
    sensor->set_exposure_ctrl = set_aec_sensor;
    sensor->set_hmirror = set_hmirror_sensor;
    sensor->set_vflip = set_vflip_sensor;

    sensor->set_whitebal = set_awb_dsp;
    sensor->set_aec2 = set_aec2;
    sensor->set_aec_value = set_aec_value;
    sensor->set_special_effect = set_special_effect;
    sensor->set_wb_mode = set_wb_mode;
    sensor->set_ae_level = set_ae_level;

    sensor->set_dcw = set_dcw_dsp;
    sensor->set_bpc = set_bpc_dsp;
    sensor->set_wpc = set_wpc_dsp;
    sensor->set_awb_gain = set_awb_gain_dsp;
    sensor->set_agc_gain = set_agc_gain;

    sensor->set_raw_gma = set_raw_gma_dsp;
    sensor->set_lenc = set_lenc_dsp;

    ESP_LOGD(TAG, "OV2640 Attached");
    return 0;
}
