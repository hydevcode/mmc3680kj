/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-21     zhourq       the first version
 */

/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-16     zhourq       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "mmc3680kj.h"

#define MMC3680KJ_REG_DATA                                0x00
#define MMC3680KJ_REG_XL                                  0x00
#define MMC3680KJ_REG_XH                                  0x01
#define MMC3680KJ_REG_YL                                  0x02
#define MMC3680KJ_REG_YH                                  0x03
#define MMC3680KJ_REG_ZL                                  0x04
#define MMC3680KJ_REG_ZH                                  0x05
#define MMC3680KJ_REG_TEMP                                0x06
#define MMC3680KJ_REG_STATUS                              0x07
#define MMC3680KJ_REG_CTRL0                               0x08
#define MMC3680KJ_REG_CTRL1                               0x09
#define MMC3680KJ_REG_CTRL2                               0x0a
#define MMC3680KJ_REG_X_THD                               0x0b
#define MMC3680KJ_REG_Y_THD                               0x0c
#define MMC3680KJ_REG_Z_THD                               0x0d
#define MMC3680KJ_REG_SELFTEST                            0x0e
#define MMC3680KJ_REG_PASSWORD                            0x0f
#define MMC3680KJ_REG_OTPMODE                             0x12
#define MMC3680KJ_REG_TESTMODE                            0x13
#define MMC3680KJ_REG_SR_PWIDTH                           0x20
#define MMC3680KJ_REG_OTP                                 0x2a
#define MMC3680KJ_REG_PRODUCTID                           0x2f

#define MMC3680KJ_CMD_REFILL                              0x20
#define MMC3680KJ_CMD_RESET                               0x10
#define MMC3680KJ_CMD_SET                                 0x08
#define MMC3680KJ_CMD_TM_M                                0x01
#define MMC3680KJ_CMD_TM_T                                0x02
#define MMC3680KJ_CMD_START_MDT                           0x04
#define MMC3680KJ_CMD_100HZ                               0x00
#define MMC3680KJ_CMD_200HZ                               0x01
#define MMC3680KJ_CMD_400HZ                               0x02
#define MMC3680KJ_CMD_600HZ                               0x03
#define MMC3680KJ_CMD_CM_14HZ                             0x01
#define MMC3680KJ_CMD_CM_5HZ                              0x02
#define MMC3680KJ_CMD_CM_1HZ                              0x04
#define MMC3680KJ_CMD_SW_RST                              0x80
#define MMC3680KJ_CMD_PASSWORD                            0xe1
#define MMC3680KJ_CMD_OTP_OPER                            0x11
#define MMC3680KJ_CMD_OTP_MR                              0x80
#define MMC3680KJ_CMD_OTP_ACT                             0x80
#define MMC3680KJ_CMD_OTP_NACT                            0x00
#define MMC3680KJ_CMD_STSET_OPEN                          0x02
#define MMC3680KJ_CMD_STRST_OPEN                          0x04
#define MMC3680KJ_CMD_ST_CLOSE                            0x00
#define MMC3680KJ_CMD_INT_MD_EN                           0x40
#define MMC3680KJ_CMD_INT_MDT_EN                          0x20

#define MMC3680KJ_PRODUCT_ID                              0x0a
#define MMC3680KJ_OTP_READ_DONE_BIT                       0x10
#define MMC3680KJ_PUMP_ON_BIT                             0x08
#define MMC3680KJ_MDT_BIT                                 0x04
#define MMC3680KJ_MEAS_T_DONE_BIT                         0x02
#define MMC3680KJ_MEAS_M_DONE_BIT                         0x01

#define MMC3680KJ_I2C_SLAVE_ADDR                          0x30
#define MMC3680KJ_ADDR_TRANS(n)                           ((n)<<1)
#define MMC3680KJ_I2C_ADDR                                MMC3680KJ_ADDR_TRANS(MMC3680KJ_I2C_SLAVE_ADDR)

#define MMC3680KJ_OFFSET                                  32768
#define MMC3680KJ_SENSITIVITY                             1024
#define MMC3680KJ_T_ZERO                                  -75
#define MMC3680KJ_T_SENSITIVITY                           80

#define MMC3680KJ_MAG_DATA_SIZE                           6
#define OTP_CONVERT(REG)                                  (((REG) >=32 ? (32 - (REG)) : (REG)) * 6)
static int32_t g_otp_matrix[3] = {1000000, 1000000, 1350000};
int32_t mmc3680_get_id(struct mmc3680kj_dev *dev,uint8_t* id)
{
    return dev->read_reg(dev->Address,MMC3680KJ_REG_PRODUCTID,id,1);
}

int32_t mmc3680kj_set(struct mmc3680kj_dev *dev)
{
    uint8_t value = 0;
    if(dev == RT_NULL){
        return -RT_ERROR;
    }
    value = MMC3680KJ_CMD_SET;
    return dev->write_reg(dev->Address,MMC3680KJ_REG_CTRL0,&value,1);
}

int32_t mmc3680kj_check_otp(struct mmc3680kj_dev *dev)
{
    uint8_t value = 0,ret = 0;
    if(dev == RT_NULL){
        return -RT_ERROR;
    }

    ret = dev->read_reg(dev->Address,MMC3680KJ_REG_STATUS,&value,1);
    if(ret != RT_EOK)
        return ret;

    if ((value & MMC3680KJ_OTP_READ_DONE_BIT) != MMC3680KJ_OTP_READ_DONE_BIT) {
        return -RT_ERROR;
    }
    return RT_EOK;
}

int32_t mmc3680kj_get_comp_matrix(struct mmc3680kj_dev *dev)
{
    uint8_t value = 0;
    uint8_t reg_data[2] = {0};
    int ret = RT_EOK;

    if(dev == RT_NULL){
        return -RT_ERROR;
    }

    value = MMC3680KJ_CMD_PASSWORD;
    ret = dev->write_reg(dev->Address, MMC3680KJ_REG_PASSWORD, &value, 1);
    if (ret != RT_EOK) {
        return ret;
    }

    value = MMC3680KJ_CMD_OTP_OPER;
    ret = dev->write_reg(dev->Address, MMC3680KJ_REG_OTPMODE, &value, 1);
    if (ret != RT_EOK) {
        return ret;
    }

    value = MMC3680KJ_CMD_OTP_MR;
    ret = dev->write_reg(dev->Address, MMC3680KJ_REG_TESTMODE, &value, 1);
    if (ret != RT_EOK) {
        return ret;
    }

    value = MMC3680KJ_CMD_OTP_ACT;
    ret = dev->write_reg(dev->Address, MMC3680KJ_REG_CTRL2, &value, 1);
    if (ret != RT_EOK) {
        return ret;
    }

    ret = dev->read_reg(dev->Address, MMC3680KJ_REG_OTP, reg_data, 2);
    if (ret != RT_EOK) {
        return ret;
    }

    value = MMC3680KJ_CMD_OTP_NACT;
    ret = dev->write_reg(dev->Address, MMC3680KJ_REG_CTRL2, &value,1);
    if (ret != RT_EOK) {
        return ret;
    }

    g_otp_matrix[0] = 1000000;
    g_otp_matrix[1] = OTP_CONVERT(reg_data[0] & 0x3f) * 1000 + 1000000;
    g_otp_matrix[2] = (OTP_CONVERT((reg_data[1] & 0x0f) << 2 | (reg_data[0] & 0xc0) >> 6 ) + 1000) * 1350;

    return RT_EOK;
}

int32_t mmc3680kj_set_pulse_width(struct mmc3680kj_dev *dev)
{
    uint8_t value = 0;
    int ret = RT_EOK;

    if(dev == RT_NULL){
        return -RT_ERROR;
    }

    value = MMC3680KJ_CMD_OTP_NACT;
    ret = dev->write_reg(dev->Address, MMC3680KJ_REG_CTRL2, &value, 1);
    if (ret != RT_EOK) {
        return ret;
    }

    value = MMC3680KJ_CMD_PASSWORD;
    ret = dev->write_reg(dev->Address, MMC3680KJ_REG_PASSWORD, &value, 1);
    if (ret != RT_EOK) {
        return ret;
    }

    ret = dev->read_reg(dev->Address, MMC3680KJ_REG_SR_PWIDTH, &value, 1);
    if (ret != RT_EOK) {
        return ret;
    }

    value &= 0xe7;

    ret = dev->write_reg(dev->Address, MMC3680KJ_REG_SR_PWIDTH, &value, 1);
    if (ret != RT_EOK) {
        return ret;
    }

    return 0;
}

int32_t mmc3680kj_set_output_resolution(struct mmc3680kj_dev *dev, uint16_t res)
{
    int ret = 0;
    uint8_t value = 0;

    if(dev == RT_NULL){
        return -RT_ERROR;
    }

    switch(res){
    case 100:
        value = MMC3680KJ_CMD_100HZ;
        break;
    case 200:
        value = MMC3680KJ_CMD_200HZ;
        break;
    case 400:
        value = MMC3680KJ_CMD_400HZ;
        break;
    case 600:
        value = MMC3680KJ_CMD_600HZ;
        break;
    }
    ret = dev->write_reg(dev->Address, MMC3680KJ_REG_CTRL1, &value, 1);
    if (ret != RT_EOK) {
        return ret;
    }

    return 0;
}

static int mmc3680kj_enable(struct mmc3680kj_dev *dev)
{
    uint8_t value = 0;
    int ret = RT_EOK;

    if(dev == RT_NULL){
        return -RT_ERROR;
    }

    value = MMC3680KJ_CMD_TM_M;
    ret = dev->write_reg(dev->Address, MMC3680KJ_REG_CTRL0, &value, 1);
    if (ret != RT_EOK) {
        return ret;
    }

    do {
        dev->sleep(10);
        ret = dev->read_reg(dev->Address, MMC3680KJ_REG_STATUS, &value, 1);
        if (ret != RT_EOK) {
            return ret;
        }
    } while ((value & 0x01) != 0x01);

    return RT_EOK;
}

int32_t mmc3680kj_mag_read(struct mmc3680kj_dev *dev,uint32_t *buf)
{
    int ret = RT_EOK;
    uint8_t reg_raw[MMC3680KJ_MAG_DATA_SIZE] = {0};
    uint16_t data_raw[3] = {0};
    uint32_t mag_raw[3] = {0};

    if(dev == RT_NULL || buf == RT_NULL){
        return -RT_ERROR;
    }

    ret = mmc3680kj_enable(dev);
    if (ret != RT_EOK) {
        return ret;
    }

    ret = dev->read_reg(dev->Address, MMC3680KJ_REG_DATA, reg_raw, MMC3680KJ_MAG_DATA_SIZE);
    if (ret != RT_EOK) {
        return ret;
    }

    data_raw[0] = (uint16_t)(reg_raw[1] << 8 | reg_raw[0]);
    data_raw[1] = (uint16_t)(reg_raw[3] << 8 | reg_raw[2]);
    data_raw[2] = (uint16_t)(reg_raw[5] << 8 | reg_raw[4]);
    mag_raw[0] = (uint32_t)(data_raw[0]);
    mag_raw[1] = (uint32_t)(data_raw[1] - data_raw[2] + MMC3680KJ_OFFSET);
    mag_raw[2] = (uint32_t)(data_raw[1] + data_raw[2] - MMC3680KJ_OFFSET);

    buf[0] = (int32_t)(mag_raw[0] - MMC3680KJ_OFFSET) * g_otp_matrix[0] / MMC3680KJ_SENSITIVITY;
    buf[1] = (int32_t)(mag_raw[1] - MMC3680KJ_OFFSET) * g_otp_matrix[1] / MMC3680KJ_SENSITIVITY;
    buf[2] = (int32_t)(mag_raw[2] - MMC3680KJ_OFFSET) * g_otp_matrix[2] / MMC3680KJ_SENSITIVITY;

    return 0;
}




