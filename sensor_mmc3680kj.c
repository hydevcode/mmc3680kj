/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-16     zhourq       the first version
 */
#include "stdio.h"
#include "string.h"
#define DBG_ENABLE
#define DBG_LEVEL DBG_INFO
#define DBG_SECTION_NAME  "sensor.memsic.mmc3680kj"
#define DBG_COLOR
#include <rtdbg.h>

#include "sensor_mmc3680kj.h"

#define SENSOR_MAG_RANGE_MIN    -30000
#define SENSOR_MAG_RANGE_MAX    30000
#define SENSOR_TEMP_RANGE_MIN   -75
#define SENSOR_TEMP_RANGE_MAX   125
#define SENSOR_DEFAULT_ADDRESS  "i2c4"

static struct rt_i2c_bus_device *i2c_bus_dev;
static struct mmc3680kj_dev dev;
static int32_t rt_i2c_write_reg(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len);
static int32_t rt_i2c_read_reg(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len);
static void rt_sleep(uint32_t ms);

static rt_err_t _mmc3680kj_init(struct rt_sensor_intf *intf)
{
    rt_uint8_t i2c_addr = (rt_uint32_t)(intf->user_data) & 0xff;

    i2c_bus_dev = rt_i2c_bus_device_find((intf->dev_name == RT_NULL) ?
                                               SENSOR_DEFAULT_ADDRESS :
                                               intf->dev_name);
    if(i2c_bus_dev == RT_NULL)
        return -RT_ERROR;

    dev.Address     = i2c_addr;
    dev.init        = RT_NULL;
    dev.deinit      = RT_NULL;
    dev.read_reg    = rt_i2c_read_reg;
    dev.write_reg   = rt_i2c_write_reg;
    dev.sleep       = rt_sleep;


    if(mmc3680kj_set(&dev) != RT_EOK)
        return -RT_ERROR;

    if(mmc3680kj_check_otp(&dev) != RT_EOK)
        return -RT_ERROR;

    if(mmc3680kj_get_comp_matrix(&dev) != RT_EOK)
        return -RT_ERROR;

    if(mmc3680kj_set_pulse_width(&dev) != RT_EOK)
        return -RT_ERROR;

    if(mmc3680kj_set_output_resolution(&dev,100) != RT_EOK)
        return -RT_ERROR;
    return RT_EOK;
}

static RT_SIZE_TYPE _mmc3680kj_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    if (sensor->info.type == RT_SENSOR_CLASS_MAG)
    {
        uint32_t buf[3];
        mmc3680kj_mag_read(&dev,buf);
        data->data.mag.x = buf[0];
        data->data.mag.y = buf[1];
        data->data.mag.z = buf[2];
        data->timestamp = rt_sensor_get_ts();
    }

    return 1;
}

static RT_SIZE_TYPE _mmc3680kj_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _mmc3680kj_polling_get_data(sensor, buf);
    }
    else
    {
        LOG_E("only RT_SENSOR_MODE_POLLING could get");
        return 0;
    }

}

static rt_err_t _mmc3680kj_get_id(rt_sensor_t sensor, uint16_t * args)
{
    uint8_t id = 0;
    mmc3680_get_id(&dev,&id);
    *args = id;
    return RT_EOK;
}

static rt_err_t _mmc3680kj_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    LOG_D("Setting range is not supported!");
    return RT_EOK;
}

static rt_err_t _mmc3680kj_set_odr(rt_sensor_t sensor, rt_uint16_t odr)
{
    if (sensor->info.type == RT_SENSOR_CLASS_MAG)
    {
        if(odr == 100 ||odr == 200 ||odr == 400 ||odr == 600)
        {
            mmc3680kj_set_output_resolution(&dev,odr);
            LOG_D("mag set odr %d", odr);
        }
        else {
            LOG_E("please set odr 100 200 400 or 600!");
            return -RT_ERROR;
        }
    }

    return RT_EOK;
}


static rt_err_t _mmc3680kj_set_POWER(rt_sensor_t sensor, rt_uint8_t power)
{
    if (power == RT_SENSOR_POWER_DOWN)
    {
        if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
        {
//            LSM6DSL_ACC_Disable(&lsm6dsl);
        }
        else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
        {
//            LSM6DSL_GYRO_Disable(&lsm6dsl);
        }
        else if (sensor->info.type == RT_SENSOR_CLASS_STEP)
        {
//            LSM6DSL_ACC_Disable(&lsm6dsl);
//            LSM6DSL_ACC_Disable_Pedometer(&lsm6dsl);
        }
        LOG_D("set power down");
    }
    else if (power == RT_SENSOR_POWER_NORMAL)
    {
//        lsm6dsl_xl_power_mode_set(&lsm6dsl.Ctx, LSM6DSL_XL_NORMAL);

        if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
        {
//            LSM6DSL_ACC_Enable(&lsm6dsl);
        }
        else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
        {
//            LSM6DSL_GYRO_Enable(&lsm6dsl);
        }
        else if (sensor->info.type == RT_SENSOR_CLASS_STEP)
        {
//            LSM6DSL_ACC_Enable(&lsm6dsl);
//            LSM6DSL_ACC_Enable_Pedometer(&lsm6dsl);
        }

        LOG_D("set power normal");
    }
    else if (power == RT_SENSOR_POWER_HIGH)
    {
//        lsm6dsl_xl_power_mode_set(&lsm6dsl.Ctx, LSM6DSL_XL_HIGH_PERFORMANCE);

        if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
        {
//            LSM6DSL_ACC_Enable(&lsm6dsl);
        }
        else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
        {
//            LSM6DSL_GYRO_Enable(&lsm6dsl);
        }

        LOG_D("set power high");
    }
    else
    {
        LOG_W("Unsupported mode, code is %d", power);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t _mmc3680kj_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        result = _mmc3680kj_get_id(sensor,args);
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _mmc3680kj_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = _mmc3680kj_set_odr(sensor,(rt_uint32_t)args & 0xffff);
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _mmc3680kj_set_POWER(sensor,(rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        result = -RT_ERROR;
        break;
    default:
         LOG_E("only RT_SENSOR_CTRL_GET_ID,RT_SENSOR_CTRL_SET_POWER,RT_SENSOR_CTRL_SET_ODR could set");
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    _mmc3680kj_fetch_data,
    _mmc3680kj_control
};

int rt_hw_mmc3680kj_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_mag = RT_NULL;
//    rt_sensor_t sensor_temp = RT_NULL;

    {
        sensor_mag = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_mag == RT_NULL)
            return -1;

        sensor_mag->info.type       = RT_SENSOR_CLASS_MAG;
        sensor_mag->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
        sensor_mag->info.model      = "mmc3680kj";
        sensor_mag->info.unit       = RT_SENSOR_UNIT_MGAUSS;
        sensor_mag->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_mag->info.range_max  = SENSOR_MAG_RANGE_MAX;
        sensor_mag->info.range_min  = SENSOR_MAG_RANGE_MIN;
        sensor_mag->info.period_min = 5;

        rt_memcpy(&sensor_mag->config, cfg, sizeof(struct rt_sensor_config));
        sensor_mag->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_mag, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code0: %d", result);
            goto __exit;
        }
    }
//
//    {
//        sensor_temp = rt_calloc(1, sizeof(struct rt_sensor_device));
//        if (sensor_temp == RT_NULL)
//            return -1;
//
//        sensor_temp->info.type       = RT_SENSOR_CLASS_TEMP;
//        sensor_temp->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
//        sensor_temp->info.model      = "mmc3680kj";
//        sensor_temp->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
//        sensor_temp->info.intf_type  = RT_SENSOR_INTF_I2C;
//        sensor_temp->info.range_max  = SENSOR_TEMP_RANGE_MAX;
//        sensor_temp->info.range_min  = SENSOR_TEMP_RANGE_MIN;
//        sensor_temp->info.period_min = 5;
//
//        rt_memcpy(&sensor_temp->config, cfg, sizeof(struct rt_sensor_config));
//        sensor_temp->ops = &sensor_ops;
//
//        result = rt_hw_sensor_register(sensor_temp, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
//        if (result != RT_EOK)
//        {
//            LOG_E("device register err code1: %d", result);
//            goto __exit;
//        }
//    }

    result = _mmc3680kj_init(&cfg->intf);
    if (result != RT_EOK)
    {
        LOG_E("device register err code2: %d", result);
        goto __exit;
    }
    return RT_EOK;

__exit:
    if (sensor_mag)
        rt_free(sensor_mag);
    return -RT_ERROR;
}

static int32_t rt_i2c_write_reg(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static int32_t rt_i2c_read_reg(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}
static void rt_sleep(uint32_t ms)
{
    rt_thread_mdelay(ms);
}

