/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#include "sensor_melexis_mlx9039x.h"

#define DBG_TAG "sensor.melexis.mlx9039x"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define mlx_dev ((struct mlx9039x_device *)sensor->parent.user_data)

static struct mlx9039x_device *_mlx9039x_init(struct rt_sensor_intf *intf)
{
    rt_uint8_t i2c_addr = (rt_uint32_t)(intf->user_data) & 0xff;

    return mlx9039x_init(intf->dev_name, i2c_addr);
}

static rt_err_t _mlx9039x_set_range(rt_sensor_t sensor, rt_int32_t range)
{
//    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
//    {
//        rt_uint8_t range_ctr;
//
//        if (range < 2000)
//            range_ctr = MPU6XXX_ACCEL_RANGE_2G;
//        else if (range < 4000)
//            range_ctr = MPU6XXX_ACCEL_RANGE_4G;
//        else if (range < 8000)
//            range_ctr = MPU6XXX_ACCEL_RANGE_8G;
//        else
//            range_ctr = MPU6XXX_ACCEL_RANGE_16G;
//
//        LOG_D("acce set range %d", range_ctr);
//
//        return mlx9039x_set_param(mpu_dev, MPU6XXX_ACCEL_RANGE, range_ctr);
//    }
//    else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
//    {
//        rt_uint8_t range_ctr;
//
//        if (range < 250000UL)
//            range_ctr = MPU6XXX_GYRO_RANGE_250DPS;
//        else if (range < 500000UL)
//            range_ctr = MPU6XXX_GYRO_RANGE_500DPS;
//        else if (range < 1000000UL)
//            range_ctr = MPU6XXX_GYRO_RANGE_1000DPS;
//        else
//            range_ctr = MPU6XXX_GYRO_RANGE_2000DPS;
//
//        LOG_D("gyro set range %d", range);
//
//        return mlx9039x_set_param(mpu_dev, MPU6XXX_GYRO_RANGE, range_ctr);
//    }
    return RT_EOK;
}

static rt_err_t _mlx9039x_acc_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
{
    if (mode == RT_SENSOR_MODE_POLLING)
    {
        LOG_D("set mode to POLLING");
    }
    else
    {
        LOG_D("Unsupported mode, code is %d", mode);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t _mlx9039x_nop(rt_sensor_t sensor)
{
    mlx9039x_nop((struct mlx9039x_device *)sensor->parent.user_data);
}

static rt_err_t _mlx9039x_reset(rt_sensor_t sensor)
{
    mlx9039x_reset((struct mlx9039x_device *)sensor->parent.user_data);
}

static rt_size_t _mlx9039x_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
//    if (sensor->info.type == RT_SENSOR_CLASS_MPS)
//    {
//        struct mlx9039x_3axes acce;
//        if (mlx9039x_get_accel(mpu_dev, &acce) != RT_EOK)
//        {
//            return 0;
//        }
//
//        data->type = RT_SENSOR_CLASS_ACCE;
//        data->data.acce.x = acce.x;
//        data->data.acce.y = acce.y;
//        data->data.acce.z = acce.z;
//        data->timestamp = rt_sensor_get_ts();
//    }

    return 1;
}

static rt_size_t mlx9039x_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _mlx9039x_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t mlx9039x_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(rt_uint8_t *)args = mlx_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _mlx9039x_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _mlx9039x_acc_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _mlx9039x_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    mlx9039x_fetch_data,
    mlx9039x_control
};

int rt_hw_mlx9039x_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    struct mlx9039x_device *mlx_dev_temp;
    rt_sensor_t sensor_mps = RT_NULL;

    mlx_dev_temp = _mlx9039x_init(&cfg->intf);
    if (mlx_dev_temp == RT_NULL)
    {
        LOG_E("_mlx9039x init err!");
        goto __exit;
    }

    /* MPS sensor register */
    {
        sensor_mps = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_mps == RT_NULL)
            goto __exit;

        sensor_mps->info.type       = RT_SENSOR_CLASS_MAG;
        sensor_mps->info.vendor     = RT_SENSOR_VENDOR_MELEXIS;
        sensor_mps->info.model      = "mlx9039x";
        sensor_mps->info.unit       = RT_SENSOR_UNIT_MG;
        sensor_mps->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_mps->info.range_max  = 16000;
        sensor_mps->info.range_min  = 2000;
        sensor_mps->info.period_min = 5;

        rt_memcpy(&sensor_mps->config, cfg, sizeof(struct rt_sensor_config));
        sensor_mps->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_mps, name, RT_DEVICE_FLAG_RDWR, mlx_dev_temp);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    LOG_I("sensor init success");
    return RT_EOK;

__exit:
    if (mlx_dev_temp)
        mlx9039x_deinit(mlx_dev_temp);

    return -RT_ERROR;
}
