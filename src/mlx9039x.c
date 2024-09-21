/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-20     lgnq         the first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "mlx9039x.h"

#include <string.h>
#include <stdlib.h>

#include <math.h>

/**
 * This function reads the value of register for mlx9039x
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx9039x
 * @param val read data pointer
 *
 * @return the reading status, RT_EOK represents reading the value of register successfully.
 */
static rt_err_t mlx9039x_mem_direct_read(struct mlx9039x_device *dev, rt_uint8_t *recv_buf, rt_uint8_t recv_len)
{
    rt_err_t res = RT_EOK;

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        struct rt_i2c_msg msgs;

        msgs.addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs.flags = RT_I2C_RD;        /* Read flag */
        msgs.buf   = recv_buf;         /* Read data pointer */
        msgs.len   = recv_len;         /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            rt_kprintf("rt_i2c_transfer error\r\n");
            res = -RT_ERROR;
        }
#endif
    }

    return res;
}

/**
 * This function reads the value of register for mlx9039x
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx9039x
 * @param val read data pointer
 *
 * @return the reading status, RT_EOK represents reading the value of register successfully.
 */
static rt_err_t mlx9039x_mem_read(struct mlx9039x_device *dev, rt_uint8_t start_addr, rt_uint8_t *recv_buf, rt_uint8_t recv_len)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf = start_addr;

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        struct rt_i2c_msg msgs[2];

        msgs[0].addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &send_buf;        /* Write data pointer */
        msgs[0].len   = 1;                /* Number of bytes write */

        msgs[1].addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = recv_buf;         /* Read data pointer */
        msgs[1].len   = recv_len;         /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            res = RT_EOK;
        }
        else
        {
            rt_kprintf("rt_i2c_transfer error\r\n");
            res = -RT_ERROR;
        }
#endif
    }

    return res;
}

/**
 * This function reads the value of register for mlx9039x
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx9039x
 * @param val read data pointer
 *
 * @return the reading status, RT_EOK represents reading the value of register successfully.
 */
//send_buf = start register address + data1 + data2 + ...
static rt_err_t mlx9039x_mem_write(struct mlx9039x_device *dev, rt_uint8_t *send_buf, rt_uint8_t send_len)
{
    rt_err_t res = RT_EOK;

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        struct rt_i2c_msg msgs;

        msgs.addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs.flags = RT_I2C_WR;        /* Read flag */
        msgs.buf   = send_buf;         /* Read data pointer */
        msgs.len   = send_len;         /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            rt_kprintf("rt_i2c_transfer error\r\n");
            res = -RT_ERROR;
        }
#endif
    }

    return res;
}

rt_err_t mlx9039x_reset(struct mlx9039x_device *dev)
{
    rt_err_t res = RT_EOK;

    return res;
}

static rt_err_t mlx9039x_address_reset(struct mlx9039x_device *dev)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    send_buf[0] = 0x11;
    send_buf[1] = 0x06;

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        struct rt_i2c_msg msgs;

        msgs.addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs.flags = RT_I2C_WR;        /* Read flag */
        msgs.buf   = send_buf;         /* Read data pointer */
        msgs.len   = 2;                /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            rt_kprintf("rt_i2c_transfer error\r\n");
            res = -RT_ERROR;
        }
#endif
    }

    return res;
}

static rt_err_t mlx9039x_get_stat1(struct mlx9039x_device *dev, union mlx9039x_stat1 *stat1)
{
    rt_err_t res = RT_EOK;

    res = mlx9039x_mem_read(dev, MEM_ADDRESS_STAT1, (rt_uint8_t *)stat1, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("error\r\n");
    }
    else
    {
//        rt_kprintf("STAT1 = 0x%x, DRDY = 0x%x\r\n", stat1->byte_val, stat1->drdy);
    }

    return res;
}

static rt_err_t mlx9039x_get_stat2(struct mlx9039x_device *dev, union mlx9039x_stat2 *stat2)
{
    rt_err_t res = RT_EOK;

    res = mlx9039x_mem_read(dev, MEM_ADDRESS_STAT2, (rt_uint8_t *)stat2, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("error\r\n");
    }
    else
    {
//        rt_kprintf("STAT2 = 0x%x\r\n", stat2->byte_val);
    }

    return res;
}

static rt_bool_t mlx9039x_is_data_ready(struct mlx9039x_device *dev)
{
    union mlx9039x_stat1 stat1;

    mlx9039x_get_stat1(dev, &stat1);
    if (stat1.drdy)
    {
        return RT_TRUE;
    }
    else
    {
        return RT_FALSE;
    }
}

static rt_bool_t mlx9039x_is_data_overrun(struct mlx9039x_device *dev)
{
    union mlx9039x_stat2 stat2;

    mlx9039x_get_stat2(dev, &stat2);

#if MLX9039x == MLX90397RLQ_AAA_000
    if (stat2.dor || stat2.hovf_x || stat2.hovf_y || stat2.hovf_z)
    {
        return RT_TRUE;
    }
    else
    {
        return RT_FALSE;
    }
#else
    if (stat2.dor || stat2.hovf)
    {
        return RT_TRUE;
    }
    else
    {
        return RT_FALSE;
    }
#endif
}

rt_err_t mlx9039x_get_x(struct mlx9039x_device *dev, rt_int16_t *x)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    res = mlx9039x_mem_read(dev, MEM_ADDRESS_X, recv_buf, 2);
    if (res == RT_EOK)
    {
        *x = recv_buf[1]<<8 | recv_buf[0];
    }

    return res;
}

rt_err_t mlx9039x_get_y(struct mlx9039x_device *dev, rt_int16_t *y)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    res = mlx9039x_mem_read(dev, MEM_ADDRESS_Y, recv_buf, 2);
    if (res == RT_EOK)
    {
        *y = recv_buf[1]<<8 | recv_buf[0];
    }

    return res;
}

rt_err_t mlx9039x_get_z(struct mlx9039x_device *dev, rt_int16_t *z)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    res = mlx9039x_mem_read(dev, MEM_ADDRESS_Z, recv_buf, 2);
    if (res == RT_EOK)
    {
        *z = recv_buf[1]<<8 | recv_buf[0];
    }

    return res;
}

rt_err_t mlx9039x_get_x_flux(struct mlx9039x_device *dev, float *x)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    while (mlx9039x_is_data_ready(dev) == RT_FALSE)
    {
        rt_thread_delay(100);
    }

    res = mlx9039x_mem_read(dev, MEM_ADDRESS_X, recv_buf, 2);
    if (res == RT_EOK)
    {
        *x = (float)(((rt_int16_t)recv_buf[1] << 8) | recv_buf[0])*MAGNETIC_SENSITIVITY_XY;
    }

    return res;
}

rt_err_t mlx9039x_get_y_flux(struct mlx9039x_device *dev, float *y)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    while (mlx9039x_is_data_ready(dev) == RT_FALSE)
    {
        rt_thread_delay(100);
    }

    res = mlx9039x_mem_read(dev, MEM_ADDRESS_Y, recv_buf, 2);
    if (res == RT_EOK)
    {
        *y = (float)(((rt_int16_t)recv_buf[1] << 8) | recv_buf[0])*MAGNETIC_SENSITIVITY_XY;
    }

    return res;
}

rt_err_t mlx9039x_get_z_flux(struct mlx9039x_device *dev, float *z)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    while (mlx9039x_is_data_ready(dev) == RT_FALSE)
    {
        rt_thread_delay(100);
    }

    res = mlx9039x_mem_read(dev, MEM_ADDRESS_Z, recv_buf, 2);
    if (res == RT_EOK)
    {
        *z = (float)(((rt_int16_t)recv_buf[1] << 8) | recv_buf[0])*MAGNETIC_SENSITIVITY_Z;
    }

    return res;
}

rt_err_t mlx9039x_get_t(struct mlx9039x_device *dev, rt_int16_t *t)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    res = mlx9039x_mem_read(dev, MEM_ADDRESS_T, recv_buf, 2);
    if (res == RT_EOK)
    {
        *t = recv_buf[1]<<8 | recv_buf[0];
    }

    return res;
}

rt_err_t mlx9039x_get_temperature(struct mlx9039x_device *dev, float *t)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    while (mlx9039x_is_data_ready(dev) == RT_FALSE)
    {
        rt_thread_delay(100);
    }

    res = mlx9039x_mem_read(dev, MEM_ADDRESS_T, recv_buf, 2);
    if (res == RT_EOK)
    {
        *t = (float)(((rt_int16_t)recv_buf[1] << 8 ) | recv_buf[0] ) / TEMPERATURE_RES;
    }

    return res;
}

rt_err_t mlx9039x_get_cid(struct mlx9039x_device *dev, rt_uint8_t *cid)
{
    rt_err_t res = RT_EOK;

    res = mlx9039x_mem_read(dev, MEM_ADDRESS_CID, cid, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("Read CID is error\r\n");
    }

    return res;
}

rt_err_t mlx9039x_get_did(struct mlx9039x_device *dev, rt_uint8_t *did)
{
    rt_err_t res = RT_EOK;

    res = mlx9039x_mem_read(dev, MEM_ADDRESS_DID, did, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("Read DID is error\r\n");
    }

    return res;
}

rt_err_t mlx9039x_get_mode(struct mlx9039x_device *dev, rt_uint8_t *mode)
{
    rt_err_t res = RT_EOK;

    res = mlx9039x_mem_read(dev, 0x10, mode, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("Read MODE is error\r\n");
    }

    return res;
}

rt_err_t mlx9039x_set_mode(struct mlx9039x_device *dev, rt_uint8_t application_mode)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    send_buf[0] = MEM_ADDRESS_CTRL;
    send_buf[1] = application_mode;
    res = mlx9039x_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        rt_kprintf("set application mode error\r\n");
    }
    else
    {
        switch (application_mode & 0xF)
        {
        case POWER_DOWN_MODE:
            rt_kprintf("POWER_DOWN_MODE\r\n");
            break;
        case SINGLE_MEASUREMENT_MODE:
            rt_kprintf("SINGLE_MEASUREMENT_MODE\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_10HZ:
            rt_kprintf("CONTINUOUS_MEASUREMENT_MODE_10HZ\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_20HZ:
            rt_kprintf("CONTINUOUS_MEASUREMENT_MODE_20HZ\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_50HZ:
            rt_kprintf("CONTINUOUS_MEASUREMENT_MODE_50HZ\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_100HZ:
            rt_kprintf("CONTINUOUS_MEASUREMENT_MODE_100HZ\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_200HZ:
            rt_kprintf("CONTINUOUS_MEASUREMENT_MODE_200HZ\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_500HZ:
            rt_kprintf("CONTINUOUS_MEASUREMENT_MODE_500HZ\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_700HZ:
            rt_kprintf("CONTINUOUS_MEASUREMENT_MODE_700HZ\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_1400HZ:
            rt_kprintf("CONTINUOUS_MEASUREMENT_MODE_1400HZ");
            break;
        default:
            rt_kprintf("unknown application mode\r\n");
            break;
        }
    }

    return res;
}

rt_err_t mlx9039x_get_osr_dig_filt(struct mlx9039x_device *dev, union mlx9039x_osr_dig_filt *val)
{
    rt_err_t res = RT_EOK;

    res = mlx9039x_mem_read(dev, 0x14, (rt_uint8_t *)val, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("Get OSR_DIG_FILT error\r\n");
    }

    return res;
}

rt_err_t mlx9039x_set_osr_dig_filt(struct mlx9039x_device *dev, union mlx9039x_osr_dig_filt val)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    send_buf[0] = 0x14;
    send_buf[1] = val.byte_val;
    res = mlx9039x_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        rt_kprintf("Set OSR_DIG_FILT error\r\n");
    }

    return res;
}

rt_err_t mlx9039x_get_cust_ctrl(struct mlx9039x_device *dev, union mlx9039x_cust_ctrl *val)
{
    rt_err_t res = RT_EOK;

    res = mlx9039x_mem_read(dev, 0x15, (rt_uint8_t *)val, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("Get CUST_CTRL error\r\n");
    }

    return res;
}

rt_err_t mlx9039x_set_cust_ctrl(struct mlx9039x_device *dev, union mlx9039x_cust_ctrl val)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    send_buf[0] = 0x15;
    send_buf[1] = val.byte_val;
    res = mlx9039x_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        rt_kprintf("Set CUST_CTRL error\r\n");
    }

    return res;
}

rt_err_t mlx9039x_set_cust_ctrl2(struct mlx9039x_device *dev, union mlx9039x_cust_ctrl2 val)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    send_buf[0] = 0xF;
    send_buf[1] = val.byte_val;
    res = mlx9039x_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        rt_kprintf("Set CUST_CTRL2 error\r\n");
    }

    return res;
}

rt_err_t mlx9039x_get_cust_ctrl2(struct mlx9039x_device *dev, union mlx9039x_cust_ctrl2 *val)
{
    rt_err_t res = RT_EOK;

    res = mlx9039x_mem_read(dev, 0xF, (rt_uint8_t *)val, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("Get CUST_CTRL2 error\r\n");
    }

    return res;
}

rt_err_t mlx9039x_get_magnetic_sensitivity(struct mlx9039x_device *dev, float *sensitivity_xy, float *sensitivity_z)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t val;

    res = mlx9039x_mem_read(dev, 0xF, &val, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("Get Magnetic sensitivity error\r\n");
    }

#if MLX9039x == MLX90397RLQ_AAA_000
    switch (val)
    {
    case MAGNETIC_RANGE_XYZ_25MT_25MT_25MT:
        *sensitivity_xy = 0.75;
        *sensitivity_z  = 0.75;
        break;
    case MAGNETIC_RANGE_XYZ_50MT_50MT_25MT:
        *sensitivity_xy = 1.5;
        *sensitivity_z  = 0.75;
        break;
    case MAGNETIC_RANGE_XYZ_25MT_25MT_50MT:
        *sensitivity_xy = 0.75;
        *sensitivity_z  = 1.5;
        break;
    case MAGNETIC_RANGE_XYZ_50MT_50MT_50MT:
        *sensitivity_xy = 1.5;
        *sensitivity_z  = 1.5;
        break;
    case MAGNETIC_RANGE_XYZ_25MT_25MT_100MT:
        *sensitivity_xy = 0.75;
        *sensitivity_z  = 3;
        break;
    case MAGNETIC_RANGE_XYZ_50MT_50MT_100MT:
        *sensitivity_xy = 1.5;
        *sensitivity_z  = 3;
        break;
    case MAGNETIC_RANGE_XYZ_25MT_25MT_200MT:
        *sensitivity_xy = 0.75;
        *sensitivity_z  = 6;
        break;
    case MAGNETIC_RANGE_XYZ_50MT_50MT_200MT:
        *sensitivity_xy = 1.5;
        *sensitivity_z  = 6;
        break;
    default:
        break;
    }
#elif MLX9039x == MLX90392ELQ_AAA_010
    *sensitivity_xy = 0.15;
    *sensitivity_z  = 0.15;
#elif MLX9039x == MLX90392ELQ_AAA_011
    *sensitivity_xy = 1.5;
    *sensitivity_z  = 1.5;
#elif MLX9039x == MLX90392ELQ_AAA_013
    *sensitivity_xy = 1.5;
    *sensitivity_z  = 1.5;
#endif

    return res;
}

rt_err_t mlx9039x_set_temperature(struct mlx9039x_device *dev, rt_uint8_t onoff)
{
    rt_err_t res = RT_EOK;
    union mlx9039x_cust_ctrl val;

    res = mlx9039x_get_cust_ctrl(dev, &val);

    if (1 == onoff)
    {
        if (val.t_comp_en == 0)
        {
            val.t_comp_en = 1;
            res = mlx9039x_set_cust_ctrl(dev, val);
        }
    }
    else
    {
        if (val.t_comp_en == 1)
        {
            val.t_comp_en = 0;
            res = mlx9039x_set_cust_ctrl(dev, val);
        }
    }

    return res;
}

rt_err_t mlx9039x_get_xyz(struct mlx9039x_device *dev, struct mlx9039x_xyz *xyz)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[6];

    while (mlx9039x_is_data_ready(dev) == RT_FALSE)
    {
        rt_thread_delay(100);
    }

    res = mlx9039x_mem_read(dev, 0x1, recv_buf, 6);
    if (res == RT_EOK)
    {
        xyz->x = recv_buf[1]<<8 | recv_buf[0];
        xyz->y = recv_buf[3]<<8 | recv_buf[2];
        xyz->z = recv_buf[5]<<8 | recv_buf[4];
    }

    if (mlx9039x_is_data_overrun(dev))
        res = RT_ERROR;

    return res;
}

rt_err_t mlx9039x_get_xyz_flux(struct mlx9039x_device *dev, struct mlx9039x_xyz_flux *xyz)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[6];

    while (mlx9039x_is_data_ready(dev) == RT_FALSE)
    {
        rt_thread_delay(100);
    }

    res = mlx9039x_mem_read(dev, 0x1, recv_buf, 6);
    if (res == RT_EOK)
    {
        xyz->x = (float)(((rt_int16_t)recv_buf[1] << 8) | recv_buf[0]) * MAGNETIC_SENSITIVITY_XY;
        xyz->y = (float)(((rt_int16_t)recv_buf[3] << 8) | recv_buf[2]) * MAGNETIC_SENSITIVITY_XY;
        xyz->z = (float)(((rt_int16_t)recv_buf[5] << 8) | recv_buf[4]) * MAGNETIC_SENSITIVITY_Z;
    }

    if (mlx9039x_is_data_overrun(dev))
        res = RT_ERROR;

    return res;
}

rt_err_t mlx9039x_set_hallconf(struct mlx9039x_device *dev, rt_uint8_t hallconf)
{
    rt_err_t res = 0;

//    rt_uint16_t register_val;
//    union mlx9039x_register0 reg;
//
//    res = mlx9039x_read_reg(dev, 0, &register_val);
//    if (res == -RT_ERROR)
//        return res;
//
//    reg.word_val = register_val;
//    reg.hallconf = hallconf;
//    res = mlx9039x_write_reg(dev, 0, reg.word_val);
//    if (res == -RT_ERROR)
//        return res;
        
    return res;
}

rt_err_t mlx9039x_set_oversampling(struct mlx9039x_device *dev, mlx9039x_oversampling_t osr)
{
    rt_err_t res = 0;

//    rt_uint16_t register_val;
//    union mlx9039x_register2 reg;
//
//    res = mlx9039x_read_reg(dev, 2, &register_val);
//    if (res == -RT_ERROR)
//        return res;
//
//    reg.word_val = register_val;
//    reg.osr = osr;
//    res = mlx9039x_write_reg(dev, 2, reg.word_val);
//    if (res == -RT_ERROR)
//        return res;

    return res;
}

rt_err_t mlx9039x_get_oversampling(struct mlx9039x_device *dev, mlx9039x_oversampling_t *osr)
{
    rt_err_t res = 0;

//    rt_uint16_t register_val;
//    union mlx9039x_register2 reg;
//
//    res = mlx9039x_read_reg(dev, 2, &register_val);
//    if (res == -RT_ERROR)
//        return res;
//
//    reg.word_val = register_val;
//    *osr = reg.osr;
        
    return res;
}

rt_err_t mlx9039x_set_digital_filtering(struct mlx9039x_device *dev, mlx9039x_filter_t dig_filt)
{
    rt_err_t res = 0;

//    rt_uint16_t register_val;
//    union mlx9039x_register2 reg;
//
//    res = mlx9039x_read_reg(dev, 2, &register_val);
//    if (res == -RT_ERROR)
//        return res;
//
//    reg.word_val = register_val;
//    reg.dig_filt = dig_filt;
//    res = mlx9039x_write_reg(dev, 2, reg.word_val);
//    if (res == -RT_ERROR)
//        return res;

    return res;
}

rt_err_t mlx9039x_get_digital_filtering(struct mlx9039x_device *dev, mlx9039x_filter_t *dig_filt)
{
    rt_err_t res = 0;

//    rt_uint16_t register_val;
//    union mlx9039x_register2 reg;
//
//    res = mlx9039x_read_reg(dev, 2, &register_val);
//    if (res == -RT_ERROR)
//        return res;
//
//    reg.word_val = register_val;
//    *dig_filt = reg.dig_filt;

    return res;
}

void mlx9039x_setup(struct mlx9039x_device *dev)
{
//    mlx9039x_reset(dev);

//    rt_thread_delay(10000);

//    mlx9039x_set_gain_sel(dev, 4);
//    mlx9039x_set_resolution(dev, 0, 0, 0);
//    mlx9039x_set_oversampling(dev, 3);
//    mlx9039x_set_digital_filtering(dev, 7);
//    mlx9039x_set_temperature_compensation(dev, 0);
}

/**
 * This function gets the raw data of mlx9039x
 *
 * @param dev the pointer of device driver structure
 * @param xyz the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK represents  reading the data successfully.
 */
static rt_err_t mlx9039x_continuous_measurement(struct mlx9039x_device *dev, struct mlx9039x_xyz *xyz, rt_uint16_t freq)
{
    rt_uint8_t status = RT_EOK;

    struct mlx9039x_xyz_flux xyz_flux;

    switch (freq)
    {
    case 10:
        rt_kprintf("10Hz");
        status = mlx9039x_set_mode(dev, (0x7<<4) | CONTINUOUS_MEASUREMENT_MODE_10HZ);
        break;
    case 20:
        rt_kprintf("20Hz");
        status = mlx9039x_set_mode(dev, (0x7<<4) | CONTINUOUS_MEASUREMENT_MODE_20HZ);
        break;
    case 50:
        rt_kprintf("50Hz");
        status = mlx9039x_set_mode(dev, (0x7<<4) | CONTINUOUS_MEASUREMENT_MODE_50HZ);
        break;
    case 100:
        rt_kprintf("100Hz");
        status = mlx9039x_set_mode(dev, (0x7<<4) | CONTINUOUS_MEASUREMENT_MODE_100HZ);
        break;
    case 200:
        rt_kprintf("200Hz");
        status = mlx9039x_set_mode(dev, (0x7<<4) | CONTINUOUS_MEASUREMENT_MODE_200HZ);
        break;
    case 500:
        rt_kprintf("500Hz");
        status = mlx9039x_set_mode(dev, (0x7<<4) | CONTINUOUS_MEASUREMENT_MODE_500HZ);
        break;
    case 700:
        rt_kprintf("700Hz");
        status = mlx9039x_set_mode(dev, (0x7<<4) | CONTINUOUS_MEASUREMENT_MODE_700HZ);
        break;
    case 1400:
        rt_kprintf("1400Hz");
        status = mlx9039x_set_mode(dev, (0x7<<4) | CONTINUOUS_MEASUREMENT_MODE_1400HZ);
        break;
    default:
        rt_kprintf("wrong frequency\r\n");
        break;
    }

    while (1)
    {
        status = mlx9039x_get_xyz(dev, xyz);
        if (status == RT_EOK)
        {
            float angle;
            float alpha;
            float beta;

            rt_int16_t a;
            rt_uint16_t b;

            xyz_flux.x = (float)xyz->x * dev->magnetic_sensitivity_xy;
            xyz_flux.y = (float)xyz->y * dev->magnetic_sensitivity_xy;
            xyz_flux.z = (float)xyz->z * dev->magnetic_sensitivity_z;

            a = (rt_int16_t)xyz_flux.x;
            b = fabs(xyz_flux.x - a) * 100;
            rt_kprintf("X = 0x%x[%d.%02duT]\t", xyz->x, a, b);

            a = (rt_int16_t)xyz_flux.y;
            b = fabs(xyz_flux.y - a) * 100;
            rt_kprintf("Y = 0x%x[%d.%02duT]\t", xyz->y, a, b);

            a = (rt_int16_t)xyz_flux.z;
            b = fabs(xyz_flux.z - a) * 100;
            rt_kprintf("Z = 0x%x[%d.%02duT]\t", xyz->z, a, b);

            angle = atan2f(xyz->y, xyz->x);
            if (angle < 0)
                angle = angle + 2*PI;
            a = (int)angle;
            b = fabs(angle-a) * 100;
            rt_kprintf("Angle = %d.%02d\t", a, b);

            alpha = atan2f(xyz->z, xyz->x);
            if (alpha < 0)
                alpha = alpha + 2*PI;
            a = (int)alpha;
            b = fabs(alpha-a) * 100;
            rt_kprintf("Alpha = %d.%02d\t", a, b);

            beta = atan2f(xyz->z, xyz->y);
            if (beta < 0)
                beta = beta + 2*PI;
            a = (int)beta;
            b = fabs(beta-a) * 100;
            rt_kprintf("Beta = %d.%02d\r\n", a, b);
        }

        rt_thread_delay(500);
    }

    return status;
}

static rt_err_t mlx9039x_single_measurement(struct mlx9039x_device *dev, struct mlx9039x_xyz_flux *xyz)
{
    rt_uint8_t status = RT_EOK;
    union mlx9039x_stat1 stat1;

    status = mlx9039x_set_mode(dev, SINGLE_MEASUREMENT_MODE);

    stat1.byte_val = 0;
    while (stat1.drdy == 0)
    {
        status = mlx9039x_get_stat1(dev, &stat1);
        rt_thread_delay(100);
    }

    status = mlx9039x_get_xyz_flux(dev, xyz);

    return status;
}

/**
 * This function initialize the mlx9039x device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication, RT_NULL for spi
 *
 * @return the pointer of device driver structure, RT_NULL represents  initialization failed.
 */
struct mlx9039x_device *mlx9039x_init(const char *dev_name, rt_uint8_t param)
{
    struct mlx9039x_device *dev = RT_NULL;

    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct mlx9039x_device));
    if (dev == RT_NULL)
    {
        rt_kprintf("Can't allocate memory for mlx9039x device on '%s' ", dev_name);
        goto __exit;
    }

    dev->bus = rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        rt_kprintf("Can't find device:'%s'", dev_name);
        goto __exit;
    }

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        if (param != RT_NULL)
        {
            dev->i2c_addr = param;
        }
        else
        {
            rt_uint8_t id[2];

            /* find mlx9039x device at address: 0x0C */
            dev->i2c_addr = MLX9039x_I2C_ADDRESS;
            if (mlx9039x_mem_read(dev, 0x0A, id, 2) != RT_EOK)
            {
                rt_kprintf("Can't find device at '%s'!", dev_name);
                goto __exit;
            }
            else
            {
                rt_kprintf("CID is 0x%x\r\n", id[0]);
                rt_kprintf("DID is 0x%x\r\n", id[1]);

                mlx9039x_set_mode(dev, SINGLE_MEASUREMENT_MODE);
                mlx9039x_set_temperature(dev, 1);

                mlx9039x_get_magnetic_sensitivity(dev, &(dev->magnetic_sensitivity_xy), &(dev->magnetic_sensitivity_z));
            }

            rt_kprintf("Device i2c address is:'0x%x'!\r\n", dev->i2c_addr);
        }
#endif        
    }
    else
    {
        rt_kprintf("Unsupported device:'%s'!", dev_name);
        goto __exit;
    }

    return dev;

__exit:
    if (dev != RT_NULL)
    {
        rt_free(dev);
    }
    return RT_NULL;
}

/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
void mlx9039x_deinit(struct mlx9039x_device *dev)
{
    RT_ASSERT(dev);

    rt_free(dev);
}

static void mlx9039x(int argc, char **argv)
{
    static struct mlx9039x_device *dev = RT_NULL;

    /* If the number of arguments less than 2 */
    if (argc < 2)
    {
        rt_kprintf("\n");
        rt_kprintf("mlx9039x [OPTION] [PARAM]\n");
        rt_kprintf("         probe <dev_name>      Probe mlx9039x by given name, ex:i2c2\n");
        rt_kprintf("         id                    Print CID and DID\n");
        rt_kprintf("         stat1                 Print stat1\n");
        rt_kprintf("                               var = [0 - 3] means [250 - 2000DPS]\n");
        rt_kprintf("         ar <var>              Set accel range to var\n");
        rt_kprintf("                               var = [0 - 3] means [2 - 16G]\n");
        rt_kprintf("         sleep <var>           Set sleep status\n");
        rt_kprintf("                               var = 0 means disable, = 1 means enable\n");
        rt_kprintf("         read [num]            read [num] times mlx9039x\n");
        rt_kprintf("                               num default 5\n");
        return;
    }
    else
    {
        if (!strcmp(argv[1], "probe"))
        {
            if (dev)
            {
                mlx9039x_deinit(dev);
            }

            if (argc == 2)
                dev = mlx9039x_init("i2c2", RT_NULL);
            else if (argc == 3)
                dev = mlx9039x_init(argv[2], RT_NULL);
        }
        else if (dev == RT_NULL)
        {
            rt_kprintf("Please probe mlx9039x first!\n");
            return;
        }
        else if (!strcmp(argv[1], "id"))
        {
            rt_uint8_t id[2];
            rt_uint8_t start_addr = MEM_ADDRESS_CID;
            rt_uint8_t len = 2;

            mlx9039x_mem_read(dev, start_addr, id, len);
            rt_kprintf("CID = 0x%x\r\n", id[0]);
            rt_kprintf("DID = 0x%x\r\n", id[1]);
        }
        else if (!strcmp(argv[1], "stat1"))
        {
            union mlx9039x_stat1 stat1;

            mlx9039x_get_stat1(dev, &stat1);
        }
        else if (!strcmp(argv[1], "mode"))
        {
            mlx9039x_set_mode(dev, atoi(argv[2]));
        }
        else if (!strcmp(argv[1], "x"))
        {
            rt_int16_t x;

            mlx9039x_get_x(dev, &x);
            rt_kprintf("x = 0x%x\r\n", x);
        }
        else if (!strcmp(argv[1], "y"))
        {
            rt_int16_t y;

            mlx9039x_get_y(dev, &y);
            rt_kprintf("y = 0x%x\r\n", y);
        }
        else if (!strcmp(argv[1], "z"))
        {
            rt_int16_t z;

            mlx9039x_get_z(dev, &z);
            rt_kprintf("z = 0x%x\r\n", z);
        }
        else if (!strcmp(argv[1], "t"))
        {
            float t;

            mlx9039x_get_temperature(dev, &t);
            rt_kprintf("t = %d.%d\r\n", (rt_int16_t)t, (rt_uint16_t)(t*100)%100);
        }
        else if (!strcmp(argv[1], "rr"))
        {
            rt_uint8_t val;
            mlx9039x_mem_read(dev, atoi(argv[2]), &val, 1);

            rt_kprintf("Reading REG[%d] = 0x%x...\r\n", atoi(argv[2]), val);
        }
        else if (!strcmp(argv[1], "setup"))
        {
            mlx9039x_setup(dev);
        }
        else if (!strcmp(argv[1], "xyz"))
        {
            struct mlx9039x_xyz_flux xyz;

//            mlx9039x_single_measurement(dev, &xyz);
            mlx9039x_get_xyz_flux(dev, &xyz);
            rt_kprintf("x = %d.%d\r\n", (rt_int16_t)xyz.x, (rt_int16_t)(xyz.x*10)%10);
            rt_kprintf("y = %d.%d\r\n", (rt_int16_t)xyz.y, (rt_int16_t)(xyz.y*10)%10);
            rt_kprintf("z = %d.%d\r\n", (rt_int16_t)xyz.z, (rt_int16_t)(xyz.z*10)%10);
        }
        else if (!strcmp(argv[1], "continuous"))
        {
            struct mlx9039x_xyz xyz;

            mlx9039x_continuous_measurement(dev, &xyz, atoi(argv[2]));
        }
        else
        {
            rt_kprintf("Unknown command, please enter 'mlx9039x' get help information!\n");
        }
    }
}

#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(mlx9039x, mlx9039x sensor function);
#endif

#ifdef FINSH_USING_MSH
    MSH_CMD_EXPORT(mlx9039x, mlx9039x sensor function);
#endif
