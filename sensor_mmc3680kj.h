/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-16     zhourq       the first version
 */
#ifndef APPLICATIONS_MMC3680KJ_MMC3680KJ_H_
#define APPLICATIONS_MMC3680KJ_MMC3680KJ_H_
#include "sensor.h"
#include "mmc3680kj.h"

#define MMC3680KJ_I2CBUS_NAME "i2c4"
#define MMC3680KJ_ADDR_DEFAULT 0x30

int rt_hw_mmc3680kj_init(const char *name, struct rt_sensor_config *cfg);


#endif /* APPLICATIONS_MMC3680KJ_MMC3680KJ_H_ */
