/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-16     zhourq       the first version
 */
#ifndef _MMC3680KJ_H
#define _MMC3680KJ_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t (*mmc3680kj_init)(void);
typedef int32_t (*mmc3680kj_deinit)(void);
typedef int32_t (*mmc3680kj_read_reg)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*mmc3680kj_write_reg)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef void    (*mmc3680kj_sleep)(uint32_t ms);

struct mmc3680kj_dev{
    uint8_t             Address;
    mmc3680kj_init      init;
    mmc3680kj_deinit    deinit;
    mmc3680kj_read_reg  read_reg;
    mmc3680kj_write_reg write_reg;
    mmc3680kj_sleep     sleep;
};


int32_t mmc3680_get_id(struct mmc3680kj_dev *dev,uint8_t* id);
int32_t mmc3680kj_set(struct mmc3680kj_dev *dev);
int32_t mmc3680kj_check_otp(struct mmc3680kj_dev *dev);
int32_t mmc3680kj_get_comp_matrix(struct mmc3680kj_dev *dev);
int32_t mmc3680kj_set_pulse_width(struct mmc3680kj_dev *dev);
int32_t mmc3680kj_set_output_resolution(struct mmc3680kj_dev *dev, uint16_t res);
int32_t mmc3680kj_mag_read(struct mmc3680kj_dev *dev,uint32_t *buf);
#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif/*_MMC3680KJ_H*/
