// SPDX-License-Identifier: GPL-2.0
/*
 * TI SN65DSI84 init driver
 *
 * Currently supported and tested on rk3568:
 * - SN65DSI84
 *   = 1x Single-link DSI ~ 2x Single-link or 1x Dual-link LVDS
 *   - Supported
 *   - Dual-link LVDS mode tested
 *   - 2x Single-link LVDS mode unsupported
 *     (should be easy to add by someone who has the HW)
 *  TODO: 
 *   - add auto detect of format (should be easy to add to DTS)
 *   - add remove of GPIO after remove driver
 *   - add .h file :)))
 * Copyright (C) 2024 Kulichkov Vitaliy <shizgiz1337@gmail.com>
 *
 * Based on previous work of:
 * Marek Vasut <marex@denx.de>
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_gpio.h> 
#include <linux/regmap.h>

#include <drm/drm_mipi_dsi.h>

/* ID registers */
#define REG_ID(n)				(0x00 + (n))
/* Reset and clock registers */
#define REG_RC_RESET				0x09
#define  REG_RC_RESET_SOFT_RESET		BIT(0)
#define REG_RC_LVDS_PLL				0x0a
#define  REG_RC_LVDS_PLL_PLL_EN_STAT		BIT(7)
#define  REG_RC_LVDS_PLL_LVDS_CLK_RANGE(n)	(((n) & 0x7) << 1)
#define  REG_RC_LVDS_PLL_HS_CLK_SRC_DPHY	BIT(0)
#define REG_RC_DSI_CLK				0x0b
#define  REG_RC_DSI_CLK_DSI_CLK_DIVIDER(n)	(((n) & 0x1f) << 3)
#define  REG_RC_DSI_CLK_REFCLK_MULTIPLIER(n)	((n) & 0x3)
#define REG_RC_PLL_EN				0x0d
#define  REG_RC_PLL_EN_PLL_EN			BIT(0)
/* DSI registers */
#define REG_DSI_LANE				0x10
#define  REG_DSI_LANE_LEFT_RIGHT_PIXELS		BIT(7)	/* DSI85-only */
#define  REG_DSI_LANE_DSI_CHANNEL_MODE_DUAL	0	/* DSI85-only */
#define  REG_DSI_LANE_DSI_CHANNEL_MODE_2SINGLE	BIT(6)	/* DSI85-only */
#define  REG_DSI_LANE_DSI_CHANNEL_MODE_SINGLE	BIT(5)
#define  REG_DSI_LANE_CHA_DSI_LANES(n)		(((n) & 0x3) << 3)
#define  REG_DSI_LANE_CHB_DSI_LANES(n)		(((n) & 0x3) << 1)
#define  REG_DSI_LANE_SOT_ERR_TOL_DIS		BIT(0)
#define REG_DSI_EQ				0x11
#define  REG_DSI_EQ_CHA_DSI_DATA_EQ(n)		(((n) & 0x3) << 6)
#define  REG_DSI_EQ_CHA_DSI_CLK_EQ(n)		(((n) & 0x3) << 2)
#define REG_DSI_CLK				0x12
#define  REG_DSI_CLK_CHA_DSI_CLK_RANGE(n)	((n) & 0xff)
/* LVDS registers */
#define REG_LVDS_FMT				0x18
#define  REG_LVDS_FMT_DE_NEG_POLARITY		BIT(7)
#define  REG_LVDS_FMT_HS_NEG_POLARITY		BIT(6)
#define  REG_LVDS_FMT_VS_NEG_POLARITY		BIT(5)
#define  REG_LVDS_FMT_LVDS_LINK_CFG		BIT(4)	/* 0:AB 1:A-only */
#define  REG_LVDS_FMT_CHA_24BPP_MODE		BIT(3)
#define  REG_LVDS_FMT_CHB_24BPP_MODE		BIT(2)
#define  REG_LVDS_FMT_CHA_24BPP_FORMAT1		BIT(1)
#define  REG_LVDS_FMT_CHB_24BPP_FORMAT1		BIT(0)
#define REG_LVDS_VCOM				0x19
#define  REG_LVDS_VCOM_CHA_LVDS_VOCM		BIT(6)
#define  REG_LVDS_VCOM_CHB_LVDS_VOCM		BIT(4)
#define  REG_LVDS_VCOM_CHA_LVDS_VOD_SWING(n)	(((n) & 0x3) << 2)
#define  REG_LVDS_VCOM_CHB_LVDS_VOD_SWING(n)	((n) & 0x3)
#define REG_LVDS_LANE				0x1a
#define  REG_LVDS_LANE_EVEN_ODD_SWAP		BIT(6)
#define  REG_LVDS_LANE_CHA_REVERSE_LVDS		BIT(5)
#define  REG_LVDS_LANE_CHB_REVERSE_LVDS		BIT(4)
#define  REG_LVDS_LANE_CHA_LVDS_TERM		BIT(1)
#define  REG_LVDS_LANE_CHB_LVDS_TERM		BIT(0)
#define REG_LVDS_CM				0x1b
#define  REG_LVDS_CM_CHA_LVDS_CM_ADJUST(n)	(((n) & 0x3) << 4)
#define  REG_LVDS_CM_CHB_LVDS_CM_ADJUST(n)	((n) & 0x3)
/* Video registers */
#define REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW	0x20
#define REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH	0x21
#define REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW	0x24
#define REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH	0x25
#define REG_VID_CHA_SYNC_DELAY_LOW		0x28
#define REG_VID_CHA_SYNC_DELAY_HIGH		0x29
#define REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW	0x2c
#define REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH	0x2d
#define REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW	0x30
#define REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH	0x31
#define REG_VID_CHA_HORIZONTAL_BACK_PORCH	0x34
#define REG_VID_CHA_VERTICAL_BACK_PORCH		0x36
#define REG_VID_CHA_HORIZONTAL_FRONT_PORCH	0x38
#define REG_VID_CHA_VERTICAL_FRONT_PORCH	0x3a
#define REG_VID_CHA_TEST_PATTERN		0x3c
/* IRQ registers */
#define REG_IRQ_GLOBAL				0xe0
#define  REG_IRQ_GLOBAL_IRQ_EN			BIT(0)
#define REG_IRQ_EN				0xe1
#define  REG_IRQ_EN_CHA_SYNCH_ERR_EN		BIT(7)
#define  REG_IRQ_EN_CHA_CRC_ERR_EN		BIT(6)
#define  REG_IRQ_EN_CHA_UNC_ECC_ERR_EN		BIT(5)
#define  REG_IRQ_EN_CHA_COR_ECC_ERR_EN		BIT(4)
#define  REG_IRQ_EN_CHA_LLP_ERR_EN		BIT(3)
#define  REG_IRQ_EN_CHA_SOT_BIT_ERR_EN		BIT(2)
#define  REG_IRQ_EN_CHA_PLL_UNLOCK_EN		BIT(0)
#define REG_IRQ_STAT				0xe5
#define  REG_IRQ_STAT_CHA_SYNCH_ERR		BIT(7)
#define  REG_IRQ_STAT_CHA_CRC_ERR		BIT(6)
#define  REG_IRQ_STAT_CHA_UNC_ECC_ERR		BIT(5)
#define  REG_IRQ_STAT_CHA_COR_ECC_ERR		BIT(4)
#define  REG_IRQ_STAT_CHA_LLP_ERR		BIT(3)
#define  REG_IRQ_STAT_CHA_SOT_BIT_ERR		BIT(2)
#define  REG_IRQ_STAT_CHA_PLL_UNLOCK		BIT(0)


enum sn65dsi84_model {
	MODEL_sn65dsi84,
	MODEL_SN65DSI84,
};

struct display_timings {
    u32 format;
    u32 lanes;
    u32 dsi_clk;
    u32 clock_frequency;
    u32 hactive;
    u32 vactive;
    u32 hback_porch;
    u32 hfront_porch;
    u32 hsync_len;
    u32 vback_porch;
    u32 vfront_porch;
    u32 vsync_len;
    u32 hsync_active;
    u32 vsync_active;
    u32 de_active;
    u32 pixelclk_active;
};

struct sn65dsi84 {
	struct device			*dev;
	struct regmap			*regmap;
	struct device_node		*host_node;
	struct gpio_desc		*enable_gpio;
    struct display_timings *display_timings;
	int				dsi_lanes;
};

static const struct regmap_range sn65dsi84_readable_ranges[] = {
	regmap_reg_range(REG_ID(0), REG_ID(8)),
	regmap_reg_range(REG_RC_LVDS_PLL, REG_RC_DSI_CLK),
	regmap_reg_range(REG_RC_PLL_EN, REG_RC_PLL_EN),
	regmap_reg_range(REG_DSI_LANE, REG_DSI_CLK),
	regmap_reg_range(REG_LVDS_FMT, REG_LVDS_CM),
	regmap_reg_range(REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW,
			 REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH),
	regmap_reg_range(REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW,
			 REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH),
	regmap_reg_range(REG_VID_CHA_SYNC_DELAY_LOW,
			 REG_VID_CHA_SYNC_DELAY_HIGH),
	regmap_reg_range(REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW,
			 REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH),
	regmap_reg_range(REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW,
			 REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH),
	regmap_reg_range(REG_VID_CHA_HORIZONTAL_BACK_PORCH,
			 REG_VID_CHA_HORIZONTAL_BACK_PORCH),
	regmap_reg_range(REG_VID_CHA_VERTICAL_BACK_PORCH,
			 REG_VID_CHA_VERTICAL_BACK_PORCH),
	regmap_reg_range(REG_VID_CHA_HORIZONTAL_FRONT_PORCH,
			 REG_VID_CHA_HORIZONTAL_FRONT_PORCH),
	regmap_reg_range(REG_VID_CHA_VERTICAL_FRONT_PORCH,
			 REG_VID_CHA_VERTICAL_FRONT_PORCH),
	regmap_reg_range(REG_VID_CHA_TEST_PATTERN, REG_VID_CHA_TEST_PATTERN),
	regmap_reg_range(REG_IRQ_GLOBAL, REG_IRQ_EN),
	regmap_reg_range(REG_IRQ_STAT, REG_IRQ_STAT),
};

static const struct regmap_access_table sn65dsi84_readable_table = {
	.yes_ranges = sn65dsi84_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(sn65dsi84_readable_ranges),
};

static const struct regmap_range sn65dsi84_writeable_ranges[] = {
	regmap_reg_range(REG_RC_RESET, REG_RC_DSI_CLK),
	regmap_reg_range(REG_RC_PLL_EN, REG_RC_PLL_EN),
	regmap_reg_range(REG_DSI_LANE, REG_DSI_CLK),
	regmap_reg_range(REG_LVDS_FMT, REG_LVDS_CM),
	regmap_reg_range(REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW,
			 REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH),
	regmap_reg_range(REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW,
			 REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH),
	regmap_reg_range(REG_VID_CHA_SYNC_DELAY_LOW,
			 REG_VID_CHA_SYNC_DELAY_HIGH),
	regmap_reg_range(REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW,
			 REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH),
	regmap_reg_range(REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW,
			 REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH),
	regmap_reg_range(REG_VID_CHA_HORIZONTAL_BACK_PORCH,
			 REG_VID_CHA_HORIZONTAL_BACK_PORCH),
	regmap_reg_range(REG_VID_CHA_VERTICAL_BACK_PORCH,
			 REG_VID_CHA_VERTICAL_BACK_PORCH),
	regmap_reg_range(REG_VID_CHA_HORIZONTAL_FRONT_PORCH,
			 REG_VID_CHA_HORIZONTAL_FRONT_PORCH),
	regmap_reg_range(REG_VID_CHA_VERTICAL_FRONT_PORCH,
			 REG_VID_CHA_VERTICAL_FRONT_PORCH),
	regmap_reg_range(REG_VID_CHA_TEST_PATTERN, REG_VID_CHA_TEST_PATTERN),
	regmap_reg_range(REG_IRQ_GLOBAL, REG_IRQ_EN),
	regmap_reg_range(REG_IRQ_STAT, REG_IRQ_STAT),
};

static const struct regmap_access_table sn65dsi84_writeable_table = {
	.yes_ranges = sn65dsi84_writeable_ranges,
	.n_yes_ranges = ARRAY_SIZE(sn65dsi84_writeable_ranges),
};

static const struct regmap_range sn65dsi84_volatile_ranges[] = {
	regmap_reg_range(REG_RC_RESET, REG_RC_RESET),
	regmap_reg_range(REG_RC_LVDS_PLL, REG_RC_LVDS_PLL),
	regmap_reg_range(REG_IRQ_STAT, REG_IRQ_STAT),
};

static const struct regmap_access_table sn65dsi84_volatile_table = {
	.yes_ranges = sn65dsi84_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(sn65dsi84_volatile_ranges),
};

static const struct regmap_config sn65dsi84_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.rd_table = &sn65dsi84_readable_table,
	.wr_table = &sn65dsi84_writeable_table,
	.volatile_table = &sn65dsi84_volatile_table,
	.cache_type = REGCACHE_RBTREE,
	.max_register = REG_IRQ_STAT,
};

static u8 get_lvds_clk_range(struct sn65dsi84 *ctx) {
    struct display_timings *display_timings = ctx->display_timings;
    // LVDS CLOCK = DSI_CLK * DSI_LANES/BPP*2
    u32 lvds_clk;

    lvds_clk = (display_timings->dsi_clk * 2 * 1000/ 24);
    
    if (lvds_clk >= 2500 && lvds_clk < 37500) {
        return 0b000; // 25 MHz <= LVDS_CLK < 37.5 MHz
    } else if (lvds_clk >= 37500 && lvds_clk < 62500) {
        return 0b001; // 37.5 MHz <= LVDS_CLK < 62.5 MHz
    } else if (lvds_clk >= 62500 && lvds_clk < 87500) {
        return 0b010; // 62.5 MHz <= LVDS_CLK < 87.5 MHz
    } else if (lvds_clk >= 87500 && lvds_clk < 112500) {
        return 0b011; // 87.5 MHz <= LVDS_CLK < 112.5 MHz
    } else if (lvds_clk >= 112500 && lvds_clk < 137500) {
        return 0b100; // 112.5 MHz <= LVDS_CLK < 137.5 MHz
    } else if (lvds_clk >= 137500 && lvds_clk <= 154000) {
        return 0b101; // 137.5 MHz <= LVDS_CLK <= 154 MHz
    } else {
        // Выход за пределы допустимого диапазона
        dev_err(ctx->dev, "LVDS clock is out of possible range");
        return -1; // Ошибка: частота вне допустимого диапазона
    }
}

unsigned char get_dsi_clk_range(struct sn65dsi84 *ctx) {
    struct display_timings *display_timings = ctx->display_timings;
    // Проверяем диапазон частоты
    unsigned int new_dsi_clk;
    unsigned char code;
    new_dsi_clk = display_timings->dsi_clk / 2;
    if (new_dsi_clk < 40 || new_dsi_clk > 500) {
        return 0xFF; // ошибка, если частота вне допустимого диапазона
    }

    // Рассчитываем код в диапазоне от 40 до 500 MHz с шагом 5 MHz
    code = (unsigned char)(((new_dsi_clk - 40) / 5) + 0x08);
    return code;
}

static u8 get_dsi_lvds_divider(struct sn65dsi84 *ctx) {
    struct display_timings *display_timings = ctx->display_timings;
    u32 lvds_clk;
    unsigned int new_dsi_clk;

    unsigned int divider; 

    new_dsi_clk = display_timings->dsi_clk * 1000 / 2; // 423 000
    lvds_clk = (display_timings->dsi_clk * 2 * 1000/ 24); // 70500

    divider = new_dsi_clk / lvds_clk;

    dev_info(ctx->dev, "Divider is %u, hex 0x%X", divider, REG_RC_DSI_CLK_DSI_CLK_DIVIDER(divider-1));
    
    return (divider-1);
}

static u8 get_low_byte(u32 value) {
    return (value & 0xFF);
}

static u8 get_high_byte(u32 value) {
    return ((value >> 8) & 0xFF);
}

static int sn65dsi84_setup(struct sn65dsi84 *ctx) {
    /* Function to setup the brigde */
    struct display_timings *display_timings = ctx->display_timings;
    unsigned int pval;
	__le16 le16val;
	u16 val;
    const bool lvds_format_jeida = false;
    const bool lvds_dual_link = true;
    int ret;
    // TODO: choose JEIDA OR SWG format
    
    /* Clear reset, disable PLL */
    regmap_write(ctx->regmap, REG_RC_RESET, 0x00);
	regmap_write(ctx->regmap, REG_RC_PLL_EN, 0x00);

    /* Reference clock derived from DSI link clock. */
	regmap_write(ctx->regmap, REG_RC_LVDS_PLL,
		     REG_RC_LVDS_PLL_LVDS_CLK_RANGE(get_lvds_clk_range(ctx)) |
		     REG_RC_LVDS_PLL_HS_CLK_SRC_DPHY);

    /* Setup DSI CLK RANGE */
	regmap_write(ctx->regmap, REG_DSI_CLK,
		     REG_DSI_CLK_CHA_DSI_CLK_RANGE((get_dsi_clk_range(ctx))));

    /* get_dsi_lvds_divider(ctx); */
	regmap_write(ctx->regmap, REG_RC_DSI_CLK,
		     REG_RC_DSI_CLK_DSI_CLK_DIVIDER(get_dsi_lvds_divider(ctx)));

    /* Set number of DSI lanes and LVDS link config. */
    regmap_write(ctx->regmap, REG_DSI_LANE, 0x26);

    /* No equalization. */
    regmap_write(ctx->regmap, REG_DSI_EQ, 0x00);
	// /* Set up sync signal polarity. */

	val = (display_timings->de_active & 0xFF ? 0 : REG_LVDS_FMT_DE_NEG_POLARITY) | (display_timings->hsync_active & 0XFF ?
	       0 : REG_LVDS_FMT_HS_NEG_POLARITY) |
	      (display_timings->vsync_active & 0XFF ?
	       0 : REG_LVDS_FMT_VS_NEG_POLARITY);

	// /* Set up bits-per-pixel, 18bpp or 24bpp. */
    switch (display_timings->format)
    {
    case MIPI_DSI_FMT_RGB888:
        val |= REG_LVDS_FMT_CHA_24BPP_MODE;
        if (lvds_dual_link)
            val |= REG_LVDS_FMT_CHB_24BPP_MODE;
        /* code */
        break;
    case MIPI_DSI_FMT_RGB666:
        break;
    default:
        val |= REG_LVDS_FMT_CHA_24BPP_MODE;
        break;
    }
    
    if (display_timings->format == MIPI_DSI_FMT_RGB888) {
        dev_info(ctx->dev, "Format is RGB888");
    }

	// /* Set up LVDS format, JEIDA/Format 1 or SPWG/Format 2 */
	if (lvds_format_jeida) {
		val |= REG_LVDS_FMT_CHA_24BPP_FORMAT1;
		if (lvds_dual_link)
			val |= REG_LVDS_FMT_CHB_24BPP_FORMAT1;
	}

	// /* Set up LVDS output config (DSI84,DSI85) */
	if (!lvds_dual_link)
		val |= REG_LVDS_FMT_LVDS_LINK_CFG;

	regmap_write(ctx->regmap, REG_LVDS_FMT, val);
	regmap_write(ctx->regmap, REG_LVDS_VCOM, 0x00);
    regmap_write(ctx->regmap, REG_LVDS_LANE, 0x00);

	regmap_write(ctx->regmap, REG_LVDS_CM, 0x00);


    /* HACTIVE */
    regmap_write(ctx->regmap, REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW, get_low_byte(display_timings->hactive));
    regmap_write(ctx->regmap, REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH, get_high_byte(display_timings->hactive));


    /* VACTIVE */
    regmap_write(ctx->regmap, REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW, get_low_byte(display_timings->vactive));
    regmap_write(ctx->regmap, REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH, get_high_byte(display_timings->vactive));

    /* CHA SYNC DELAY WIDTH */
	le16val = cpu_to_le16(32 + 1);
	regmap_bulk_write(ctx->regmap, REG_VID_CHA_SYNC_DELAY_LOW, &le16val, 2);

    /* CHA HSYNC PULSE WIDTH */
    regmap_write(ctx->regmap, REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH, get_high_byte(display_timings->hsync_len / 2));
    regmap_write(ctx->regmap, REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW, get_low_byte(display_timings->hsync_len / 2));

    /* CHA VSYNC PULSE WIDTH */
    regmap_write(ctx->regmap, REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH, get_high_byte(display_timings->vsync_len));
    regmap_write(ctx->regmap, REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW, get_low_byte(display_timings->vsync_len));

    /* CHA HBP */
    regmap_write(ctx->regmap, REG_VID_CHA_HORIZONTAL_BACK_PORCH, display_timings->hback_porch / 2);

    /* CHA VBP */
    regmap_write(ctx->regmap, REG_VID_CHA_VERTICAL_BACK_PORCH, display_timings->vback_porch);

    /* CHA HFP */
    regmap_write(ctx->regmap, REG_VID_CHA_HORIZONTAL_FRONT_PORCH, display_timings->hfront_porch / 2);

    /* CHA VFP*/
    regmap_write(ctx->regmap, REG_VID_CHA_VERTICAL_FRONT_PORCH, display_timings->vfront_porch);

    /* ENABLE PLL */
	regmap_write(ctx->regmap, REG_RC_PLL_EN, REG_RC_PLL_EN_PLL_EN);
	usleep_range(3000, 4000);
	ret = regmap_read_poll_timeout(ctx->regmap, REG_RC_LVDS_PLL, pval,
				       pval & REG_RC_LVDS_PLL_PLL_EN_STAT,
				       1000, 100000);
	if (ret) {
		dev_err(ctx->dev, "failed to lock PLL, ret=%i\n", ret);
		/* On failure, disable PLL again and exit. */
		regmap_write(ctx->regmap, REG_RC_PLL_EN, 0x00);
		return -1;
	}

	/* Trigger reset after CSR register update. */
	regmap_write(ctx->regmap, REG_RC_RESET, REG_RC_RESET_SOFT_RESET);

	/* Clear all errors that got asserted during initialization. */
	regmap_read(ctx->regmap, REG_IRQ_STAT, &pval);
	regmap_write(ctx->regmap, REG_IRQ_STAT, pval);

    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%lX", REG_RC_LVDS_PLL, (REG_RC_LVDS_PLL_LVDS_CLK_RANGE(get_lvds_clk_range(ctx)) |
		     REG_RC_LVDS_PLL_HS_CLK_SRC_DPHY));
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_DSI_CLK, REG_DSI_CLK_CHA_DSI_CLK_RANGE((get_dsi_clk_range(ctx))));
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_RC_DSI_CLK, REG_RC_DSI_CLK_DSI_CLK_DIVIDER(get_dsi_lvds_divider(ctx)));
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_DSI_LANE, 0x26);
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_DSI_EQ, 0x00);
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_LVDS_FMT, val);
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_LVDS_VCOM, 0x00);
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_LVDS_LANE, 0x00);
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_LVDS_CM, 0x00);
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH, get_high_byte(display_timings->hactive));
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW, get_low_byte(display_timings->hactive));
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH, get_high_byte(display_timings->vactive));
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW, get_low_byte(display_timings->vactive));
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_VID_CHA_SYNC_DELAY_HIGH, 0x00);
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_VID_CHA_SYNC_DELAY_LOW, le16val);
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH, get_high_byte(display_timings->hsync_len / 2));
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW, get_low_byte(display_timings->hsync_len / 2));
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH, get_high_byte(display_timings->vsync_len));
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW, get_low_byte(display_timings->vsync_len));
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_VID_CHA_HORIZONTAL_BACK_PORCH, display_timings->hback_porch / 2);
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_VID_CHA_VERTICAL_BACK_PORCH, display_timings->vback_porch);
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_VID_CHA_HORIZONTAL_FRONT_PORCH, display_timings->hfront_porch / 2);
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%X", REG_VID_CHA_VERTICAL_FRONT_PORCH, display_timings->vfront_porch);
    dev_info(ctx->dev, "Reg: 0x%02X:, value: 0x%lX", REG_RC_PLL_EN, REG_RC_PLL_EN_PLL_EN);

    return 0;
}

/* Function for parse device tree 
This is an example of my dsi node in DTB
dsi0: dsi@fe060000 {
    compatible = "rockchip,rk3568-mipi-dsi";
    reg = <0x0 0xfe060000 0x0 0x10000>;
    interrupts = <GIC_SPI 68 IRQ_TYPE_LEVEL_HIGH>;
    clocks = <&cru PCLK_DSITX_0>, <&cru HCLK_VO>, <&video_phy0>;
    clock-names = "pclk", "hclk", "hs_clk";
    resets = <&cru SRST_P_DSITX_0>;
    reset-names = "apb";
    phys = <&video_phy0>;
    phy-names = "mipi_dphy";
    power-domains = <&power RK3568_PD_VO>;
    rockchip,grf = <&grf>;
    #address-cells = <1>;
    #size-cells = <0>;
    status = "disabled";


&dsi0 {
    status = "okay";
    rockchip,lane-rate = <846>;
    panel { 
        clock-frequency = <140912640>; 
        hactive = <1920>;              
        vactive = <1080>;              
        hback-porch = <60>;           
        hfront-porch = <62>;           
        hsync-len = <62>;              
        vback-porch = <24>;            
        vfront-porch = <6>;            
        vsync-len = <6>;               
        hsync-active = <0>;            
        vsync-active = <0>;            
        de-active = <1>;               
        pixelclk-active = <0>;   
    };
*/
static int sn65dsi84_parse_dt(struct sn65dsi84 *ctx, enum sn65dsi84_model model, struct display_timings *display_timings)
{
    struct device *dev = ctx->dev;
    struct device_node *dsi_node, *panel_node, *timing_node, *timings; 
    int ret;
    u32 format, lanes, dsi_clk, clock_frequency, hactive, vactive, hback_porch, hfront_porch, hsync_len, vback_porch, vfront_porch, vsync_len, hsync_active, vsync_active, de_active, pixelclk_active;


    // Find DSI node via compatible
    dsi_node = of_find_compatible_node(NULL, NULL, "rockchip,rk3568-mipi-dsi");
    if (!dsi_node) {
        dev_err(dev, "Cannot find DSI0 node by compatible property\n");
        return -ENODEV;
    }

    // Get DSI-clk
    ret = of_property_read_u32(dsi_node, "rockchip,lane-rate", &dsi_clk);
    if (ret) {
        dev_err(dev, "Cannot get dsi clock rate\n");
        of_node_put(dsi_node);
        return ret;
    }
    
    // Find panel in DSI node
    panel_node = of_get_child_by_name(dsi_node, "panel");
    if (!panel_node) {
        dev_err(dev, "Cannot find panel in DSI node\n");
        return -ENODEV;
    }

    // Find dsi bus format
    ret = of_property_read_u32(panel_node, "dsi,format", &format);
    if (ret) {
        dev_err(dev, "Cannot get dsi bus format\n");
        of_node_put(dsi_node);
        return ret;
    }

    switch (format) {
    case MIPI_DSI_FMT_RGB888:
        dev_info(dev, "DSI format is RGB888\n");
        break;
    case MIPI_DSI_FMT_RGB666:
        dev_info(dev, "DSI format is RGB666\n");
        break;
    case MIPI_DSI_FMT_RGB565:
        dev_info(dev, "DSI format is RGB565\n");
        break;
    default:
        dev_warn(dev, "Unknown DSI format: %u\n", format);
        break;
    }

    // Find count of used dsi lanes
    ret = of_property_read_u32(panel_node, "dsi,lanes", &lanes);
    if (ret) {
        dev_err(dev, "Cannot get count of used dsi lanes\n");
        of_node_put(dsi_node);
        return ret;
    }

    // Get timings node in panel node 
    timing_node = of_get_child_by_name(panel_node, "display-timings");
    if (!timing_node) {
        dev_err(dev, "Не удалось найти тайминги дисплея\n");
        of_node_put(timing_node);
        of_node_put(panel_node);
        of_node_put(dsi_node);
        return -EINVAL;
    }

    // Get timings array in timings node
    timings = of_get_child_by_name(timing_node, "timing0");
    if (!timing_node) {
        dev_err(dev, "Не удалось найти тайминги дисплея(параметры)\n");
        of_node_put(timings);
        of_node_put(timing_node);
        of_node_put(panel_node);
        of_node_put(dsi_node);
        return -EINVAL;
    }

    // Get clock-frequency
    ret = of_property_read_u32(timings, "clock-frequency", &clock_frequency);
    if (ret) {
        dev_err(dev, "Cannot get clock-frequency of panel\n");
        of_node_put(timings);
        of_node_put(timing_node);
        of_node_put(panel_node);
        of_node_put(dsi_node);
        return -EINVAL;
    }

    // Get hactive
    ret = of_property_read_u32(timings, "hactive", &hactive);
    if (ret) {
        dev_err(dev, "Cannot get hactive of panel\n");
        of_node_put(timings);
        of_node_put(timing_node);
        of_node_put(panel_node);
        of_node_put(dsi_node);
        return ret;
    }
    

    // Get vactive
    ret = of_property_read_u32(timings, "vactive", &vactive);
    if (ret) {
        dev_err(dev, "Cannot get vactive of panel\n");
        of_node_put(timings);
        of_node_put(timing_node);
        of_node_put(panel_node);
        of_node_put(dsi_node);
        return ret;
    }

    // Get HBP
    ret = of_property_read_u32(timings, "hback-porch", &hback_porch);
        if (ret) {
        dev_err(dev, "Cannot get hback-porch of panel\n");
        of_node_put(timings);
        of_node_put(timing_node);
        of_node_put(panel_node);
        of_node_put(dsi_node);
        return ret;
    }

    //Get HFP
    ret = of_property_read_u32(timings, "hfront-porch", &hfront_porch);
        if (ret) {
        dev_err(dev, "Cannot get hfront-porch of panel\n");
        of_node_put(timings);
        of_node_put(timing_node);
        of_node_put(panel_node);
        of_node_put(dsi_node);
        return ret;
    }

    // Get HSW
    ret = of_property_read_u32(timings, "hsync-len", &hsync_len);
        if (ret) {
        dev_err(dev, "Cannot get hsync-len of panel\n");
        of_node_put(timings);
        of_node_put(timing_node);
        of_node_put(panel_node);
        of_node_put(dsi_node);
        return ret;
    }

    // Get VBP
    ret = of_property_read_u32(timings, "vback-porch",&vback_porch);
        if (ret) {
        dev_err(dev, "Cannot get vback-porch of panel\n");
        of_node_put(timings);
        of_node_put(timing_node);
        of_node_put(panel_node);
        of_node_put(dsi_node);
        return ret;
    }

    // Get VFP
    ret = of_property_read_u32(timings, "vfront-porch", &vfront_porch);
        if (ret) {
        dev_err(dev, "Cannot get vfront-porch of panel\n");
        of_node_put(timings);
        of_node_put(timing_node);
        of_node_put(panel_node);
        of_node_put(dsi_node);
        return ret;
    }

    // Get VSW
    ret = of_property_read_u32(timings, "vsync-len", &vsync_len);
        if (ret) {
        dev_err(dev, "Cannot get vsync-len of panel\n");
        of_node_put(timings);
        of_node_put(timing_node);
        of_node_put(panel_node);
        of_node_put(dsi_node);
        return ret;
    }

    // Get DE flag(high/low)
    ret = of_property_read_u32(timings, "de-active", &de_active);
        if (ret) {
        dev_err(dev, "Cannot get vsync-len of panel\n");
        of_node_put(timings);
        of_node_put(timing_node);
        of_node_put(panel_node);
        of_node_put(dsi_node);
        return ret;
    }

    //Get HSYNC polarity
    ret = of_property_read_u32(timings, "hsync-active", &hsync_active);
        if (ret) {
        dev_err(dev, "Cannot get vsync-len of panel\n");
        of_node_put(timings);
        of_node_put(timing_node);
        of_node_put(panel_node);
        of_node_put(dsi_node);
        return ret;
    }

    // Get VSYNC polarity 
    ret = of_property_read_u32(timings, "vsync-active", &vsync_active);
        if (ret) {
        dev_err(dev, "Cannot get vsync-len of panel\n");
        of_node_put(timings);
        of_node_put(timing_node);
        of_node_put(panel_node);
        of_node_put(dsi_node);
        return ret;
    }

    // Get pixelclock polarity
    ret = of_property_read_u32(timings, "pixelclk-active", &pixelclk_active);
        if (ret) {
        dev_err(dev, "Cannot get vsync-len of panel\n");
        of_node_put(timings);
        of_node_put(timing_node);
        of_node_put(panel_node);
        of_node_put(dsi_node);
        return ret;
    }

    dev_info(dev, "Found panel: %p\nTimings is:\ndsi_clk: %u\nformat: %u\nlanes: %u\nclock-frequency: %u\nhactive: %u\nvactive: %u\nhback-porch: %u\nhfront-porch: %u\nhsync-len: %u\nvback-porch: %u\nvfront-porch: %u\nvsync-len: %u\n",panel_node, dsi_clk, format, lanes, clock_frequency, hactive, vactive, hback_porch, hfront_porch, hsync_len, vback_porch, vfront_porch,vsync_len);
    
    display_timings->clock_frequency = clock_frequency;
    display_timings->hactive = hactive;
    display_timings->vactive = vactive;
    display_timings->hback_porch = hback_porch;
    display_timings->hfront_porch = hfront_porch;
    display_timings->hsync_len = hsync_len;
    display_timings->vback_porch = vback_porch;
    display_timings->vfront_porch = vfront_porch;
    display_timings->vsync_len = vsync_len;
    display_timings->hsync_active = hsync_active;
    display_timings->vsync_active = vsync_active;
    display_timings->de_active = de_active;
    display_timings->pixelclk_active = pixelclk_active;
    display_timings->dsi_clk = dsi_clk;
    display_timings->format = format;
    display_timings->lanes = lanes;

    // Clear memory
    of_node_put(timings);
    of_node_put(timing_node);
    of_node_put(panel_node);
    of_node_put(dsi_node);
    return 0;

};

static int sn65dsi84_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	enum sn65dsi84_model model;
	struct sn65dsi84 *ctx;
	int ret;
    int enable_gpio;
    /* Its my numbers of backlight gpio. They r should be added in DTB*/
    int backlight_gpio = 110;
    int pwm_gpio = 19;

	printk(KERN_INFO "DSI driver: устройство инициализировано\n");

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		printk(KERN_ERR "DSI driver: не удалось выделить область памяти под устройство");
		return -ENOMEM;
	};
	ctx->dev = dev;

    ctx->display_timings = devm_kzalloc(dev, sizeof(struct display_timings), GFP_KERNEL);
    if (!ctx->display_timings) {
        printk(KERN_ERR "DSI driver: не удалось выделить память под display_timings");
        devm_kfree(dev, ctx);
        return -ENOMEM;
    }

	if (dev->of_node) {
		model = (enum sn65dsi84_model)(uintptr_t)
			of_device_get_match_data(dev);
		printk(KERN_INFO "DSI driver: Model from Device Tree: %d\n", model);
	} else {
		model = id->driver_data;
		printk(KERN_INFO "DSI driver: Model from driver: %d\n", model);
	}


    /*  
    Can use this if not planning to use gpio in system after enable 

	ctx->enable_gpio = devm_gpiod_get(ctx->dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->enable_gpio)) {
		printk(KERN_ERR "DSI driver: не удалось включить gpio");
		return PTR_ERR(ctx->enable_gpio);
	};
    */ 
    
    /* OR use this for export in linux and setup gpio as u want */
    enable_gpio = of_get_named_gpio(dev->of_node, "enable-gpios", 0);
    if (enable_gpio < 0) {
        dev_err(dev, "Failed to get number GPIO from Device Tree\n");
        //return enable_gpio;
        goto err_put_node;
    }
    dev_info(ctx->dev, "GPIO NUM IS %d", enable_gpio);

    // Request of GPIO
    ret = gpio_request(enable_gpio, "enable_gpio");
    if (ret) {
        dev_err(dev, "Failed to request GPIO\n");
        return ret;
    }

    // GPIO to out
    ret = gpio_direction_output(enable_gpio, 1);  // Устанавливаем высокий уровень
    if (ret) {
        dev_err(dev, "Failed to set GPIO direction\n");
        gpio_free(enable_gpio);  // Освобождаем GPIO
        return ret;
    }

    // GPIO to high level
    gpio_set_value(enable_gpio, 1);

    // Export GPIO to linux
    ret = gpio_export(enable_gpio, false);  // false is not pull-up resistors
    if (ret) {
        dev_err(ctx->dev, "Failed to export GPIO\n");
        gpio_free(enable_gpio);  // Free GPIO if cannot to export it
        return ret;
    }

	ret = sn65dsi84_parse_dt(ctx, model, ctx->display_timings);
	if (ret) {
		printk(KERN_ERR "DSI driver: Ошибки при парсинге device tree");
		goto err_put_node;
	};
	ctx->regmap = devm_regmap_init_i2c(client, &sn65dsi84_regmap_config);
	if (IS_ERR(ctx->regmap)) {
		ret = PTR_ERR(ctx->regmap);
		printk(KERN_ERR "DSI driver: Ошибка регмап для i2c");
		goto err_put_node;
	}


    /* SETUP THE BRIGDE */
    ret = sn65dsi84_setup(ctx);
    if (ret) {
        printk(KERN_ERR "DSI DRIVER: ошибка настройки панели");
        goto err_put_node;
    };
    
    ssleep(2);
    //==================================================================================================
    /*  ENABLE PWM */
    ret = gpio_request(pwm_gpio, NULL);
    if (ret) {
        dev_err(dev, "Failed to request PWM GPIO %d\n", pwm_gpio);
        return ret;
    }

    // GPIO to out
    ret = gpio_direction_output(pwm_gpio, 1);  // Устанавливаем высокий уровень
    if (ret) {
        dev_err(dev, "Failed to set PWM GPIO %d direction\n", pwm_gpio);
        gpio_free(pwm_gpio);  // Освобождаем GPIO
        return ret;
    }

    // GPIO to high level
    gpio_set_value(pwm_gpio, 1);

    // Export GPIO to linux
    ret = gpio_export(pwm_gpio, false);  // false is not pull-up resistors
    if (ret) {
        dev_err(ctx->dev, "Failed to export PWM GPIO %d\n", pwm_gpio);
        gpio_free(pwm_gpio);  // Free GPIO if cannot to export it
        return ret;
    }

    ssleep(2);
    
    // ===============================================================================================
    /* ENABLE 12V */
    ret = gpio_request(backlight_gpio, NULL);
    if (ret) {
        dev_err(dev, "Failed to request 12v GPIO %d\n", backlight_gpio);
        return ret;
    }

    // GPIO to out
    ret = gpio_direction_output(backlight_gpio, 1);  // Устанавливаем высокий уровень
    if (ret) {
        dev_err(dev, "Failed to set 12 V GPIO %d direction\n", backlight_gpio);
        gpio_free(backlight_gpio);  // Освобождаем GPIO
        return ret;
    }

    // GPIO to high level
    gpio_set_value(backlight_gpio, 1);

    // Export GPIO to linux
    ret = gpio_export(backlight_gpio, false);  // false is not pull-up resistors
    if (ret) {
        dev_err(ctx->dev, "Failed to export 12v GPIO %d\n", backlight_gpio);
        gpio_free(backlight_gpio);  // Free GPIO if cannot to export it
        return ret;
    }


	dev_set_drvdata(dev, ctx);
	i2c_set_clientdata(client, ctx);
    
	pr_info("driver dsi: probe ended");
	return 0;

err_put_node:
	of_node_put(ctx->host_node);
	return ret;
}

static int sn65dsi84_remove(struct i2c_client *client)
{
	struct sn65dsi84 *ctx = i2c_get_clientdata(client);

	printk(KERN_INFO "DSI driver: remove func started");
	of_node_put(ctx->host_node);

	return 0;
}

static struct i2c_device_id sn65dsi84_id[] = {
	{ "ti,sn65dsi84", MODEL_SN65DSI84 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sn65dsi84_id);

static const struct of_device_id sn65dsi84_match_table[] = {
	{ .compatible = "ti,sn65dsi84", .data = (void *)MODEL_sn65dsi84 },
	{},
};
MODULE_DEVICE_TABLE(of, sn65dsi84_match_table);

static struct i2c_driver sn65dsi84_driver = {
	.probe = sn65dsi84_probe,
	.remove = sn65dsi84_remove,
	.id_table = sn65dsi84_id,
	.driver = {
		.name = "sn65dsi84",
		.of_match_table = sn65dsi84_match_table,
	},
};

static int __init sn65dsi84_init(void)
{
    pr_info("DSI driver: init function called\n");
    return i2c_add_driver(&sn65dsi84_driver);
}

static void __exit sn65dsi84_exit(void)
{
    pr_info("DSI driver: exit function called\n");
    i2c_del_driver(&sn65dsi84_driver);
}

module_init(sn65dsi84_init);
module_exit(sn65dsi84_exit);
//module_i2c_driver(sn65dsi84_driver);

MODULE_AUTHOR("Kulichkov Vitaliy <shizgiz1337@gmail.com>");
MODULE_DESCRIPTION("TI SN65DSI84 DSI to LVDS bridge driver");
MODULE_LICENSE("GPL v2");