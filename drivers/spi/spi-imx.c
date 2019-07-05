/*
 * Copyright 2004-2007, 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2008 Juergen Beisert
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation
 * 51 Franklin Street, Fifth Floor
 * Boston, MA  02110-1301, USA.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/platform_data/dma-imx.h>
#include <linux/platform_data/spi-imx.h>

#define DRIVER_NAME "spi_imx"

#define MXC_CSPIRXDATA		0x00
#define MXC_CSPITXDATA		0x04
#define MXC_CSPICTRL		0x08
#define MXC_CSPIINT		0x0c
#define MXC_RESET		0x1c

/* generic defines to abstract from the different register layouts */
#define MXC_INT_RR	(1 << 0) /* Receive data ready interrupt */
#define MXC_INT_TE	(1 << 1) /* Transmit FIFO empty interrupt */
#define MXC_INT_TCEN    (1 << 7)   /* Transfer complete */

/* The maximum  bytes that a sdma BD can transfer.*/
#define MAX_SDMA_BD_BYTES  (1 << 15)

enum spi_imx_devtype {
	IMX1_CSPI,
	IMX21_CSPI,
	IMX27_CSPI,
	IMX31_CSPI,
	IMX35_CSPI,	/* CSPI on all i.mx except above */
	IMX51_ECSPI,	/* ECSPI on i.mx51 and later */
	IMX6UL_ECSPI,
};

struct spi_imx_data;

struct spi_imx_devtype_data {
	void (*intctrl)(struct spi_imx_data *, int);
	int (*config)(struct spi_device *);
	void (*trigger)(struct spi_imx_data *);
	int (*rx_available)(struct spi_imx_data *);
	void (*reset)(struct spi_imx_data *);
	enum spi_imx_devtype devtype;
};

struct spi_imx_data {
	struct spi_bitbang bitbang;
	struct device *dev;

	struct completion xfer_done;
	void __iomem *base;
	struct clk *clk_per;
	struct clk *clk_ipg;
	unsigned long spi_clk;
	unsigned int spi_bus_clk;

	unsigned int speed_hz;
	unsigned int bits_per_word;
	unsigned int len;
	unsigned int prev_width;
	unsigned int spi_drctl;

	unsigned int count;
	void (*tx)(struct spi_imx_data *);
	void (*rx)(struct spi_imx_data *);
	void *rx_buf;
	const void *tx_buf;
	unsigned int txfifo; /* number of words pushed in tx FIFO */

	/* DMA */
	unsigned int dma_finished;
	bool usedma;
	u32 wml;
	struct completion dma_rx_completion;
	struct completion dma_tx_completion;
	struct dma_slave_config rx_config;
	struct dma_slave_config tx_config;

	const struct spi_imx_devtype_data *devtype_data;
	int idle_state_provided;
	int idle_state;
	int current_state;
};

static inline int is_imx27_cspi(struct spi_imx_data *d)
{
	return d->devtype_data->devtype == IMX27_CSPI;
}

static inline int is_imx35_cspi(struct spi_imx_data *d)
{
	return d->devtype_data->devtype == IMX35_CSPI;
}

static inline int is_imx51_ecspi(struct spi_imx_data *d)
{
	return d->devtype_data->devtype == IMX51_ECSPI;
}

static inline int is_imx6ul_ecspi(struct spi_imx_data *d)
{
	return d->devtype_data->devtype == IMX6UL_ECSPI;
}

static inline unsigned spi_imx_get_fifosize(struct spi_imx_data *d)
{
	return (is_imx51_ecspi(d) || is_imx6ul_ecspi(d)) ? 64 : 8;
}

#define MXC_SPI_BUF_RX(type)						\
static void spi_imx_buf_rx_##type(struct spi_imx_data *spi_imx)		\
{									\
	unsigned int val = readl(spi_imx->base + MXC_CSPIRXDATA);	\
									\
	if (spi_imx->rx_buf) {						\
		*(type *)spi_imx->rx_buf = val;				\
		spi_imx->rx_buf += sizeof(type);			\
	}								\
}

#define MXC_SPI_BUF_TX(type)						\
static void spi_imx_buf_tx_##type(struct spi_imx_data *spi_imx)		\
{									\
	type val = 0;							\
									\
	if (spi_imx->tx_buf) {						\
		val = *(type *)spi_imx->tx_buf;				\
		spi_imx->tx_buf += sizeof(type);			\
	}								\
									\
	spi_imx->count -= sizeof(type);					\
									\
	writel(val, spi_imx->base + MXC_CSPITXDATA);			\
}

MXC_SPI_BUF_RX(u8)
MXC_SPI_BUF_TX(u8)
MXC_SPI_BUF_RX(u16)
MXC_SPI_BUF_TX(u16)
MXC_SPI_BUF_RX(u32)
MXC_SPI_BUF_TX(u32)

/* First entry is reserved, second entry is valid only if SDHC_SPIEN is set
 * (which is currently not the case in this driver)
 */
static int mxc_clkdivs[] = {0, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192,
	256, 384, 512, 768, 1024};

/* MX21, MX27 */
static unsigned int spi_imx_clkdiv_1(unsigned int fin,
		unsigned int fspi, unsigned int max, unsigned int *fres)
{
	int i;

	for (i = 2; i < max; i++)
		if (fspi * mxc_clkdivs[i] >= fin)
			break;

	*fres = fin / mxc_clkdivs[i];
	return i;
}

/* MX1, MX31, MX35, MX51 CSPI */
static unsigned int spi_imx_clkdiv_2(unsigned int fin,
		unsigned int fspi, unsigned int *fres)
{
	int i, div = 4;

	for (i = 0; i < 7; i++) {
		if (fspi * div >= fin)
			goto out;
		div <<= 1;
	}

out:
	*fres = fin / div;
	return i;
}

static int spi_imx_bytes_per_word(const int bits_per_word)
{
	return DIV_ROUND_UP(bits_per_word, BITS_PER_BYTE);
}

static bool spi_imx_can_dma(struct spi_master *master, struct spi_device *spi,
			 struct spi_transfer *transfer)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(master);
	unsigned int bytes_per_word;

	if (!master->dma_rx)
		return false;

	bytes_per_word = spi_imx_bytes_per_word(transfer->bits_per_word);
	if (transfer->len > spi_imx_get_fifosize(spi_imx) * bytes_per_word)
		return true;
	return false;
}

#define MX51_ECSPI_CTRL		0x08
#define MX51_ECSPI_CTRL_ENABLE		(1 <<  0)
#define MX51_ECSPI_CTRL_XCH		(1 <<  2)
#define MX51_ECSPI_CTRL_SMC		(1 << 3)
#define MX51_ECSPI_CTRL_MODE_MASK	(0xf << 4)
#define MX51_ECSPI_CTRL_DRCTL(drctl)	((drctl) << 16)
#define MX51_ECSPI_CTRL_POSTDIV_OFFSET	8
#define MX51_ECSPI_CTRL_PREDIV_OFFSET	12
#define MX51_ECSPI_CTRL_CS(cs)		(((cs) & 3) << 18)
#define MX51_ECSPI_CTRL_BL_OFFSET	20

#define MX51_ECSPI_CONFIG	0x0c
#define MX51_ECSPI_CONFIG_SCLKPHA(cs)	(1 << (((cs) & 3) +  0))
#define MX51_ECSPI_CONFIG_SCLKPOL(cs)	(1 << (((cs) & 3) +  4))
#define MX51_ECSPI_CONFIG_SBBCTRL(cs)	(1 << (((cs) & 3) +  8))
#define MX51_ECSPI_CONFIG_SSBPOL(cs)	(1 << (((cs) & 3) + 12))
#define MX51_ECSPI_CONFIG_SCLKCTL(cs)	(1 << (((cs) & 3) + 20))

#define MX51_ECSPI_INT		0x10
#define MX51_ECSPI_INT_TEEN		(1 <<  0)
#define MX51_ECSPI_INT_RREN		(1 <<  3)
#define MX51_ECSPI_INT_TCEN             (1 << 7)

#define MX51_ECSPI_DMA      0x14
#define MX51_ECSPI_DMA_TX_WML(wml)	((wml) & 0x3f)
#define MX51_ECSPI_DMA_RX_WML(wml)	(((wml) & 0x3f) << 16)
#define MX51_ECSPI_DMA_RXT_WML(wml)	(((wml) & 0x3f) << 24)

#define MX51_ECSPI_DMA_TEDEN		(1 << 7)
#define MX51_ECSPI_DMA_RXDEN		(1 << 23)
#define MX51_ECSPI_DMA_RXTDEN		(1 << 31)

#define MX51_ECSPI_STAT		0x18
#define MX51_ECSPI_STAT_RR		(1 <<  3)

#define MX51_ECSPI_PERIOD		0x1c

#define MX51_ECSPI_TESTREG	0x20
#define MX51_ECSPI_TESTREG_LBC	BIT(31)

/* MX51 eCSPI */
static unsigned int mx51_ecspi_clkdiv(struct spi_imx_data *spi_imx,
				      unsigned int fspi, unsigned int *fres)
{
	/*
	 * there are two 4-bit dividers, the pre-divider divides by
	 * $pre, the post-divider by 2^$post
	 */
	unsigned int pre, post;
	unsigned int fin = spi_imx->spi_clk;

	if (unlikely(fspi > fin))
		return 0;

	post = fls(fin) - fls(fspi);
	if (fin > fspi << post)
		post++;

	/* now we have: (fin <= fspi << post) with post being minimal */

	post = max(4U, post) - 4;
	if (unlikely(post > 0xf)) {
		dev_err(spi_imx->dev, "cannot set clock freq: %u (base freq: %u)\n",
				fspi, fin);
		return 0xff;
	}

	pre = DIV_ROUND_UP(fin, fspi << post) - 1;

	dev_dbg(spi_imx->dev, "%s: fin: %u, fspi: %u, post: %u, pre: %u\n",
			__func__, fin, fspi, post, pre);

	/* Resulting frequency for the SCLK line. */
	*fres = (fin / (pre + 1)) >> post;

	return (pre << MX51_ECSPI_CTRL_PREDIV_OFFSET) |
		(post << MX51_ECSPI_CTRL_POSTDIV_OFFSET);
}

static void mx51_ecspi_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned val = 0;

	if (enable & MXC_INT_TE)
		val |= MX51_ECSPI_INT_TEEN;

	if (enable & MXC_INT_RR)
		val |= MX51_ECSPI_INT_RREN;

	if (enable & MXC_INT_TCEN)
		val |= MX51_ECSPI_INT_TCEN;

	writel(val, spi_imx->base + MX51_ECSPI_INT);
}

static void mx51_ecspi_trigger(struct spi_imx_data *spi_imx)
{
	u32 reg = readl(spi_imx->base + MX51_ECSPI_CTRL);
	/*
	 * To workaround ERR008517, SDMA script need use XCH instead of SMC
	 * just like PIO mode and it fix on i.mx6ul
	 */
	if (!spi_imx->usedma)
		reg |= MX51_ECSPI_CTRL_XCH;
	else if (!spi_imx->dma_finished &&
		 spi_imx->devtype_data->devtype == IMX6UL_ECSPI)
		reg |= MX51_ECSPI_CTRL_SMC;
	else
		reg &= ~MX51_ECSPI_CTRL_SMC;
	writel(reg, spi_imx->base + MX51_ECSPI_CTRL);
}

static int mx51_ecspi_config(struct spi_device *spi)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	u32 ctrl = MX51_ECSPI_CTRL_ENABLE;
	u32 clk = spi_imx->speed_hz, delay, reg;
	u32 cfg = readl(spi_imx->base + MX51_ECSPI_CONFIG);
	u32 bits = spi_imx->bits_per_word;
	int tx_wml;

	/*
	 * The hardware seems to have a race condition when changing modes. The
	 * current assumption is that the selection of the channel arrives
	 * earlier in the hardware than the mode bits when they are written at
	 * the same time.
	 * So set master mode for all channels as we do not support slave mode.
	 */
	ctrl |= MX51_ECSPI_CTRL_MODE_MASK;

	/*
	 * Enable SPI_RDY handling (falling edge/level triggered).
	 */
	if (spi->mode & SPI_READY)
		ctrl |= MX51_ECSPI_CTRL_DRCTL(spi_imx->spi_drctl);

	/* set clock speed */
	ctrl |= mx51_ecspi_clkdiv(spi_imx, spi_imx->speed_hz, &clk);
	spi_imx->spi_bus_clk = clk;

	/* set chip select to use */
	ctrl |= MX51_ECSPI_CTRL_CS(spi->chip_select);

	if (bits == 32) {
		unsigned len = spi_imx->len / 4;

		while (len > 128) {
			if (len & 1)
				break;
			len >>= 1;
		}
		while (len > 128) {
			if (len % 3)
				break;
			len /= 3;
		}
		if ((len > 128) || !len)
			len = 1;
		bits = len << 5;
	}
	ctrl |= (bits - 1) << MX51_ECSPI_CTRL_BL_OFFSET;

	cfg |= MX51_ECSPI_CONFIG_SBBCTRL(spi->chip_select);

	if (spi->mode & SPI_CPHA)
		cfg |= MX51_ECSPI_CONFIG_SCLKPHA(spi->chip_select);
	else
		cfg &= ~MX51_ECSPI_CONFIG_SCLKPHA(spi->chip_select);

	if (spi->mode & SPI_CPOL) {
		cfg |= MX51_ECSPI_CONFIG_SCLKPOL(spi->chip_select);
		cfg |= MX51_ECSPI_CONFIG_SCLKCTL(spi->chip_select);
	} else {
		cfg &= ~MX51_ECSPI_CONFIG_SCLKPOL(spi->chip_select);
		cfg &= ~MX51_ECSPI_CONFIG_SCLKCTL(spi->chip_select);
	}
	if (spi->mode & SPI_CS_HIGH)
		cfg |= MX51_ECSPI_CONFIG_SSBPOL(spi->chip_select);
	else
		cfg &= ~MX51_ECSPI_CONFIG_SSBPOL(spi->chip_select);

	/* CTRL register always go first to bring out controller from reset */
	writel(ctrl, spi_imx->base + MX51_ECSPI_CTRL);

	reg = readl(spi_imx->base + MX51_ECSPI_TESTREG);
	if (spi->mode & SPI_LOOP)
		reg |= MX51_ECSPI_TESTREG_LBC;
	else
		reg &= ~MX51_ECSPI_TESTREG_LBC;
	writel(reg, spi_imx->base + MX51_ECSPI_TESTREG);

	writel(cfg, spi_imx->base + MX51_ECSPI_CONFIG);

	/*
	 * Wait until the changes in the configuration register CONFIGREG
	 * propagate into the hardware. It takes exactly one tick of the
	 * SCLK clock, but we will wait two SCLK clock just to be sure. The
	 * effect of the delay it takes for the hardware to apply changes
	 * is noticable if the SCLK clock run very slow. In such a case, if
	 * the polarity of SCLK should be inverted, the GPIO ChipSelect might
	 * be asserted before the SCLK polarity changes, which would disrupt
	 * the SPI communication as the device on the other end would consider
	 * the change of SCLK polarity as a clock tick already.
	 */
	delay = (2 * 1000000) / clk;
	if (likely(delay < 10))	/* SCLK is faster than 100 kHz */
		udelay(delay);
	else			/* SCLK is _very_ slow */
		usleep_range(delay, delay + 10);

	/*
	 * Configure the DMA register: setup the watermark
	 * and enable DMA request.
	 */

	/*
	 * work around for
	 * ERR009165 eCSPI: TXFIFO empty flag glitch can cause the current
	 * FIFO transfer to be sent twice
	 */
	tx_wml = is_imx6ul_ecspi(spi_imx) ?
		spi_imx_get_fifosize(spi_imx) - spi_imx->tx_config.dst_maxburst :
		0;

	writel(MX51_ECSPI_DMA_RX_WML(spi_imx->wml - 1) |
		MX51_ECSPI_DMA_TX_WML(tx_wml) |
		MX51_ECSPI_DMA_TEDEN | MX51_ECSPI_DMA_RXDEN,
		spi_imx->base + MX51_ECSPI_DMA);

	return 0;
}

static int mx51_ecspi_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MX51_ECSPI_STAT) & MX51_ECSPI_STAT_RR;
}

static void mx51_ecspi_reset(struct spi_imx_data *spi_imx)
{
	/* drain receive buffer */
	while (mx51_ecspi_rx_available(spi_imx))
		readl(spi_imx->base + MXC_CSPIRXDATA);
}

#define MX31_INTREG_TEEN	(1 << 0)
#define MX31_INTREG_RREN	(1 << 3)

#define MX31_CSPICTRL_ENABLE	(1 << 0)
#define MX31_CSPICTRL_MASTER	(1 << 1)
#define MX31_CSPICTRL_XCH	(1 << 2)
#define MX31_CSPICTRL_SMC	(1 << 3)
#define MX31_CSPICTRL_POL	(1 << 4)
#define MX31_CSPICTRL_PHA	(1 << 5)
#define MX31_CSPICTRL_SSCTL	(1 << 6)
#define MX31_CSPICTRL_SSPOL	(1 << 7)
#define MX31_CSPICTRL_BC_SHIFT	8
#define MX35_CSPICTRL_BL_SHIFT	20
#define MX31_CSPICTRL_CS_SHIFT	24
#define MX35_CSPICTRL_CS_SHIFT	12
#define MX31_CSPICTRL_DR_SHIFT	16

#define MX31_CSPI_DMAREG	0x10
#define MX31_DMAREG_RH_DEN	(1<<4)
#define MX31_DMAREG_TH_DEN	(1<<1)

#define MX31_CSPISTATUS		0x14
#define MX31_STATUS_RR		(1 << 3)

#define MX31_CSPI_TESTREG	0x1C
#define MX31_TEST_LBC		(1 << 14)

/* These functions also work for the i.MX35, but be aware that
 * the i.MX35 has a slightly different register layout for bits
 * we do not use here.
 */
static void mx31_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned int val = 0;

	if (enable & MXC_INT_TE)
		val |= MX31_INTREG_TEEN;
	if (enable & MXC_INT_RR)
		val |= MX31_INTREG_RREN;

	writel(val, spi_imx->base + MXC_CSPIINT);
}

static void mx31_trigger(struct spi_imx_data *spi_imx)
{
	unsigned int reg;

	reg = readl(spi_imx->base + MXC_CSPICTRL);
	reg |= MX31_CSPICTRL_XCH;
	writel(reg, spi_imx->base + MXC_CSPICTRL);
}

static int mx31_config(struct spi_device *spi)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	unsigned int reg = MX31_CSPICTRL_ENABLE | MX31_CSPICTRL_MASTER;
	unsigned int clk;

	reg |= spi_imx_clkdiv_2(spi_imx->spi_clk, spi_imx->speed_hz, &clk) <<
		MX31_CSPICTRL_DR_SHIFT;
	spi_imx->spi_bus_clk = clk;

	if (is_imx35_cspi(spi_imx)) {
		reg |= (spi_imx->bits_per_word - 1) << MX35_CSPICTRL_BL_SHIFT;
		reg |= MX31_CSPICTRL_SSCTL;
	} else {
		reg |= (spi_imx->bits_per_word - 1) << MX31_CSPICTRL_BC_SHIFT;
	}

	if (spi->mode & SPI_CPHA)
		reg |= MX31_CSPICTRL_PHA;
	if (spi->mode & SPI_CPOL)
		reg |= MX31_CSPICTRL_POL;
	if (spi->mode & SPI_CS_HIGH)
		reg |= MX31_CSPICTRL_SSPOL;
	if (spi->cs_gpio < 0)
		reg |= (spi->cs_gpio + 32) <<
			(is_imx35_cspi(spi_imx) ? MX35_CSPICTRL_CS_SHIFT :
						  MX31_CSPICTRL_CS_SHIFT);

	if (spi_imx->usedma)
		reg |= MX31_CSPICTRL_SMC;

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	reg = readl(spi_imx->base + MX31_CSPI_TESTREG);
	if (spi->mode & SPI_LOOP)
		reg |= MX31_TEST_LBC;
	else
		reg &= ~MX31_TEST_LBC;
	writel(reg, spi_imx->base + MX31_CSPI_TESTREG);

	if (spi_imx->usedma) {
		/* configure DMA requests when RXFIFO is half full and
		   when TXFIFO is half empty */
		writel(MX31_DMAREG_RH_DEN | MX31_DMAREG_TH_DEN,
			spi_imx->base + MX31_CSPI_DMAREG);
	}

	return 0;
}

static int mx31_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MX31_CSPISTATUS) & MX31_STATUS_RR;
}

static void mx31_reset(struct spi_imx_data *spi_imx)
{
	/* drain receive buffer */
	while (readl(spi_imx->base + MX31_CSPISTATUS) & MX31_STATUS_RR)
		readl(spi_imx->base + MXC_CSPIRXDATA);
}

#define MX21_INTREG_RR		(1 << 4)
#define MX21_INTREG_TEEN	(1 << 9)
#define MX21_INTREG_RREN	(1 << 13)

#define MX21_CSPICTRL_POL	(1 << 5)
#define MX21_CSPICTRL_PHA	(1 << 6)
#define MX21_CSPICTRL_SSPOL	(1 << 8)
#define MX21_CSPICTRL_XCH	(1 << 9)
#define MX21_CSPICTRL_ENABLE	(1 << 10)
#define MX21_CSPICTRL_MASTER	(1 << 11)
#define MX21_CSPICTRL_DR_SHIFT	14
#define MX21_CSPICTRL_CS_SHIFT	19

static void mx21_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned int val = 0;

	if (enable & MXC_INT_TE)
		val |= MX21_INTREG_TEEN;
	if (enable & MXC_INT_RR)
		val |= MX21_INTREG_RREN;

	writel(val, spi_imx->base + MXC_CSPIINT);
}

static void mx21_trigger(struct spi_imx_data *spi_imx)
{
	unsigned int reg;

	reg = readl(spi_imx->base + MXC_CSPICTRL);
	reg |= MX21_CSPICTRL_XCH;
	writel(reg, spi_imx->base + MXC_CSPICTRL);
}

static int mx21_config(struct spi_device *spi)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	unsigned int reg = MX21_CSPICTRL_ENABLE | MX21_CSPICTRL_MASTER;
	unsigned int max = is_imx27_cspi(spi_imx) ? 16 : 18;
	unsigned int clk;

	reg |= spi_imx_clkdiv_1(spi_imx->spi_clk, spi_imx->speed_hz, max, &clk)
		<< MX21_CSPICTRL_DR_SHIFT;
	spi_imx->spi_bus_clk = clk;

	reg |= spi_imx->bits_per_word - 1;

	if (spi->mode & SPI_CPHA)
		reg |= MX21_CSPICTRL_PHA;
	if (spi->mode & SPI_CPOL)
		reg |= MX21_CSPICTRL_POL;
	if (spi->mode & SPI_CS_HIGH)
		reg |= MX21_CSPICTRL_SSPOL;
	if (spi->cs_gpio < 0)
		reg |= (spi->cs_gpio + 32) << MX21_CSPICTRL_CS_SHIFT;

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int mx21_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MXC_CSPIINT) & MX21_INTREG_RR;
}

static void mx21_reset(struct spi_imx_data *spi_imx)
{
	writel(1, spi_imx->base + MXC_RESET);
}

#define MX1_INTREG_RR		(1 << 3)
#define MX1_INTREG_TEEN		(1 << 8)
#define MX1_INTREG_RREN		(1 << 11)

#define MX1_CSPICTRL_POL	(1 << 4)
#define MX1_CSPICTRL_PHA	(1 << 5)
#define MX1_CSPICTRL_XCH	(1 << 8)
#define MX1_CSPICTRL_ENABLE	(1 << 9)
#define MX1_CSPICTRL_MASTER	(1 << 10)
#define MX1_CSPICTRL_DR_SHIFT	13

static void mx1_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned int val = 0;

	if (enable & MXC_INT_TE)
		val |= MX1_INTREG_TEEN;
	if (enable & MXC_INT_RR)
		val |= MX1_INTREG_RREN;

	writel(val, spi_imx->base + MXC_CSPIINT);
}

static void mx1_trigger(struct spi_imx_data *spi_imx)
{
	unsigned int reg;

	reg = readl(spi_imx->base + MXC_CSPICTRL);
	reg |= MX1_CSPICTRL_XCH;
	writel(reg, spi_imx->base + MXC_CSPICTRL);
}

static int mx1_config(struct spi_device *spi)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	unsigned int reg = MX1_CSPICTRL_ENABLE | MX1_CSPICTRL_MASTER;
	unsigned int clk;

	reg |= spi_imx_clkdiv_2(spi_imx->spi_clk, spi_imx->speed_hz, &clk) <<
		MX1_CSPICTRL_DR_SHIFT;
	spi_imx->spi_bus_clk = clk;

	reg |= spi_imx->bits_per_word - 1;

	if (spi->mode & SPI_CPHA)
		reg |= MX1_CSPICTRL_PHA;
	if (spi->mode & SPI_CPOL)
		reg |= MX1_CSPICTRL_POL;

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int mx1_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MXC_CSPIINT) & MX1_INTREG_RR;
}

static void mx1_reset(struct spi_imx_data *spi_imx)
{
	writel(1, spi_imx->base + MXC_RESET);
}

static struct spi_imx_devtype_data imx1_cspi_devtype_data = {
	.intctrl = mx1_intctrl,
	.config = mx1_config,
	.trigger = mx1_trigger,
	.rx_available = mx1_rx_available,
	.reset = mx1_reset,
	.devtype = IMX1_CSPI,
};

static struct spi_imx_devtype_data imx21_cspi_devtype_data = {
	.intctrl = mx21_intctrl,
	.config = mx21_config,
	.trigger = mx21_trigger,
	.rx_available = mx21_rx_available,
	.reset = mx21_reset,
	.devtype = IMX21_CSPI,
};

static struct spi_imx_devtype_data imx27_cspi_devtype_data = {
	/* i.mx27 cspi shares the functions with i.mx21 one */
	.intctrl = mx21_intctrl,
	.config = mx21_config,
	.trigger = mx21_trigger,
	.rx_available = mx21_rx_available,
	.reset = mx21_reset,
	.devtype = IMX27_CSPI,
};

static struct spi_imx_devtype_data imx31_cspi_devtype_data = {
	.intctrl = mx31_intctrl,
	.config = mx31_config,
	.trigger = mx31_trigger,
	.rx_available = mx31_rx_available,
	.reset = mx31_reset,
	.devtype = IMX31_CSPI,
};

static struct spi_imx_devtype_data imx35_cspi_devtype_data = {
	/* i.mx35 and later cspi shares the functions with i.mx31 one */
	.intctrl = mx31_intctrl,
	.config = mx31_config,
	.trigger = mx31_trigger,
	.rx_available = mx31_rx_available,
	.reset = mx31_reset,
	.devtype = IMX35_CSPI,
};

static struct spi_imx_devtype_data imx51_ecspi_devtype_data = {
	.intctrl = mx51_ecspi_intctrl,
	.config = mx51_ecspi_config,
	.trigger = mx51_ecspi_trigger,
	.rx_available = mx51_ecspi_rx_available,
	.reset = mx51_ecspi_reset,
	.devtype = IMX51_ECSPI,
};

static struct spi_imx_devtype_data imx6ul_ecspi_devtype_data = {
	.intctrl = mx51_ecspi_intctrl,
	.config = mx51_ecspi_config,
	.trigger = mx51_ecspi_trigger,
	.rx_available = mx51_ecspi_rx_available,
	.reset = mx51_ecspi_reset,
	.devtype = IMX6UL_ECSPI,
};

static const struct platform_device_id spi_imx_devtype[] = {
	{
		.name = "imx1-cspi",
		.driver_data = (kernel_ulong_t) &imx1_cspi_devtype_data,
	}, {
		.name = "imx21-cspi",
		.driver_data = (kernel_ulong_t) &imx21_cspi_devtype_data,
	}, {
		.name = "imx27-cspi",
		.driver_data = (kernel_ulong_t) &imx27_cspi_devtype_data,
	}, {
		.name = "imx31-cspi",
		.driver_data = (kernel_ulong_t) &imx31_cspi_devtype_data,
	}, {
		.name = "imx35-cspi",
		.driver_data = (kernel_ulong_t) &imx35_cspi_devtype_data,
	}, {
		.name = "imx51-ecspi",
		.driver_data = (kernel_ulong_t) &imx51_ecspi_devtype_data,
	}, {
		.name = "imx6ul-ecspi",
		.driver_data = (kernel_ulong_t) &imx6ul_ecspi_devtype_data,
	}, {
		/* sentinel */
	}
};

static const struct of_device_id spi_imx_dt_ids[] = {
	{ .compatible = "fsl,imx1-cspi", .data = &imx1_cspi_devtype_data, },
	{ .compatible = "fsl,imx21-cspi", .data = &imx21_cspi_devtype_data, },
	{ .compatible = "fsl,imx27-cspi", .data = &imx27_cspi_devtype_data, },
	{ .compatible = "fsl,imx31-cspi", .data = &imx31_cspi_devtype_data, },
	{ .compatible = "fsl,imx35-cspi", .data = &imx35_cspi_devtype_data, },
	{ .compatible = "fsl,imx51-ecspi", .data = &imx51_ecspi_devtype_data, },
	{ .compatible = "fsl,imx6ul-ecspi", .data = &imx6ul_ecspi_devtype_data, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spi_imx_dt_ids);

static void spi_imx_chip_select(struct spi_imx_data *spi_imx, int is_active,
		int chip_select, int mode, int set_direction)
{
	int gpio;
	int active = is_active != BITBANG_CS_INACTIVE;
	int val;

	if (spi_imx->idle_state_provided) {
		int i;
		int change;

		if (!active)
			chip_select = spi_imx->idle_state;

		change = spi_imx->current_state ^ chip_select;
		spi_imx->current_state = chip_select;
		while (change) {
			i = active ? __ffs(change) : __fls(change);
			change &= ~(1 << i);
			gpio = spi_imx->bitbang.master->cs_gpios[i];
			val = (chip_select >> i) & 1;
			if (set_direction)
				gpio_direction_output(gpio, val);
			else
				gpio_set_value(gpio, val);
		}
	} else {
		gpio = spi_imx->bitbang.master->cs_gpios[chip_select];
		if (!gpio_is_valid(gpio))
			return;

		val = !(mode & SPI_CS_HIGH) ^ active;
		if (set_direction)
			gpio_direction_output(gpio, val);
		else
			gpio_set_value(gpio, val);
	}
}

static void spi_imx_chipselect(struct spi_device *spi, int is_active)
{
	spi_imx_chip_select(spi_master_get_devdata(spi->master), is_active, spi->chip_select, spi->mode, 0);
}

static void spi_imx_push(struct spi_imx_data *spi_imx)
{
	while (spi_imx->txfifo < spi_imx_get_fifosize(spi_imx)) {
		if (!spi_imx->count)
			break;
		spi_imx->tx(spi_imx);
		spi_imx->txfifo++;
	}

	spi_imx->devtype_data->trigger(spi_imx);
}

static irqreturn_t spi_imx_isr(int irq, void *dev_id)
{
	struct spi_imx_data *spi_imx = dev_id;

	while (spi_imx->devtype_data->rx_available(spi_imx)) {
		spi_imx->rx(spi_imx);
		spi_imx->txfifo--;
	}

	if (spi_imx->count) {
		spi_imx_push(spi_imx);
		return IRQ_HANDLED;
	}

	if (spi_imx->txfifo) {
		/* No data left to push, but still waiting for rx data,
		 * enable receive data available interrupt.
		 */
		spi_imx->devtype_data->intctrl(
				spi_imx, MXC_INT_RR);
		return IRQ_HANDLED;
	}

	spi_imx->devtype_data->intctrl(spi_imx, 0);
	complete(&spi_imx->xfer_done);

	return IRQ_HANDLED;
}

static int spi_imx_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	int bits_per_word;
	int width;
	int ret;
	int burst;

	if (!t)
		return 0;

	if (spi_imx_can_dma(spi_imx->bitbang.master, spi, t))
		spi_imx->usedma = 1;
	else
		spi_imx->usedma = 0;

	bits_per_word = t->bits_per_word;
	spi_imx->bits_per_word = bits_per_word;
	spi_imx->speed_hz  = t->speed_hz;
	spi_imx->len  = t->len;

	/* Initialize the functions for transfer */
	if (bits_per_word <= 8) {
		spi_imx->rx = spi_imx_buf_rx_u8;
		spi_imx->tx = spi_imx_buf_tx_u8;
		width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	} else if (bits_per_word <= 16) {
		spi_imx->rx = spi_imx_buf_rx_u16;
		spi_imx->tx = spi_imx_buf_tx_u16;
		width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	} else {
		spi_imx->rx = spi_imx_buf_rx_u32;
		spi_imx->tx = spi_imx_buf_tx_u32;
		width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	}
	/*
	 * Try to improve performance for workaround
	 * ERR009165 eCSPI: TXFIFO empty flag glitch can cause the current
	 * FIFO transfer to be sent twice
	 */
	burst = (is_imx6ul_ecspi(spi_imx) || (spi_imx->speed_hz > 40000000)) ?
			spi_imx->wml : spi_imx_get_fifosize(spi_imx);
	spi_imx->rx_config.src_addr_width = width;
	spi_imx->tx_config.dst_addr_width = width;

	if (spi_imx->usedma && (spi_imx->prev_width != width ||
			spi_imx->tx_config.dst_maxburst != burst)) {
		spi_imx->prev_width = width;
		spi_imx->tx_config.dst_maxburst = burst;

		ret = dmaengine_slave_config(spi_imx->bitbang.master->dma_rx,
						&spi_imx->rx_config);
		if (ret) {
			dev_err(spi_imx->dev, "error in RX dma configuration.\n");
			return ret;
		}

		ret = dmaengine_slave_config(spi_imx->bitbang.master->dma_tx,
						&spi_imx->tx_config);
		if (ret) {
			dev_err(spi_imx->dev, "error in TX dma configuration.\n");
			return ret;
		}
	}
	spi_imx->devtype_data->config(spi);
	return 0;
}

static void spi_imx_sdma_exit(struct spi_imx_data *spi_imx)
{
	struct spi_master *master = spi_imx->bitbang.master;

	if (master->dma_rx) {
		dma_release_channel(master->dma_rx);
		master->dma_rx = NULL;
	}

	if (master->dma_tx) {
		dma_release_channel(master->dma_tx);
		master->dma_tx = NULL;
	}
}

static int spi_imx_sdma_init(struct device *dev, struct spi_imx_data *spi_imx,
			     struct spi_master *master,
			     const struct resource *res)
{
	int ret;
	int fifosize = spi_imx_get_fifosize(spi_imx);

	spi_imx->wml = fifosize / 2;

	/* Prepare for TX DMA: */
	master->dma_tx = dma_request_slave_channel_reason(dev, "tx");
	if (IS_ERR(master->dma_tx)) {
		ret = PTR_ERR(master->dma_tx);
		master->dma_tx = NULL;
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_err(dev, "can't get the TX DMA channel, error %d!\n", ret);
		return ret;
	}

	spi_imx->tx_config.direction = DMA_MEM_TO_DEV;
	spi_imx->tx_config.dst_addr = res->start + MXC_CSPITXDATA;
	spi_imx->tx_config.dst_maxburst = (is_imx6ul_ecspi(spi_imx) ||
		(spi_imx->speed_hz > 40000000)) ? spi_imx->wml : fifosize;

	/* Prepare for RX : */
	master->dma_rx = dma_request_slave_channel_reason(dev, "rx");
	if (IS_ERR(master->dma_rx)) {
		ret = PTR_ERR(master->dma_rx);
		dev_dbg(dev, "can't get the RX DMA channel, error %d\n", ret);
		master->dma_rx = NULL;
		goto err;
	}

	spi_imx->rx_config.direction = DMA_DEV_TO_MEM;
	spi_imx->rx_config.src_addr = res->start + MXC_CSPIRXDATA;
	spi_imx->rx_config.src_maxburst = spi_imx->wml;
	init_completion(&spi_imx->dma_rx_completion);
	init_completion(&spi_imx->dma_tx_completion);
	master->can_dma = spi_imx_can_dma;
	master->max_dma_len = MAX_SDMA_BD_BYTES;
	spi_imx->bitbang.master->flags = SPI_MASTER_MUST_RX |
					 SPI_MASTER_MUST_TX;


	/*
	 * I have no idea why this is needed, but a dma error
	 * happens on 1st dma without it
	 */
	spi_imx->tx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	ret = dmaengine_slave_config(spi_imx->bitbang.master->dma_tx,
			&spi_imx->tx_config);
	if (ret) {
		dev_err(spi_imx->dev, "error(%d) in TX dma configuration.\n", ret);
		goto err;
	}
	return 0;
err:
	spi_imx_sdma_exit(spi_imx);
	return ret;
}

static void spi_imx_dma_rx_callback(void *cookie)
{
	struct spi_imx_data *spi_imx = (struct spi_imx_data *)cookie;

	complete(&spi_imx->dma_rx_completion);
}

static void spi_imx_dma_tx_callback(void *cookie)
{
	struct spi_imx_data *spi_imx = (struct spi_imx_data *)cookie;

	complete(&spi_imx->dma_tx_completion);
}

static void spi_imx_tail_pio_set(struct spi_imx_data *spi_imx, int left)
{

	switch (spi_imx->rx_config.src_addr_width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		spi_imx->rx = spi_imx_buf_rx_u8;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		spi_imx->rx = spi_imx_buf_rx_u16;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		spi_imx->rx = spi_imx_buf_rx_u32;
		break;
	default:
		spi_imx->rx = spi_imx_buf_rx_u8;
		break;
	}

	spi_imx->txfifo = left / spi_imx->rx_config.src_addr_width;
}

static int spi_imx_calculate_timeout(struct spi_imx_data *spi_imx, int size)
{
	uint64_t timeout = 0;

	/* Time with actual data transfer and CS change delay related to HW */
	timeout = ((8 + 4) * size);
	timeout *= MSEC_PER_SEC;
	do_div(timeout, spi_imx->spi_bus_clk);

	/* Add extra second for scheduler related activities */
	timeout += MSEC_PER_SEC;

	/* Double calculated timeout */
	return msecs_to_jiffies(timeout * 2);
}

static int spi_imx_dma_transfer(struct spi_imx_data *spi_imx,
				struct spi_transfer *transfer)
{
	struct dma_async_tx_descriptor *desc_tx, *desc_rx;
	int ret;
	unsigned long transfer_timeout;
	int left = 0;
	struct spi_master *master = spi_imx->bitbang.master;
	struct sg_table *tx = &transfer->tx_sg, *rx = &transfer->rx_sg;
	unsigned nents;
	int rem;
	u32 bpw;

	/* Trigger the cspi module. */
	spi_imx->dma_finished = 0;

	nents = rx->nents;
	bpw = spi_imx->tx_config.dst_addr_width;
	/*
	 * Adjust the transfer lenth of the last scattlist if there are
	 * some tail data, use PIO read to get the tail data since DMA
	 * sometimes miss the last tail interrupt.
	 */
	left = rem = transfer->len % (spi_imx->tx_config.dst_maxburst * bpw);
	while (rem) {
		struct scatterlist *sgl_last = &rx->sgl[nents - 1];

		if (sgl_last->length > rem) {
			sgl_last->length -= rem;
			break;
		}
		rem -= sgl_last->length;
		sgl_last->length = 0;
		nents--;
		rx->nents--;
	}
	/*
	 * The TX DMA setup starts the transfer, so make sure RX is configured
	 * before TX.
	 */
	desc_rx = dmaengine_prep_slave_sg(master->dma_rx,
				rx->sgl, rx->nents, DMA_DEV_TO_MEM,
				DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc_rx)
		return -EINVAL;

	desc_rx->callback = spi_imx_dma_rx_callback;
	desc_rx->callback_param = (void *)spi_imx;
	reinit_completion(&spi_imx->dma_rx_completion);

	desc_tx = dmaengine_prep_slave_sg(master->dma_tx,
				tx->sgl, tx->nents, DMA_MEM_TO_DEV,
				DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc_tx)
		return -EINVAL;

	desc_tx->callback = spi_imx_dma_tx_callback;
	desc_tx->callback_param = (void *)spi_imx;
	reinit_completion(&spi_imx->dma_tx_completion);

	dmaengine_submit(desc_rx);
	dma_async_issue_pending(master->dma_rx);

	dmaengine_submit(desc_tx);
	dma_async_issue_pending(master->dma_tx);
	spi_imx->devtype_data->trigger(spi_imx);

	transfer_timeout = spi_imx_calculate_timeout(spi_imx, transfer->len);

	/* Wait SDMA to finish the data transfer.*/
	ret = wait_for_completion_timeout(&spi_imx->dma_tx_completion,
					  transfer_timeout);
	if (!ret) {
		dev_err(spi_imx->dev, "I/O Error in DMA TX:%x %x %x %x %x\n",
				transfer->len,
				readl(spi_imx->base + MX51_ECSPI_STAT),
				readl(spi_imx->base + MX51_ECSPI_TESTREG),
				readl(spi_imx->base + MX51_ECSPI_DMA),
				readl(spi_imx->base + MX51_ECSPI_CTRL));
		dmaengine_terminate_all(master->dma_tx);
		dmaengine_terminate_all(master->dma_rx);
		return -ETIMEDOUT;
	}

	ret = wait_for_completion_timeout(&spi_imx->dma_rx_completion,
			transfer_timeout);
	if (!ret) {
		dev_err(spi_imx->dev, "I/O Error in DMA RX:%x %x %x\n",
			transfer->len,
			readl(spi_imx->base + MX51_ECSPI_STAT),
			readl(spi_imx->base + MX51_ECSPI_TESTREG));
		spi_imx->devtype_data->reset(spi_imx);
		dmaengine_terminate_all(master->dma_rx);
		return -ETIMEDOUT;
	}

	if (left) {
		/* read the tail data by PIO */
		dma_sync_sg_for_cpu(master->dma_rx->device->dev,
				    &rx->sgl[rx->nents - 1], 1,
				    DMA_FROM_DEVICE);
		spi_imx->rx_buf = transfer->rx_buf
					+ (transfer->len - left);
		spi_imx_tail_pio_set(spi_imx, left);
		reinit_completion(&spi_imx->xfer_done);

		spi_imx->devtype_data->intctrl(spi_imx, MXC_INT_TCEN);

		ret = wait_for_completion_timeout(&spi_imx->xfer_done,
					transfer_timeout);
		if (!ret) {
			dev_err(spi_imx->dev, "I/O Error in RX tail\n");
			return -ETIMEDOUT;
		}
	}

	spi_imx->dma_finished = 1;
	if (is_imx6ul_ecspi(spi_imx))
		spi_imx->devtype_data->trigger(spi_imx);

	return transfer->len;
}

static int spi_imx_pio_transfer(struct spi_device *spi,
				struct spi_transfer *transfer)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	unsigned long transfer_timeout;
	unsigned long timeout;

	spi_imx->tx_buf = transfer->tx_buf;
	spi_imx->rx_buf = transfer->rx_buf;
	spi_imx->count = transfer->len;
	spi_imx->txfifo = 0;

	reinit_completion(&spi_imx->xfer_done);

	spi_imx_push(spi_imx);

	spi_imx->devtype_data->intctrl(spi_imx, MXC_INT_TE);

	transfer_timeout = spi_imx_calculate_timeout(spi_imx, transfer->len);

	timeout = wait_for_completion_timeout(&spi_imx->xfer_done,
					      transfer_timeout);
	if (!timeout) {
		dev_err(&spi->dev, "I/O Error in PIO\n");
		spi_imx->devtype_data->reset(spi_imx);
		return -ETIMEDOUT;
	}

	return transfer->len;
}

static int spi_imx_transfer(struct spi_device *spi,
				struct spi_transfer *transfer)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);

	if (spi_imx->usedma)
		return spi_imx_dma_transfer(spi_imx, transfer);
	else
		return spi_imx_pio_transfer(spi, transfer);
}

static int spi_imx_setup(struct spi_device *spi)
{
	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n", __func__,
		 spi->mode, spi->bits_per_word, spi->max_speed_hz);
	spi_imx_chip_select(spi_master_get_devdata(spi->master),
			BITBANG_CS_INACTIVE, spi->chip_select, spi->mode, 1);
	return 0;
}

static void spi_imx_cleanup(struct spi_device *spi)
{
}

static int
spi_imx_prepare_message(struct spi_master *master, struct spi_message *msg)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(master);
	int ret;

	ret = clk_prepare_enable(spi_imx->clk_per);
	if (ret)
		return ret;

	ret = clk_prepare_enable(spi_imx->clk_ipg);
	if (ret) {
		clk_disable_unprepare(spi_imx->clk_per);
		return ret;
	}

	return 0;
}

static int
spi_imx_unprepare_message(struct spi_master *master, struct spi_message *msg)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(master);

	clk_disable_unprepare(spi_imx->clk_ipg);
	clk_disable_unprepare(spi_imx->clk_per);
	return 0;
}

static int get_default_speed(struct device_node *np)
{
	struct device_node *nc;
	int speed_hz;

	if (np) {
		for_each_available_child_of_node(np, nc) {
			int ret = of_property_read_u32(nc, "spi-max-frequency", &speed_hz);

			if (ret >= 0)
				return speed_hz;
		}
	}
	return 20000000;
}

static int spi_imx_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id =
			of_match_device(spi_imx_dt_ids, &pdev->dev);
	struct spi_imx_master *mxc_platform_info =
			dev_get_platdata(&pdev->dev);
	struct spi_master *master;
	struct spi_imx_data *spi_imx;
	struct resource *res;
	int i, ret, num_cs, irq, idle_state, spi_drctl;
	u32 speed_hz;

	if (!np && !mxc_platform_info) {
		dev_err(&pdev->dev, "can't get the platform data\n");
		return -EINVAL;
	}

	master = spi_alloc_master(&pdev->dev, sizeof(struct spi_imx_data));
	if (!master)
		return -ENOMEM;

	ret = of_property_read_u32(np, "fsl,spi-rdy-drctl", &spi_drctl);
	if ((ret < 0) || (spi_drctl >= 0x3)) {
		/* '11' is reserved */
		spi_drctl = 0;
	}
	speed_hz = get_default_speed(np);

	platform_set_drvdata(pdev, master);

	master->bits_per_word_mask = SPI_BPW_RANGE_MASK(1, 32);
	master->bus_num = pdev->id;

	spi_imx = spi_master_get_devdata(master);
	spi_imx->bitbang.master = master;
	spi_imx->dev = &pdev->dev;
	num_cs = of_gpio_named_count(np, "cs-gpios");
	spi_imx->speed_hz = speed_hz;

	ret = of_property_read_u32(np, "idle-state", &idle_state);
	if (ret >= 0) {
		spi_imx->idle_state = idle_state;
		spi_imx->idle_state_provided = 1;
		if (idle_state >= (1 << num_cs)) {
			dev_err(&pdev->dev, "idle-state(0x%x) > 0x%x, num_cs=%d\n",
				idle_state, 1 << num_cs, num_cs);
			ret = -EINVAL;
			goto out_master_put;
		}
		master->num_chipselect = 1 << num_cs;
	}

	spi_imx->devtype_data = of_id ? of_id->data :
		(struct spi_imx_devtype_data *)pdev->id_entry->driver_data;

	if (mxc_platform_info) {
		master->num_chipselect = mxc_platform_info->num_chipselect;
		master->cs_gpios = devm_kzalloc(&master->dev,
			sizeof(int) * master->num_chipselect, GFP_KERNEL);
		if (!master->cs_gpios)
			return -ENOMEM;

		for (i = 0; i < master->num_chipselect; i++)
			master->cs_gpios[i] = mxc_platform_info->chipselect[i];
 	}

	spi_imx->bitbang.chipselect = spi_imx_chipselect;
	spi_imx->bitbang.setup_transfer = spi_imx_setupxfer;
	spi_imx->bitbang.txrx_bufs = spi_imx_transfer;
	spi_imx->bitbang.master->setup = spi_imx_setup;
	spi_imx->bitbang.master->cleanup = spi_imx_cleanup;
	spi_imx->bitbang.master->prepare_message = spi_imx_prepare_message;
	spi_imx->bitbang.master->unprepare_message = spi_imx_unprepare_message;
	spi_imx->bitbang.master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	if (is_imx35_cspi(spi_imx) || is_imx51_ecspi(spi_imx))
		spi_imx->bitbang.master->mode_bits |= SPI_LOOP | SPI_READY;

	spi_imx->spi_drctl = spi_drctl;

	init_completion(&spi_imx->xfer_done);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	spi_imx->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(spi_imx->base)) {
		ret = PTR_ERR(spi_imx->base);
		goto out_master_put;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto out_master_put;
	}

	ret = devm_request_irq(&pdev->dev, irq, spi_imx_isr, 0,
			       dev_name(&pdev->dev), spi_imx);
	if (ret) {
		dev_err(&pdev->dev, "can't get irq%d: %d\n", irq, ret);
		goto out_master_put;
	}

	spi_imx->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(spi_imx->clk_ipg)) {
		ret = PTR_ERR(spi_imx->clk_ipg);
		goto out_master_put;
	}

	spi_imx->clk_per = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(spi_imx->clk_per)) {
		ret = PTR_ERR(spi_imx->clk_per);
		goto out_master_put;
	}

	ret = clk_prepare_enable(spi_imx->clk_per);
	if (ret)
		goto out_master_put;

	ret = clk_prepare_enable(spi_imx->clk_ipg);
	if (ret)
		goto out_put_per;

	spi_imx->spi_clk = clk_get_rate(spi_imx->clk_per);
	/*
	 * Only validated on i.mx35 and i.mx6 now, can remove the constraint
	 * if validated on other chips.
	 */
	if (is_imx35_cspi(spi_imx) || is_imx51_ecspi(spi_imx) ||
			is_imx6ul_ecspi(spi_imx)) {
		ret = spi_imx_sdma_init(&pdev->dev, spi_imx, master, res);
		if (ret == -EPROBE_DEFER)
			goto out_clk_put;

		if (ret < 0)
			dev_err(&pdev->dev, "dma setup error %d, use pio\n",
				ret);
	}

	spi_imx->devtype_data->reset(spi_imx);

	spi_imx->devtype_data->intctrl(spi_imx, 0);

	master->dev.of_node = pdev->dev.of_node;
	ret = spi_bitbang_start(&spi_imx->bitbang);
	if (ret) {
		dev_err(&pdev->dev, "bitbang start failed with %d\n", ret);
		goto out_clk_put;
	}

	if (!master->cs_gpios) {
		dev_err(&pdev->dev, "No CS GPIOs available\n");
		ret = -EINVAL;
		goto out_clk_put;
	}

	for (i = 0; i < master->num_chipselect; i++) {
		if (!gpio_is_valid(master->cs_gpios[i]))
			continue;

		ret = devm_gpio_request(&pdev->dev, master->cs_gpios[i],
					DRIVER_NAME);
		if (ret) {
			dev_err(&pdev->dev, "Can't get CS GPIO %i\n",
				master->cs_gpios[i]);
			goto out_clk_put;
		}
	}
	if (spi_imx->idle_state_provided) {
		spi_imx->current_state = ~spi_imx->idle_state &
			((1 << num_cs) - 1);
		spi_imx_chip_select(spi_imx, BITBANG_CS_INACTIVE, 0, 0, 1);
	}

	dev_info(&pdev->dev, "probed\n");

	clk_disable_unprepare(spi_imx->clk_ipg);
	clk_disable_unprepare(spi_imx->clk_per);
	return ret;

out_clk_put:
	clk_disable_unprepare(spi_imx->clk_ipg);
out_put_per:
	clk_disable_unprepare(spi_imx->clk_per);
out_master_put:
	spi_master_put(master);

	return ret;
}

static int spi_imx_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct spi_imx_data *spi_imx = spi_master_get_devdata(master);

	spi_bitbang_stop(&spi_imx->bitbang);

	writel(0, spi_imx->base + MXC_CSPICTRL);
	clk_unprepare(spi_imx->clk_ipg);
	clk_unprepare(spi_imx->clk_per);
	spi_imx_sdma_exit(spi_imx);
	spi_master_put(master);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int spi_imx_suspend(struct device *dev)
{
	pinctrl_pm_select_sleep_state(dev);
	return 0;
}

static int spi_imx_resume(struct device *dev)
{
	pinctrl_pm_select_default_state(dev);
	return 0;
}

static SIMPLE_DEV_PM_OPS(imx_spi_pm, spi_imx_suspend, spi_imx_resume);
#define IMX_SPI_PM       (&imx_spi_pm)
#else
#define IMX_SPI_PM       NULL
#endif

static struct platform_driver spi_imx_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = spi_imx_dt_ids,
		   .pm = IMX_SPI_PM,
	},
	.id_table = spi_imx_devtype,
	.probe = spi_imx_probe,
	.remove = spi_imx_remove,
};
module_platform_driver(spi_imx_driver);

MODULE_DESCRIPTION("SPI Master Controller driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);