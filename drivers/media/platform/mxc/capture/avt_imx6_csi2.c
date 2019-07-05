/*
 * Copyright (C) 2019 Allied Vision Technologies , Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/proc_fs.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <linux/mipi_csi2.h>
#include <media/v4l2-chip-ident.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"
#include <linux/platform_data/i2c-imx.h>
#include <linux/byteorder/little_endian.h>
#include <linux/lcm.h>
#include <linux/math64.h>
#include "avt_imx6_csi2.h"
#include "avt_imx6_csi2_local.h"
#include "avt_debug.h"

#ifdef WANDBOARD_IMX6
#pragma message("WANDBOARD_IMX6")
#include "../../../../../drivers/mxc/mipi/mxc_mipi_csi2.h"
#endif

#define __FILENAME__ 		"avt_imx6_csi2.c"

static int debug;
MODULE_PARM_DESC(debug, "Debug");
module_param(debug, int, 0600);/* S_IRUGO */

#define BCRM_WAIT_HANDSHAKE_TIMEOUT	3000
#define DEFAULT_FPS 			30
#define CRC32MASKREV			0xEDB88320u
#define UINT32_MAXIMUM			0xffffffff

#define AV_CAM_XCLK_MIN			IMX6_CSI_HOST_CLK_MIN_FREQ_HZ
#define AV_CAM_XCLK_MAX			CSI_HOST_CLK_MAX_FREQ_HZ_3L

#define IO_LIMIT			1024

static int lanes_auto_conf;
static int clk_auto_conf;
static int avail_pixformats;
static uint32_t clk;
static bool is_bcrm_write_handshake_available = false;	

/*!
 * Maintains the information on the current state of the sensor.
 */
static struct sensor_data av_cam_data;
static struct avt_fmtdesc fmtdesc_avt[TOTAL_SUPP_PIXFORMATS];
static int av_cam_i2c_clock;
static int av_cam_probe(struct i2c_client *adapter, const struct i2c_device_id *device_id);
static int av_cam_remove(struct i2c_client *client);
static uint64_t wait_for_bcrm_write_handshake(uint64_t timeout_ms);
static bool check_bcrm_write_byte_result(u16 reg, u8 val);
static bool check_bcrm_write_reg_result(struct v4l2_i2c *i2c_reg);
static s32 i2c_bcrm_read_byte_reg(u16 reg, u8 *val);
static s32 i2c_bcrm_write_byte_reg(u16 reg, u8 val);
static int i2c_bcrm_write_reg(struct v4l2_i2c *i2c_reg);
static int i2c_bcrm_write_reg_ctrl(struct v4l2_control *vc, unsigned int reg, int length);
static int i2c_bcrm_write_reg_ctrl_ext(struct v4l2_ext_control *vc, unsigned int reg, int length);
static ssize_t i2c_generic_read_buffer(__u32 reg, __u32 reg_size, __u32 count, char *buffer);
static ssize_t i2c_generic_read(struct v4l2_i2c *i2c_reg);
static int set_bayer_format(__u8 value);
static int bcrm_version_check(void);
static void bcrm_get_device_fw_version(void);
static void bcrm_get_write_handshake_availibility(void);
static int gcprm_version_check(void);
static int read_cci_registers(void);
static int read_gencp_registers(void);
static uint32_t crc32(const void *pSrc, uint32_t nByteCount, const uint32_t *pnStartCrcValue);
static void swapbytes(void *_object, size_t _size);
static int convert_bcrm_to_v4l2(struct bcrm_to_v4l2 *bcrmv4l2, int conversion, bool abs);
static uint32_t mipi_csi2_datatye = MIPI_DT_RGB888; /* RGB888 mode default. */
static __s32 convert_s_ctrl(__s32 val, __s32 min, __s32 max, __s32 step);
static __s64 convert_s_ext_ctrl(__s64 val, __s64 min, __s64 max, __s64 step);
static int ioctl_send_command(struct v4l2_int_device *s, struct v4l2_send_command_control *vc);
static int supported_pixformat(void);
static int ioctl_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qctrl);
static int ioctl_queryctrl_new(struct v4l2_int_device *s, struct v4l2_queryctrl *qctrl);
static int ioctl_ext_queryctrl(struct v4l2_int_device *s, struct v4l2_query_ext_ctrl *q_ext_qctrl);
static void bcrm_dump(void);
static void dump_bcrm_reg_8(u16 nOffset, const char *pRegName);
static void dump_bcrm_reg_32(u16 nOffset, const char *pRegName);
static void dump_bcrm_reg_64(u16 nOffset, const char *pRegName);
static uint8_t compute_dphy_clock_reg(uint32_t camera_clock_hz);
static void dump_dphy_state_reg(uint32_t reg);
static void dump_dphy_errstate1_reg(uint32_t reg);
static void dump_dphy_errstate2_reg(uint32_t reg);
static uint8_t get_bcrm_reg_access_mode(uint16_t reg);
static int ioctl_try_fmt(struct v4l2_int_device *s, struct v4l2_format *tryfmt);

static const struct i2c_device_id av_cam_id[] = {
	{AV_QUERYCAP_CAM_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, av_cam_id);

static struct i2c_driver av_cam_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = AV_QUERYCAP_CAM_NAME,
		  },
	.probe  = av_cam_probe,
	.remove = av_cam_remove,
	.id_table = av_cam_id,
};

enum av_cam_mode {
	av_cam_mode = 0,
	av_cam_mode_INIT = 0xff, /*only for sensor init*/
};

struct av_cam_mode_info {
	const char *name;
	enum av_cam_mode mode;
	u32 width;
	u32 height;
	u32 fps;
	u32 min_width;
	u32 min_height;
	u32 max_width;
	u32 max_height;
	u32 step_width;
	u32 step_height;
	__u32 flags;
	__u32 pixelformat;
};

/* list of image formats supported by Allied Vision's camera sensor, refer to BCRM doc */
static const struct v4l2_fmtdesc av_cam_formats[] = {
/* 1 */
	{
		.description	= "YUV422 8bit",
		.pixelformat	= V4L2_PIX_FMT_UYVY,
		.flags		= MIPI_DT_YUV422
	},
/* 2 */
	{
		.description	= "RGB888 (RGB24)",
		.pixelformat	= V4L2_PIX_FMT_RGB24,
		.flags		= MIPI_DT_RGB888
	},
/* 3 */
	{
		.description	= "RGB888 (BGR24)",
		.pixelformat	= V4L2_PIX_FMT_BGR24,
		.flags		= MIPI_DT_RGB888
	},
/* 4 */
	{
		.description	= "RGB565 (RGB16)",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
		.flags		= MIPI_DT_RGB565
	},
/* 5 */
	{
		.description	= "RAW8 (GREY)",
		.pixelformat	= V4L2_PIX_FMT_GREY,
		.flags		= MIPI_DT_RAW8
	},
/* 6 */
	{
		.description	= "RAW8 (SBGGR8)",
		.pixelformat	= V4L2_PIX_FMT_SBGGR8,
		.flags		= MIPI_DT_RAW8
	},
/* 7 */
	{
		.description	= "RAW8 (SGBRG8)",
		.pixelformat	= V4L2_PIX_FMT_SGBRG8,
		.flags		= MIPI_DT_RAW8
	},
/* 8 */
	{
		.description	= "RAW8 (SGRBG8)",
		.pixelformat	= V4L2_PIX_FMT_SGRBG8,
		.flags		= MIPI_DT_RAW8
	},
/* 9 */
	{
		.description	= "RAW8 (SRGGB8)",
		.pixelformat	= V4L2_PIX_FMT_SRGGB8,
		.flags		= MIPI_DT_RAW8
	},
/* 10 */
	{
		.description	= "RAW10 (GREY)",
		.pixelformat	= V4L2_PIX_FMT_Y10P,
		.flags		= MIPI_DT_RAW10
	},
/* 11 */
	{
		.description	= "RAW10 (SBGGR10P)",
		.pixelformat	= V4L2_PIX_FMT_SBGGR10P,
		.flags		= MIPI_DT_RAW10
	},
/* 12 */
	{
		.description	= "RAW10 (SGBRG10P)",
		.pixelformat	= V4L2_PIX_FMT_SGBRG10P,
		.flags		= MIPI_DT_RAW10
	},
/* 13 */
	{
		.description	= "RAW10 (SGRBG10P)",
		.pixelformat	= V4L2_PIX_FMT_SGRBG10P,
		.flags		= MIPI_DT_RAW10
	},
/* 14 */
	{
		.description	= "RAW10 (SRGGB10P)",
		.pixelformat	= V4L2_PIX_FMT_SRGGB10P,
		.flags		= MIPI_DT_RAW10
	},
/* 15 */
	{
		.description	= "RAW12 (GREY)",
		.pixelformat	= V4L2_PIX_FMT_Y12P,
		.flags		= MIPI_DT_RAW12
	},
/* 16 */
	{
		.description	= "RAW12 (SBGGR12P)",
		.pixelformat	= V4L2_PIX_FMT_SBGGR12P,
		.flags		= MIPI_DT_RAW12
	},
/* 17 */
	{
		.description	= "RAW12 (SGBRG12P)",
		.pixelformat	= V4L2_PIX_FMT_SGBRG12P,
		.flags		= MIPI_DT_RAW12
	},
/* 18 */
	{
		.description	= "RAW12 (SGRBG12P)",
		.pixelformat	= V4L2_PIX_FMT_SGRBG12P,
		.flags		= MIPI_DT_RAW12
	},
/* 19 */
	{
		.description	= "RAW12 (SRGGB12P)",
		.pixelformat	= V4L2_PIX_FMT_SRGGB12P,
		.flags		= MIPI_DT_RAW12
	},

/* 20 */
/*
	{
		.description	= "JPEG",
		.pixelformat	= V4L2_PIX_FMT_JPEG,
		.flags		= MIPI_DT_JPEG
	},
*/
};

/* Must be sorted from low to high control ID! */
struct avt_ctrl_mapping_t avt_ctrl_mappings[] = {
	{
		.id 			= V4L2_CID_BRIGHTNESS,
		.attr			= AV_ATTR_BRIGHTNESS,
		.min_offset 		= BCRM_BLACK_LEVEL_MIN_32R,
		.max_offset 		= BCRM_BLACK_LEVEL_MAX_32R,
		.reg_offset		= BCRM_BLACK_LEVEL_32RW,
		.step_offset		= BCRM_BLACK_LEVEL_INC_32R,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_32,
		.type 			= V4L2_CTRL_TYPE_INTEGER,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{
		.id 			= V4L2_CID_CONTRAST,
		.attr			= AV_ATTR_CONTRAST,
		.min_offset 		= BCRM_CONTRAST_VALUE_MIN_32R,
		.max_offset 		= BCRM_CONTRAST_VALUE_MAX_32R,
		.reg_offset		= BCRM_CONTRAST_VALUE_32RW,
		.step_offset		= BCRM_CONTRAST_VALUE_INC_32R,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_32,
		.type 			= V4L2_CTRL_TYPE_INTEGER,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{
		.id 			= V4L2_CID_SATURATION,
		.attr			= AV_ATTR_SATURATION,
		.min_offset 		= BCRM_SATURATION_MIN_32R,
		.max_offset 		= BCRM_SATURATION_MAX_32R,
		.reg_offset		= BCRM_SATURATION_32RW,
		.step_offset		= BCRM_SATURATION_INC_32R,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_32,
		.type 			= V4L2_CTRL_TYPE_INTEGER,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{
		.id 			= V4L2_CID_HUE,
		.attr			= AV_ATTR_HUE,
		.min_offset 		= BCRM_HUE_MIN_32R,
		.max_offset 		= BCRM_HUE_MAX_32R,
		.reg_offset		= BCRM_HUE_32RW,
		.step_offset		= BCRM_HUE_INC_32R,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_32,
		.type 			= V4L2_CTRL_TYPE_INTEGER,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{
		.id 			= V4L2_CID_AUTO_WHITE_BALANCE,
		.attr			= AV_ATTR_WHITEBALANCE_AUTO,
		.reg_offset		= BCRM_WHITE_BALANCE_AUTO_8RW,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_8,
		.type 			= V4L2_CTRL_TYPE_BOOLEAN,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{
		.id 			= V4L2_CID_DO_WHITE_BALANCE,
		.attr			= AV_ATTR_WHITEBALANCE,
		.reg_offset		= BCRM_WHITE_BALANCE_AUTO_8RW,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_8,
		.type 			= V4L2_CTRL_TYPE_BUTTON,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{
		.id 			= V4L2_CID_RED_BALANCE,
		.attr			= AV_ATTR_RED_BALANCE,
		.min_offset 		= BCRM_RED_BALANCE_RATIO_MIN_64R,
		.max_offset 		= BCRM_RED_BALANCE_RATIO_MAX_64R,
		.reg_offset		= BCRM_RED_BALANCE_RATIO_64RW,
		.step_offset		= BCRM_RED_BALANCE_RATIO_INC_64R,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_64,
		.type 			= V4L2_CTRL_TYPE_INTEGER64,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{
		.id 			= V4L2_CID_BLUE_BALANCE,
		.attr			= AV_ATTR_BLUE_BALANCE,
		.min_offset 		= BCRM_BLUE_BALANCE_RATIO_MIN_64R,
		.max_offset 		= BCRM_BLUE_BALANCE_RATIO_MAX_64R,
		.reg_offset		= BCRM_BLUE_BALANCE_RATIO_64RW,
		.step_offset		= BCRM_BLUE_BALANCE_RATIO_INC_64R,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_64,
		.type 			= V4L2_CTRL_TYPE_INTEGER64,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{
		.id 			= V4L2_CID_GAMMA,
		.attr			= AV_ATTR_GAMMA,
		.min_offset 		= BCRM_GAMMA_MIN_64R,
		.max_offset 		= BCRM_GAMMA_MAX_64R,
		.reg_offset		= BCRM_GAMMA_64RW,
		.step_offset		= BCRM_GAMMA_INC_64R,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_64,
		.type 			= V4L2_CTRL_TYPE_INTEGER64,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{
		.id 			= V4L2_CID_EXPOSURE,
		.attr			= AV_ATTR_EXPOSURE,
		.min_offset 		= BCRM_EXPOSURE_TIME_MIN_64R,
		.max_offset 		= BCRM_EXPOSURE_TIME_MAX_64R,
		.reg_offset		= BCRM_EXPOSURE_TIME_64RW,
		.step_offset		= BCRM_EXPOSURE_TIME_INC_64R,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_64,
		.type 			= V4L2_CTRL_TYPE_INTEGER64,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{
		.id 			= V4L2_CID_AUTOGAIN,
		.attr			= AV_ATTR_AUTOGAIN,
		.reg_offset		= BCRM_GAIN_AUTO_8RW,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_8,
		.type 			= V4L2_CTRL_TYPE_BOOLEAN,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{
		.id 			= V4L2_CID_GAIN,
		.attr			= AV_ATTR_GAIN,
		.min_offset 		= BCRM_GAIN_MIN_64R,
		.max_offset 		= BCRM_GAIN_MAX_64R,
		.reg_offset		= BCRM_GAIN_64RW,
		.step_offset		= BCRM_GAIN_INC_64R,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_64,
		.type 			= V4L2_CTRL_TYPE_INTEGER64,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{
		.id 			= V4L2_CID_HFLIP,
		.attr			= AV_ATTR_REVERSE_X,
		.reg_offset		= BCRM_IMG_REVERSE_X_8RW,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_8,
		.type 			= V4L2_CTRL_TYPE_BOOLEAN,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{
		.id 			= V4L2_CID_VFLIP,
		.attr			= AV_ATTR_REVERSE_Y,
		.reg_offset		= BCRM_IMG_REVERSE_Y_8RW,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_8,
		.type 			= V4L2_CTRL_TYPE_BOOLEAN,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{
		.id 			= V4L2_CID_SHARPNESS,
		.attr			= AV_ATTR_SHARPNESS,
		.min_offset 		= BCRM_SHARPNESS_MIN_32R,
		.max_offset 		= BCRM_SHARPNESS_MAX_32R,
		.reg_offset		= BCRM_SHARPNESS_32RW,
		.step_offset		= BCRM_SHARPNESS_INC_32R,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_32,
		.type 			= V4L2_CTRL_TYPE_INTEGER,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{
		.id 			= V4L2_CID_EXPOSURE_AUTO,
		.attr			= AV_ATTR_EXPOSURE_AUTO,
		.reg_offset		= BCRM_EXPOSURE_AUTO_8RW,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_8,
		.type			= V4L2_CTRL_TYPE_MENU,
		.get_default_val	= avt_get_menu_def_val,
		.flags 			= 0,
		.menu_count		= 2,
	},
	{
		.id 			= V4L2_CID_EXPOSURE_ABSOLUTE,
		.attr			= AV_ATTR_EXPOSURE_ABS,
		.min_offset 		= BCRM_EXPOSURE_TIME_MIN_64R,
		.max_offset 		= BCRM_EXPOSURE_TIME_MAX_64R,
		.reg_offset		= BCRM_EXPOSURE_TIME_64RW,
		.step_offset		= BCRM_EXPOSURE_TIME_INC_64R,
		.reg_size 		= AV_CAM_REG_SIZE,
		.data_size 		= AV_CAM_DATA_SIZE_64,
		.type 			= V4L2_CTRL_TYPE_INTEGER64,
		.get_default_val	= avt_get_int32_64_def_val,
		.flags 			= 0,
		.menu_count		= 0,
	},
	{ 

	},
};

#if 0
/* Must be sorted from low to high control ID! */
/* User class ioctls */
static const u32 avt_user_ctrls[] = {
	V4L2_CID_BRIGHTNESS,
	V4L2_CID_CONTRAST,
	V4L2_CID_SATURATION,
	V4L2_CID_HUE,
	V4L2_CID_AUTO_WHITE_BALANCE,
	V4L2_CID_DO_WHITE_BALANCE,
	V4L2_CID_RED_BALANCE,
	V4L2_CID_BLUE_BALANCE,
	V4L2_CID_GAMMA,
	V4L2_CID_WHITENESS,
	V4L2_CID_EXPOSURE,
	V4L2_CID_AUTOGAIN,
	V4L2_CID_GAIN,
	V4L2_CID_HFLIP,
	V4L2_CID_VFLIP,
	V4L2_CID_SHARPNESS,
	0
};


/* Must be sorted from low to high control ID! */
/* Camera class ioctls */
static const u32 avt_camera_ctrls[] = {
	V4L2_CID_EXPOSURE_AUTO,
	V4L2_CID_EXPOSURE_ABSOLUTE,
	0
};
static const u32 *ctrl_classes[] = {
	avt_user_ctrls,
	avt_camera_ctrls,
	NULL
};
#endif

static const u32 avt_v4l2_ctrls[] = {
	V4L2_CID_BRIGHTNESS,
	V4L2_CID_CONTRAST,
	V4L2_CID_SATURATION,
	V4L2_CID_HUE,
	V4L2_CID_AUTO_WHITE_BALANCE,
	V4L2_CID_DO_WHITE_BALANCE,
	V4L2_CID_RED_BALANCE,
	V4L2_CID_BLUE_BALANCE,
	V4L2_CID_GAMMA,
	V4L2_CID_WHITENESS,
	V4L2_CID_EXPOSURE,
	V4L2_CID_AUTOGAIN,
	V4L2_CID_GAIN,
	V4L2_CID_HFLIP,
	V4L2_CID_VFLIP,
	V4L2_CID_SHARPNESS,
	V4L2_CID_EXPOSURE_AUTO,
	V4L2_CID_EXPOSURE_ABSOLUTE,
	0
};



#if LINUX_VERSION_CODE > KERNEL_VERSION(3,14,52)
/* ctrl_classes points to an array of u32 pointers, the last element is
   a NULL pointer. Each u32 array is a 0-terminated array of control IDs.
   Each array must be sorted low to high and belong to the same control
   class. The array of u32 pointers must also be sorted, from low class IDs
   to high class IDs.

   This function returns the first ID that follows after the given ID.
   When no more controls are available 0 is returned. */
u32 v4l2_ctrl_next(const u32 * const * ctrl_classes, u32 id)
{
	u32 ctrl_class = V4L2_CTRL_ID2CLASS(id);
	const u32 *pctrl;

	if (ctrl_classes == NULL)
		return 0;

	/* if no query is desired, then check if the ID is part of ctrl_classes */
	if ((id & V4L2_CTRL_FLAG_NEXT_CTRL) == 0) {
		/* find class */
		while (*ctrl_classes && V4L2_CTRL_ID2CLASS(**ctrl_classes) != ctrl_class)
			ctrl_classes++;
		if (*ctrl_classes == NULL)
			return 0;
		pctrl = *ctrl_classes;
		/* find control ID */
		while (*pctrl && *pctrl != id) pctrl++;
		return *pctrl ? id : 0;
	}
	id &= V4L2_CTRL_ID_MASK;
	id++;	/* select next control */
	/* find first class that matches (or is greater than) the class of
	   the ID */
	while (*ctrl_classes && V4L2_CTRL_ID2CLASS(**ctrl_classes) < ctrl_class)
		ctrl_classes++;
	/* no more classes */
	if (*ctrl_classes == NULL)
		return 0;
	pctrl = *ctrl_classes;
	/* find first ctrl within the class that is >= ID */
	while (*pctrl && *pctrl < id) pctrl++;
	if (*pctrl)
		return *pctrl;
	/* we are at the end of the controls of the current class. */
	/* continue with next class if available */
	ctrl_classes++;
	if (*ctrl_classes == NULL)
		return 0;
	return **ctrl_classes;
}
EXPORT_SYMBOL(v4l2_ctrl_next);
#else
#define V4L2_CTRL_FLAG_NEXT_COMPOUND	0x40000000
#endif

static void av_cam_reset(void)
{
	return;
}

/**
 * @brief Since the camera needs a few ms to process written data, we need to poll 
   the handshake register to make sure to continue not too early with the next write access.
 * 
 * @param timeout_ms : Timeout value in ms
 * @return uint64_t : Duration in ms
 */
static uint64_t wait_for_bcrm_write_handshake(uint64_t timeout_ms)
{
	static const int poll_interval_ms = 10;
	static const int default_wait_time_ms = 50;
	int status = 0;
	u8 buffer[3] = {0};
	u8 handshake_val = 0;
	bool handshake_valid = false;		
	struct timeval tstart;
	struct timeval tend;
	uint64_t start_time_ms = 0;
	uint64_t duration_ms = 0;	

	do_gettimeofday(&tstart);
	start_time_ms = (tstart.tv_sec * (uint64_t)1000) + (tstart.tv_usec / 1000);

	if (is_bcrm_write_handshake_available)
	{
		/* We need to poll the handshake register and wait until the camera has processed the data */	
		AV_DEBUG(" Wait for 'write done' bit (0x81) ...");
		do
		{
			msleep(poll_interval_ms);
			/* Read handshake register */
			status = i2c_bcrm_read_byte_reg(cci_reg.bcrm_address + BCRM_WRITE_HANDSHAKE_REG_8RW, &handshake_val);

			do_gettimeofday(&tend);
			duration_ms = ((tend.tv_sec * (uint64_t)1000) + (tend.tv_usec / 1000)) - start_time_ms;

			if (status >= 0)
			{
				/* Check, if handshake bit is set */
				if ((handshake_val & 0x01) == 1)
				{
					/* Handshake set by camera. We should to reset it */
					buffer[0] = (cci_reg.bcrm_address + BCRM_WRITE_HANDSHAKE_REG_8RW) >> 8;
					buffer[1] = (cci_reg.bcrm_address + BCRM_WRITE_HANDSHAKE_REG_8RW) & 0xff;
					buffer[2] = (handshake_val & 0xFE);	/* Reset LSB (handshake bit)*/
					status = i2c_master_send(av_cam_data.i2c_client, buffer, sizeof(buffer));

					/* Since the camera needs a few ms for every write access to finish, we need to poll here too */
					AV_DEBUG(" Wait for reset of 'write done' bit (0x80) ...");
					do{
						msleep(poll_interval_ms);
						/* We need to wait again until the bit is reset */
						status = i2c_bcrm_read_byte_reg(cci_reg.bcrm_address + BCRM_WRITE_HANDSHAKE_REG_8RW, &handshake_val);
						
						do_gettimeofday(&tend);
						duration_ms = ((tend.tv_sec * (uint64_t)1000) + (tend.tv_usec / 1000)) - start_time_ms;

						if (status >= 0)
						{
							if ((handshake_val & 0x01) == 0)	/* Verify write */
							{
								handshake_valid = true;
								break;		
							}
						}
						else
						{
							AV_ERR(" Error while reading BCRM_WRITE_HANDSHAKE_REG_8RW register.");
							break;
						}
					}
					while (duration_ms <= timeout_ms);
			
					break;		
				}
			}
			else
			{
				AV_ERR(" Error while reading BCRM_WRITE_HANDSHAKE_REG_8RW register.");
				break;
			}
		}
		while (duration_ms <= timeout_ms);

		if (!handshake_valid)
		{
			AV_ERR(" Write handshake timeout!");
		}		
	}
	else
	{
		/* Handshake not supported. Use static sleep at least once as fallback */
		msleep(default_wait_time_ms);	
		duration_ms = (uint64_t)default_wait_time_ms;	
	}

	return duration_ms;
}

/**
 * @brief Low level function to write one byte to a BCRM register (16 bit address)
 *        Uses write handshake if supported by camera.
 * 
 * @param reg : 16 bit register address
 * @param val : Byte to write
 * @return s32 : If <0 error, else successfull
 */
static s32 i2c_bcrm_write_byte_reg(u16 reg, u8 val)
{
	int ret = 0;
	uint64_t duration = 0;
	u8 buffer[3] = {0};

	buffer[0] = reg >> 8;
	buffer[1] = reg & 0xff;
	buffer[2] = val;
	
	ret = i2c_master_send(av_cam_data.i2c_client, buffer, sizeof(buffer));

	if (ret >= 0)
	{
		duration = wait_for_bcrm_write_handshake(BCRM_WAIT_HANDSHAKE_TIMEOUT);
		AV_DEBUG("I2C write success. reg=0x%04x, val=0x%x, duration=%lldms, ret=%d", reg, val, duration, ret);	
	
	/*	if (!check_bcrm_write_byte_result(reg, val))
		{
			AV_ERR("I2C write check FAILED. reg=0x%04x. Value not accepted by camera.", reg); 		
		}
		else
		{
			AV_DEBUG("I2C write check OK. reg=0x%04x", reg);
		}
	*/
	}
	else
	{
		AV_ERR("I2C write FAILED. reg=0x%04x, val=0x%x, duration=%lldms, ret=%d", reg, val, duration, ret);
	}

	return ret;
}

/**
 * @brief Low level function to read one byte from a BCRM register (16 bit address)
 * 
 * @param reg : 16 bit register address
 * @param val : Pointer to value
 * @return s32 : If <0 error, else successfull
 */
static s32 i2c_bcrm_read_byte_reg(u16 reg, u8 *val)
{
	struct sensor_data *sensor;
	struct i2c_client *client;
	struct i2c_msg msgs[2];
	u8 buffer[2];
	int ret = 0;

	sensor = &av_cam_data;
	client = sensor->i2c_client;

	buffer[0] = reg >> 8;
	buffer[1] = reg & 0xff;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = buffer;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buffer;
    
	ret = i2c_transfer(client->adapter, msgs, 2);

	if (ret >= 0)
	{
		*val = buffer[0];
		AV_DEBUG("I2C read success. reg=0x%04x, *val=0x%x, ret=%d", reg, *val, ret);	
	}
	else
	{
		AV_ERR("I2C read FAILED. reg=0x%04x, ret=%d", reg, ret);
	}

	return ret;
}


static bool check_bcrm_write_byte_result(u16 reg, u8 val)
{
	int status = 0;
	u8 read_val = 0;

	if (get_bcrm_reg_access_mode(reg) & ACCESS_MODE_R)
	{
		status = i2c_bcrm_read_byte_reg(reg, &read_val);
		AV_DEBUG("write check: read_val=0x%x, val=0x%x", read_val, val);
	}
	else
	{
		return false;	// Read only register
	}

	return ((status >= 0) && (read_val == val)) ? true : false;
}

static uint8_t get_bcrm_reg_access_mode(uint16_t reg)
{
	int i;
	for (i = 0; i<sizeof(bcrm_reg_struct_table)/sizeof(struct bcrm_reg_struct_struct); i++)
	{
		if (bcrm_reg_struct_table[i].reg == reg)
		{
			return bcrm_reg_struct_table[i].access_mode;
		}
	}

	return 0;
}



/* Wrapper for i2c_generic_read
*/
static int i2c_generic_read_buffer(__u32 reg, __u32 reg_size, __u32 count, char *buffer)
{
	int ret = 0;
	struct v4l2_i2c i2c_reg;
	i2c_reg.reg = reg;
	i2c_reg.reg_size = reg_size;
	i2c_reg.count = count;
	i2c_reg.buffer = buffer;
    
	ret = i2c_generic_read(&i2c_reg);
	swapbytes(buffer, count);

	return (ret >= 0) ? ret : -EINVAL;
}

/* Low level function for generic read from a given 16 bit register
*/
static ssize_t i2c_generic_read(struct v4l2_i2c *i2c_reg)
{
	struct i2c_msg msg[2];
	u8 msgbuf[i2c_reg->reg_size];
	struct sensor_data *sensor = &av_cam_data;
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0, i = 0, j = 0, reg_size_bkp;

	reg_size_bkp = i2c_reg->reg_size;

	/* clearing i2c msg with 0's */
	memset(msg, 0, sizeof(msg));

	if (i2c_reg->count > IO_LIMIT) 
	{
		AV_ERR("IO_LIMIT exceeded! i2c_reg->count (%d) > IO_LIMIT (%d)", i2c_reg->count, IO_LIMIT);
		i2c_reg->count = IO_LIMIT;
	}

	/* find start address of buffer */
	for (i = --i2c_reg->reg_size ; i >= 0 ; i--, j++)
		msgbuf[i] = ((i2c_reg->reg >> (8*j)) & 0xFF);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = reg_size_bkp;
	msg[0].buf = msgbuf;

	msg[1].addr = client->addr;  	/* slave address */
	msg[1].flags = I2C_M_RD;  	/* Read flag setting */
	msg[1].len = i2c_reg->count;
	msg[1].buf = (char *) i2c_reg->buffer; /* dest buffer */

	ret = i2c_transfer(client->adapter, msg, 2);

	if (ret >= 0)
	{
		AV_DEBUG("I2C read success. reg=0x%04x, count=%d, ret=%d", i2c_reg->reg, i2c_reg->count, ret);	
	}
	else
	{
		AV_ERR("I2C read FAILED. reg=0x%04x, count=%d, ret=%d", i2c_reg->reg, i2c_reg->count, ret);
	}

	return ret;
}

static s32 ioctl_gencam_i2cread_reg(struct v4l2_int_device *s, struct v4l2_i2c *i2c_reg)
{
	return i2c_generic_read(i2c_reg);
}

static s32 ioctl_gencam_i2cwrite_reg(struct v4l2_int_device *s, struct v4l2_i2c *i2c_reg)
{
	int j = 0, i = 0, reg_size_bkp = 0;
	int total_size = i2c_reg->count + i2c_reg->reg_size;
	u8 i2c_w_buf[total_size];
	char *temp_buffer_bkp;
	int ret = 0;
	char *temp_buff;

	/* count exceeds writing IO_LIMIT characters */
	if (i2c_reg->count > IO_LIMIT) {
		AV_ERR("Limit excedded! i2c_reg->count > IO_LIMIT");
		i2c_reg->count = IO_LIMIT;
	}

	temp_buff = (char *) kmalloc(i2c_reg->count, GFP_KERNEL);

	if (temp_buff == NULL) {
		AV_ERR("Failed to allocate memory:");
		return -1;
	}

	temp_buffer_bkp = temp_buff;

	memset(temp_buff, 0, sizeof(i2c_reg->count));
	memcpy((char *)temp_buff, (const char *)i2c_reg->buffer, i2c_reg->count);

	/* Backup the size of register which needs to be read after filling the address buffer */
	reg_size_bkp = i2c_reg->reg_size;

	/* Fill the address in buffer upto size of address want to write */
	for (i = --i2c_reg->reg_size; i >= 0 ; i--, j++)
		i2c_w_buf[i] = ((i2c_reg->reg >> (8*j)) & 0xFF);

	/* Append the data value in the same buffer  */
	for (i = reg_size_bkp; i < (i2c_reg->count + reg_size_bkp) ; i++, temp_buff++)
		i2c_w_buf[i] = *temp_buff;

	ret = i2c_master_send(av_cam_data.i2c_client, i2c_w_buf, total_size);

	if (ret < 0)
		AV_ERR("I2C write failed. (Status=%d)", ret);

	kfree(temp_buffer_bkp);
	return ret;
}

/* Low level bcrm write register function
*/
static int i2c_bcrm_write_reg(struct v4l2_i2c *i2c_reg)
{
	int j = 0, i = 0, reg_size_bkp = 0;
	int total_size = i2c_reg->count + i2c_reg->reg_size;
	u8 i2c_w_buf[total_size];
	char *temp_buffer_bkp;
	int ret = 0;
	char *temp_buff;
	uint64_t duration = 0;

	// Check address range
	if ((i2c_reg->reg < cci_reg.bcrm_address) || (i2c_reg->reg > cci_reg.bcrm_address + _BCRM_LAST_ADDR))
	{
		AV_ERR("INVALID BCRM write call");
		return -1;
	}

	/* count exceeds writing IO_LIMIT characters */
	if (i2c_reg->count > IO_LIMIT) {
		AV_ERR("Limit excedded! i2c_reg->count > IO_LIMIT");
		i2c_reg->count = IO_LIMIT;
	}

	temp_buff = (char *) kmalloc(i2c_reg->count, GFP_KERNEL);

	if (temp_buff == NULL) {
		AV_ERR("Failed to allocate memory:");
		return -1;
	}

	temp_buffer_bkp = temp_buff;

	memset(temp_buff, 0, sizeof(i2c_reg->count));
	memcpy((char *)temp_buff, (const char *)i2c_reg->buffer, i2c_reg->count);

	/* Backup the size of register which needs to be read after filling the address buffer */
	reg_size_bkp = i2c_reg->reg_size;

	/* Fill the address in buffer upto size of address want to write */
	for (i = --i2c_reg->reg_size; i >= 0 ; i--, j++)
		i2c_w_buf[i] = ((i2c_reg->reg >> (8*j)) & 0xFF);

	/* Append the data value in the same buffer  */
	for (i = reg_size_bkp; i < (i2c_reg->count + reg_size_bkp) ; i++, temp_buff++)
		i2c_w_buf[i] = *temp_buff;

	ret = i2c_master_send(av_cam_data.i2c_client, i2c_w_buf, total_size);

	if (ret >= 0)
	{
		AV_DEBUG(" Wait for write handshake. reg=0x%04x", i2c_reg->reg);
		duration = wait_for_bcrm_write_handshake(BCRM_WAIT_HANDSHAKE_TIMEOUT);
		//msleep(500);
		AV_DEBUG("I2C write success. reg=0x%04x, count=%d, duration=%lldms, ret=%d", i2c_reg->reg, i2c_reg->count, duration, ret);
	
	/* TODO
		if (!check_bcrm_write_reg_result(i2c_reg))
		{
			AV_ERR("I2C write check FAILED. reg=0x%04x. Value not accepted by camera.", i2c_reg->reg); 		
		}
		else
		{
			AV_DEBUG("I2C write check OK. reg=0x%04x", i2c_reg->reg);				
		}
	*/	
	}
	else
	{
		AV_ERR("I2C write FAILED. reg=0x%04x, count=%d, duration=%lldms, ret=%d", i2c_reg->reg, i2c_reg->count, duration, ret);
	}

	kfree(temp_buffer_bkp);
	return ret;
}


static bool check_bcrm_write_reg_result(struct v4l2_i2c *i2c_reg)
{
	int status = 0;
	bool ret = false;
	struct v4l2_i2c local_i2c_reg;
	int count=0;

	local_i2c_reg.reg = i2c_reg->reg;
	local_i2c_reg.reg_size = i2c_reg->reg_size;
	local_i2c_reg.count = i2c_reg->count;
	local_i2c_reg.buffer = (char *) kmalloc(i2c_reg->count, GFP_KERNEL);

	status = i2c_generic_read(&local_i2c_reg);

	if (status < 0)
	{
		AV_ERR("read failed");
	}
	if (memcmp(local_i2c_reg.buffer, i2c_reg->buffer, i2c_reg->count) == 0)
	{
		ret = true;
	}
	else
	{
		AV_ERR("count=%d", i2c_reg->count);
		for(count=0; count<i2c_reg->count; count++)
		{
			AV_ERR("%X - %X", local_i2c_reg.buffer[count], i2c_reg->buffer[count]);
		}


		AV_ERR("memcmp failed");
	}
	
	kfree(local_i2c_reg.buffer);

	return ret;
}


/* Wrapper for i2c_bcrm_write_reg. Uses v4l2_control as input.
*/
static int i2c_bcrm_write_reg_ctrl(struct v4l2_control *vc, unsigned int reg, int length)
{
	struct v4l2_i2c i2c_reg = {0};
	__u64 temp = 0;

// TODO

	if (length > AV_CAM_DATA_SIZE_32)
		temp = vc->value;

	if (length > AV_CAM_DATA_SIZE_32)
		swapbytes(&temp, length);
	else
		swapbytes(&vc->value, length);

	i2c_reg.reg = reg;
	i2c_reg.reg_size = AV_CAM_REG_SIZE;
	i2c_reg.count = length;

	AV_DEBUG("vc_value=%d (0x%x), temp=%lld (0x%llx)", vc->value, vc->value, temp, temp);


	if (length > AV_CAM_DATA_SIZE_32)
		i2c_reg.buffer = (const char *) &temp;
	else
		i2c_reg.buffer = (const char *) &vc->value;

	return i2c_bcrm_write_reg(&i2c_reg);
}

/* Wrapper for i2c_bcrm_write_reg. Uses v4l2_ext_control as input.
*/
static int i2c_bcrm_write_reg_ctrl_ext(struct v4l2_ext_control *vc, unsigned int reg, int length)
{
	//struct v4l2_int_device s;
	struct v4l2_i2c i2c_reg = {0};
	int ret = 0;

	memset(&i2c_reg, 0, sizeof(i2c_reg));

	if (length > AV_CAM_DATA_SIZE_32)
		swapbytes(&vc->value64, length);
	else
		swapbytes(&vc->value, length);

	i2c_reg.reg = reg;
	i2c_reg.reg_size = AV_CAM_REG_SIZE;
	i2c_reg.count = length;

	AV_DEBUG("vc_value=%d (0x%x), vc->value64=%lld (0x%llx)", vc->value, vc->value, vc->value64, vc->value64);

	if (length > AV_CAM_DATA_SIZE_32)
		i2c_reg.buffer = (const char *) &vc->value64;
	else
		i2c_reg.buffer = (const char *) &vc->value;

	ret = i2c_bcrm_write_reg(&i2c_reg);

	if (length > AV_CAM_DATA_SIZE_32)
		swapbytes(&vc->value64, length);
	else
		swapbytes(&vc->value, length);

	return ret;
}

static uint32_t crc32(const void *pSrc, uint32_t nByteCount, const uint32_t *pnStartCrcValue)
{
	register uint32_t   nBit;
	register uint32_t   nCrc32 = UINT32_MAXIMUM;
	const uint8_t *pSrc8  = (const uint8_t *) pSrc;
    
	/*  start value provided ? */
	if (NULL != pnStartCrcValue)
		nCrc32 = *pnStartCrcValue;

	while (0u != nByteCount--) {
		nCrc32 ^= *(pSrc8++);

		nBit = 8;
		while (0u != nBit--) {
			if (nCrc32 & 0x01u)
				nCrc32 = (nCrc32 >> 1) ^ CRC32MASKREV;

			else
				nCrc32 = (nCrc32 >> 1);
		}
	}
	//AV_DEBUG("CRC Checksum: 0x%x", nCrc32);
	return nCrc32;
}

static void swapbytes(void *_object, size_t _size)
{
	unsigned char *start, *end;
	/* If the board is little endian, swap the CCI registers as its big endian */
	if (!is_bigendian()) {
		//AV_DEBUG("swapbytes, SoC is little endian!, need to swap the CCI registers");

		for (start = (unsigned char *)_object, end = start + _size - 1; start < end; ++start, --end) {
			unsigned char swap = *start;
			*start = *end;
			*end = swap;
		}
	}
}

static int read_cci_registers(void)
{
	int status = 0;
	struct v4l2_i2c i2c_reg = {0};
	uint32_t CRC = 0;
	uint32_t crc_byte_count = 0;

	i2c_reg.reg = cci_cmd_tbl[CCI_REGISTER_LAYOUT_VERSION].address;
	i2c_reg.reg_size = AV_CAM_REG_SIZE;
	i2c_reg.count = sizeof(cci_reg) - 3;    /* Avoid last 3 bytes read as its WRITE only register except CURRENT MODE REG */

	i2c_reg.buffer = (char *)&cci_reg;
	/*  Calculate CRC byte count */
	crc_byte_count = sizeof(cci_reg) - 7 ;

	AV_DEBUG("Read all CCI registers...");
	AV_DEBUG("crc_byte_count=%d, i2c_reg.count=%d", crc_byte_count, i2c_reg.count);

	/*  Read  CCI registers */
	status = i2c_generic_read(&i2c_reg);

	if (status < 0) {
	//	AV_ERR("I2C read failed. (Status=%d)", status);
		return status;
	}

	/* CRC calculation */
	CRC = crc32(&cci_reg, crc_byte_count, 0);

	/* Swap bytes if neccessary */
	swapbytes(&cci_reg.cci_register_layout_version, 4);
	swapbytes(&cci_reg.device_capabilities, 8);
	swapbytes(&cci_reg.gcprm_address, 2);
	swapbytes(&cci_reg.bcrm_address, 2);
	swapbytes(&cci_reg.checksum, 4);

	/* Check the checksum of received with calculated. */
	if (CRC != cci_reg.checksum) {
		AV_ERR("CRC check failed: CRC calculated=0x%x. CRC received=0x%x", CRC, cci_reg.checksum);
		return AV_CAM_ERR_CRC_FAIL;
	} else {
		AV_DEBUG("CRC check passed: CRC calculated=0x%x. CRC received=0x%x", CRC, cci_reg.checksum);
	}

	AV_DEBUG("cci_register_layout_version=0x%x", cci_reg.cci_register_layout_version);
	AV_DEBUG("device_capabilities=0x%llx", cci_reg.device_capabilities);
	AV_DEBUG(" GCPRM offset=0x%x", cci_reg.gcprm_address);
	AV_DEBUG(" BCRM offset=0x%x", cci_reg.bcrm_address);
	AV_DEBUG(" UserDefinedName supported=%d", (uint32_t)((cci_reg.device_capabilities>>0) & 1));
	AV_DEBUG(" BCRM supported=%d", (uint32_t)((cci_reg.device_capabilities>>1) & 1));
	AV_DEBUG(" GenCP supported=%d", (uint32_t)((cci_reg.device_capabilities>>2) & 1));
	AV_DEBUG(" CCI checksum=0x%x", cci_reg.checksum);
	switch((cci_reg.device_capabilities>>4) & 0xF)
	{
	case CCI_STRING_ENC_ASCII:
		AV_DEBUG(" String Encoding=%d (ASCII)", (uint32_t)((cci_reg.device_capabilities>>4) & 0xF));
		break;
	case CCI_STRING_ENC_UTF8:
		AV_DEBUG(" String Encoding=%d (UTF8)", (uint32_t)((cci_reg.device_capabilities>>4) & 0xF));
		break;
	case CCI_STRING_ENC_UTF16:
		AV_DEBUG(" String Encoding=%d (UTF16)", (uint32_t)((cci_reg.device_capabilities>>4) & 0xF));
		break;
	default:
		AV_DEBUG(" String Encoding=%d (Unknown)", (uint32_t)((cci_reg.device_capabilities>>4) & 0xF));
		break;
	}	

	AV_DEBUG(" FamiliyName supported=%d", (uint32_t)((cci_reg.device_capabilities>>8) & 1));
	AV_DEBUG("cci_reg.device_guid=%s", cci_reg.device_guid);

	return AV_CAM_ERR_SUCCESS;
}

static int bcrm_version_check(void)
{
	u32 value = 0;
	int status;

	/* Reading the BCRM version */
	status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_VERSION_REG_32R,
				 	AV_CAM_REG_SIZE,
					AV_CAM_DATA_SIZE_32,
					(char *) &value);
	if (status < 0) {
		return -EINVAL;
	}

	AV_DEBUG("Driver BCRM version: 0x%08lx", BCRM_DEVICE_VERSION);
	AV_DEBUG("Driver BCRM major version: 0x%04lx", BCRM_MAJOR_VERSION);
	AV_DEBUG("Driver BCRM minor version: 0x%04lx", BCRM_MINOR_VERSION);

	AV_DEBUG("Camera BCRM version: 0x%08x", value);
	AV_DEBUG("Camera BCRM major version: 0x%04x", value>>16);
	AV_DEBUG("Camera BCRM minor version: 0x%04x", (int16_t) value);

	return (value>>16) == BCRM_MAJOR_VERSION?1:0 ;
}

static void bcrm_get_device_fw_version(void)
{
	u64 value = 0;
	int status;

	/* Reading the device firmware version from camera */
	status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_DEVICE_FIRMWARE_VERSION_REG_64R,
				 	AV_CAM_REG_SIZE,
					AV_CAM_DATA_SIZE_64,
					(char *) &value);
	if (status < 0) {
		AV_WARNING("Could not retrieve device firmware version (status=%d)", status);
		return;
	}

	AV_DEBUG( "Device firmware version: %d.%d.%d.%d (0x%llX)", 
		(u8)(value & 0xFF),
		(u8)((value>>8) &0xFF),
		(u16)((value>>16) &0xFFFF),
		(u32)((value>>32) &0xFFFFFFFF),
		value);
}

static void bcrm_get_write_handshake_availibility(void)
{
	u8 value = 0;
	int status;

	/* Reading the device firmware version from camera */
	status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_WRITE_HANDSHAKE_REG_8RW,
				 	AV_CAM_REG_SIZE,
					AV_CAM_DATA_SIZE_8,
					(char *) &value);

	if ((status >= 0) && (value & 0x80))
	{
		is_bcrm_write_handshake_available = true;
		AV_DEBUG("BCRM_WRITE_HANDSHAKE_REG_8RW register supported!");	
	}
	else
	{
		is_bcrm_write_handshake_available = false;
		AV_WARNING("BCRM_WRITE_HANDSHAKE_REG_8RW register NOT supported!");
	}
}

static void bcrm_dump(void)
{
	/* Dump all BCRM registers (except write only ones) */

	dump_bcrm_reg_32(BCRM_VERSION_REG_32R, 				" BCRM_VERSION_REG_32R");
	dump_bcrm_reg_64(BCRM_FEATURE_INQUIRY_REG_64R, 			" BCRM_FEATURE_INQUIRY_REG_64R");
	dump_bcrm_reg_64(BCRM_DEVICE_FIRMWARE_VERSION_REG_64R, 		" BCRM_DEVICE_FIRMWARE_VERSION_REG_64R");
	dump_bcrm_reg_8(BCRM_WRITE_HANDSHAKE_REG_8RW, 			" BCRM_WRITE_HANDSHAKE_REG_8RW");

	/* Streaming Control Registers */
	dump_bcrm_reg_8(BCRM_SUPPORTED_CSI2_LANE_COUNTS_8R, 		" BCRM_SUPPORTED_CSI2_LANE_COUNTS_8R");
	dump_bcrm_reg_8(BCRM_CSI2_LANE_COUNT_8RW, 			" BCRM_CSI2_LANE_COUNT_8RW");
	dump_bcrm_reg_32(BCRM_CSI2_CLOCK_MIN_32R, 			" BCRM_CSI2_CLOCK_MIN_32R");
	dump_bcrm_reg_32(BCRM_CSI2_CLOCK_MAX_32R, 			" BCRM_CSI2_CLOCK_MAX_32R");
	dump_bcrm_reg_32(BCRM_CSI2_CLOCK_32RW, 				" BCRM_CSI2_CLOCK_32RW");
	dump_bcrm_reg_32(BCRM_BUFFER_SIZE_32R, 				" BCRM_BUFFER_SIZE_32R");
	dump_bcrm_reg_32(BCRM_PHY_RESET_REG_8RW, 			" BCRM_PHY_RESET_REG_8RW");

	/* Acquisition Control Registers */
	dump_bcrm_reg_8(BCRM_ACQUISITION_START_8RW, 			" BCRM_ACQUISITION_START_8RW");
	dump_bcrm_reg_8(BCRM_ACQUISITION_STOP_8RW, 			" BCRM_ACQUISITION_STOP_8RW");
	dump_bcrm_reg_8(BCRM_ACQUISITION_ABORT_8RW, 			" BCRM_ACQUISITION_ABORT_8RW");
	dump_bcrm_reg_8(BCRM_ACQUISITION_STATUS_8R,			" BCRM_ACQUISITION_STATUS_8R");
	dump_bcrm_reg_64(BCRM_ACQUISITION_FRAME_RATE_64RW, 		" BCRM_ACQUISITION_FRAME_RATE_64RW");
	dump_bcrm_reg_64(BCRM_ACQUISITION_FRAME_RATE_MIN_64R, 		" BCRM_ACQUISITION_FRAME_RATE_MIN_64R");
	dump_bcrm_reg_64(BCRM_ACQUISITION_FRAME_RATE_MAX_64R, 		" BCRM_ACQUISITION_FRAME_RATE_MAX_64R");
	dump_bcrm_reg_64(BCRM_ACQUISITION_FRAME_RATE_INC_64R, 		" BCRM_ACQUISITION_FRAME_RATE_INC_64R");

	dump_bcrm_reg_8(BCRM_FRAME_RATE_ENABLE_8RW, 			" BCRM_FRAME_RATE_ENABLE_8RW");
	dump_bcrm_reg_8(BCRM_FRAME_START_TRIGGER_MODE_8RW, 		" BCRM_FRAME_START_TRIGGER_MODE_8RW");
	dump_bcrm_reg_8(BCRM_FRAME_START_TRIGGER_SOURCE_8RW,		" BCRM_FRAME_START_TRIGGER_SOURCE_8RW");
	dump_bcrm_reg_8(BCRM_FRAME_START_TRIGGER_ACTIVATION_8RW,	" BCRM_FRAME_START_TRIGGER_ACTIVATION_8RW");
	
	/* Image Format Control Registers */
	dump_bcrm_reg_32(BCRM_IMG_WIDTH_32RW, 				" BCRM_IMG_WIDTH_32RW");
	dump_bcrm_reg_32(BCRM_IMG_WIDTH_MIN_32R, 			" BCRM_IMG_WIDTH_MIN_32R");
	dump_bcrm_reg_32(BCRM_IMG_WIDTH_MAX_32R, 			" BCRM_IMG_WIDTH_MAX_32R");
	dump_bcrm_reg_32(BCRM_IMG_WIDTH_INC_32R, 			" BCRM_IMG_WIDTH_INC_32R");

	dump_bcrm_reg_32(BCRM_IMG_HEIGHT_32RW,				" BCRM_IMG_HEIGHT_32RW");
	dump_bcrm_reg_32(BCRM_IMG_HEIGHT_MIN_32R,			" BCRM_IMG_HEIGHT_MIN_32R");
	dump_bcrm_reg_32(BCRM_IMG_HEIGHT_MAX_32R,			" BCRM_IMG_HEIGHT_MAX_32R");
	dump_bcrm_reg_32(BCRM_IMG_HEIGHT_INC_32R, 			" BCRM_IMG_HEIGHT_INC_32R");

	dump_bcrm_reg_32(BCRM_IMG_OFFSET_X_32RW, 			" BCRM_IMG_OFFSET_X_32RW");
	dump_bcrm_reg_32(BCRM_IMG_OFFSET_X_MIN_32R, 			" BCRM_IMG_OFFSET_X_MIN_32R");
	dump_bcrm_reg_32(BCRM_IMG_OFFSET_X_MAX_32R, 			" BCRM_IMG_OFFSET_X_MAX_32R");
	dump_bcrm_reg_32(BCRM_IMG_OFFSET_X_INC_32R, 			" BCRM_IMG_OFFSET_X_INC_32R");

	dump_bcrm_reg_32(BCRM_IMG_OFFSET_Y_32RW, 			" BCRM_IMG_OFFSET_Y_32RW");
	dump_bcrm_reg_32(BCRM_IMG_OFFSET_Y_MIN_32R, 			" BCRM_IMG_OFFSET_Y_MIN_32R");
	dump_bcrm_reg_32(BCRM_IMG_OFFSET_Y_MAX_32R, 			" BCRM_IMG_OFFSET_Y_MAX_32R");
	dump_bcrm_reg_32(BCRM_IMG_OFFSET_Y_INC_32R, 			" BCRM_IMG_OFFSET_Y_INC_32R");

	dump_bcrm_reg_32(BCRM_IMG_MIPI_DATA_FORMAT_32RW, 		" BCRM_IMG_MIPI_DATA_FORMAT_32RW");
	dump_bcrm_reg_64(BCRM_IMG_AVAILABLE_MIPI_DATA_FORMATS_64R, 	" BCRM_IMG_AVAILABLE_MIPI_DATA_FORMATS_64R");

	dump_bcrm_reg_8(BCRM_IMG_BAYER_PATTERN_INQUIRY_8R, 		" BCRM_IMG_BAYER_PATTERN_INQUIRY_8R");
	dump_bcrm_reg_8(BCRM_IMG_BAYER_PATTERN_8RW, 			" BCRM_IMG_BAYER_PATTERN_8RW");

	dump_bcrm_reg_8(BCRM_IMG_REVERSE_X_8RW, 			" BCRM_IMG_REVERSE_X_8RW");
	dump_bcrm_reg_8(BCRM_IMG_REVERSE_Y_8RW, 			" BCRM_IMG_REVERSE_Y_8RW");

	dump_bcrm_reg_32(BCRM_SENSOR_WIDTH_32R, 			" BCRM_SENSOR_WIDTH_32R");
	dump_bcrm_reg_32(BCRM_SENSOR_HEIGHT_32R, 			" BCRM_SENSOR_HEIGHT_32R");

	dump_bcrm_reg_32(BCRM_WIDTH_MAX_32R, 				" BCRM_WIDTH_MAX_32R");
	dump_bcrm_reg_32(BCRM_HEIGHT_MAX_32R, 				" BCRM_HEIGHT_MAX_32R");

	/* Brightness Control Registers */
	dump_bcrm_reg_64(BCRM_EXPOSURE_TIME_64RW, 			" BCRM_EXPOSURE_TIME_64RW");
	dump_bcrm_reg_64(BCRM_EXPOSURE_TIME_MIN_64R, 			" BCRM_EXPOSURE_TIME_MIN_64R");
	dump_bcrm_reg_64(BCRM_EXPOSURE_TIME_MAX_64R, 			" BCRM_EXPOSURE_TIME_MAX_64R");
	dump_bcrm_reg_64(BCRM_EXPOSURE_TIME_INC_64R, 			" BCRM_EXPOSURE_TIME_INC_64R");
	dump_bcrm_reg_8(BCRM_EXPOSURE_AUTO_8RW, 			" BCRM_EXPOSURE_AUTO_8RW");

	dump_bcrm_reg_8(BCRM_INTENSITY_AUTO_PRECEDENCE_8RW, 		" BCRM_INTENSITY_AUTO_PRECEDENCE_8RW");
	dump_bcrm_reg_32(BCRM_INTENSITY_AUTO_PRECEDENCE_VALUE_32RW, 	" BCRM_INTENSITY_AUTO_PRECEDENCE_VALUE_32RW");
	dump_bcrm_reg_32(BCRM_INTENSITY_AUTO_PRECEDENCE_MIN_32R, 	" BCRM_INTENSITY_AUTO_PRECEDENCE_MIN_32R");
	dump_bcrm_reg_32(BCRM_INTENSITY_AUTO_PRECEDENCE_MAX_32R, 	" BCRM_INTENSITY_AUTO_PRECEDENCE_MAX_32R");
	dump_bcrm_reg_32(BCRM_INTENSITY_AUTO_PRECEDENCE_INC_32R, 	" BCRM_INTENSITY_AUTO_PRECEDENCE_INC_32R");

	dump_bcrm_reg_32(BCRM_BLACK_LEVEL_32RW, 			" BCRM_BLACK_LEVEL_32RW");
	dump_bcrm_reg_32(BCRM_BLACK_LEVEL_MIN_32R, 			" BCRM_BLACK_LEVEL_MIN_32R");
	dump_bcrm_reg_32(BCRM_BLACK_LEVEL_MAX_32R, 			" BCRM_BLACK_LEVEL_MAX_32R");
	dump_bcrm_reg_32(BCRM_BLACK_LEVEL_INC_32R, 			" BCRM_BLACK_LEVEL_INC_32R");

	dump_bcrm_reg_64(BCRM_GAIN_64RW, 				" BCRM_GAIN_64RW");
	dump_bcrm_reg_64(BCRM_GAIN_MIN_64R,			 	" BCRM_GAIN_MIN_64R");
	dump_bcrm_reg_64(BCRM_GAIN_MAX_64R, 				" BCRM_GAIN_MAX_64R");
	dump_bcrm_reg_64(BCRM_GAIN_INC_64R, 				" BCRM_GAIN_INC_64R");
	dump_bcrm_reg_8(BCRM_GAIN_AUTO_8RW, 				" BCRM_GAIN_AUTO_8RW");

	dump_bcrm_reg_64(BCRM_GAMMA_64RW, 				" BCRM_GAMMA_64RW");
	dump_bcrm_reg_64(BCRM_GAMMA_MIN_64R, 				" BCRM_GAMMA_MIN_64R");
	dump_bcrm_reg_64(BCRM_GAMMA_MAX_64R, 				" BCRM_GAMMA_MAX_64R");
	dump_bcrm_reg_64(BCRM_GAMMA_INC_64R, 				" BCRM_GAMMA_INC_64R");

	dump_bcrm_reg_8(BCRM_CONTRAST_ENABLE_8RW, 			" BCRM_CONTRAST_ENABLE_8RW");
	dump_bcrm_reg_32(BCRM_CONTRAST_VALUE_32RW, 			" BCRM_CONTRAST_VALUE_32RW");
	dump_bcrm_reg_32(BCRM_CONTRAST_VALUE_MIN_32R, 			" BCRM_CONTRAST_VALUE_MIN_32R");
	dump_bcrm_reg_32(BCRM_CONTRAST_VALUE_MAX_32R, 			" BCRM_CONTRAST_VALUE_MAX_32R");
	dump_bcrm_reg_32(BCRM_CONTRAST_VALUE_INC_32R, 			" BCRM_CONTRAST_VALUE_INC_32R");

	/* Color Management Registers */
	dump_bcrm_reg_32(BCRM_SATURATION_32RW,				" BCRM_SATURATION_32RW");
	dump_bcrm_reg_32(BCRM_SATURATION_MIN_32R,			" BCRM_SATURATION_MIN_32R");
	dump_bcrm_reg_32(BCRM_SATURATION_MAX_32R, 			" BCRM_SATURATION_MAX_32R");
	dump_bcrm_reg_32(BCRM_SATURATION_INC_32R, 			" BCRM_SATURATION_INC_32R");

	dump_bcrm_reg_32(BCRM_HUE_32RW,				 	" BCRM_HUE_32RW");
	dump_bcrm_reg_32(BCRM_HUE_MIN_32R, 				" BCRM_HUE_MIN_32R");
	dump_bcrm_reg_32(BCRM_HUE_MAX_32R,				" BCRM_HUE_MAX_32R");
	dump_bcrm_reg_32(BCRM_HUE_INC_32R, 				" BCRM_HUE_INC_32R");

	dump_bcrm_reg_64(BCRM_ALL_BALANCE_RATIO_64RW,			" BCRM_ALL_BALANCE_RATIO_64RW");
	dump_bcrm_reg_64(BCRM_ALL_BALANCE_RATIO_MIN_64R, 		" BCRM_ALL_BALANCE_RATIO_MIN_64R");
	dump_bcrm_reg_64(BCRM_ALL_BALANCE_RATIO_MAX_64R, 		" BCRM_ALL_BALANCE_RATIO_MAX_64R");

	dump_bcrm_reg_64(BCRM_RED_BALANCE_RATIO_64RW, 			" BCRM_RED_BALANCE_RATIO_64RW");
	dump_bcrm_reg_64(BCRM_RED_BALANCE_RATIO_MIN_64R, 		" BCRM_RED_BALANCE_RATIO_MIN_64R");
	dump_bcrm_reg_64(BCRM_RED_BALANCE_RATIO_MAX_64R, 		" BCRM_RED_BALANCE_RATIO_MAX_64R");
	dump_bcrm_reg_64(BCRM_RED_BALANCE_RATIO_INC_64R, 		" BCRM_RED_BALANCE_RATIO_INC_64R");

	dump_bcrm_reg_64(BCRM_GREEN_BALANCE_RATIO_64RW,			" BCRM_GREEN_BALANCE_RATIO_64RW");
	dump_bcrm_reg_64(BCRM_GREEN_BALANCE_RATIO_MIN_64R,		" BCRM_GREEN_BALANCE_RATIO_MIN_64R");
	dump_bcrm_reg_64(BCRM_GREEN_BALANCE_RATIO_MAX_64R, 		" BCRM_GREEN_BALANCE_RATIO_MAX_64R");
	dump_bcrm_reg_64(BCRM_GREEN_BALANCE_RATIO_INC_64R,		" BCRM_GREEN_BALANCE_RATIO_INC_64R");

	dump_bcrm_reg_64(BCRM_BLUE_BALANCE_RATIO_64RW, 			" BCRM_BLUE_BALANCE_RATIO_64RW");
	dump_bcrm_reg_64(BCRM_BLUE_BALANCE_RATIO_MIN_64R, 		" BCRM_BLUE_BALANCE_RATIO_MIN_64R");
	dump_bcrm_reg_64(BCRM_BLUE_BALANCE_RATIO_MAX_64R, 		" BCRM_BLUE_BALANCE_RATIO_MAX_64R");
	dump_bcrm_reg_64(BCRM_BLUE_BALANCE_RATIO_INC_64R, 		" BCRM_BLUE_BALANCE_RATIO_INC_64R");

	dump_bcrm_reg_8(BCRM_WHITE_BALANCE_AUTO_8RW, 			" BCRM_WHITE_BALANCE_AUTO_8RW");

	/* Other Registers */
	dump_bcrm_reg_32(BCRM_SHARPNESS_32RW, 				" BCRM_SHARPNESS_32RW");
	dump_bcrm_reg_32(BCRM_SHARPNESS_MIN_32R, 			" BCRM_SHARPNESS_MIN_32R");
	dump_bcrm_reg_32(BCRM_SHARPNESS_MAX_32R, 			" BCRM_SHARPNESS_MAX_32R");
	dump_bcrm_reg_32(BCRM_SHARPNESS_INC_32R, 			" BCRM_SHARPNESS_INC_32R");

	dump_bcrm_reg_32(BCRM_DEVICE_TEMPERATURE_32R, 			" BCRM_DEVICE_TEMPERATURE_32R");
	dump_bcrm_reg_64(BCRM_ANALOG_GAIN_64RW, 			" BCRM_ANALOG_GAIN_64RW");
	dump_bcrm_reg_64(BCRM_ANALOG_GAIN_MIN_64RW, 			" BCRM_ANALOG_GAIN_MIN_64RW");
	dump_bcrm_reg_64(BCRM_ANALOG_GAIN_MAX_64RW, 			" BCRM_ANALOG_GAIN_MAX_64RW");
}

static void dump_bcrm_reg_8(u16 nOffset, const char *pRegName)
{
	struct v4l2_i2c i2c_reg = {0};
	int status = 0;
	u8 data = 0;
	i2c_reg.reg = cci_reg.bcrm_address + nOffset;
	i2c_reg.reg_size = AV_CAM_REG_SIZE;
	i2c_reg.count = AV_CAM_DATA_SIZE_8;	
	i2c_reg.buffer = (char *)&data;
	status = i2c_generic_read(&i2c_reg);

	if (status >= 0)
		AV_DEBUG("%s: %u (0x%x)", pRegName, data, data);
	else
		AV_ERR("%s: ERROR", pRegName);
}

static void dump_bcrm_reg_32(u16 nOffset, const char *pRegName)
{
	struct v4l2_i2c i2c_reg = {0};
	int status = 0;
	u32 data = 0;
	i2c_reg.reg = cci_reg.bcrm_address + nOffset;
	i2c_reg.reg_size = AV_CAM_REG_SIZE;
	i2c_reg.count = AV_CAM_DATA_SIZE_32;	
	i2c_reg.buffer = (char *)&data;
	status = i2c_generic_read(&i2c_reg);

	swapbytes(&data, i2c_reg.count);
	if (status >= 0)
		AV_DEBUG("%s: %u (0x%08x)", pRegName, data, data);
	else
		AV_ERR("%s: ERROR", pRegName);
}

static void dump_bcrm_reg_64(u16 nOffset, const char *pRegName)
{
	struct v4l2_i2c i2c_reg = {0};
	int status = 0;
	u64 data = 0;
	i2c_reg.reg = cci_reg.bcrm_address + nOffset;
	i2c_reg.reg_size = AV_CAM_REG_SIZE;
	i2c_reg.count = AV_CAM_DATA_SIZE_64;	
	i2c_reg.buffer = (char *)&data;
	status = i2c_generic_read(&i2c_reg);

	swapbytes(&data, i2c_reg.count);
	if (status >= 0)
		AV_DEBUG("%s: %llu (0x%016llx)", pRegName, data, data);
	else
		AV_ERR("%s: ERROR", pRegName);
}

static int gcprm_version_check(void)
{
	AV_DEBUG("Driver GCPRM version: 0x%08lx", GCPRM_DEVICE_VERSION);
	AV_DEBUG("Driver GCPRM major version: 0x%04lx", GCPRM_MAJOR_VERSION);
	AV_DEBUG("Driver GCPRM minor version: 0x%04lx", GCPRM_MINOR_VERSION);

	AV_DEBUG("Camera GCPRM version: 0x%08lx", gencp_reg.gcprm_layout_version);
	AV_DEBUG("Camera GCPRM major version: 0x%04x", gencp_reg.gcprm_layout_version>>16);
	AV_DEBUG("Camera GCPRM minor version: 0x%04x", (int16_t) gencp_reg.gcprm_layout_version);

	return (gencp_reg.gcprm_layout_version>>16) == GCPRM_MAJOR_VERSION ? 1 : 0;
}

static int read_gencp_registers(void)
{
	int status = 0;
	struct v4l2_i2c i2c_reg = {};

	AV_DEBUG("cci_reg.gcprm_address=0x%x, sizeof(gencp_reg)=%d", cci_reg.gcprm_address, sizeof(gencp_reg));

	i2c_reg.reg = cci_reg.gcprm_address + 0x0000;
	i2c_reg.reg_size = AV_CAM_REG_SIZE;
	i2c_reg.count = sizeof(gencp_reg);

	i2c_reg.buffer = (char *)&gencp_reg;
	/*  Read  CCI registers */
	status = i2c_generic_read(&i2c_reg);

	if (status < 0) {
		return status;
	}

	swapbytes(&gencp_reg.gcprm_layout_version, AV_CAM_DATA_SIZE_32);
	swapbytes(&gencp_reg.gencp_out_buffer_address, AV_CAM_DATA_SIZE_16);
	swapbytes(&gencp_reg.gencp_in_buffer_address, AV_CAM_DATA_SIZE_16);
	swapbytes(&gencp_reg.gencp_out_buffer_size, AV_CAM_DATA_SIZE_16);
	swapbytes(&gencp_reg.gencp_in_buffer_size, AV_CAM_DATA_SIZE_16);

	AV_DEBUG("gencp_reg.gcprm_layout_version=0x%04x ", gencp_reg.gcprm_layout_version);
	AV_DEBUG("gencp_reg.gencp_out_buffer_address=0x%04x ", gencp_reg.gencp_out_buffer_address);
	AV_DEBUG("gencp_reg.gencp_in_buffer_address=0x%04x ", gencp_reg.gencp_in_buffer_address);
	AV_DEBUG("gencp_reg.gencp_out_buffer_size=0x%04x ", gencp_reg.gencp_out_buffer_size);
	AV_DEBUG("gencp_reg.gencp_in_buffer_size=0x%04x ", gencp_reg.gencp_in_buffer_size);

	read_gencp_out_buffer_size = gencp_reg.gencp_out_buffer_size;
	read_gencp_in_buffer_size = gencp_reg.gencp_in_buffer_size;

	return AV_CAM_ERR_SUCCESS;
}


/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	struct v4l2_standard standard;
	int ret = 0;
	int status = 0;
	__u64 value = 0;
	
	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:

		/* Reading the Current Frame Interval (FPS) */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_ACQUISITION_FRAME_RATE_64RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value);

		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_ACQUISITION_FRAME_RATE_64RW). (Status=%d)", status);
			return -EINVAL;
		}

		if (value == 0)
		{
			value = 1;
		}
		
		// camera stores framerate in uHz but V4L2 wants "time between frames in seconds" -> inverse fraction and convert
		standard.frameperiod.denominator = value;
		standard.frameperiod.numerator = MICRO_HZ;

		AV_DEBUG("BCRM_ACQUISITION_FRAME_RATE_64RW read from camera: %llu", value);

		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe = standard.frameperiod;
		cparm->capturemode = sensor->streamcap.capturemode;
		memset(cparm->reserved, 0, sizeof(cparm->reserved));
		cparm->readbuffers = 0;/* V4L2_CAP_STREAMING is set */

		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		AV_ERR("Unknown type %d", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	__u64 value_min = 0;
	__u64 value_max = 0;
	__u64 value_step = 0;
	int status = 0;
	__u64 value64 = 0;
	struct v4l2_i2c i2c_reg = {};

	int ret = 0;
	u32 tgt_fps;	/* target framerate in uHz (micro Hertz) */

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:

		/* Setting '0' to the Frame Start Trigger mode */
		i2c_bcrm_write_byte_reg(cci_reg.bcrm_address + BCRM_FRAME_START_TRIGGER_MODE_8RW, 0);/* Disable the trigger */

		/* Setting true ('1') in Acquisition Frame Rate Enable */
		i2c_bcrm_write_byte_reg(cci_reg.bcrm_address + BCRM_FRAME_RATE_ENABLE_8RW, 1);/* Enable the frame rate */

		/* Reading the Minimum Frame Interval (FPS) */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_ACQUISITION_FRAME_RATE_MIN_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *)&value_min);

		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_ACQUISITION_FRAME_RATE_MIN_64R). (Status=%d)", status);
			return -EINVAL;
		}

		/* Reading the Maximum Frame Interval (FPS) */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_ACQUISITION_FRAME_RATE_MAX_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value_max);

		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_ACQUISITION_FRAME_RATE_MAX_64R). (Status=%d)", status);
			return -EINVAL;
		}

		/* Reading the Step Frame Interval (FPS) */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_ACQUISITION_FRAME_RATE_INC_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value_step);

		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_ACQUISITION_FRAME_RATE_INC_64R). (Status=%d)", status);
			return -EINVAL;
		}

		AV_DEBUG("Convert v4l2 timeperframe: %d/%d", timeperframe->numerator, timeperframe->denominator);

		/* convert from "time between frames in seconds" to framerate in uHz (micro Hertz) */
		value64 = (__u64)timeperframe->denominator * (__u64)MICRO_HZ;
		value64 = div64_u64(value64, timeperframe->numerator);

		tgt_fps = (s32)value64;

		/* adjust value according to camera's min, max, inc restrictions */
		tgt_fps = convert_s_ctrl(tgt_fps, (__u32)value_min, (__u32)value_max, (__u32)value_step);

		value64 = (__u64)tgt_fps;

		AV_DEBUG("Write framerate to camera (BCRM_ACQUISITION_FRAME_RATE_64RW): %llu", value64);

		swapbytes(&value64, AV_CAM_DATA_SIZE_64);
		i2c_reg.reg = BCRM_ACQUISITION_FRAME_RATE_64RW + cci_reg.bcrm_address;
		i2c_reg.reg_size = AV_CAM_REG_SIZE;
		i2c_reg.count = AV_CAM_DATA_SIZE_64;
		i2c_reg.buffer = (const char *)&value64;

		//status = ioctl_gencam_i2cwrite_reg(s, &i2c_reg);
		status = i2c_bcrm_write_reg(&i2c_reg);

		if (status < 0) {
			return -EINVAL;
		}

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode = (u32)a->parm.capture.capturemode;
		a->parm.capture.capability = sensor->streamcap.capability;
		memset(a->parm.capture.reserved, 0, sizeof(a->parm.capture.reserved));

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		AV_ERR("Type is not V4L2_BUF_TYPE_VIDEO_CAPTURE but %d", a->type);
		ret = -EINVAL;
		break;

	default:
		AV_ERR("Unknown type %d", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}


/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;
	int retval = 0;
	struct v4l2_send_command_control ct;

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:

	/* Read the default width from allied vision camera */
		CLEAR(ct);
		ct.id = V4L2_AV_IMX_CSI2_WIDTH_R;
		retval = ioctl_send_command(s, &ct);

		if (retval < 0)
			return -EINVAL;
		av_cam_data.pix.width = ct.value0;

	/* Read the default height from allied vision camera */
		CLEAR(ct);
		ct.id = V4L2_AV_IMX_CSI2_HEIGHT_R;
		retval = ioctl_send_command(s, &ct);

		if (retval < 0)
			return -EINVAL;

		av_cam_data.pix.height = ct.value0;

		f->fmt.pix = sensor->pix;
		f->fmt.pix.width = av_cam_data.pix.width;
		f->fmt.pix.height = av_cam_data.pix.height;
		sensor->pix = f->fmt.pix;
		AV_DEBUG("w=%d, h=%d", sensor->pix.width, sensor->pix.height);
		break;

#ifndef WANDBOARD_IMX6
	case V4L2_BUF_TYPE_SENSOR:
		AV_DEBUG("l=%d, t=%d, w=%d, h=%d",
			sensor->spix.left, 
			sensor->spix.top,
			sensor->spix.swidth,
			sensor->spix.sheight);
		f->fmt.spix = sensor->spix;
		break;
#endif

	case V4L2_BUF_TYPE_PRIVATE:
		break;

	default:
		f->fmt.pix = sensor->pix;
		break;
	}

	return 0;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	int retval = 0;
	struct v4l2_send_command_control ct;
	struct v4l2_format tryfmt;

	if (fsize->index > 0)
		return -EINVAL;

	/* Read the default width from allied vision camera */
	CLEAR(ct);
	ct.id = V4L2_AV_IMX_CSI2_WIDTH_R;
	retval = ioctl_send_command(s, &ct);

	if (retval < 0)
		return -EINVAL;
	av_cam_data.pix.width = ct.value0;

	/* Read the default height from allied vision camera */
	CLEAR(ct);
	ct.id = V4L2_AV_IMX_CSI2_HEIGHT_R;
	retval = ioctl_send_command(s, &ct);

	if (retval < 0)
		return -EINVAL;

	av_cam_data.pix.height = ct.value0;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width =	av_cam_data.pix.width;
	fsize->discrete.height = av_cam_data.pix.height;

	/* Check if pixelformat is supported */
	CLEAR(tryfmt);
	tryfmt.fmt.pix.width = av_cam_data.pix.width;
	tryfmt.fmt.pix.height = av_cam_data.pix.height;
	tryfmt.fmt.pix.pixelformat = fsize->pixel_format;
	if (ioctl_try_fmt(s, &tryfmt))
	{
		AV_DEBUG("VIDIOC_ENUM_FRAMESIZES: pixelformat not supported!");
		return -EINVAL;
	}

	return 0;
}

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	struct v4l2_streamparm streamparm;
	struct v4l2_send_command_control ct;
	int ret = 0;

	if (fival->index > 0) {
		AV_DEBUG("enum frameintervals failed! fival->index=%d ", fival->index);
		return -EINVAL;
	}

	/* Read the default width from allied vision camera */
	CLEAR(ct);
	ct.id = V4L2_AV_IMX_CSI2_WIDTH_R;
	ret = ioctl_send_command(s, &ct);

	if (ret < 0)
		return -EINVAL;
	av_cam_data.pix.width = ct.value0;

	/* Read the default height from allied vision camera */
	CLEAR(ct);
	ct.id = V4L2_AV_IMX_CSI2_HEIGHT_R;
	ret = ioctl_send_command(s, &ct);

	if (ret < 0)
		return -EINVAL;

	av_cam_data.pix.height = ct.value0;

	if (!(fival->width == av_cam_data.pix.width
			&& fival->height == av_cam_data.pix.height))
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = MICRO_HZ;

	CLEAR(streamparm);
	streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = ioctl_g_parm(s, &streamparm);

	if (ret < 0) {
		AV_ERR("Failed to read FPS rate from camera");
		return -EINVAL;
	}

	fival->discrete.denominator = streamparm.parm.capture.timeperframe.denominator;

	AV_DEBUG("streamparm.parm.capture.timeperframe.denominator=%u", streamparm.parm.capture.timeperframe.denominator);
	AV_DEBUG("fival->pixel_format=%u, fival->width=%d, fival->height=%d, fival->index=%d",
			fival->pixel_format, fival->width, fival->height, fival->index);
	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type = V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
		AV_QUERYCAP_CAM_NAME);

	return 0;
}

static int convert_bcrm_to_v4l2(struct bcrm_to_v4l2 *bcrmv4l2, int conversion, bool abs)
{
	int32_t value = 0;
	int32_t min = 0;
	int32_t max = 0;
	int32_t step = 0;
	int32_t result = 0;
	int32_t valuedown = 0;
	int32_t valueup = 0;

	step = 1;
	max = S32_MAX;

    	/* 1. convert to double */
	if (conversion == min_enum) {
		value = (int32_t)bcrmv4l2->min_bcrm;
		//min = 1;
	} else if (conversion == max_enum) {
		value = (int32_t)bcrmv4l2->max_bcrm;
		min = bcrmv4l2->min_bcrm;
	} else if (conversion == step_enum) {
		value = (int32_t)bcrmv4l2->step_bcrm;
		min = S32_MIN;
	}

	/* 2. convert the units */
	/*	value *= factor; */
	if (abs) {
		if (value != 0)
			do_div(value, 1UL);
		if (min != 0)
			do_div(min, 1UL);
		if (max != 0)
			do_div(max, 1UL);
		
	}
	/* V4L2_CID_EXPOSURE_ABSOLUTE */
	else {
		if (value != 0)
			do_div(value, 100000UL);
		if (min != 0)
			do_div(min, 100000UL);
		if (max != 0)
			do_div(max, 100000UL);
	}

	/* 3. Round value to next integer */
	if (value < S32_MIN)
		result = S32_MIN;
	else if (value > S32_MAX)
		result = S32_MAX;
	else
		result = value;

	/* 4. Clamp to limits */
	if (result > max)
		result = max;
	else if (result < min)
		result = min;

	/* 5. Use nearest increment */
	valuedown = result - ((result - min) % (step));
	valueup = valuedown + step;

	if (result >= 0) {
		if (((valueup - result) <= (result - valuedown))
		&&  (valueup <= bcrmv4l2->max_bcrm))
			result = valueup;
		else
			result = valuedown;
	} else {
		if (((valueup - result) < (result - valuedown))
			&&  (valueup <= bcrmv4l2->max_bcrm))
			result = valueup;
		else
			result = valuedown;
	}

	if (conversion == min_enum)
		bcrmv4l2->min_v4l2 = result;
	else if (conversion == max_enum)
		bcrmv4l2->max_v4l2 = result;
	else if (conversion == step_enum)
		bcrmv4l2->step_v4l2 = result;
	return 0;
}

static int set_crop(struct v4l2_int_device *s, struct v4l2_crop *s_crop)
{
	int retval = 0;
	struct v4l2_send_command_control ctrl;
	struct sensor_data *sensor = s->priv;
	__s32 min = 0;
	__s32 max = 0;
	__s32 inc = 0;
	struct v4l2_rect *crop_current;

	crop_current = &sensor->crop_current;

	AV_DEBUG("SET CROPPING START:");
	AV_DEBUG("crop_current->left=%u", crop_current->left);
	AV_DEBUG("crop_current->top=%u", crop_current->top);
	AV_DEBUG("crop_current->width=%u", crop_current->width);
	AV_DEBUG("crop_current->height=%u", crop_current->height);

	AV_DEBUG("s_crop->c.left=%u", s_crop->c.left);
	AV_DEBUG("s_crop->c.top=%u", s_crop->c.top);
	AV_DEBUG("s_crop->c.width=%u", s_crop->c.width);
	AV_DEBUG("s_crop->c.height=%u", s_crop->c.height);

/*
*       As per the crop concept document, the following steps should be followed before setting crop to the sensor.
*
* i)    If new width is less or equal than current width, write the width register first then write offset X (left) register,
*       both values should be within the range (min, max and inc).
* ii)   If new width is higher than current width, write the offset X (left) register first then write the width register,
*       both values should be within the range (min, max, and inc)
* iii)  If new height is less or equal than current height, write the height register first then write offset Y (top) register,
*       both values should be within the range (min, max and inc).
* iv)   If new height is higher than current height, write the offset Y (top) register first then write the height register,
*       both values should be within the range (min, max, and inc)
*/
	if (s_crop->c.width <= crop_current->width) { /* step i) New width is lesser or equal than current */

		AV_DEBUG("New width is less or equal than current width");

		/* Write the width first then offset X */
		/* i) Read the Width Min value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_WIDTH_MINVAL_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		min = ctrl.value0;

		/* ii) Read the Width Max value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_WIDTH_MAXVAL_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		max = ctrl.value0;

		/* iii) Read the Width Increment value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_WIDTH_INCVAL_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		inc = ctrl.value0;

		/* Check whether the values are within the range of min, max and increment(step) */
		s_crop->c.width = convert_s_ctrl(s_crop->c.width, min, max, inc);

		/* iv) Write the Width value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_WIDTH_W;
		ctrl.value0 = s_crop->c.width;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		/* v) Read the Offset X Min value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_X_MIN_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		min = ctrl.value0;

		/* vi) Read the Offset X Max value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_X_MAX_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		max = ctrl.value0;

		/* vii) Read the Offset X Increment value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_X_INC_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		inc = ctrl.value0;

		/* Check whether the values are within the range of min, max and increment(step) */
		s_crop->c.left = convert_s_ctrl(s_crop->c.left, min, max, inc);

		/* viii) Write the Offset X value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_X_W;
		ctrl.value0 = s_crop->c.left;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

	} else { /*  step ii) New width is higher than current */

		AV_DEBUG("New width is higher than current width");

		/* Write the offset X first then width */

		/* i) Read the Offset X Min value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_X_MIN_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		min = ctrl.value0;

		/* ii) Read the Offset X Max value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_X_MAX_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		max = ctrl.value0;

		/* iii) Read the Offset X Increment value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_X_INC_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		inc = ctrl.value0;

		AV_DEBUG("OFFSET_X_MIN=%d", min);
		AV_DEBUG("OFFSET_X_MAX=%d", max);
		AV_DEBUG("OFFSET_X_INC=%d", inc);
		AV_DEBUG("s_crop->c.left=%d", s_crop->c.left);

		/* Check whether the values are within the range of min, max and increment(step) */
		s_crop->c.left = convert_s_ctrl(s_crop->c.left, min, max, inc);

		AV_DEBUG("s_crop->c.left=%d", s_crop->c.left);

		/* iv) Write the Offset X value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_X_W;
		ctrl.value0 = s_crop->c.left;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		/* v) Read the Width Min value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_WIDTH_MINVAL_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		min = ctrl.value0;

		/* vi) Read the Width Max value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_WIDTH_MAXVAL_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		max = ctrl.value0;

		/* vii) Read the Width Increment value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_WIDTH_INCVAL_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		inc = ctrl.value0;

		AV_DEBUG("WIDTH_MINVAL=%d", min);
		AV_DEBUG("WIDTH_MAXVAL=%d", max);
		AV_DEBUG("WIDTH_INCVAL=%d", inc);
		AV_DEBUG("s_crop->c.width=%d", s_crop->c.width);

		/* Check whether the values are within the range of min, max and increment(step) */
		s_crop->c.width = convert_s_ctrl(s_crop->c.width, min, max, inc);

		AV_DEBUG("s_crop->c.width=%d", s_crop->c.width);

		/* viii) Write the Width value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_WIDTH_W;
		ctrl.value0 = s_crop->c.width;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;
	}

	if (s_crop->c.height <= crop_current->height) { /* step iii) New height is lesser or equal than current */

		/* Write the height first then offset Y */

		AV_DEBUG("New height is less or equal than current height");

		/* i) Read the Height Min value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_HEIGHT_MINVAL_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		min = ctrl.value0;

		/* ii) Read the Height Max value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_HEIGHT_MAXVAL_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		max = ctrl.value0;

		/* iii) Read the Height Increment value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_HEIGHT_INCVAL_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		inc = ctrl.value0;

		AV_DEBUG("HEIGHT_MINVAL=%d", min);
		AV_DEBUG("HEIGHT_MAXVAL=%d", max);
		AV_DEBUG("HEIGHT_INCVAL=%d", inc);
		AV_DEBUG("s_crop->c.height=%d", s_crop->c.height);

		/* Check whether the values are within the range of min, max and increment(step) */
		s_crop->c.height = convert_s_ctrl(s_crop->c.height, min, max, inc);

		AV_DEBUG("s_crop->c.height=%d", s_crop->c.height);

		/* iv) Write the Height value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_HEIGHT_W;
		ctrl.value0 = s_crop->c.height;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		/* v) Read the Offset Y Min value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_Y_MIN_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		min = ctrl.value0;

		/* vi) Read the Offset Y Max value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_Y_MAX_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		max = ctrl.value0;

		/* vii) Read the Offset Y Increment value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_Y_INC_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		inc = ctrl.value0;

		AV_DEBUG("OFFSET_Y_MIN=%d", min);
		AV_DEBUG("OFFSET_Y_MAX=%d", max);
		AV_DEBUG("OFFSET_Y_INC=%d", inc);
		AV_DEBUG("s_crop->c.top=%d", s_crop->c.top);

		/* Check whether the values are within the range of min, max and increment(step) */
		s_crop->c.top = convert_s_ctrl(s_crop->c.top, min, max, inc);

		AV_DEBUG("s_crop->c.top=%d", s_crop->c.top);
		
		/* viii) Write the Offset Y value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_Y_W;
		ctrl.value0 = s_crop->c.top;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

	} else { /* step iv) New height is higher than current */

		AV_DEBUG("New height is higher than current height");

		/* Write the offset Y first then height */

		/* i) Read the Offset Y Min value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_Y_MIN_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		min = ctrl.value0;

		/* ii) Read the Offset Y Max value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_Y_MAX_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		max = ctrl.value0;

		/* iii) Read the Offset Y Increment value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_Y_INC_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		inc = ctrl.value0;

		AV_DEBUG("OFFSET_Y_MIN=%d", min);
		AV_DEBUG("OFFSET_Y_MAX=%d", max);
		AV_DEBUG("OFFSET_Y_INC=%d", inc);
		AV_DEBUG("s_crop->c.top=%d", s_crop->c.top);

		/* Check whether the values are within the range of min, max and increment(step) */
		s_crop->c.top = convert_s_ctrl(s_crop->c.top, min, max, inc);

		AV_DEBUG("s_crop->c.top=%d", s_crop->c.top);

		/* iv) Write the Offset Y value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_Y_W;
		ctrl.value0 = s_crop->c.top;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		/* v) Read the Height Min value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_HEIGHT_MINVAL_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		min = ctrl.value0;

		/* vi) Read the Height Max value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_HEIGHT_MAXVAL_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		max = ctrl.value0;

		/* vii) Read the Height Increment value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_HEIGHT_INCVAL_R;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;

		inc = ctrl.value0;

		AV_DEBUG("HEIGHT_MINVAL=%d", min);
		AV_DEBUG("HEIGHT_MAXVAL=%d", max);
		AV_DEBUG("HEIGHT_INCVAL=%d", inc);
		AV_DEBUG("s_crop->c.height=%d", s_crop->c.height);

		/* Check whether the values are within the range of min, max and increment(step) */
		s_crop->c.height = convert_s_ctrl(s_crop->c.height, min, max, inc);

		AV_DEBUG("s_crop->c.height=%d", s_crop->c.height);

		/* viii) Write the Height value */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_HEIGHT_W;
		ctrl.value0 = s_crop->c.height;
		retval = ioctl_send_command(s, &ctrl);

		if (retval < 0)
			return -EINVAL;
	}

	AV_DEBUG("crop_current->left=%u", crop_current->left);
	AV_DEBUG("crop_current->top=%u", crop_current->top);
	AV_DEBUG("crop_current->width=%u", crop_current->width);
	AV_DEBUG("crop_current->height=%u", crop_current->height);

	AV_DEBUG("s_crop->c.left=%u", s_crop->c.left);
	AV_DEBUG("s_crop->c.top=%u", s_crop->c.top);
	AV_DEBUG("s_crop->c.width=%u", s_crop->c.width);
	AV_DEBUG("s_crop->c.height=%u", s_crop->c.height);
	AV_DEBUG("SET CROPPING END");

	return 0;
}

static int get_crop_default(struct v4l2_int_device *s, struct v4l2_cropcap *s_crop)
{
	int retval = 0;
	struct v4l2_send_command_control ctrl;
	struct sensor_data *sensor = s->priv;
	struct v4l2_rect *crop_bounds = &sensor->crop_bounds;
	struct v4l2_rect *crop_defrect = &sensor->crop_defrect;

	/* Get the Minimum Left (Offset X) supported by the camera */
	CLEAR(ctrl);
	ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_X_MIN_R;
	retval = ioctl_send_command(s, &ctrl);

	if (retval < 0)
		return -EINVAL;

	crop_defrect->left = crop_bounds->left = ctrl.value0;
	AV_DEBUG("crop_defrect->left=%u, crop_bounds->left=%u", crop_defrect->left, crop_bounds->left);

	/* Get the Minimum Top (Offset Y) supported by the camera */
	CLEAR(ctrl);
	ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_Y_MIN_R;
	retval = ioctl_send_command(s, &ctrl);

	if (retval < 0)
		return -EINVAL;

	crop_defrect->top = crop_bounds->top = ctrl.value0;
	AV_DEBUG("crop_defrect->top=%u, crop_bounds->top=%u", crop_defrect->top, crop_bounds->top);

	/* Get the Maximum Width supported by the camera */
	CLEAR(ctrl);
	ctrl.id = V4L2_AV_IMX_CSI2_MAX_WIDTH_R;
        retval = ioctl_send_command(s, &ctrl);

        if (retval < 0)
                return -EINVAL;

        crop_defrect->width = crop_bounds->width = ctrl.value0;
        AV_DEBUG("crop_defrect->width=%u, crop_bounds->width=%u", crop_defrect->width, crop_bounds->width);

	/* Get the Maximum Height supported by the camera */
	CLEAR(ctrl);
	ctrl.id = V4L2_AV_IMX_CSI2_MAX_HEIGHT_R;
	retval = ioctl_send_command(s, &ctrl);

	if (retval < 0)
		return -EINVAL;

	crop_bounds->height = crop_defrect->height = ctrl.value0;
	AV_DEBUG("crop_bounds->height=%u, crop_defrect->height=%u", crop_bounds->height, crop_defrect->height );

	return 0;
}

static int ioctl_cropcap(struct v4l2_int_device *s, struct v4l2_cropcap *cropcap)
{
	return get_crop_default(s, cropcap);
}

static int ioctl_g_crop(struct v4l2_int_device *s, struct v4l2_crop *g_crop)
{
	int retval = 0;
	struct v4l2_send_command_control ctrl;

	AV_DEBUG("\n");

	/* Read the Width value */
	CLEAR(ctrl);
	ctrl.id = V4L2_AV_IMX_CSI2_WIDTH_R;
	retval = ioctl_send_command(s, &ctrl);

	if (retval < 0)
		return -EINVAL;

	g_crop->c.width = ctrl.value0;

	/* Read the Height value */
	CLEAR(ctrl);
	ctrl.id = V4L2_AV_IMX_CSI2_HEIGHT_R;
	retval = ioctl_send_command(s, &ctrl);

	if (retval < 0)
		return -EINVAL;

	g_crop->c.height = ctrl.value0;

	/* Read the Offset X (left) value */
	CLEAR(ctrl);
	ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_X_R;
	retval = ioctl_send_command(s, &ctrl);

	if (retval < 0)
		return -EINVAL;

	g_crop->c.left = ctrl.value0;

	/* Read the Offset Y (top) value */
	CLEAR(ctrl);
	ctrl.id = V4L2_AV_IMX_CSI2_OFFSET_Y_R;
	retval = ioctl_send_command(s, &ctrl);

	if (retval < 0)
		return -EINVAL;

	g_crop->c.top = ctrl.value0;

	AV_DEBUG("g_crop->c.width=%u", g_crop->c.width);
	AV_DEBUG("g_crop->c.height=%u", g_crop->c.height);
	AV_DEBUG("g_crop->c.left=%u", g_crop->c.left);
	AV_DEBUG("g_crop->c.top=%u", g_crop->c.top);

	return 0;
}

static int ioctl_s_crop(struct v4l2_int_device *s, struct v4l2_crop *s_crop)
{
	return set_crop(s, s_crop);
}

static int ioctl_querycap(struct v4l2_int_device *s, struct v4l2_capability *v4l2cap)
{
	memset(v4l2cap, 0, sizeof(*v4l2cap));
	strlcpy(v4l2cap->driver, AV_QUERYCAP_CAM_NAME, sizeof(v4l2cap->driver));
	v4l2cap->version = LINUX_VERSION_CODE;
	strlcpy(v4l2cap->card, cci_reg.model_name, sizeof(v4l2cap->card));
	snprintf(v4l2cap->bus_info, sizeof(v4l2cap->bus_info), "platform:%s", AV_QUERYCAP_BUSINFO);
	return 0;
}

static int avt_get_ctrl(struct v4l2_ext_control *ctrl)
{
	ssize_t status = 0;
	unsigned int reg = 0;
	int length = 0;
	uint64_t val64 = 0;
	uint32_t val32 = 0;

	switch (ctrl->id) {

		case V4L2_CID_HFLIP: {
			AV_DEBUG("V4L2_CID_HFLIP");
			reg = BCRM_IMG_REVERSE_X_8RW;
			length = AV_CAM_DATA_SIZE_8;
			break;
		}
		case V4L2_CID_VFLIP: {
			AV_DEBUG("V4L2_CID_VFLIP");
			reg = BCRM_IMG_REVERSE_Y_8RW;
			length = AV_CAM_DATA_SIZE_8;
			break;
		}
		/* BLACK LEVEL is deprecated and thus we use Brightness */
		case V4L2_CID_BRIGHTNESS: {
			AV_DEBUG("V4L2_CID_BRIGHTNESS");
			reg = BCRM_BLACK_LEVEL_32RW;
			length = AV_CAM_DATA_SIZE_32;
			break;
		}
		case V4L2_CID_GAMMA: {
			AV_DEBUG("V4L2_CID_GAMMA");
			reg = BCRM_GAMMA_64RW;
			length = AV_CAM_DATA_SIZE_64;
			break;
		}
		case V4L2_CID_CONTRAST: {
			AV_DEBUG("V4L2_CID_CONTRAST");
			reg = BCRM_CONTRAST_VALUE_32RW;
			length = AV_CAM_DATA_SIZE_32;
			break;
		}
		case V4L2_CID_DO_WHITE_BALANCE: {
			AV_DEBUG("V4L2_CID_DO_WHITE_BALANCE");
			reg = BCRM_WHITE_BALANCE_AUTO_8RW;
			length = AV_CAM_DATA_SIZE_8;
			break;
		}
		case V4L2_CID_AUTO_WHITE_BALANCE: {
			AV_DEBUG("V4L2_CID_AUTO_WHITE_BALANCE");
			reg = BCRM_WHITE_BALANCE_AUTO_8RW;
			length = AV_CAM_DATA_SIZE_8;
			break;
		}
		case V4L2_CID_SATURATION: {
			AV_DEBUG("V4L2_CID_SATURATION");
			reg = BCRM_SATURATION_32RW;
			length = AV_CAM_DATA_SIZE_32;
			break;
		}
		case V4L2_CID_HUE: {
			AV_DEBUG("V4L2_CID_HUE");
			reg = BCRM_HUE_32RW;
			length = AV_CAM_DATA_SIZE_32;
			break;
		}
		case V4L2_CID_RED_BALANCE: {
			AV_DEBUG("V4L2_CID_RED_BALANCE");
			reg = BCRM_RED_BALANCE_RATIO_64RW;
			length = AV_CAM_DATA_SIZE_64;
			break;
		}
		case V4L2_CID_BLUE_BALANCE: {
			AV_DEBUG("V4L2_CID_BLUE_BALANCE");
			reg = BCRM_BLUE_BALANCE_RATIO_64RW;
			length = AV_CAM_DATA_SIZE_64;
			break;
		}
		case V4L2_CID_EXPOSURE: {
			AV_DEBUG("V4L2_CID_EXPOSURE");
			reg = BCRM_EXPOSURE_TIME_64RW;
			length = AV_CAM_DATA_SIZE_64;
			break;
		}
		case V4L2_CID_GAIN: {
			AV_DEBUG("V4L2_CID_GAIN");
			reg = BCRM_GAIN_64RW;
			length = AV_CAM_DATA_SIZE_64;
			break;
		}
		case V4L2_CID_AUTOGAIN: {
			AV_DEBUG("V4L2_CID_AUTOGAIN");
			reg = BCRM_GAIN_AUTO_8RW;
			length = AV_CAM_DATA_SIZE_8;
			break;
		}
		case V4L2_CID_SHARPNESS: {
			AV_DEBUG("V4L2_CID_SHARPNESS");
			reg = BCRM_SHARPNESS_32RW;
			length = AV_CAM_DATA_SIZE_32;
			break;
		}

		case V4L2_CID_EXPOSURE_ABSOLUTE: {
			AV_DEBUG("V4L2_CID_EXPOSURE_ABSOLUTE");
			reg = BCRM_EXPOSURE_TIME_64RW;
			length = AV_CAM_DATA_SIZE_64;
			break;
		}

		case V4L2_CID_EXPOSURE_AUTO: {
			AV_DEBUG("V4L2_CID_EXPOSURE_AUTO");
			reg = BCRM_EXPOSURE_AUTO_8RW;
			length = AV_CAM_DATA_SIZE_8;
			break;
		}

		default: {
			AV_ERR("case default ctrl->id=%d", ctrl->id);
			return -EINVAL;
		}
	}

	if (length > AV_CAM_DATA_SIZE_32) {
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + reg,
						AV_CAM_REG_SIZE,
						length,
						(char *) &val64);
	} else {
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + reg,
						AV_CAM_REG_SIZE,
						length,
						(char *) &val32);
	}

	if (length > AV_CAM_DATA_SIZE_32)
		ctrl->value64 = val64;
	else {
		ctrl->value = val32;
		ctrl->value64 = ctrl->value;
		ctrl->value64 &= 0x00000000FFFFFFFF;
	}

	if (ctrl->id == V4L2_CID_EXPOSURE_ABSOLUTE) {
		if (ctrl->value64 != 0)
			do_div(ctrl->value64, 100000UL);/* Exposure absolute is 64bit REG. */
	}

	if (ctrl->id == V4L2_CID_EXPOSURE_AUTO)
	{
		/* Translate values from BCRM space to V4L2 space */
		AV_DEBUG("V4L2_CID_EXPOSURE_AUTO: BCRM input: ctrl->value=%d", ctrl->value);

		switch (ctrl->value)
		{
		case EXPOSURE_AUTO_CONTINUOUS:
			AV_DEBUG("EXPOSURE_AUTO_CONTINUOUS");
			ctrl->value = V4L2_EXPOSURE_AUTO;
			break;
		case EXPOSURE_AUTO_ONCE:
			AV_DEBUG("EXPOSURE_AUTO_ONCE");
			ctrl->value = V4L2_EXPOSURE_MANUAL;
			break;
		case EXPOSURE_AUTO_OFF:
			AV_DEBUG("EXPOSURE_AUTO_OFF");
			ctrl->value = V4L2_EXPOSURE_MANUAL;
			break;
		default:
			/* unsupported */
			AV_ERR("Unsupported V4L2_CID_EXPOSURE_AUTO value (%d)", ctrl->value);
			return -EINVAL;

		}

		AV_DEBUG("V4L2_CID_EXPOSURE_AUTO: v4l2 output: ctrl->value=%d", ctrl->value);
	}
	else
	if (ctrl->id == V4L2_CID_AUTOGAIN)
	{
		/* Translate values from BCRM space to V4L2 space */
		switch (ctrl->value)
		{
		case GAIN_AUTO_OFF:
			AV_DEBUG("GAIN_AUTO_OFF");
			ctrl->value = false;/* false (OFF) for off & once mode */
			break;
		case GAIN_AUTO_ONCE:
			AV_DEBUG("GAIN_AUTO_ONCE");
			ctrl->value = true; /* true (ON) for continous moe, */
			break;
		case GAIN_AUTO_CONTINUOUS:
			AV_DEBUG("GAIN_AUTO_CONTINUOUS");
			ctrl->value = true; /* true (ON) for continous moe, */
			break;
		default:
			/* unsupported */
			AV_ERR("Unsupported V4L2_CID_AUTOGAIN value (%d)", ctrl->value);
			return -EINVAL;

		}
	}
	else
	if (ctrl->id == V4L2_CID_AUTO_WHITE_BALANCE)
	{
		/* Translate values from BCRM space to V4L2 space */
		switch (ctrl->value)
		{
		case WHITEBALANCE_AUTO_OFF:
			AV_DEBUG("WHITEBALANCE_AUTO_OFF");
			ctrl->value = false;/* false (OFF) for off & once mode */
			break;
		case WHITEBALANCE_AUTO_ONCE:
			AV_DEBUG("WHITEBALANCE_AUTO_ONCE");
			ctrl->value = true; /* true (ON) for continous mode, */
			break;
		case WHITEBALANCE_AUTO_CONTINUOUS:
			AV_DEBUG("WHITEBALANCE_AUTO_CONTINUOUS");
			ctrl->value = true; /* true (ON) for continous mode, */
			break;
		default:
			/* unsupported */
			AV_ERR("Unsupported V4L2_CID_AUTOGAIN value (%d)", ctrl->value);
			return -EINVAL;

		}
	}

	AV_DEBUG("status=%d", status);

	return (status < 0) ? -EINVAL : 0;

#if 0
	status = i2c_generic_read_buffer(cci_reg.bcrm_address + reg,
					 AV_CAM_REG_SIZE, length, (char *) &val64);

	if (ctrl->id == V4L2_CID_EXPOSURE_ABSOLUTE) {
		do_div(val64, 100000UL);
		ctrl->value64 = val64;
	}

	if (ctrl->id == V4L2_CID_EXPOSURE_AUTO) {
		if (val64 == 2)
			ctrl->value64 = true;/* true (ON) for continous mode, Refer BCRM doc */
		else
			ctrl->value64 = false;/* false (OFF) for off & once mode, Refer BCRM doc */
	}

	return status < 0 ? AV_CAM_ERR_I2CREAD_FAIL:0;
#endif
}

static int avt_try_ctrl(struct v4l2_ext_control *ctrl)
{
	int err = 0;
	struct v4l2_int_device s;
	struct v4l2_queryctrl qctrl; // <- 32 bit values. v4l2_ext_control -> 64 bit values

	qctrl.id = ctrl->id;
	err = ioctl_queryctrl_new(&s, &qctrl);

	if (err < 0) {
		AV_ERR("queryctl failed. (Status=%d)", err);
		return -EINVAL;
	}

	if ((ctrl->value64 < (__s64)qctrl.minimum) || (ctrl->value64 > (__s64)qctrl.maximum)) {
		AV_ERR("Control 0x%x: Requested value not supported by camera. Value (%lld) out of range (min=%lld, max=%lld)", 
			qctrl.id, 
			ctrl->value64, 
			(__s64)qctrl.minimum, 
			(__s64)qctrl.maximum);
		return -EINVAL;
	}

	return 0;
}

static __s64 convert_s_ext_ctrl(__s64 val, __s64 min, __s64 max, __s64 step)
{
	int64_t valuedown = 0, valueup = 0;

	if (val > max)
		val = max;

	else if (val < min)
		val = min;

	if (step != 0)
		div64_u64_rem(val - min, step, &valuedown);

	valuedown = val - valuedown;

	valueup = valuedown + step;

	if (val >= 0) {
		if (((valueup - val) <= (val - valuedown)) &&  (valueup <= max))
			val = valueup;
		else
			val = valuedown;
	} else {
		if (((valueup - val) < (val - valuedown)) &&  (valueup <= max))
			val = valueup;
		else
			val = valuedown;
	}

	return val;
}

static int avt_set_ctrl(struct v4l2_ext_control *ctrl)
{
	unsigned int reg = 0;
	int length = 0;
	struct v4l2_control vc;
	struct v4l2_query_ext_ctrl qctrl;
	struct v4l2_int_device s;
	ssize_t status;
	int retval = 0;
	qctrl.id = ctrl->id;

	switch (ctrl->id) {

		case V4L2_CID_HFLIP: {
			AV_DEBUG("V4L2_CID_HFLIP");
			reg = BCRM_IMG_REVERSE_X_8RW;
			length = AV_CAM_DATA_SIZE_8;
			break;
		}
		case V4L2_CID_VFLIP: {
			AV_DEBUG("V4L2_CID_VFLIP");
			reg = BCRM_IMG_REVERSE_Y_8RW;
			length = AV_CAM_DATA_SIZE_8;
			break;
		}
		case V4L2_CID_DO_WHITE_BALANCE: {
			AV_DEBUG("V4L2_CID_DO_WHITE_BALANCE ctrl->value=%u", ctrl->value);
			reg = BCRM_WHITE_BALANCE_AUTO_8RW;
			length = AV_CAM_DATA_SIZE_8;
			ctrl->value = 1; /* Set 'once' in White Balance Auto Register. */
			break;
		}
		case V4L2_CID_BRIGHTNESS: {
			AV_DEBUG("V4L2_CID_BRIGHTNESS ctrl->value=%u", ctrl->value);
			reg = BCRM_BLACK_LEVEL_32RW;
			length = AV_CAM_DATA_SIZE_32;
			break;
		}
		case V4L2_CID_CONTRAST: {
			AV_DEBUG("V4L2_CID_CONTRAST ctrl->value=%u", ctrl->value);
			reg = BCRM_CONTRAST_VALUE_32RW;
			length = AV_CAM_DATA_SIZE_32;
			break;
		}
		case V4L2_CID_SATURATION: {
			AV_DEBUG("V4L2_CID_SATURATION ctrl->value=%u", ctrl->value);
			reg = BCRM_SATURATION_32RW;
			length = AV_CAM_DATA_SIZE_32;
			break;
		}
		case V4L2_CID_HUE: {
			AV_DEBUG("V4L2_CID_HUE ctrl->value=%u", ctrl->value);
			reg = BCRM_HUE_32RW;
			length = AV_CAM_DATA_SIZE_32;
			break;
		}
		case V4L2_CID_RED_BALANCE: {
			AV_DEBUG("V4L2_CID_RED_BALANCE ctrl->value64=%llu", ctrl->value64);
			reg = BCRM_RED_BALANCE_RATIO_64RW;
			length = AV_CAM_DATA_SIZE_64;

			retval = ioctl_ext_queryctrl(&s, &qctrl);

			if (retval < 0) {
				AV_ERR("ioctl_ext_queryctrl failed: retval=%d", retval);
				return -EINVAL;
			}

			ctrl->value64 = convert_s_ext_ctrl(ctrl->value64, qctrl.minimum, qctrl.maximum, qctrl.step);

			break;
		}
		case V4L2_CID_BLUE_BALANCE: {
			AV_DEBUG("V4L2_CID_BLUE_BALANCE ctrl->value64=%llu", ctrl->value64);
			reg = BCRM_BLUE_BALANCE_RATIO_64RW;
			length = AV_CAM_DATA_SIZE_64;

			retval = ioctl_ext_queryctrl(&s, &qctrl);

			if (retval < 0) {
				AV_ERR("ioctl_ext_queryctrl failed, retval=%d", retval);
				return -EINVAL;
			}

			ctrl->value64 = convert_s_ext_ctrl(ctrl->value64, qctrl.minimum, qctrl.maximum, qctrl.step);

			break;
		}
		case V4L2_CID_AUTO_WHITE_BALANCE: {
			AV_DEBUG("V4L2_CID_AUTO_WHITE_BALANCE ctrl->value=%u", ctrl->value);
			reg = BCRM_WHITE_BALANCE_AUTO_8RW;
			length = AV_CAM_DATA_SIZE_8;

			/* Translate values from V4L2 space to BCRM space */
			if (ctrl->value == true)
				ctrl->value = WHITEBALANCE_AUTO_CONTINUOUS;	/* Continouous mode */
			else
				ctrl->value = WHITEBALANCE_AUTO_OFF;		/* 1; OFF/once mode */

			break;
		}
		case V4L2_CID_GAMMA: {
			AV_DEBUG("V4L2_CID_GAMMA ctrl->value64=%llu", ctrl->value64);
			reg = BCRM_GAMMA_64RW;
			length = AV_CAM_DATA_SIZE_64;

			retval = ioctl_ext_queryctrl(&s, &qctrl);

			if (retval < 0) {
				AV_ERR("ioctl_ext_queryctrl failed: retval=%d", retval);
				return -EINVAL;
			}

			ctrl->value64 = convert_s_ext_ctrl(ctrl->value64, qctrl.minimum, qctrl.maximum, qctrl.step);

			break;
		}
		case V4L2_CID_EXPOSURE: {
			__s64 bkp_val = 0;
			AV_DEBUG("V4L2_CID_EXPOSURE, cci_reg.bcrm_address=0x%x, ctrl->value64=%llu", cci_reg.bcrm_address, ctrl->value64);

			bkp_val = ctrl->value64;
			/*  i) Setting 'Manual' in Exposure Auto reg. */
			ctrl->value = EXPOSURE_AUTO_OFF;

			AV_DEBUG("V4L2_CID_EXPOSURE, cci_reg.bcrm_address=0x%x, ctrl->value=%u", cci_reg.bcrm_address, ctrl->value);

			status = i2c_bcrm_write_reg_ctrl_ext(ctrl, BCRM_EXPOSURE_AUTO_8RW + cci_reg.bcrm_address, AV_CAM_DATA_SIZE_8);

			if (status < 0) {
				return -EINVAL;
			}

			/*  ii) Setting value in Exposure reg. */
			ctrl->value64 = bkp_val;

			retval = ioctl_ext_queryctrl(&s, &qctrl);

			if (retval < 0) {
				AV_ERR("ioctl_ext_queryctrl failed: retval=%d", retval);
				return -EINVAL;
			}

			ctrl->value64 = convert_s_ext_ctrl(ctrl->value64, qctrl.minimum, qctrl.maximum, qctrl.step);

			reg = BCRM_EXPOSURE_TIME_64RW;
			length = AV_CAM_DATA_SIZE_64;
			break;
		}
		case V4L2_CID_AUTOGAIN: {
			AV_DEBUG("V4L2_CID_AUTOGAIN vc->value=%u", ctrl->value);
			reg = BCRM_GAIN_AUTO_8RW;
			length = AV_CAM_DATA_SIZE_8;

			/* Translate values from V4L2 space to BCRM space */
			if (ctrl->value == true)
				ctrl->value = GAIN_AUTO_CONTINUOUS;	/* Continouous mode */
			else
				ctrl->value = GAIN_AUTO_OFF;		/* 1; OFF/once mode */

			break;
		}
		case V4L2_CID_GAIN: {
			AV_DEBUG("V4L2_CID_GAIN, ctrl->value64=%llu", ctrl->value64);
			reg = BCRM_GAIN_64RW;
			length = AV_CAM_DATA_SIZE_64;

			retval = ioctl_ext_queryctrl(&s, &qctrl);

			if (retval < 0) {
				AV_ERR("ioctl_ext_queryctrl failed: retval=%d", retval);
				return -EINVAL;
			}

			ctrl->value64 = convert_s_ext_ctrl(ctrl->value64, qctrl.minimum, qctrl.maximum, qctrl.step);

			break;
		}
		case V4L2_CID_SHARPNESS: {
			AV_DEBUG("V4L2_CID_SHARPNESS, ctrl->value=%u", ctrl->value);
			reg = BCRM_SHARPNESS_32RW;
			length = AV_CAM_DATA_SIZE_32;
			break;
		}

		case V4L2_CID_EXPOSURE_ABSOLUTE: {
			AV_DEBUG("V4L2_CID_EXPOSURE_ABSOLUTE");
			/* i) Setting 'Manual' in Exposure Auto reg.  */
			vc.value = EXPOSURE_AUTO_OFF;

			status = i2c_bcrm_write_reg_ctrl(&vc, BCRM_EXPOSURE_AUTO_8RW + cci_reg.bcrm_address, AV_CAM_DATA_SIZE_8);

			if (status < 0) {
				return -EINVAL;
			}

			/*  ii) Setting value in Exposure reg. */
			reg = BCRM_EXPOSURE_TIME_64RW;
			length = AV_CAM_DATA_SIZE_64;

			retval = ioctl_ext_queryctrl(&s, &qctrl);

			if (retval < 0) {
				AV_ERR("ioctl_ext_queryctrl failed: retval=%d", retval);
				return -EINVAL;
			}

			ctrl->value64 = convert_s_ext_ctrl(ctrl->value64, qctrl.minimum, qctrl.maximum, qctrl.step);

			ctrl->value64 = ctrl->value64 * EXP_ABS;

			break;
		}

		case V4L2_CID_EXPOSURE_AUTO: {
			AV_DEBUG("V4L2_CID_EXPOSURE_AUTO");
			reg = BCRM_EXPOSURE_AUTO_8RW;
			length = AV_CAM_DATA_SIZE_8;
			/* Translate values from V4L2 space to BCRM space */
			switch (ctrl->value64)
			{
			case V4L2_EXPOSURE_AUTO:
				
				ctrl->value64 = EXPOSURE_AUTO_CONTINUOUS;
				AV_DEBUG("V4L2_EXPOSURE_AUTO");
				break;
			case V4L2_EXPOSURE_MANUAL:
				AV_DEBUG("V4L2_EXPOSURE_MANUAL");
				ctrl->value64 = EXPOSURE_AUTO_OFF;
				break;
			default:
				/* unsupported */
				AV_ERR("Unsupported V4L2_CID_EXPOSURE_AUTO value (%lld)", ctrl->value64);
				return -EINVAL;

			}

			AV_DEBUG("V4L2_CID_EXPOSURE_AUTO: BCRM output: ctrl->value64=%lld", ctrl->value64);

			break;
		
		}
		default: {
			AV_ERR("case default, ctrl->id=%d", ctrl->id);
			return -EINVAL;
		}
	}

	status = i2c_bcrm_write_reg_ctrl_ext(ctrl, reg + cci_reg.bcrm_address, length);

	if (ctrl->id == V4L2_CID_EXPOSURE_ABSOLUTE) {
		if (ctrl->value64 != 0)
			do_div(ctrl->value64, EXP_ABS);
	}

	return (status < 0) ? -EINVAL : 0;

#if 0
	if (length > AV_CAM_DATA_SIZE_32)
		temp = ctrl->value64;

	if (length > AV_CAM_DATA_SIZE_32)
		swapbytes(&temp, length);
	else
		swapbytes(&ctrl->value64, length);

	i2c_reg.reg = reg + cci_reg.bcrm_address;
	i2c_reg.reg_size = AV_CAM_REG_SIZE;
	i2c_reg.count = length;
	if (length > AV_CAM_DATA_SIZE_32)
		i2c_reg.buffer = (const char *) &temp;
	else
		i2c_reg.buffer = (const char *) &ctrl->value64;

	status = ioctl_gencam_i2cwrite_reg(&s, &i2c_reg);

	if (status < 0) {
		AV_ERR("I2C write failed");
		return -1;
	}

	return 0;
#endif
}


static int ext_controls(struct v4l2_ext_controls *ctrls, unsigned int cmd)
{
	int err = 0;
	int i;

	if (cmd == VIDIOC_G_EXT_CTRLS) {

		for (i = 0; i < ctrls->count; i++) {
			struct v4l2_ext_control *ctrl = ctrls->controls + i;
			err = avt_get_ctrl(ctrl);

			if (err) {
				AV_ERR("avt_get_ctrl error=%d", err);
				ctrls->error_idx = i;
				break;
			}
		}
		return err;
	}

	if (cmd == VIDIOC_S_EXT_CTRLS) {

		for (i = 0; i < ctrls->count; i++) {
			struct v4l2_ext_control *ctrl = ctrls->controls + i;

			err = avt_set_ctrl(ctrl);
			if (err) {
				AV_ERR("avt_set_ctrl error=%d", err);
				ctrls->error_idx = i;
				break;
			}
		}
		return err;
	}

	if (cmd == VIDIOC_TRY_EXT_CTRLS) {

		for (i = 0; i < ctrls->count; i++) {
			struct v4l2_ext_control *ctrl = ctrls->controls + i;

			err = avt_try_ctrl(ctrl);
			if (err) {
				ctrls->error_idx = i;
				break;
			}
		}
		return err;
	}

	return err;
}

static int ioctl_g_ext_ctrl(struct v4l2_int_device *s, struct v4l2_ext_controls *g_ext_ctrls)
{
	return ext_controls(g_ext_ctrls, VIDIOC_G_EXT_CTRLS);
}

static int ioctl_s_ext_ctrl(struct v4l2_int_device *s, struct v4l2_ext_controls *s_ext_ctrls)
{
	return ext_controls(s_ext_ctrls, VIDIOC_S_EXT_CTRLS);
}

static int ioctl_try_ext_ctrl(struct v4l2_int_device *s, struct v4l2_ext_controls *try_ext_ctrls)
{
	return ext_controls(try_ext_ctrls, VIDIOC_TRY_EXT_CTRLS);
}

static int ioctl_query_menu(struct v4l2_int_device *s, struct v4l2_querymenu *query_menu)
{
	if ((NULL == s) || (NULL == query_menu))
	{
		return -1;
	}
	
	AV_DEBUG("query_menu->index=%d", query_menu->index);
	AV_DEBUG("query_menu->id=%d", query_menu->id);
	//AV_DEBUG("query_menu->value=%lld", query_menu->value);

	/*
	It is possible for VIDIOC_QUERYMENU to return an EINVAL error code for some indices 
	between minimum and maximum. In that case that particular menu item is not supported by 
	this driver. Also note that the minimum value is not necessarily 0.

	V4L2_EXPOSURE_AUTO 		Automatic exposure time, automatic iris aperture.
	V4L2_EXPOSURE_MANUAL 		Manual exposure time, manual iris.
	V4L2_EXPOSURE_SHUTTER_PRIORITY 	Manual exposure time, auto iris.
	V4L2_EXPOSURE_APERTURE_PRIORITY Auto exposure time, manual iris.
	*/

	switch (query_menu->id) 
	{
		case V4L2_CID_EXPOSURE_AUTO:
			switch (query_menu->index)
			{
				case 0:
					strcpy(query_menu->name, "Auto");
					break;
				case 1:
					strcpy(query_menu->name, "Manual");
					break;
				default:
					return -1;	
			}
			break;

		default:
			return -1;
	}

	AV_DEBUG("query_menu->name=%s", query_menu->name);
	return 0;
}

static int ioctl_queryctrl_new(struct v4l2_int_device *s, struct v4l2_queryctrl *qctrl)
{
	u64 value_feature = 0;
	u32 value = 0;
	u64 value64 = 0;
	int status;
	bcrm_feature_reg_t feature_inquiry_reg;
	struct bcrm_to_v4l2 bcrm_v4l2;

	/* Reading the Feature inquiry register */
	status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_FEATURE_INQUIRY_REG_64R,
				 	AV_CAM_REG_SIZE,
					AV_CAM_DATA_SIZE_64,
					(char *) &value_feature);
	if (status < 0) {
		return -EINVAL;
	}

	feature_inquiry_reg.value = value_feature;
	AV_DEBUG("feature_inquiry_reg.value=0x%llx", feature_inquiry_reg.value);

	switch (qctrl->id) {

	/* BLACK LEVEL is deprecated and thus we use Brightness */
	case V4L2_CID_BRIGHTNESS: {
		AV_DEBUG("case V4L2_CID_BRIGHTNESS");

		if (!feature_inquiry_reg.feature_inq.black_level_avail) {
			AV_DEBUG("Camera firmware doesn't support Black level/Brightness");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* Reading the current Black Level value */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_BLACK_LEVEL_32RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0)
			AV_ERR("I2C read failed (BCRM_BLACK_LEVEL_32RW). (Status=%d)", status);

		qctrl->default_value = value;

		/* Reading the Minimum Black Level */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_BLACK_LEVEL_MIN_32R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0)
			AV_ERR("I2C read failed (BCRM_BLACK_LEVEL_MIN_32R). (Status=%d)", status);

		qctrl->minimum = value;

		/* Reading the Maximum Black Level  */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_BLACK_LEVEL_MAX_32R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0)
			AV_ERR("I2C read failed (BCRM_BLACK_LEVEL_MAX_32R). (Status=%d)", status);

		qctrl->maximum = value;

		/* Reading the Black Level step increment */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_BLACK_LEVEL_INC_32R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0)
			AV_ERR("I2C read failed (BCRM_BLACK_LEVEL_INC_32R). (Status=%d)", status);

		qctrl->step = value;

		if ((qctrl->minimum > qctrl->maximum) || (qctrl->step <= 0)) {
			AV_ERR("BCRM value error. qctrl->minimum=%d, qctrl->maximum=%d, qctrl->step=%d", qctrl->minimum, qctrl->maximum, qctrl->step);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		strcpy(qctrl->name, "Brightness");
		break;
		}


	case V4L2_CID_EXPOSURE: {
		AV_DEBUG("case V4L2_CID_EXPOSURE");

		CLEAR(bcrm_v4l2);
		
		// Exposure time is mandatory!
		/*
		if (!feature_inquiry_reg.feature_inq.exposure_time_avail) {
			AV_DEBUG("Camera firmware doesn't support Exposure time");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}
		*/

		/* Reading the Exposure time (ns)*/
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_EXPOSURE_TIME_64RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0)
			AV_ERR("I2C read failed (BCRM_EXPOSURE_TIME_64RW). (Status=%d)", status);
		qctrl->default_value = (__s32) value64;

		/* Reading the Minimum Exposure time */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_EXPOSURE_TIME_MIN_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0)
			AV_ERR("I2C read failed (BCRM_EXPOSURE_TIME_MIN_64R). (Status=%d)", status);

		bcrm_v4l2.min_bcrm = value64;

		/* Reading the Maximum Exposure time  */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_EXPOSURE_TIME_MAX_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0)
			AV_ERR("I2C read failed (BCRM_EXPOSURE_TIME_MAX_64R). (Status=%d)", status);

		bcrm_v4l2.max_bcrm = value64;

		/* Reading the Exposure time step increment */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_EXPOSURE_TIME_INC_64R,
						AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0)
			AV_ERR("I2C read failed (BCRM_EXPOSURE_TIME_INC_64R). (Status=%d)", status);	
		
		bcrm_v4l2.step_bcrm = value64;

		convert_bcrm_to_v4l2(&bcrm_v4l2, min_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, max_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, step_enum, true);

		qctrl->minimum = bcrm_v4l2.min_v4l2;
		qctrl->maximum = bcrm_v4l2.max_v4l2;
		qctrl->step = bcrm_v4l2.step_v4l2;

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		strcpy(qctrl->name, "Exposure");
		break;
		}

	case V4L2_CID_EXPOSURE_ABSOLUTE: {
		AV_DEBUG("case V4L2_CID_EXPOSURE_ABSOLUTE");
		/* 100us unit */

		CLEAR(bcrm_v4l2);

		// Exposure time is mandatory!
		/*
		if (!feature_inquiry_reg.feature_inq.exposure_time_avail) {
			AV_DEBUG("Camera firmware doesn't support Exposure time (absolute)");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}
		*/

		/* Reading the Exposure time */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_EXPOSURE_TIME_64RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_EXPOSURE_TIME_64RW). (Status=%d)", status);
			return -EINVAL;
		}

		qctrl->default_value = (__s32) value64;
		if (qctrl->default_value != 0)
			do_div(qctrl->default_value, 100000UL);/* absolute conversion */

		/* Reading the Minimum Exposure time */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_EXPOSURE_TIME_MIN_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_EXPOSURE_TIME_MIN_64R). (Status=%d)", status);
			return -EINVAL;
		}

		bcrm_v4l2.min_bcrm = value64;

		/* Reading the Maximum Exposure time  */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_EXPOSURE_TIME_MAX_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_EXPOSURE_TIME_MAX_64R). (Status=%d)", status);
			return -EINVAL;
		}

		bcrm_v4l2.max_bcrm = value64;

		/* Reading the Exposure time step increment */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_EXPOSURE_TIME_INC_64R,
						AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_EXPOSURE_TIME_INC_64R). (Status=%d)", status);
			return -EINVAL;
		}

		bcrm_v4l2.step_bcrm = value64;

		convert_bcrm_to_v4l2(&bcrm_v4l2, min_enum, false);
		convert_bcrm_to_v4l2(&bcrm_v4l2, max_enum, false);
		qctrl->minimum = bcrm_v4l2.min_v4l2;
		qctrl->maximum = bcrm_v4l2.max_v4l2;
		qctrl->step = 1;

		qctrl->type = V4L2_CTRL_TYPE_INTEGER64;
		strcpy(qctrl->name, "Exposure Absolute");
		break;
		}

	case V4L2_CID_EXPOSURE_AUTO: {
		
		AV_DEBUG("case V4L2_CID_EXPOSURE_AUTO");

		if (!feature_inquiry_reg.feature_inq.exposure_auto) {
			AV_WARNING("Camera firmware doesn't support Exposure Auto");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* Reading the current exposure auto value */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_EXPOSURE_AUTO_8RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_8,
						(char *) &value);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_EXPOSURE_AUTO_8RW). (Status=%d)", status);
			return -EINVAL;
		}

		if (value == EXPOSURE_AUTO_CONTINUOUS)
			qctrl->default_value = V4L2_EXPOSURE_AUTO;/* true (ON) for continous mode, Refer BCRM doc */
		else
			qctrl->default_value = V4L2_EXPOSURE_MANUAL;/* false (OFF) */

		qctrl->minimum = 0;
		qctrl->step = 1;
		qctrl->maximum = 1;

		qctrl->type = V4L2_CTRL_TYPE_MENU;

		if ((qctrl->minimum > qctrl->maximum) || (qctrl->step <= 0)) {
			AV_ERR("BCRM value error. qctrl->minimum=%d, qctrl->maximum=%d, qctrl->step=%d", qctrl->minimum, qctrl->maximum, qctrl->step);
			return -EINVAL;
		}

		strcpy(qctrl->name, "Exposure Auto");
		break;
	}

	case V4L2_CID_GAIN: {
		AV_DEBUG("case V4L2_CID_GAIN");

		CLEAR(bcrm_v4l2);

		if (!feature_inquiry_reg.feature_inq.gain_avail) {
			AV_DEBUG("Camera firmware doesn't support Gain");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* Reading the Gain value */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_GAIN_64RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_GAIN_64RW). (Status=%d)", status);
			return -EINVAL;
		}

		qctrl->default_value = (__s32) value64;

		/* Reading the Minimum Gain value */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_GAIN_MIN_64R,
						AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_GAIN_MIN_64R). (Status=%d)", status);
			return -EINVAL;
		}

		bcrm_v4l2.min_bcrm = value64;

		/* Reading the Maximum Gain value  */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_GAIN_MAX_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_GAIN_MAX_64R). (Status=%d)", status);
			return -EINVAL;
		}

		bcrm_v4l2.max_bcrm = value64;

		/* Reading the Gain step increment */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_GAIN_INC_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_GAIN_INC_64R). (Status=%d)", status);
			return -EINVAL;
		}

		bcrm_v4l2.step_bcrm = value64;

		convert_bcrm_to_v4l2(&bcrm_v4l2, min_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, max_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, step_enum, true);

		qctrl->minimum = bcrm_v4l2.min_v4l2;
		qctrl->maximum = bcrm_v4l2.max_v4l2;
		qctrl->step = bcrm_v4l2.step_v4l2;
		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		strcpy(qctrl->name, "Gain");
		break;
	}


	case V4L2_CID_AUTOGAIN: {
		AV_DEBUG("case V4L2_CID_AUTOGAIN");

		if (!feature_inquiry_reg.feature_inq.gain_auto) {
			AV_DEBUG("Camera firmware doesn't support Gain Auto");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* Reading the Auto Gain value */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_GAIN_AUTO_8RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_8,
						(char *) &value);

		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_GAIN_AUTO_8RW). (Status=%d)", status);
			return -EINVAL;
		}

		if (value == 2)
			qctrl->default_value = true;/* true (ON) for continous mode, Refer BCRM doc */
		else
			qctrl->default_value = false;/* false (OFF) */

		qctrl->minimum = 0;
		qctrl->step = 1;
		qctrl->maximum = 1;

		qctrl->type = V4L2_CTRL_TYPE_BOOLEAN;

		if ((qctrl->minimum > qctrl->maximum) ||  (qctrl->step <= 0)) {
			AV_ERR("BCRM values read error ");
			return -EINVAL;
		}

#if 0
		if (status < 0)
			AV_ERR("I2C read failed, status %d", status);

		if (value == 2)
			qctrl->default_value = true;/* true (ON) */
		else
			qctrl->default_value = false;/* false (OFF) */

		qctrl->minimum = 0;
		qctrl->step = qctrl->maximum = 1;
		qctrl->type = V4L2_CTRL_TYPE_BOOLEAN;
#endif

		strcpy(qctrl->name, "Auto Gain");
		break;
	}

	case V4L2_CID_HFLIP:
		AV_DEBUG("case V4L2_CID_HFLIP\n");

		if (!feature_inquiry_reg.feature_inq.reverse_x_avail) {
			AV_DEBUG("Camera firmware doesn't support Reversing X (Horizontal Flip)");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* Reading the Reverse X value */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_IMG_REVERSE_X_8RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_8,
						(char *) &value);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_IMG_REVERSE_X_8RW). (Status=%d)", status);
			return -EINVAL;
		}

		qctrl->default_value = value;

		qctrl->minimum = 0;
		qctrl->step = qctrl->maximum = 1;
		qctrl->type = V4L2_CTRL_TYPE_BOOLEAN;
		strcpy(qctrl->name, "Reverse X");

		break;

	case V4L2_CID_VFLIP:
		AV_DEBUG("case V4L2_CID_VFLIP\n");
		if (!feature_inquiry_reg.feature_inq.reverse_y_avail) {
			AV_DEBUG("Camera firmware doesn't support Reversing Y (Vertical Flip) ");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* Reading the Reverse Y value */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_IMG_REVERSE_Y_8RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_8,
						(char *) &value);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_IMG_REVERSE_Y_8RW). (Status=%d)", status);
			return -EINVAL;
		}

		qctrl->default_value = value;

		qctrl->minimum = 0;
		qctrl->step = qctrl->maximum = 1;
		qctrl->type = V4L2_CTRL_TYPE_BOOLEAN;
		strcpy(qctrl->name, "Reverse Y");

		break;

	case V4L2_CID_GAMMA: {
		AV_DEBUG("case V4L2_CID_GAMMA");

		CLEAR(bcrm_v4l2);

		if (!feature_inquiry_reg.feature_inq.gamma_avail) {
			AV_DEBUG("Camera firmware doesn't support Gamma");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* Reading the Gamma value */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_GAMMA_64RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_GAMMA_64RW). (Status=%d)", status);
			return -EINVAL;
		}

		qctrl->default_value = (__s32) value64;

		/* Reading the Minimum Gamma */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_GAMMA_MIN_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_GAMMA_MIN_64R). (Status=%d)", status);
			return -EINVAL;
		}

		bcrm_v4l2.min_bcrm = value64;

		/* Reading the Maximum Gamma  */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_GAMMA_MAX_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_GAMMA_MAX_64R). (Status=%d)", status);
			return -EINVAL;
		}

		bcrm_v4l2.max_bcrm = value64;

		/* Reading the Gamma step increment */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_GAMMA_INC_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_GAMMA_INC_64R). (Status=%d)", status);
			return -EINVAL;
		}

		bcrm_v4l2.step_bcrm = value64;

		convert_bcrm_to_v4l2(&bcrm_v4l2, min_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, max_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, step_enum, true);

		qctrl->minimum = bcrm_v4l2.min_v4l2;
		qctrl->maximum = bcrm_v4l2.max_v4l2;
		qctrl->step = bcrm_v4l2.step_v4l2;

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		strcpy(qctrl->name, "Gamma");
		break;
		}

	case V4L2_CID_CONTRAST: {
		AV_DEBUG("case V4L2_CID_CONTRAST");

		if (!feature_inquiry_reg.feature_inq.contrast_avail) {
			AV_DEBUG("Camera firmware doesn't support Contrast");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* Reading the Contrast value */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_CONTRAST_VALUE_32RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_CONTRAST_VALUE_32RW). (Status=%d)", status);
			return -EINVAL;
		}

		qctrl->default_value = value;

		/* Reading the Minimum Contrast */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_CONTRAST_VALUE_MIN_32R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_CONTRAST_VALUE_MIN_32R). (Status=%d)", status);
			return -EINVAL;
		}

		qctrl->minimum = value;

		/* Reading the Maximum Contrast  */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_CONTRAST_VALUE_MAX_32R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_CONTRAST_VALUE_MAX_32R). (Status=%d)", status);
			return -EINVAL;
		}

		qctrl->maximum = value;

		/* Reading the Contrast step increment */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_CONTRAST_VALUE_INC_32R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_CONTRAST_VALUE_INC_32R). (Status=%d)", status);
			return -EINVAL;
		}

		qctrl->step = value;

		if ((qctrl->minimum > qctrl->maximum) ||  (qctrl->step <= 0)) {
			AV_ERR("BCRM value error. qctrl->minimum=%d, qctrl->maximum=%d, qctrl->step=%d", qctrl->minimum, qctrl->maximum, qctrl->step);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		strcpy(qctrl->name, "Contrast");
		break;

		}

	case V4L2_CID_AUTO_WHITE_BALANCE: {
		AV_DEBUG("case V4L2_CID_AUTO_WHITE_BALANCE");

		if (!feature_inquiry_reg.feature_inq.white_balance_auto_avail) {
			AV_DEBUG("Camera firmware doesn't support White balance Auto");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* Reading the White balance auto value */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_WHITE_BALANCE_AUTO_8RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_8,
						(char *) &value);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_WHITE_BALANCE_AUTO_8RW). (Status=%d)", status);
			return -EINVAL;
		}

		if (value == 2)
			qctrl->default_value = true;/* true (ON) */
		else
			qctrl->default_value = false;/* false (OFF) */


		qctrl->minimum = 0;
		qctrl->step = 1;
		qctrl->maximum = 1;
		qctrl->type = V4L2_CTRL_TYPE_BOOLEAN;
		strcpy(qctrl->name, "White Balance Auto");
		break;
		}

	case V4L2_CID_DO_WHITE_BALANCE: {
		AV_DEBUG("case V4L2_CID_DO_WHITE_BALANCE");

		if (!feature_inquiry_reg.feature_inq.white_balance_avail) {
			AV_DEBUG("Camera firmware doesn't support White balance");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* Reading the White balance auto reg */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_WHITE_BALANCE_AUTO_8RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_8,
						(char *) &value);
		if (status < 0) {
			AV_ERR("I2C read failed (BCMR BCRM_WHITE_BALANCE_AUTO_8RW). (Status=%d)", status);
			return -EINVAL;
		}

		if (value == 1)/* Once */
			qctrl->default_value = true;/* true (ON) */
		else
			qctrl->default_value = false;/* false (OFF) */


		qctrl->minimum = 0;
		qctrl->step = 1;
		qctrl->maximum = 1;
		qctrl->type = V4L2_CTRL_TYPE_BUTTON;
		strcpy(qctrl->name, "White Balance");
		break;
		}

	case V4L2_CID_SATURATION: {
		AV_DEBUG("case V4L2_CID_SATURATION");

		if (!feature_inquiry_reg.feature_inq.saturation_avail) {
			AV_DEBUG("Camera firmware doesn't support Saturation");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* Reading the Saturation value */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_SATURATION_32RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_SATURATION_32RW). (Status=%d)", status);
			return -EINVAL;
		}

		qctrl->default_value = value;

		/* Reading the Minimum Saturation */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_SATURATION_MIN_32R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_SATURATION_MIN_32R). (Status=%d)", status);
			return -EINVAL;
		}

		qctrl->minimum = value;

		/* Reading the Maximum Saturation  */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_SATURATION_MAX_32R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_SATURATION_MAX_32R). (Status=%d)", status);
			return -EINVAL;
		}

		qctrl->maximum = value;

		/* Reading the Saturation step increment */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_SATURATION_INC_32R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_SATURATION_INC_32R). (Status=%d)", status);
			return -EINVAL;
		}

		qctrl->step = value;

		if ((qctrl->minimum > qctrl->maximum) ||  (qctrl->step <= 0)) {
			AV_ERR("BCRM value error. qctrl->minimum=%d, qctrl->maximum=%d, qctrl->step=%d", qctrl->minimum, qctrl->maximum, qctrl->step);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		strcpy(qctrl->name, "Saturation");
		break;
	}

	case V4L2_CID_HUE: {
		AV_DEBUG("case V4L2_CID_HUE\n");

		if (!feature_inquiry_reg.feature_inq.hue_avail) {
			AV_DEBUG("Camera firmware doesn't support Hue");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* Reading the Hue value */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_HUE_32RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_HUE_32RW). (Status=%d)", status);
			return -EINVAL;
		}

		qctrl->default_value = value;

		/* Reading the Minimum HUE */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_HUE_MIN_32R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0) {
			AV_ERR("I2C read failed (BCRM_HUE_MIN_32R). (Status=%d)", status);
			return -EINVAL;
		}

		qctrl->minimum = value;

		/* Reading the Maximum HUE  */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_HUE_MAX_32R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0)
		{
			AV_ERR("I2C read failed (BCRM_HUE_MAX_32R). (Status=%d)", status);
			return -EINVAL;
		}
		qctrl->maximum = value;

		/* Reading the HUE step increment */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_HUE_INC_32R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0)
		{
			AV_ERR("I2C read failed (BCRM_HUE_INC_32R). (Status=%d)", status);
			return -EINVAL;
		}
		qctrl->step = value;

		if ((qctrl->minimum > qctrl->maximum) ||  (qctrl->step <= 0)) {
			AV_ERR("BCRM value error. qctrl->minimum=%d, qctrl->maximum=%d, qctrl->step=%d", qctrl->minimum, qctrl->maximum, qctrl->step);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		strcpy(qctrl->name, "Hue");
		break;
	}

	case V4L2_CID_RED_BALANCE: {
		AV_DEBUG("case V4L2_CID_RED_BALANCE");

		if (!feature_inquiry_reg.feature_inq.white_balance_avail) {
			AV_DEBUG("Camera firmware doesn't support White balance");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		CLEAR(bcrm_v4l2);

		/* Reading the Red balance value */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_RED_BALANCE_RATIO_64RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0)
		{
			AV_ERR("I2C read failed (BCRM_RED_BALANCE_RATIO_64RW). (Status=%d)", status);
			return -EINVAL;
		}
		qctrl->default_value = (__s32) value64;

		/* Reading the Minimum Red balance */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_RED_BALANCE_RATIO_MIN_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0)
		{
			AV_ERR("I2C read failed (BCRM_RED_BALANCE_RATIO_MIN_64R). (Status=%d)", status);
			return -EINVAL;
		}
		bcrm_v4l2.min_bcrm = value64;

		/* Reading the Maximum Red balance  */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_RED_BALANCE_RATIO_MAX_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0)
		{
			AV_ERR("I2C read failed (BCRM_RED_BALANCE_RATIO_MAX_64R). (Status=%d)", status);
			return -EINVAL;
		}
		bcrm_v4l2.max_bcrm = value64;

		/* Reading the Red balance step increment */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_RED_BALANCE_RATIO_INC_64R,
						AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0)
		{
			AV_ERR("I2C read failed (BCRM_RED_BALANCE_RATIO_INC_64R). (Status=%d)", status);
			return -EINVAL;
		}
		bcrm_v4l2.step_bcrm = value64;

		convert_bcrm_to_v4l2(&bcrm_v4l2, min_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, max_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, step_enum, true);

		qctrl->minimum = bcrm_v4l2.min_v4l2;
		qctrl->maximum = bcrm_v4l2.max_v4l2;
		qctrl->step = bcrm_v4l2.step_v4l2;

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		strcpy(qctrl->name, "Red Balance");
		break;
	}

	case V4L2_CID_BLUE_BALANCE: {
		AV_DEBUG("case V4L2_CID_BLUE_BALANCE");

		if (!feature_inquiry_reg.feature_inq.white_balance_avail) {
			AV_DEBUG("Camera firmware doesn't support White balance");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		CLEAR(bcrm_v4l2);

		/* Reading the Blue balance value */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_BLUE_BALANCE_RATIO_64RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0)
		{
			AV_ERR("I2C read failed (BCRM_BLUE_BALANCE_RATIO_64RW). (Status=%d)", status);
			return -EINVAL;
		}
		qctrl->default_value = (__s32) value64;

		/* Reading the Minimum Blue balance */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_BLUE_BALANCE_RATIO_MIN_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0)
		{
			AV_ERR("I2C read failed (BCRM_BLUE_BALANCE_RATIO_MIN_64R). (Status=%d)", status);
			return -EINVAL;
		}
		bcrm_v4l2.min_bcrm = value64;

		/* Reading the Maximum Blue balance  */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_BLUE_BALANCE_RATIO_MAX_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0)
		{
			AV_ERR("I2C read failed (BCRM_BLUE_BALANCE_RATIO_MAX_64R). (Status=%d)", status);
			return -EINVAL;
		}
		bcrm_v4l2.max_bcrm = value64;

		/* Reading the Blue balance step increment */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_BLUE_BALANCE_RATIO_INC_64R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &value64);
		if (status < 0)
		{
			AV_ERR("I2C read failed (BCRM_BLUE_BALANCE_RATIO_INC_64R). (Status=%d)", status);
			return -EINVAL;
		}
		bcrm_v4l2.step_bcrm = value64;

		convert_bcrm_to_v4l2(&bcrm_v4l2, min_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, max_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, step_enum, true);

		qctrl->minimum = bcrm_v4l2.min_v4l2;
		qctrl->maximum = bcrm_v4l2.max_v4l2;
		qctrl->step = bcrm_v4l2.step_v4l2;

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		strcpy(qctrl->name, "Blue Balance");
		break;
	}

	case V4L2_CID_SHARPNESS: {
		AV_DEBUG("case V4L2_CID_SHARPNESS");

		if (!feature_inquiry_reg.feature_inq.sharpness_avail) {
			AV_DEBUG("Camera firmware doesn't support Sharpness");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* Reading the Sharpness value */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_SHARPNESS_32RW,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0)
		{
			AV_ERR("I2C read failed (BCRM_SHARPNESS_32RW). (Status=%d)", status);
			return -EINVAL;
		}
		qctrl->default_value = value;

		/* Reading the Minimum sharpness */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_SHARPNESS_MIN_32R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0)
		{
			AV_ERR("I2C read failed (BCRM_SHARPNESS_MIN_32R). (Status=%d)", status);
			return -EINVAL;
		}
		qctrl->minimum = value;

		/* Reading the Maximum sharpness  */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_SHARPNESS_MAX_32R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0)
		{
			AV_ERR("I2C read failed (BCRM_SHARPNESS_MAX_32R). (Status=%d)", status);
			return -EINVAL;
		}
		qctrl->maximum = value;

		/* Reading the sharpness step increment */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_SHARPNESS_INC_32R,
					 	AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_32,
						(char *) &value);
		if (status < 0)
		{
			AV_ERR("I2C read failed (BCRM_SHARPNESS_INC_32R). (Status=%d)", status);
			return -EINVAL;
		}
		qctrl->step = value;

		if ((qctrl->minimum > qctrl->maximum) ||  (qctrl->step <= 0)) {
			AV_ERR("BCRM value error. qctrl->minimum=%d, qctrl->maximum=%d, qctrl->step=%d", qctrl->minimum, qctrl->maximum, qctrl->step);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		strcpy(qctrl->name, "Sharpness");
		break;
	}

	default:
		AV_DEBUG("case default or not supported qctrl->id 0x%x", qctrl->id);
		qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
		return 0;
	}

	AV_DEBUG("qctrl->id=%u (0x%08x)", qctrl->id, qctrl->id);
	AV_DEBUG("qctrl->type=%d", qctrl->type);
	AV_DEBUG("qctrl->name=%s", qctrl->name);
	AV_DEBUG("qctrl->minimum=%d", qctrl->minimum);
	AV_DEBUG("qctrl->maximum=%d", qctrl->maximum);
	AV_DEBUG("qctrl->step=%d", qctrl->step);
	AV_DEBUG("qctrl->default_value=%d", qctrl->default_value);
	AV_DEBUG("qctrl->flags=0x%x", qctrl->flags);

	return (status < 0) ? -EIO : 0;
}

static int ioctl_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *c)
{
	int i = 0; int next = 0;
	int retval = 0;
	u32 id = c->id;

	memset(c, 0, sizeof(*c));

	next = !!(id & V4L2_CTRL_FLAG_NEXT_CTRL);
	c->id = id & ~V4L2_CTRL_FLAG_NEXT_CTRL;

	if (next)
	{
		AV_DEBUG("V4L2_CTRL_FLAG_NEXT_CTRL");

		/* Loop through all possible controls */
		for (i = 0; i < ARRAY_SIZE(avt_v4l2_ctrls); i++) 
		{
			// Next control?
			if (c->id < avt_v4l2_ctrls[i])
				c->id = avt_v4l2_ctrls[i];	// Yes, test this id
			else
				continue;

			if (c->id == avt_v4l2_ctrls[i])
			{	
				AV_DEBUG("   Try control id: %d", c->id);
				retval = ioctl_queryctrl_new(s, c);
				if (!!(c->flags & V4L2_CTRL_FLAG_DISABLED))
				{
					AV_DEBUG("   Control disabled. Try next one...");
					memset(c, 0, sizeof(*c));
					continue;
				}
				else
				{
					AV_DEBUG("   Return next control id: %d (retval=%d)", c->id, retval);
					return retval;	
				}
			}
		}
	}
	else
	{
		return ioctl_queryctrl_new(s, c);
	}

	AV_DEBUG("Exit: -EINVAL");
	return -EINVAL;
}

static int avt_get_int32_64_def_val (struct avt_ctrl_mapping_t *avt_ctrl_mapping, s64 *def_val)
{
	int status;
	*def_val = 0;

	status = i2c_generic_read_buffer(cci_reg.bcrm_address + avt_ctrl_mapping->reg_offset,
					avt_ctrl_mapping->reg_size,
					avt_ctrl_mapping->data_size,
					(char *)def_val);
	if (status < 0)
		AV_ERR("I2C read failed. (Status=%d)", status);

	if (avt_ctrl_mapping->id == V4L2_CID_EXPOSURE_ABSOLUTE) {
		if (*def_val != 0)
			do_div(*def_val, 100000UL);/* Exposure absolute is 64bit REG. */
	}

	return status;
}


static int avt_get_menu_def_val (struct avt_ctrl_mapping_t *avt_ctrl_mapping, s64 *def_val)
{
	if (NULL == def_val)
		return -1;

	switch (avt_ctrl_mapping->id)
	{
		case V4L2_CID_EXPOSURE_AUTO:
			*def_val = 0;
			break;
		default:
			return -1;
	}
	
	return 0;
}


#if 0
static int avt_get_bool_def_val (struct avt_ctrl_mapping_t *avt_ctrl_mapping, s64 *def_val) {

	int status = avt_get_int32_64_def_val(avt_ctrl_mapping, def_val);
	*def_val = (*def_val == 2) ? true : false;	
	return status;
}

#define AVT_FILL_IDS(x, y) \
		memset(x, 0, sizeof *x); \
		x->id 	 = y->id; \
		x->type  = y->type; \
		x->flags = 0;

static avt_fill_ids (void *ctrl, struct avt_ctrl_mapping_t *avt_ctrl_mapping, bool ext_ctl)
{
	if(ext_ctl) {
		struct v4l2_query_ext_ctrl *qec = ctrl;
		AVT_FILL_IDS(qec, avt_ctrl_mapping);
	} else { 
		struct v4l2_queryctrl *qc = ctrl;
		AVT_FILL_IDS(qc, avt_ctrl_mapping);
	}
}
#endif

static int avt_v4l2_ext_queryctrl(struct v4l2_query_ext_ctrl *qec, struct avt_ctrl_mapping_t *avt_ctrl_mapping)
{
	u64 features = 0;
	__s64 value64;
	__s32 value32 = 0;
	int status;
	const struct v4l2_menu_info *menu;

	/* Reading the Feature inquiry register */
	status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_FEATURE_INQUIRY_REG_64R,
					AV_CAM_REG_SIZE,
					AV_CAM_DATA_SIZE_64,
					(char *) &features);
	if (status < 0)
		AV_ERR("I2C read failed. (Status=%d)", status);

	memset(qec, 0, sizeof *qec);
	qec->id = avt_ctrl_mapping->id;
	qec->type = avt_ctrl_mapping->type;
	qec->flags = 0;

	strcpy(qec->name, avt_ctrl_mapping->attr.name);
	qec->flags = avt_ctrl_mapping->flags;

	// Check if inquiry bit is set for the given control
	// Skip exposure because this register is mandatory, so there is no inquiry bit for that
	if (avt_ctrl_mapping->id != V4L2_CID_EXPOSURE &&
		avt_ctrl_mapping->id != V4L2_CID_EXPOSURE_ABSOLUTE &&
		!(features & (1<<(avt_ctrl_mapping->attr.feature_avail)))) {

		AV_DEBUG("Camera firmware doesn't support %s\n", avt_ctrl_mapping->attr.name);
		qec->flags = V4L2_CTRL_FLAG_DISABLED;
		return 0;
	}

	switch (qec->type) {
		
		case V4L2_CTRL_TYPE_MENU:
		   	qec->minimum = 0;
		   	qec->maximum = avt_ctrl_mapping->menu_count - 1;
		   	qec->step = 1;
		 	break;

		case V4L2_CTRL_TYPE_BOOLEAN:
			qec->minimum = 0;
			qec->maximum = 1;
			qec->step = 1;
			break;

		case V4L2_CTRL_TYPE_BUTTON:
			qec->minimum = 0;
			qec->maximum = 1;
			qec->step = 1;
			break;

		case V4L2_CTRL_TYPE_INTEGER:

			/* Reading the Minimum value */
			value32 = 0;
			status = i2c_generic_read_buffer(cci_reg.bcrm_address + avt_ctrl_mapping->min_offset,
							avt_ctrl_mapping->reg_size, 
							avt_ctrl_mapping->data_size, 
							(char *) &value32);
			qec->minimum = value32;
			
			/* Reading the Maximum value  */
			value32 = 0;
			status = i2c_generic_read_buffer(cci_reg.bcrm_address + avt_ctrl_mapping->max_offset,
							avt_ctrl_mapping->reg_size,
							avt_ctrl_mapping->data_size,
							(char *) &value32);

			qec->maximum = value32;

			/* Reading the step increment */
			value32 = 0;
			status = i2c_generic_read_buffer(cci_reg.bcrm_address + avt_ctrl_mapping->step_offset,
							avt_ctrl_mapping->reg_size,
							avt_ctrl_mapping->data_size,
							(char *) &value32);

			qec->step = value32;

			if (avt_ctrl_mapping->id == V4L2_CID_EXPOSURE_ABSOLUTE) {

				/* Exposure absolute is 64bit REG. */
				if (qec->step != 0)
					do_div(qec->step, 100000UL);
				if (qec->minimum != 0)
					do_div(qec->minimum, 100000UL);
				if (qec->maximum != 0)
					do_div(qec->maximum, 100000UL);

			}


			if (qec->minimum > qec->maximum) {
				AV_ERR("BCRM values read error qec->maximum=%lld, qec->minimum=%lld\n", qec->maximum, qec->minimum);
				//return 0;
			}
			break;
		case V4L2_CTRL_TYPE_INTEGER64:
			/* Reading the Minimum value */
			value64 = 0;
			status = i2c_generic_read_buffer(cci_reg.bcrm_address + avt_ctrl_mapping->min_offset,
							avt_ctrl_mapping->reg_size,
							avt_ctrl_mapping->data_size,
							(char *) &value64);

			qec->minimum = value64;

			/* Reading the Maximum value  */
			value64 = 0;
			status = i2c_generic_read_buffer(cci_reg.bcrm_address + avt_ctrl_mapping->max_offset,
							avt_ctrl_mapping->reg_size,
							avt_ctrl_mapping->data_size,
							(char *) &value64);

			qec->maximum = value64;

			/* Reading the step increment */
			value64 = 0;
			status = i2c_generic_read_buffer(cci_reg.bcrm_address + avt_ctrl_mapping->step_offset,
							avt_ctrl_mapping->reg_size,
							avt_ctrl_mapping->data_size,
							(char *) &value64);

			qec->step = value64;

			if (avt_ctrl_mapping->id == V4L2_CID_EXPOSURE_ABSOLUTE) {

				/* Exposure absolute is 64bit REG. */
				if (qec->step != 0)
					do_div(qec->step, 100000UL);
				if (qec->minimum != 0)
					do_div(qec->minimum, 100000UL);
				if (qec->maximum != 0)
					do_div(qec->maximum, 100000UL);

			}

			if (qec->minimum > qec->maximum) {
				AV_ERR("BCRM values read error qec->maximum %lld, qec->minimum %lld\n", qec->maximum, qec->minimum);
				//return 0;
			}
			break;

		default:
			AV_ERR("!!!Unexpected V4L2 type in Query Ctrl\n");
			return -EINVAL;
	}

	/* Reading the default value */	
	if(avt_ctrl_mapping->get_default_val(avt_ctrl_mapping, &qec->default_value) < 0) {
		//return 0;//return -EINVAL;
		AV_ERR("Error while getting default value");
	}

	if (status < 0) {
		return -EINVAL;
	} else {
		AV_DEBUG("Status=%d", status);
	}

	AV_DEBUG("qec->id=%u (0x%08X)", qec->id, qec->id);
	AV_DEBUG("qec->type=%d", qec->type);
	AV_DEBUG("qec->name=%s", qec->name);
	AV_DEBUG("qec->minimum=%lld", qec->minimum);
	AV_DEBUG("qec->maximum=%lld", qec->maximum);
	AV_DEBUG("qec->step=%llu", qec->step);
	AV_DEBUG("qec->default_value=%lld", qec->default_value);
	AV_DEBUG("qec->flags=0x%x", qec->flags);
	AV_DEBUG("qec->elem_size=%u", qec->elem_size);
	AV_DEBUG("qec->nr_of_dims=%u", qec->nr_of_dims);

	return 0;
}

static int get_avt_ctrl_mapping(u32 *id)
{
	int i = 0; int next = 0;

	next = !!(*id & (V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND));
	*id = *id & ~(V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND);

	for (i = 0; i < ARRAY_SIZE(avt_ctrl_mappings); i++) {
		if (next) {
			if (*id < avt_ctrl_mappings[i].id) {
				*id = avt_ctrl_mappings[i].id;
			}
			else {
				continue;
			}
		}

		if (*id == avt_ctrl_mappings[i].id) {
			return i;
		}

		if (*id < avt_ctrl_mappings[i].id) {
			break;
		}
	}

	return -EINVAL;
}

static int ioctl_ext_queryctrl(struct v4l2_int_device *s, struct v4l2_query_ext_ctrl *qec)
{
	int ret;
	//struct v4l2_queryctrl qc;

	ret = get_avt_ctrl_mapping(&qec->id);
	if(ret < 0)
		return ret;

	ret = avt_v4l2_ext_queryctrl(qec, &avt_ctrl_mappings[ret]);
	if(ret < 0)
		return ret;

	qec->elem_size = 4;
	qec->elems = 1;
	qec->nr_of_dims = 0;
	memset(qec->dims, 0, sizeof(qec->dims));
	memset(qec->reserved, 0, sizeof(qec->reserved));

	return ret;
#if 0
	memset(qec, 0, sizeof(*qec));
	qec->id = qc.id;
	qec->type = qc.type;
	strlcpy(qec->name, qc.name, sizeof(qec->name));
	qec->minimum = qc.minimum;
	qec->maximum = qc.maximum;
	qec->step = qc.step;
	qec->default_value = qc.default_value;
	qec->flags = qc.flags;
	qec->elem_size = 4;
	qec->elems = 1;
	qec->nr_of_dims = 0;
	memset(qec->dims, 0, sizeof(qec->dims));
	memset(qec->reserved, 0, sizeof(qec->reserved));
	return ret;
#endif
}

static int32_t convert_bcrm_to_v4l2_gctrl(struct bcrm_to_v4l2 *bcrmv4l2, int64_t val64, bool abs)
{
	int32_t value = 0;
	int32_t min = 0;
	int32_t max = 0;
	int32_t step = 0;
	int32_t result = 0;
	int32_t valuedown = 0;
	int32_t valueup = 0;

	/* 1. convert to double */
	step = bcrmv4l2->step_v4l2;
	max = bcrmv4l2->max_v4l2;
	min = bcrmv4l2->min_v4l2;
	value = (int32_t) val64;


	/* 2. convert the units */
/*	value *= factor; */

#if 0
	if (abs)/* V4L2_CID_EXPOSURE_ABSOLUTE */
		do_div(value, 100000UL);
	else
		do_div(value, 1UL);
#endif

	/* V4L2_CID_EXPOSURE_ABSOLUTE */
	if(abs) {
		if (value != 0)
			do_div(value, 100000UL);
	}

	/* 3. Round value to next integer */

	if (value < S32_MIN)
		result = S32_MIN;
	else if (value > S32_MAX)
		result = S32_MAX;
	else
		result = value;

	/* 4. Clamp to limits */
	if (result > max)
		result = max;
	else if (result < min)
		result = min;


	/* 5. Use nearest increment */
	valuedown = result - ((result - min) % (step));
	valueup = valuedown + step;

	if (result >= 0) {
		if (((valueup - result) <= (result - valuedown))
		&&  (valueup <= bcrmv4l2->max_bcrm))
			result = valueup;
		else
			result = valuedown;
	} else {
		if (((valueup - result) < (result - valuedown))
			&&  (valueup <= bcrmv4l2->max_bcrm))
			result = valueup;
		else
			result = valuedown;
	}

	return result;
}


/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	ssize_t status = 0;
	unsigned int reg = 0;
	int length = 0;
	struct bcrm_to_v4l2 bcrm_v4l2;
	struct v4l2_queryctrl qctrl;
	struct sensor_data *sensor = s->priv;
	int retval = 0;
	uint64_t val64 = 0;

	switch (vc->id) {

/* BLACK LEVEL is deprecated and thus we use Brightness */
	case V4L2_CID_BRIGHTNESS:
		AV_DEBUG("V4L2_CID_BRIGHTNESS");
		reg = BCRM_BLACK_LEVEL_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_CID_GAMMA:
		AV_DEBUG("V4L2_CID_GAMMA");
		reg = BCRM_GAMMA_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;
	case V4L2_CID_CONTRAST:
		AV_DEBUG("V4L2_CID_CONTRAST");
		vc->value = av_cam_data.contrast;
		reg = BCRM_CONTRAST_VALUE_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		AV_DEBUG("V4L2_CID_DO_WHITE_BALANCE");
		reg = BCRM_WHITE_BALANCE_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		AV_DEBUG("V4L2_CID_AUTO_WHITE_BALANCE");
		reg = BCRM_WHITE_BALANCE_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;
	case V4L2_CID_SATURATION:
		AV_DEBUG("V4L2_CID_SATURATION");
		vc->value = av_cam_data.saturation;
		reg = BCRM_SATURATION_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_CID_HUE:
		AV_DEBUG("V4L2_CID_HUE");
		vc->value = av_cam_data.hue;
		reg = BCRM_HUE_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_CID_RED_BALANCE:
		AV_DEBUG("V4L2_CID_RED_BALANCE");
		vc->value = av_cam_data.red;
		reg = BCRM_RED_BALANCE_RATIO_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;
	case V4L2_CID_BLUE_BALANCE:
		AV_DEBUG("V4L2_CID_BLUE_BALANCE");
		vc->value = av_cam_data.blue;
		reg = BCRM_BLUE_BALANCE_RATIO_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;
#if 0
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		AV_DEBUG("V4L2_CID_EXPOSURE_ABSOLUTE, sensor->ae_mode %d", sensor->ae_mode);
#endif
	case V4L2_CID_EXPOSURE:
		AV_DEBUG("V4L2_CID_EXPOSURE, sensor->ae_mode %d", sensor->ae_mode);
		vc->value = av_cam_data.expos;
		reg = BCRM_EXPOSURE_TIME_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;

#if 0
	case V4L2_CID_EXPOSURE_AUTO:
		AV_DEBUG("V4L2_CID_EXPOSURE_AUTO");
		vc->value = av_cam_data.expos_auto;
		reg = BCRM_EXPOSURE_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;
#endif

	case V4L2_CID_GAIN:
		AV_DEBUG("V4L2_CID_GAIN");
		vc->value = av_cam_data.gain;
		reg = BCRM_GAIN_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;

	case V4L2_CID_AUTOGAIN:
		AV_DEBUG("V4L2_CID_AUTOGAIN");
		vc->value = av_cam_data.gain_auto;
		reg = BCRM_GAIN_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;

	case V4L2_CID_SHARPNESS:
		AV_DEBUG("V4L2_CID_SHARPNESS");
		reg = BCRM_SHARPNESS_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;

	default:
		AV_ERR("   Unsupported control ID. vc->id=%d (0x%x)", vc->id, vc->id);
		return -EINVAL;
	}

	CLEAR(bcrm_v4l2);
	CLEAR(qctrl);

	qctrl.id = vc->id;
	retval = ioctl_queryctrl_new(s, &qctrl);

	if (retval < 0) {
		AV_ERR("queryctrl failed!");
		return -EINVAL;
	}

	bcrm_v4l2.min_v4l2 = qctrl.minimum;
	bcrm_v4l2.max_v4l2 = qctrl.maximum;
	bcrm_v4l2.step_v4l2 = qctrl.step;

	/* Overwrite the queryctrl maximum value for auto features since value 2 is 'true' (1) */
	if (vc->id == V4L2_CID_AUTOGAIN || vc->id == V4L2_CID_AUTO_WHITE_BALANCE)
		bcrm_v4l2.max_v4l2 = 2;

	/* Check values from BCRM */
	if ((bcrm_v4l2.min_v4l2 > bcrm_v4l2.max_v4l2) || (bcrm_v4l2.step_v4l2 <= 0)) {
		AV_DEBUG("Invalid BCRM values found. vc->id=%d, bcrm_v4l2.min_v4l2=%d, bcrm_v4l2.max_v4l2=%d, bcrm_v4l2.step_v4l2=%d", vc->id, bcrm_v4l2.min_v4l2, bcrm_v4l2.max_v4l2, bcrm_v4l2.step_v4l2);
		return -EINVAL;
	}

	status = i2c_generic_read_buffer(cci_reg.bcrm_address + reg,
					AV_CAM_REG_SIZE,
					length, 
					(char *) &val64);

	if (vc->id == V4L2_CID_EXPOSURE_ABSOLUTE)
		vc->value = convert_bcrm_to_v4l2_gctrl(&bcrm_v4l2, val64, true);/* Absolute */
	else
		vc->value = convert_bcrm_to_v4l2_gctrl(&bcrm_v4l2, val64, false);

	/* BCRM Auto Exposure changes */
	if (vc->id == V4L2_CID_EXPOSURE_AUTO)
	{
		if (vc->value == EXPOSURE_AUTO_CONTINUOUS)
			vc->value = V4L2_EXPOSURE_AUTO;
		else
			vc->value = V4L2_EXPOSURE_MANUAL;
		
	}
	/* BCRM Auto Gain/WB changes */
	if (vc->id == V4L2_CID_AUTOGAIN || vc->id == V4L2_CID_AUTO_WHITE_BALANCE) {

		if (vc->value == 2)
			vc->value = true;/* true (ON) for continous mode, Refer BCRM doc */
		else
			vc->value = false;/* false (OFF) for off & once mode, Refer BCRM doc */

	}

	return status < 0 ? AV_CAM_ERR_I2CREAD_FAIL:0;
}

static __s32 convert_s_ctrl(__s32 val, __s32 min, __s32 max, __s32 step)
{
	int32_t valuedown = 0, valueup = 0;

	AV_DEBUG("val=%d, min=%d, max=%d, step=%d", val, min, max, step);

	if(step < 1)
	{
		AV_WARNING("Warning: given step (increment) is 0 -> adjust step to 1");
		step = 1;
	}

	if (val > max)
		val = max;

	else if (val < min)
		val = min;

	valuedown = val - ((val - min) % step);
	valueup = valuedown + step;

	AV_DEBUG("valuedown=%d, valueup=%d", valuedown, valueup);

	if (val >= 0) {
		if (((valueup - val) <= (val - valuedown)) &&  (valueup <= max))
			val = valueup;
		else
			val = valuedown;
	} else {
		if (((valueup - val) < (val - valuedown)) &&  (valueup <= max))
			val = valueup;
		else
			val = valuedown;
	}

	AV_DEBUG("corrected val=%d", val);

	return val;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;
	unsigned int reg = 0;
	int length = 0;
	__s32 value_bkp = 0;
	struct v4l2_queryctrl qctrl;

	switch (vc->id) {

	case V4L2_CID_DO_WHITE_BALANCE:
		AV_DEBUG("V4L2_CID_DO_WHITE_BALANCE vc->value=%u", vc->value);
		reg = BCRM_WHITE_BALANCE_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;
		vc->value = 1; /*  Set 'once' in White Balance Auto Register. */
		break;

	/* BLACK LEVEL is deprecated and thus we use Brightness */
	case V4L2_CID_BRIGHTNESS:
		AV_DEBUG("V4L2_CID_BRIGHTNESS vc->value=%u", vc->value);
		reg = BCRM_BLACK_LEVEL_32RW;
		length = AV_CAM_DATA_SIZE_32;

		qctrl.id = V4L2_CID_BRIGHTNESS;/* V4L2_CID_BLACK_LEVEL; */

		retval = ioctl_queryctrl_new(s, &qctrl);

		if (retval < 0) {
			AV_ERR("queryctrl failed, retval=%d", retval);
			return -EINVAL;
		}
		vc->value = convert_s_ctrl(vc->value, qctrl.minimum, qctrl.maximum, qctrl.step);
		break;

	case V4L2_CID_CONTRAST:
		AV_DEBUG("V4L2_CID_CONTRAST vc->value=%u", vc->value);
		reg = BCRM_CONTRAST_VALUE_32RW;
		length = AV_CAM_DATA_SIZE_32;

		qctrl.id = V4L2_CID_CONTRAST;
		retval = ioctl_queryctrl_new(s, &qctrl);

		if (retval < 0) {
			AV_ERR("queryctrl failed!");
			return -EINVAL;
		}

		vc->value = convert_s_ctrl(vc->value, qctrl.minimum, qctrl.maximum, qctrl.step);
		break;
	case V4L2_CID_SATURATION:
		AV_DEBUG("V4L2_CID_SATURATION vc->value=%u", vc->value);
		reg = BCRM_SATURATION_32RW;
		length = AV_CAM_DATA_SIZE_32;

		qctrl.id = V4L2_CID_SATURATION;
		retval = ioctl_queryctrl_new(s, &qctrl);

		if (retval < 0) {
			AV_ERR("queryctrl failed!");
			return -EINVAL;
		}

		vc->value = convert_s_ctrl(vc->value, qctrl.minimum, qctrl.maximum, qctrl.step);
		break;
	case V4L2_CID_HUE:
		AV_DEBUG("V4L2_CID_HUE vc->value=%u", vc->value);
		reg = BCRM_HUE_32RW;
		length = AV_CAM_DATA_SIZE_32;

		qctrl.id = V4L2_CID_HUE;
		retval = ioctl_queryctrl_new(s, &qctrl);

		if (retval < 0) {
			AV_ERR("queryctrl failed!");
			return -EINVAL;
		}

		vc->value = convert_s_ctrl(vc->value, qctrl.minimum, qctrl.maximum, qctrl.step);

		break;
	case V4L2_CID_RED_BALANCE:
		AV_DEBUG("V4L2_CID_RED_BALANCE vc->value=%u", vc->value);
		reg = BCRM_RED_BALANCE_RATIO_64RW;
		length = AV_CAM_DATA_SIZE_64;

		qctrl.id = V4L2_CID_RED_BALANCE;
		retval = ioctl_queryctrl_new(s, &qctrl);

		if (retval < 0) {
			AV_ERR("queryctrl failed!");
			return -EINVAL;
		}

		vc->value = convert_s_ctrl(vc->value, qctrl.minimum, qctrl.maximum, qctrl.step);
		break;
	case V4L2_CID_BLUE_BALANCE:
		AV_DEBUG("V4L2_CID_BLUE_BALANCE vc->value=%u", vc->value);
		reg = BCRM_BLUE_BALANCE_RATIO_64RW;
		length = AV_CAM_DATA_SIZE_64;

		qctrl.id = V4L2_CID_BLUE_BALANCE;
		retval = ioctl_queryctrl_new(s, &qctrl);

		if (retval < 0) {
			AV_ERR("queryctrl failed!");
			return -EINVAL;
		}

		vc->value = convert_s_ctrl(vc->value, qctrl.minimum, qctrl.maximum, qctrl.step);
		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		AV_DEBUG("V4L2_CID_AUTO_WHITE_BALANCE vc->value=%u", vc->value);
		reg = BCRM_WHITE_BALANCE_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;

		/* BCRM Auto White balance changes	 */
		if (vc->value == true)
			vc->value = 2;/* Continouous mode */
		else
			vc->value = 0;/* 1; OFF/once mode */

		break;
	case V4L2_CID_GAMMA:
		AV_DEBUG("V4L2_CID_GAMMA vc->value=%u", vc->value);
		reg = BCRM_GAMMA_64RW;
		length = AV_CAM_DATA_SIZE_64;

		qctrl.id = V4L2_CID_GAMMA;
		retval = ioctl_queryctrl_new(s, &qctrl);

		if (retval < 0) {
			AV_ERR("queryctrl failed!");
			return -EINVAL;
		}
		vc->value = convert_s_ctrl(vc->value, qctrl.minimum, qctrl.maximum, qctrl.step);
		break;
	case V4L2_CID_EXPOSURE:
		AV_DEBUG("V4L2_CID_EXPOSURE, cci_reg.bcrm_address=0x%x, vc->value=%u", cci_reg.bcrm_address, vc->value);

		value_bkp = vc->value;/* backup the actual value */

		/*  i) Setting 'Manual' in Exposure Auto reg. */
		vc->value = EXPOSURE_AUTO_OFF;

		AV_DEBUG("V4L2_CID_EXPOSURE, cci_reg.bcrm_address=0x%x, vc->value=%u", BCRM_EXPOSURE_AUTO_8RW + cci_reg.bcrm_address, vc->value);

		retval = i2c_bcrm_write_reg_ctrl(vc, BCRM_EXPOSURE_AUTO_8RW + cci_reg.bcrm_address, AV_CAM_DATA_SIZE_8);

		if (retval < 0) {
			AV_ERR("I2C write failed!");
			return -EINVAL;
		}

		/*  ii) Setting value in Exposure reg. */
		vc->value = value_bkp;/* restore the actual value */
		reg = BCRM_EXPOSURE_TIME_64RW;
		length = AV_CAM_DATA_SIZE_64;

		qctrl.id = V4L2_CID_EXPOSURE;
		retval = ioctl_queryctrl_new(s, &qctrl);

		if (retval < 0) {
			AV_ERR("queryctrl failed!");
			return -EINVAL;
		}
		vc->value = convert_s_ctrl(vc->value, qctrl.minimum, qctrl.maximum, qctrl.step);
		break;

#if 0
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		AV_DEBUG("V4L2_CID_EXPOSURE_ABSOLUTE, cci_reg.bcrm_address 0x%x, vc->value %u", cci_reg.bcrm_address, vc->value);

		value_bkp = vc->value;/* backup the actual value */

		/*  i) Setting 'Manual' in Exposure Auto reg. */
		vc->value = V4L2_EXPOSURE_MANUAL;

		AV_DEBUG("V4L2_CID_EXPOSURE_ABSOLUTE, cci_reg.bcrm_address 0x%x, vc->value %u", cci_reg.bcrm_address, vc->value);

		retval = i2c_bcrm_write_reg_ctrl(vc, BCRM_EXPOSURE_AUTO_8RW + cci_reg.bcrm_address, AV_CAM_DATA_SIZE_8);

		if (retval < 0) {
			AV_ERR("I2C write failed!");
			return -EINVAL;
		}

		/*  ii) Setting value in Exposure reg. */
		vc->value = value_bkp;/* restore the actual value */
		reg = BCRM_EXPOSURE_TIME_64RW;
		length = AV_CAM_DATA_SIZE_64;

		qctrl.id = V4L2_CID_EXPOSURE;
		retval = ioctl_queryctrl_new(s, &qctrl);

		if (retval < 0) {
			AV_ERR("queryctrl failed!");
			return -EINVAL;
		}

		vc->value = vc->value * EXP_ABS;

		vc->value = convert_s_ctrl(vc->value, qctrl.minimum, qctrl.maximum, qctrl.step);
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		AV_DEBUG("V4L2_CID_EXPOSURE_AUTO vc->value %u", vc->value);
		reg = BCRM_EXPOSURE_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;

		/* BCRM Auto Gain changes */
		if (vc->value == true)
			vc->value = 2;/* Continouous mode */
		else
			vc->value = 0;/* 1; OFF/once mode */

		break;
#endif
	case V4L2_CID_AUTOGAIN:
		AV_DEBUG("V4L2_CID_AUTOGAIN vc->value=%u", vc->value);
		reg = BCRM_GAIN_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;

		/* BCRM Auto Gain changes */
		if (vc->value == true)
			vc->value = 2;/* Continouous mode */
		else
			vc->value = 0;/* 1; OFF/once mode */

		break;
	case V4L2_CID_GAIN:
		AV_DEBUG("V4L2_CID_GAIN, vc->value=%u", vc->value);
		reg = BCRM_GAIN_64RW;
		length = AV_CAM_DATA_SIZE_64;

		qctrl.id = V4L2_CID_GAIN;
		retval = ioctl_queryctrl_new(s, &qctrl);

		if (retval < 0) {
			AV_ERR("queryctrl failed!");
			return -EINVAL;
		}

		vc->value = convert_s_ctrl(vc->value, qctrl.minimum, qctrl.maximum, qctrl.step);

		break;

	case V4L2_CID_SHARPNESS:
		AV_DEBUG("V4L2_CID_SHARPNESS, vc->value=%u", vc->value);
		reg = BCRM_SHARPNESS_32RW;
		length = AV_CAM_DATA_SIZE_32;

		qctrl.id = V4L2_CID_SHARPNESS;
		retval = ioctl_queryctrl_new(s, &qctrl);

		if (retval < 0) {
			AV_ERR("queryctrl failed!");
			return -EINVAL;
		}

		vc->value = convert_s_ctrl(vc->value, qctrl.minimum, qctrl.maximum, qctrl.step);
		break;

	default:
		AV_ERR("   Unsupported control ID. vc->id=%d (0x%x)", vc->id, vc->id);
		retval = -EPERM;
		return retval;
	}

	retval = i2c_bcrm_write_reg_ctrl(vc, reg + cci_reg.bcrm_address, length);

	return retval < 0 ? -EINVAL : 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

static int supported_pixformat(void)
{
	u64 avail_mipi = 0;
	int i = 0;
	int status = 0;
	unsigned char bayer_val = 0;
	bcrm_availMipi_reg_t feature_inquiry_reg;
	bcrm_bayerInquiry_reg_t bayer_inquiry_reg;
	u64 device_supported = 0;
	const u64 host_supported = HOST_SUPPORTED_PIXFMTS;
	const s8 host_map[TOTAL_SUPP_PIXFORMATS] = {-1, -1, -1, -1, -1, 0, -1, 1, 2, -1, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 4, 7, 8, 6, 5};

/* Read the MIPI format register to check whether the camera really support the requested pixelformat format */
	status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_IMG_AVAILABLE_MIPI_DATA_FORMATS_64R,
				 	AV_CAM_REG_SIZE,
					AV_CAM_DATA_SIZE_64,
					(char *) &avail_mipi);

	if (status < 0) {
		//AV_ERR("I2C read failed. (Status=%d)", status);
		return -EINVAL;
	}

	feature_inquiry_reg.value = avail_mipi;

	AV_DEBUG("Feature Inquiry Reg value: 0x%016llx", avail_mipi);

/* Read the Bayer Inquiry register to check whether the camera really support the requested RAW format */
	status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_IMG_BAYER_PATTERN_INQUIRY_8R,
				 	AV_CAM_REG_SIZE,
					AV_CAM_DATA_SIZE_8,
					(char *) &bayer_val);

	if (status < 0) {
		//AV_ERR("I2C read failed. (Status=%d)", status);
		return -EINVAL;
	}

	AV_DEBUG("Bayer Inquiry Reg value: 0x%x", bayer_val);

	bayer_inquiry_reg.value = bayer_val;

	device_supported = (avail_mipi & MIPI_AVAIL_MASK); /* 0 ... 6 = 0 ... 6*/

	if (avail_mipi & (1 << 7)) {
		device_supported |= 3 << 7; /* 7 = 7 8 */
	}

/* Refer BCRM doc for the pixelformats list */
	device_supported |= ((avail_mipi & RGB888_SHIFT << RGB888_PIX; /* 8 ... 11 = 9 ... 12 */

	if (avail_mipi & RAW6_PIX) {
	device_supported |= (bayer_val & RAW6_SHIFT; /* 12 = 13 ... 17 */  /* RAW6 */
	}

	if (avail_mipi & RAW7_PIX) {
	device_supported |= (bayer_val & RAW7_SHIFT; /* 13 = 18 ... 22 */ /* RAW7 */
	}

	if (avail_mipi & RAW8_PIX) {
	device_supported |= (bayer_val & RAW8_SHIFT; /* 14 = 23 ... 27 */ /* RAW8 */
	}

	if (avail_mipi & RAW10_PIX) {
	device_supported |= (bayer_val & RAW10_SHIFT; /* 15 = 28 ... 32 */ /* RAW10 */
	}

	if (avail_mipi & RAW12_PIX) {
	device_supported |= (bayer_val & RAW12_SHIFT; /* 16 = 33 ... 37 */ /* RAW12 */
	}

	AV_DEBUG("device_supported=0x%llx", device_supported);

// mapping example device_supported/host_supported -> host_map -> v4l2_fmtdesc
//
// [bit] device_supported/host_supported ... V4L2 pixelformat
// [...]
// 23 ... Monochrome + RAW8 -> V4L2_PIX_FMT_GREY (GREY)
// 24 ... Bayer GR   + RAW8 -> V4L2_PIX_FMT_SGRBG8 (GRBG)
// 25 ... Bayer RG   + RAW8 -> V4L2_PIX_FMT_SRGGB8 (RGGB)
// 26 ... Bayer GB   + RAW8 -> V4L2_PIX_FMT_SGBRG8 (GBRG)
// 27 ... Bayer BG   + RAW8 -> V4L2_PIX_FMT_SBGGR8 (BA81)
// [...]
//
// index      0   1   2   3   4  5   6  7  8   9 10  11  12  13  14  15  16  17  18  19  20  21  22 23 24 25 26 27	 
// host_map {-1, -1, -1, -1, -1, 0, -1, 1, 2, -1, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 4, 7, 8, 6, 5};

	/* from the intersection of device supported and host supported pixelformats,
	   create a list with indices 0..n for the ioctl VIDIOC_ENUM_FMT */
	for (i = 0; i < TOTAL_SUPP_PIXFORMATS; i++) {
		if ((device_supported & (((u64)1)<<i)) & (host_supported & (((u64)1)<<i))) {
			fmtdesc_avt[avail_pixformats].flags = av_cam_formats[host_map[i]].flags;
			fmtdesc_avt[avail_pixformats].pixelformat = av_cam_formats[host_map[i]].pixelformat;
			strcpy(fmtdesc_avt[avail_pixformats].description, av_cam_formats[host_map[i]].description);
			avail_pixformats++;
		}
	}
	AV_DEBUG("Success, supported pixelformats avail_pixformats=%d", avail_pixformats);
	return	avail_pixformats ? 0 : -1;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	if (fmt->index >= avail_pixformats)
		return -EINVAL;

	fmt->pixelformat = fmtdesc_avt[fmt->index].pixelformat;
	strcpy(fmt->description, fmtdesc_avt[fmt->index].description);
	fmt->flags = V4L2_FMT_FLAG_COMPRESSED;/* HW limitation */
	return 0;
}

/* Compute commond range for Host and BCRM */
bool ComputeCommonRange(uint32_t nMin1, uint32_t nMax1, uint32_t nInc1,
				uint32_t nMin2, uint32_t nMax2, uint32_t nInc2,
				uint32_t *rMin, uint32_t *rMax, uint32_t *rInc)
{
	bool bResult = false;

	/* Compute the range of overlapping values */
	uint32_t nMin = max(nMin1, nMin2);
	uint32_t nMax = min(nMax1, nMax2);

	/* Check if it is overlapping at all */
	if (nMax >= nMin) {
		if (nMin1 == nMin2) { /* If both minima are equal then the computation is a bit simpler */
			uint32_t nLCM = lcm(nInc1, nInc2);
			*rMin = nMin;
			*rMax = nMax - ((nMax - nMin) % nLCM);

			if (*rMin == *rMax)
				*rInc = 1;
			else
				*rInc = nLCM;

			bResult = true;
		} else if (nMin1 > nMin2) {
			/* Find the first value that is ok for Host and BCRM */
			uint32_t nMin1Shifted = nMin1 - nMin2;
			uint32_t nMaxShifted = nMax - nMin2;
			uint32_t nValue = nMin1Shifted;
			for (; nValue <= nMaxShifted; nValue += nInc1) {
				if ((nValue % nInc2) == 0)
					break;
			}

			/* Compute common increment and maximum */
			if (nValue <= nMaxShifted) {
				uint32_t nLCM = lcm(nInc1, nInc2);
				*rMin = nValue + nMin2;
				*rMax = nMax - ((nMax - *rMin) % nLCM);

				if (*rMin == *rMax)
					*rInc = 1;
				else
					*rInc = nLCM;

				bResult = true;
			}
		} else {
			/* Find the first value that is ok for Host and BCRM */
			uint32_t nMin2Shifted = nMin2 - nMin1;
			uint32_t nMaxShifted = nMax - nMin1;
			uint32_t nValue = nMin2Shifted;
			for (; nValue <= nMaxShifted; nValue += nInc2) {
				if ((nValue % nInc1) == 0)
					break;
			}

			/* Compute common increment and maximum */
			if (nValue <= nMaxShifted) {
				uint32_t nLCM = lcm(nInc2, nInc1);
				*rMin = nValue + nMin1;
				*rMax = nMax - ((nMax - *rMin) % nLCM);
				if (*rMin == *rMax)
					*rInc = 1;
				else
					*rInc = nLCM;

				bResult = true;
			}
		}
	}
	return bResult;
}

static int set_bayer_format(__u8 value)
{
	int ret = 0;
	struct v4l2_i2c i2c_reg = {};

	AV_DEBUG("Bayer value %d", value);

	CLEAR(i2c_reg);
	i2c_reg.reg = cci_reg.bcrm_address + BCRM_IMG_BAYER_PATTERN_8RW;
	i2c_reg.reg_size = AV_CAM_REG_SIZE;
	i2c_reg.count = AV_CAM_DATA_SIZE_8;
	i2c_reg.buffer = (const char *) &value;

	//ret = ioctl_gencam_i2cwrite_reg(&s, &i2c_reg);
	ret = i2c_bcrm_write_reg(&i2c_reg);

	return (ret < 0) ? -EINVAL : 0;
}

static int ioctl_streamon_ex(struct v4l2_int_device *s, struct streamon_ex *streamon)
{
	struct sensor_data *sensor;

#ifdef WANDBOARD_IMX6
	struct mipi_csi2_info *mipi_csi2_data;

	mipi_csi2_data = kmalloc(sizeof(struct mipi_csi2_info), GFP_KERNEL);

	AV_DEBUG("streamon->virtualChannel=%d, streamon->nLaneCount %d, streamon->dataType 0x%x ", streamon->virtualChannel, streamon->nLaneCount, streamon->dataType);

	mipi_csi2_data = mipi_csi2_get_info();
	sensor = &av_cam_data;

/*	sensor->virtual_channel = sensor->csi | (sensor->ipu_id << 1); */
	mipi_csi2_set_datatype(mipi_csi2_data, streamon->dataType);
//TODO
	mipi_csi2_data->lanes = streamon->nLaneCount;
	mipi_csi2_set_lanes(mipi_csi2_data);
	return 0;

#else
	void *mipi_csi2_info;

	AV_DEBUG("streamon->virtualChannel=%d, streamon->nLaneCount=%d, streamon->dataType=0x%x ", 
		streamon->virtualChannel, 
		streamon->nLaneCount, 
		streamon->dataType);

	mipi_csi2_info = mipi_csi2_get_info();
	sensor = &av_cam_data;

/*	sensor->virtual_channel = sensor->csi | (sensor->ipu_id << 1); */
	mipi_csi2_set_datatype(mipi_csi2_info, streamon->dataType);
	sensor->virtual_channel = streamon->virtualChannel;//TODO
	mipi_csi2_set_lanes(mipi_csi2_info, streamon->nLaneCount);
	return 0;

#endif

	return 0;
}

struct avail_mipi_data_formats_struct{
	uint64_t bit_mask;
	const char* name;
};

static const struct avail_mipi_data_formats_struct avail_mipi_data_formats_table[] = {
	{0x0000000000000001, "YUV420_8_LEG"},
	{0x0000000000000002, "YUV420_8"},
	{0x0000000000000004, "YUV420_10"},
	{0x0000000000000008, "YUV420_8_CSPS"},
	{0x0000000000000010, "YUV420_10_CSPS"},
	{0x0000000000000020, "YUV422_8"},
	{0x0000000000000040, "YUV422_10"},
	{0x0000000000000080, "RGB888"},
	{0x0000000000000100, "RGB666"},
	{0x0000000000000200, "RGB565"},
	{0x0000000000000400, "RGB555"},
	{0x0000000000000800, "RGB444"},
	{0x0000000000001000, "RAW6"},
	{0x0000000000002000, "RAW7"},
	{0x0000000000004000, "RAW8"},
	{0x0000000000008000, "RAW10"},
	{0x0000000000010000, "RAW12"},
	{0x0000000000020000, "RAW14"},
	{0x0000000000040000, "JPEG"},	

};

static int ioctl_try_fmt(struct v4l2_int_device *s, struct v4l2_format *tryfmt)
{
	int retval = 0;
	int status = 0;
	bcrm_availMipi_reg_t avail_mipi_data_formats;
	bcrm_bayerInquiry_reg_t bayer_pattern_inquiry;
	struct v4l2_send_command_control ct;
	int i = 0;

	/* Read the minimum supported width from camera */
	CLEAR(ct);
	ct.id = V4L2_AV_IMX_CSI2_WIDTH_MINVAL_R;
	retval = ioctl_send_command(s, &ct);

	if (retval < 0)
		return -EINVAL;
	av_cam_data.pix.width = ct.value0;

	/* Read the minimum supported height from camera */
	CLEAR(ct);
	ct.id = V4L2_AV_IMX_CSI2_HEIGHT_MINVAL_R;
	retval = ioctl_send_command(s, &ct);

	if (retval < 0)
		return -EINVAL;

	av_cam_data.pix.height = ct.value0;

	// Check for min boundaries
	if (	tryfmt->fmt.pix.width >= av_cam_data.pix.width &&
		tryfmt->fmt.pix.height >= av_cam_data.pix.height)
	{
		/* Read the maximum supported width from camera */
		CLEAR(ct);
		ct.id = V4L2_AV_IMX_CSI2_MAX_WIDTH_R;
		retval = ioctl_send_command(s, &ct);

		if (retval < 0)
			return -EINVAL;
		av_cam_data.pix.width = ct.value0;

		/* Read the maximum supported height from camera */
		CLEAR(ct);
		ct.id = V4L2_AV_IMX_CSI2_MAX_HEIGHT_R;
		retval = ioctl_send_command(s, &ct);

		if (retval < 0)
			return -EINVAL;
		av_cam_data.pix.height = ct.value0;

		if (	tryfmt->fmt.pix.width > av_cam_data.pix.width ||
			tryfmt->fmt.pix.height > av_cam_data.pix.height)
		{
			AV_ERR("Image width/height above maximum");
			AV_DEBUG(" tryfmt->fmt.pix.width=%d", tryfmt->fmt.pix.width);
			AV_DEBUG(" av_cam_data.pix.width=%d", av_cam_data.pix.width);
			AV_DEBUG(" tryfmt->fmt.pix.height=%d", tryfmt->fmt.pix.height);
			AV_DEBUG(" av_cam_data.pix.height=%d", av_cam_data.pix.height);
			return -EINVAL;
		}

		/* Read the MIPI format register to check whether the camera really support the requested pixelformat format */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_IMG_AVAILABLE_MIPI_DATA_FORMATS_64R,
						AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &avail_mipi_data_formats.value);
		if (status < 0)
			return -EINVAL;

		/* Read the Bayer Inquiry register to check whether the camera really support the requested RAW format */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_IMG_BAYER_PATTERN_INQUIRY_8R,
						AV_CAM_REG_SIZE, 
						AV_CAM_DATA_SIZE_8, 
						(char *)&bayer_pattern_inquiry);
		if (status < 0)
			return -EINVAL;

		AV_DEBUG("Camera avail_mipi_data_formats.value=0x%llx", avail_mipi_data_formats.value);

		for (i = 0; i<sizeof(avail_mipi_data_formats_table)/sizeof(struct avail_mipi_data_formats_struct); i++)
		{
			if (avail_mipi_data_formats.value & avail_mipi_data_formats_table[i].bit_mask)
			{
				AV_DEBUG(" %s", avail_mipi_data_formats_table[i].name);
			}
		}

		AV_DEBUG("try_fmt->fmt.pix.pixelformat=0x%x (%c%c%c%c)", tryfmt->fmt.pix.pixelformat,
			(tryfmt->fmt.pix.pixelformat >> 0) & 0xff,
			(tryfmt->fmt.pix.pixelformat>> 8) & 0xff,
			(tryfmt->fmt.pix.pixelformat >> 16) & 0xff,
			(tryfmt->fmt.pix.pixelformat >> 24) & 0xff);

		AV_DEBUG("Camera bayer pattern inquiry: 0x%x", bayer_pattern_inquiry.value);
		if (bayer_pattern_inquiry.value & 0x01)
			AV_DEBUG(" Grey available");
		if (bayer_pattern_inquiry.value & 0x02)
			AV_DEBUG(" Bayer GR available");
		if (bayer_pattern_inquiry.value & 0x04)
			AV_DEBUG(" Bayer RG available");
		if (bayer_pattern_inquiry.value & 0x08)
			AV_DEBUG(" Bayer GB available");
		if (bayer_pattern_inquiry.value & 0x10)
			AV_DEBUG(" Bayer BG available");

		// Check, if format is available
		switch(tryfmt->fmt.pix.pixelformat)
		{
		case V4L2_PIX_FMT_GREY:
			if (	avail_mipi_data_formats.avail_mipi.raw8_avail &&
			 	bayer_pattern_inquiry.bayer_pattern.monochrome_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;
			}
			break;

		case V4L2_PIX_FMT_SBGGR8:
			if (	avail_mipi_data_formats.avail_mipi.raw8_avail &&
			 	bayer_pattern_inquiry.bayer_pattern.bayer_BG_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;
			}
			break;

		case V4L2_PIX_FMT_SGBRG8:
			if (	avail_mipi_data_formats.avail_mipi.raw8_avail &&
			 	bayer_pattern_inquiry.bayer_pattern.bayer_GB_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;
			}
			break;

		case V4L2_PIX_FMT_SGRBG8:
			if (	avail_mipi_data_formats.avail_mipi.raw8_avail &&
			 	bayer_pattern_inquiry.bayer_pattern.bayer_GR_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;
			}
			break;

		case V4L2_PIX_FMT_SRGGB8:
			if (	avail_mipi_data_formats.avail_mipi.raw8_avail &&
			 	bayer_pattern_inquiry.bayer_pattern.bayer_RG_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;
			}
			break;

		case V4L2_PIX_FMT_RGB24:
		case V4L2_PIX_FMT_BGR24:
			if (avail_mipi_data_formats.avail_mipi.rgb888_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;
			}
			break;

		case V4L2_PIX_FMT_UYVY:
			if (avail_mipi_data_formats.avail_mipi.yuv422_8_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;
			}
			break;

		case V4L2_PIX_FMT_GREY12P:
		case V4L2_PIX_FMT_Y12P:
			if (	avail_mipi_data_formats.avail_mipi.raw12_avail &&
			 	bayer_pattern_inquiry.bayer_pattern.monochrome_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;
			}
			break;

		case V4L2_PIX_FMT_Y10P:
			if (	avail_mipi_data_formats.avail_mipi.raw10_avail &&
			 	bayer_pattern_inquiry.bayer_pattern.monochrome_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;
			}
			break;

		case V4L2_PIX_FMT_SBGGR12P:
			if (	avail_mipi_data_formats.avail_mipi.raw12_avail &&
			 	bayer_pattern_inquiry.bayer_pattern.bayer_BG_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;		
			}
			break;

		case V4L2_PIX_FMT_SGBRG12P:
			if (	avail_mipi_data_formats.avail_mipi.raw12_avail &&
			 	bayer_pattern_inquiry.bayer_pattern.bayer_GB_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;		
			}
			break;

		case V4L2_PIX_FMT_SGRBG12P:
			if (	avail_mipi_data_formats.avail_mipi.raw12_avail &&
			 	bayer_pattern_inquiry.bayer_pattern.bayer_GR_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;		
			}
			break;

		case V4L2_PIX_FMT_SRGGB12P:
			if (	avail_mipi_data_formats.avail_mipi.raw12_avail &&
			 	bayer_pattern_inquiry.bayer_pattern.bayer_RG_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;		
			}
			break;

		case V4L2_PIX_FMT_SBGGR10P:
			if (	avail_mipi_data_formats.avail_mipi.raw10_avail &&
			 	bayer_pattern_inquiry.bayer_pattern.bayer_BG_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;		
			}
			break;

		case V4L2_PIX_FMT_SGBRG10P:
			if (	avail_mipi_data_formats.avail_mipi.raw10_avail &&
			 	bayer_pattern_inquiry.bayer_pattern.bayer_GB_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;		
			}
			break;

		case V4L2_PIX_FMT_SGRBG10P:
			if (	avail_mipi_data_formats.avail_mipi.raw10_avail &&
			 	bayer_pattern_inquiry.bayer_pattern.bayer_GR_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;		
			}
			break;

		case V4L2_PIX_FMT_SRGGB10P:
			if (	avail_mipi_data_formats.avail_mipi.raw10_avail &&
			 	bayer_pattern_inquiry.bayer_pattern.bayer_RG_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;		
			}
			break;

		case V4L2_PIX_FMT_RGB565:
			if (avail_mipi_data_formats.avail_mipi.rgb565_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;
			}
			break;

		case V4L2_PIX_FMT_MJPEG:
		case V4L2_PIX_FMT_JPEG:
			if (avail_mipi_data_formats.avail_mipi.jpeg_avail) {
				AV_DEBUG("%c%c%c%c format available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff, (tryfmt->fmt.pix.pixelformat>> 8) & 0xff, (tryfmt->fmt.pix.pixelformat >> 16) & 0xff, (tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
				return 0;
			}
			break;	

		default:
			AV_ERR("Unexpected pixel format");
			break;
		}

		AV_ERR("%c%c%c%c format not available", (tryfmt->fmt.pix.pixelformat >> 0) & 0xff,
			(tryfmt->fmt.pix.pixelformat>> 8) & 0xff,
			(tryfmt->fmt.pix.pixelformat >> 16) & 0xff,
			(tryfmt->fmt.pix.pixelformat >> 24) & 0xff);
	}
	else
	{
		AV_ERR("Image width/height below minimum");
		AV_DEBUG(" tryfmt->fmt.pix.width=%d", tryfmt->fmt.pix.width);
		AV_DEBUG(" av_cam_data.pix.width=%d", av_cam_data.pix.width);
		AV_DEBUG(" tryfmt->fmt.pix.height=%d", tryfmt->fmt.pix.height);
		AV_DEBUG(" av_cam_data.pix.height=%d", av_cam_data.pix.height);
	}

	return -EINVAL;
}

static int ioctl_send_command(struct v4l2_int_device *s, struct v4l2_send_command_control *vc)
{
	int status = 0;
	int ret = 0;
	unsigned int reg = 0;
	int length = 0;
	struct v4l2_i2c i2c_reg = {};
	void *mipi_csi2_info;
	int r_wn = 0;/* Write -> r_wn = 0, Read -> r_wn = 1 */
	u64 avail_mipi = 0;
	bcrm_availMipi_reg_t feature_inquiry_reg;
	bcrm_bayerInquiry_reg_t bayer_inquiry_reg;
	unsigned char bayer_val = 0;
	u64 temp = 0;
	int is_gencp_mode_local = 0;/* Default BCRM mode */
	__u8 bayer_temp = 0;
	u8 bayer_pattern;

	bayer_inquiry_reg.value = 0;
	feature_inquiry_reg.value = 0;

	if (vc->id == V4L2_AV_IMX_CSI2_PIXELFORMAT_W) {
		/* Read the MIPI format register to check whether the camera really support the requested pixelformat format */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_IMG_AVAILABLE_MIPI_DATA_FORMATS_64R,
						AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_64,
						(char *) &avail_mipi);

		feature_inquiry_reg.value = avail_mipi;

		AV_DEBUG("Feature Inquiry Reg value : 0x%016llx", avail_mipi);

		/* Read the Bayer Inquiry register to check whether the camera really support the requested RAW format */
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_IMG_BAYER_PATTERN_INQUIRY_8R,
						AV_CAM_REG_SIZE,
						AV_CAM_DATA_SIZE_8,
						(char *) &bayer_val);

		AV_DEBUG("Bayer Inquiry Reg value: 0x%x", bayer_val);

		bayer_inquiry_reg.value = bayer_val;
	}

	if (vc->id == V4L2_AV_IMX_CSI2_PIXELFORMAT_R) {
		status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_IMG_BAYER_PATTERN_8RW,
						AV_CAM_REG_SIZE,
				 		AV_CAM_DATA_SIZE_8,
						(char *) &bayer_pattern);

		if (status < 0) {
			AV_ERR("I2C read failed. (Status=%d)", status);
			return -EINVAL;
		}
		AV_DEBUG("Bayer Pattern Reg value: 0x%x", bayer_val);
	}

	switch (vc->id) {
	case V4L2_AV_IMX_CSI2_STREAMON_W:
		reg = BCRM_ACQUISITION_START_8RW;
		length = AV_CAM_DATA_SIZE_8;
		r_wn = 0;
		break;
	case V4L2_AV_IMX_CSI2_STREAMOFF_W:
		reg = BCRM_ACQUISITION_STOP_8RW;
		length = AV_CAM_DATA_SIZE_8;
		r_wn = 0;
		break;
	case V4L2_AV_IMX_CSI2_ABORT_W:
		reg = BCRM_ACQUISITION_ABORT_8RW;
		length = AV_CAM_DATA_SIZE_8;
		r_wn = 0;
		break;
	case V4L2_AV_IMX_CSI2_WIDTH_W:
		reg = BCRM_IMG_WIDTH_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 0;
		break;
	case V4L2_AV_IMX_CSI2_HEIGHT_W:
		reg = BCRM_IMG_HEIGHT_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 0;
		break;
	case V4L2_AV_IMX_CSI2_OFFSET_X_W:
		reg = BCRM_IMG_OFFSET_X_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 0;
		break;
	case V4L2_AV_IMX_CSI2_OFFSET_Y_W:
		reg = BCRM_IMG_OFFSET_Y_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 0;
		break;
	case V4L2_AV_IMX_CSI2_HFLIP_W:
		reg = BCRM_IMG_REVERSE_X_8RW;
		length = AV_CAM_DATA_SIZE_8;
		r_wn = 0;
		break;
	case V4L2_AV_IMX_CSI2_VFLIP_W:
		reg = BCRM_IMG_REVERSE_Y_8RW;
		length = AV_CAM_DATA_SIZE_8;
		r_wn = 0;
		break;
	case V4L2_AV_IMX_CSI2_PIXELFORMAT_W:
		reg = BCRM_IMG_MIPI_DATA_FORMAT_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 0;

		AV_DEBUG("\"%c%c%c%c\"", 
			(vc->value0 >> 0) & 0xff,
			(vc->value0 >> 8) & 0xff,
			(vc->value0 >> 16) & 0xff,
			(vc->value0 >> 24) & 0xff);

		switch (vc->value0) {
		case V4L2_PIX_FMT_RGB565:
			if (!feature_inquiry_reg.avail_mipi.rgb565_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format RGB565");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RGB565;
			break;
		case V4L2_PIX_FMT_RGB24:
		case V4L2_PIX_FMT_BGR24:
			if (!feature_inquiry_reg.avail_mipi.rgb888_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format RGB888 (RGB24/BGR24)");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RGB888;
			break;
		case V4L2_PIX_FMT_CUSTOM:
			vc->value0 = MIPI_DT_CUSTOM;
			break;
		case V4L2_PIX_FMT_YUV422P:
			if (!feature_inquiry_reg.avail_mipi.yuv422_8_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format YUV422P (YUV422_8)");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_YUV420_LEGACY;
			break;
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_YUYV:
			vc->value0 = MIPI_DT_YUV422;
			break;
		case V4L2_PIX_FMT_YUV420:
		case V4L2_PIX_FMT_YVU420:
			vc->value0 = MIPI_DT_YUV420;
			break;
		case V4L2_PIX_FMT_GREY:
			if (!bayer_inquiry_reg.bayer_pattern.monochrome_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format GREY (8bit Mono)");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RAW8;
			bayer_temp = monochrome;
			break;

		case V4L2_PIX_FMT_SBGGR8:
			if (!bayer_inquiry_reg.bayer_pattern.bayer_BG_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format SBGGR8 Bayer BG");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RAW8;
			bayer_temp = bayer_bg;
			break;
		case V4L2_PIX_FMT_SGBRG8:
			if (!bayer_inquiry_reg.bayer_pattern.bayer_GB_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format SGBRG8 Bayer GB");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RAW8;
			bayer_temp = bayer_gb;
			break;
		case V4L2_PIX_FMT_SGRBG8:
			if (!bayer_inquiry_reg.bayer_pattern.bayer_GR_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format SGRBG8 Bayer GR");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RAW8;
			bayer_temp = bayer_gr;
			break;
		case V4L2_PIX_FMT_SRGGB8:
			if (!bayer_inquiry_reg.bayer_pattern.bayer_RG_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format SRGGB8 Bayer RG");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RAW8;
			bayer_temp = bayer_rg;
			break;
		case V4L2_PIX_FMT_Y10P:
			if (!bayer_inquiry_reg.bayer_pattern.monochrome_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format V4L2_PIX_FMT_Y10P (10bit Mono)");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RAW10;
			bayer_temp = monochrome;
			break;
		case V4L2_PIX_FMT_SBGGR10P:
			if (!bayer_inquiry_reg.bayer_pattern.bayer_BG_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format SBGGR10P Bayer BG");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RAW10;
			bayer_temp = bayer_bg;
			break;
		case V4L2_PIX_FMT_SGBRG10P:
			if (!bayer_inquiry_reg.bayer_pattern.bayer_GB_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format SGBRG10P Bayer GB");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RAW10;
			bayer_temp = bayer_gb;
			break;
		case V4L2_PIX_FMT_SGRBG10P:
			if (!bayer_inquiry_reg.bayer_pattern.bayer_GR_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format SGRBG10P Bayer GR");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RAW10;
			bayer_temp = bayer_gr;
			break;
		case V4L2_PIX_FMT_SRGGB10P:
			if (!bayer_inquiry_reg.bayer_pattern.bayer_RG_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format SRGGB10 Bayer RG");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RAW10;
			bayer_temp = bayer_rg;
			break;
		case V4L2_PIX_FMT_GREY12P:
		case V4L2_PIX_FMT_Y12P:
			if (!bayer_inquiry_reg.bayer_pattern.monochrome_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format V4L2_PIX_FMT_Y12P (12bit Mono)");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RAW12;
			bayer_temp = monochrome;
			break;
		case V4L2_PIX_FMT_SBGGR12P:
			if (!bayer_inquiry_reg.bayer_pattern.bayer_BG_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format SBGGR12P Bayer BG");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RAW12;
			bayer_temp = bayer_bg;
			break;
		case V4L2_PIX_FMT_SGBRG12P:
			if (!bayer_inquiry_reg.bayer_pattern.bayer_GB_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format SGBRG12P Bayer GB");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RAW12;
			bayer_temp = bayer_gb;
			break;
		case V4L2_PIX_FMT_SGRBG12P:
			if (!bayer_inquiry_reg.bayer_pattern.bayer_GR_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format SGRBG12P Bayer GR");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RAW12;
			bayer_temp = bayer_gr;
			break;
		case V4L2_PIX_FMT_SRGGB12P:
			if (!bayer_inquiry_reg.bayer_pattern.bayer_RG_avail) {
				AV_ERR("Camera firmware doesn't support for this MIPI data format SRGGB12P Bayer RG");
				return -EINVAL;
			}
			vc->value0 = MIPI_DT_RAW12;
			bayer_temp = bayer_rg;
			break;

		default:
			AV_ERR("Unsupported pixelformat by the host 0x%x", vc->id);
			return -EINVAL;
		}
		break;
	case V4L2_AV_IMX_CSI2_WIDTH_R:
		reg = BCRM_IMG_WIDTH_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_WIDTH_MINVAL_R:
		reg = BCRM_IMG_WIDTH_MIN_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_WIDTH_MAXVAL_R:
		reg = BCRM_IMG_WIDTH_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_WIDTH_INCVAL_R:
		reg = BCRM_IMG_WIDTH_INC_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_HEIGHT_R:
		reg = BCRM_IMG_HEIGHT_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_HEIGHT_MINVAL_R:
		reg = BCRM_IMG_HEIGHT_MIN_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_HEIGHT_MAXVAL_R:
		reg = BCRM_IMG_HEIGHT_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_HEIGHT_INCVAL_R:
		reg = BCRM_IMG_HEIGHT_INC_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_OFFSET_X_R:
		reg = BCRM_IMG_OFFSET_X_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_OFFSET_X_MIN_R:
		reg = BCRM_IMG_OFFSET_X_MIN_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_OFFSET_X_MAX_R:
		reg = BCRM_IMG_OFFSET_X_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_OFFSET_X_INC_R:
		reg = BCRM_IMG_OFFSET_X_INC_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_OFFSET_Y_R:
		reg = BCRM_IMG_OFFSET_Y_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_OFFSET_Y_MIN_R:
		reg = BCRM_IMG_OFFSET_Y_MIN_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_OFFSET_Y_MAX_R:
		reg = BCRM_IMG_OFFSET_Y_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_OFFSET_Y_INC_R:
		reg = BCRM_IMG_OFFSET_Y_INC_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_SENSOR_WIDTH_R:
		reg = BCRM_SENSOR_WIDTH_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_SENSOR_HEIGHT_R:
		reg = BCRM_SENSOR_HEIGHT_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_MAX_WIDTH_R:
		reg = BCRM_WIDTH_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_MAX_HEIGHT_R:
		reg = BCRM_HEIGHT_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_PIXELFORMAT_R:
		reg = BCRM_IMG_MIPI_DATA_FORMAT_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_PAYLOADSIZE_R:
		reg = BCRM_BUFFER_SIZE_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_ACQ_STATUS_R:
		reg = BCRM_ACQUISITION_STATUS_8R;
		length = AV_CAM_DATA_SIZE_8;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_HFLIP_R:
		reg = BCRM_IMG_REVERSE_X_8RW;
		length = AV_CAM_DATA_SIZE_8;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_VFLIP_R:
		reg = BCRM_IMG_REVERSE_Y_8RW;
		length = AV_CAM_DATA_SIZE_8;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_CURRENTMODE_R:
		reg = GENCP_CURRENTMODE_8R;
		length = AV_CAM_DATA_SIZE_8;
		is_gencp_mode_local = 1;
		r_wn = 1;
		break;
	case V4L2_AV_IMX_CSI2_CHANGEMODE_W:
		reg = GENCP_CHANGEMODE_8W;
		length = AV_CAM_DATA_SIZE_8;
		is_gencp_mode_local = 1;
		r_wn = 0;
		break;
	default:
		AV_ERR("Unknown ctrl 0x%x", vc->id);
		return -EINVAL;
	}

	if (r_wn) {/* Read (r_wn=1) */

		if (is_gencp_mode_local) {

			status = i2c_generic_read_buffer(reg,
							AV_CAM_REG_SIZE,
						 	length,
							(char *) &vc->value0);

			if (status < 0) {
				return -EINVAL;
			}
			return 0;
		}

		status = i2c_generic_read_buffer(cci_reg.bcrm_address + reg, 
						AV_CAM_REG_SIZE,
					 	length,
						(char *) &vc->value0);

		if (status < 0) {
			return -EINVAL;
		}

		if (vc->id == V4L2_AV_IMX_CSI2_PIXELFORMAT_R) {

			switch (vc->value0) {
			case	MIPI_DT_RGB888:
				vc->value0 = V4L2_PIX_FMT_BGR24;
				break;
			case	MIPI_DT_RGB666:
				vc->value0 = V4L2_PIX_FMT_RGB666;
				break;
			case	MIPI_DT_RGB565:
				vc->value0 = V4L2_PIX_FMT_RGB565;
				break;
			case	MIPI_DT_RGB555:
				vc->value0 = V4L2_PIX_FMT_RGB555;
				break;
			case	MIPI_DT_RGB444:
				vc->value0 = V4L2_PIX_FMT_RGB444;
				break;
			case	MIPI_DT_YUV422:
				vc->value0 = V4L2_PIX_FMT_UYVY;
				break;
			case	MIPI_DT_YUV420:
				vc->value0 = V4L2_PIX_FMT_YVU420;
				break;
			case	MIPI_DT_CUSTOM:
				vc->value0 = V4L2_PIX_FMT_CUSTOM;
				break;
			case	MIPI_DT_RAW8:
					switch (bayer_pattern) {
					case	monochrome:
						vc->value0 = V4L2_PIX_FMT_GREY;
						break;
					case	bayer_gr:
						vc->value0 = V4L2_PIX_FMT_SGRBG8;
						break;
					case	bayer_rg:
						vc->value0 = V4L2_PIX_FMT_SRGGB8;
						break;
					case	bayer_gb:
						vc->value0 = V4L2_PIX_FMT_SGBRG8;
						break;
					case	bayer_bg:
						vc->value0 = V4L2_PIX_FMT_SBGGR8;
						break;
					default:
						AV_ERR("Unknown RAW8 pixelformat read 0x%x, bayer_pattern %d",
								 vc->id, bayer_pattern);
						return -EINVAL;
					}

				break;
			case	MIPI_DT_RAW10:
					switch (bayer_pattern) {
					case	monochrome:
						vc->value0 = V4L2_PIX_FMT_Y10P;
						break;
					case	bayer_gr:
						vc->value0 = V4L2_PIX_FMT_SGRBG10P;
						break;
					case	bayer_rg:
						vc->value0 = V4L2_PIX_FMT_SRGGB10P;
						break;
					case	bayer_gb:
						vc->value0 = V4L2_PIX_FMT_SGBRG10P;
						break;
					case	bayer_bg:
						vc->value0 = V4L2_PIX_FMT_SBGGR10P;
						break;
					default:
						AV_ERR("Unknown RAW10 pixelformat read 0x%x, bayer_pattern %d",
								 vc->id, bayer_pattern);
						return -EINVAL;
					}

				break;
			case	MIPI_DT_RAW12:
					switch (bayer_pattern) {
					case	monochrome:
						vc->value0 = V4L2_PIX_FMT_Y12P;
						break;
					case	bayer_gr:
						vc->value0 = V4L2_PIX_FMT_SGRBG12P;
						break;
					case	bayer_rg:
						vc->value0 = V4L2_PIX_FMT_SRGGB12P;
						break;
					case	bayer_gb:
						vc->value0 = V4L2_PIX_FMT_SGBRG12P;
						break;
					case	bayer_bg:
						vc->value0 = V4L2_PIX_FMT_SBGGR12P;
						break;
					default:
						AV_ERR("Unknown RAW12 pixelformat read 0x%x, bayer_pattern %d",
								 vc->id, bayer_pattern);
						return -EINVAL;
					}

				break;
			default:
				AV_ERR("Unknown pixelformat read=0x%x, vc->value0=0x%x", vc->id, vc->value0);
				return -EINVAL;
			}
		}

		return 0;



	} else {/* Write (r_wn=0) */
		AV_DEBUG("reg=0x%04x, length=%d, vc->value0=0x%x", reg, length, vc->value0);

		if (is_gencp_mode_local) {

			CLEAR(i2c_reg);
			i2c_reg.reg = reg;
			i2c_reg.reg_size = AV_CAM_REG_SIZE;
			i2c_reg.count = length;

			if (length > AV_CAM_DATA_SIZE_32)
				i2c_reg.buffer = (const char *) &temp;
			else
				i2c_reg.buffer = (const char *) &vc->value0;

			ret = ioctl_gencam_i2cwrite_reg(s, &i2c_reg);

			if (ret < 0) {
				return -EINVAL;
			} else {
				return 0;
			}
		}

		if (vc->id == V4L2_AV_IMX_CSI2_PIXELFORMAT_W) {
			/* Set pixelformat then set bayer format, refer OCT-2417 */
			mipi_csi2_info = mipi_csi2_get_info();
			mipi_csi2_set_datatype(mipi_csi2_info, vc->value0);
		}

		temp = vc->value0;

		if (length > AV_CAM_DATA_SIZE_32)
			swapbytes(&temp, length);
		else
			swapbytes(&vc->value0, length);



		CLEAR(i2c_reg);
		i2c_reg.reg = cci_reg.bcrm_address + reg;
		i2c_reg.reg_size = AV_CAM_REG_SIZE;
		i2c_reg.count = length;

		if (length > AV_CAM_DATA_SIZE_32)
			i2c_reg.buffer = (const char *) &temp;
		else
			i2c_reg.buffer = (const char *) &vc->value0;

		//ret = ioctl_gencam_i2cwrite_reg(s, &i2c_reg);
		ret = i2c_bcrm_write_reg(&i2c_reg);

		if (ret < 0) {
			return -EINVAL;
		}

		/* Set pixelformat then set bayer format, refer OCT-2417 */
		if (vc->id == V4L2_AV_IMX_CSI2_PIXELFORMAT_W) {
			status = set_bayer_format(bayer_temp);
			if (status < 0) {
				return -EINVAL;
			}
		}

		return 0;
	}
}

static int alliedvision_init_mode(enum av_cam_mode mode)
{
	int retval = 0;
	u32 mipi_reg = 0;
	uint8_t req_lane_count = 0;
	uint8_t cam_lane_count = 0;
	uint32_t min_clock = 0;
	uint32_t max_clock = 0;
	uint32_t inc_clock = 0;
	uint32_t min_cam_clock = 0;
	uint32_t max_cam_clock = 0;
	uint32_t cur_cam_clock = 0;
	uint32_t clock_reg = 0;
	uint32_t clock_val = 0;
	uint32_t lanes_ret = 0;
	uint32_t min_host_clock = 0;
	uint32_t max_host_clock = 0;
	uint8_t dphy_reg_val = 0x32;	// Default for 750-800MHz
	struct v4l2_i2c i2c_reg = {0};
	int status = 0;
	struct v4l2_int_device s = {0};
	bcrm_supported_lanecount_reg_t auto_lane_negot_bcrm = {0};
	imx_supported_lanecount_reg_t auto_lane_negot_host = {0};
	struct v4l2_send_command_control ct = {0};

#ifndef WANDBOARD_IMX6
	void* mipi_csi2_info = NULL;
	mipi_csi2_info = mipi_csi2_get_info();
#else
	struct mipi_csi2_info* mipi_csi2_data = NULL;
	mipi_csi2_data = kmalloc(sizeof(struct mipi_csi2_info), GFP_KERNEL);
	mipi_csi2_data = mipi_csi2_get_info();
#endif

	/* initial mipi dphy */
#ifndef WANDBOARD_IMX6
	if (mipi_csi2_info) {
		if (!mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_enable(mipi_csi2_info);

		if (mipi_csi2_get_status(mipi_csi2_info)) {
#else
	if (mipi_csi2_data) {
		if (!mipi_csi2_get_status(mipi_csi2_data))
			mipi_csi2_enable(mipi_csi2_data);

		if (mipi_csi2_get_status(mipi_csi2_data)) {
#endif

			/* ########################################### */
			/* Lane count settings */
			/* ########################################### */
			AV_DEBUG("Setting up DPHY lane count...");
			if (lanes_auto_conf) {
				AV_DEBUG("Auto negotiation of lane count settings...");

				i2c_bcrm_read_byte_reg(cci_reg.bcrm_address + BCRM_SUPPORTED_CSI2_LANE_COUNTS_8R, &cam_lane_count);

				auto_lane_negot_bcrm.value = cam_lane_count;
				auto_lane_negot_host.value = CSI_HOST_SUPPORTED_LANE_COUNT_QUAD_PLUS;

				AV_DEBUG(" Supported camera lane counts value: 0x%x", auto_lane_negot_bcrm.value);
				AV_DEBUG(" Supported host lane counts value:   0x%x", auto_lane_negot_host.value);

auto_correct:
				if (auto_lane_negot_host.lane_count.one_lane_avail & auto_lane_negot_bcrm.lane_count.one_lane_avail)
					req_lane_count = 1;
				if (auto_lane_negot_host.lane_count.two_lane_avail & auto_lane_negot_bcrm.lane_count.two_lane_avail)
					req_lane_count = 2;
				if (auto_lane_negot_host.lane_count.three_lane_avail & auto_lane_negot_bcrm.lane_count.three_lane_avail)
					req_lane_count = 3;
				if (auto_lane_negot_host.lane_count.four_lane_avail & auto_lane_negot_bcrm.lane_count.four_lane_avail)
					req_lane_count = 4;

				AV_DEBUG(" Maximum supported lane count value for camera and host is: %d", req_lane_count);

				i2c_bcrm_write_byte_reg(cci_reg.bcrm_address + BCRM_CSI2_LANE_COUNT_8RW, req_lane_count);
				/* Read back lane value from camera */
				i2c_bcrm_read_byte_reg(cci_reg.bcrm_address + BCRM_CSI2_LANE_COUNT_8RW, &cam_lane_count);

#ifndef WANDBOARD_IMX6
				lanes_ret = mipi_csi2_set_lanes(mipi_csi2_info, req_lane_count);

				AV_DEBUG(" DPHY lanes set to %d. Requested %d. Read camera value: %d", lanes_ret, req_lane_count, cam_lane_count);

				if (req_lane_count != lanes_ret) {
					auto_lane_negot_host.value = CSI_HOST_SUPPORTED_LANE_COUNT_SOLO_DL;
					AV_DEBUG(" Found SOLO/DUAL LITE board, thus correcting lanes");
					goto auto_correct;
				}
#else
				mipi_csi2_data->lanes = req_lane_count;
				lanes_ret = mipi_csi2_set_lanes(mipi_csi2_data);

				AV_DEBUG(" req_lane_count=%d, lanes_ret=%d\n", req_lane_count, lanes_ret);

				if (req_lane_count != (lanes_ret + 1)) {
					auto_lane_negot_host.value = CSI_HOST_SUPPORTED_LANE_COUNT_SOLO_DL;
					AV_DEBUG(" Found SOLO/DUAL LITE board, thus correcting lanes");
					goto auto_correct;
				}
#endif
				lane_count = req_lane_count;
			} else {
				AV_DEBUG("Manual lane count settings from DTS");
				AV_DEBUG(" Supported lane count (manual) by host and device is %d", lane_count);

				req_lane_count = lane_count;/* This back up needed for next switch () fn. */
#ifndef WANDBOARD_IMX6
				mipi_csi2_set_lanes(mipi_csi2_info, lane_count);
#else
				mipi_csi2_data->lanes = lane_count;
				mipi_csi2_set_lanes(mipi_csi2_data);
#endif
				i2c_bcrm_write_byte_reg(cci_reg.bcrm_address + BCRM_CSI2_LANE_COUNT_8RW, lane_count);
			}

			/* ########################################### */
			/* CSI clock setting */
			/* ########################################### */
			AV_DEBUG("Setting up DPHY clock...");
			min_host_clock = IMX6_CSI_HOST_CLK_MIN_FREQ_HZ;
			switch (req_lane_count) {
			case 1: /* Lane count 1 */
				max_host_clock = CSI_HOST_CLK_MAX_FREQ_HZ_1L;
				break;
			case 2: /* Lane count 2 */
				max_host_clock = CSI_HOST_CLK_MAX_FREQ_HZ_2L;
				break;
			case 3: /* Lane count 3 */
				max_host_clock = CSI_HOST_CLK_MAX_FREQ_HZ_3L;
				break;
			case 4: /* Lane count 4 */
				max_host_clock = CSI_HOST_CLK_MAX_FREQ_HZ_4L;
				break;
			default:
				AV_ERR("Unsupported lane count value");
				max_host_clock = 1;
				break;
			}

			status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_CSI2_CLOCK_MIN_32R,
						 		AV_CAM_REG_SIZE, 
								AV_CAM_DATA_SIZE_32,
								(char *) &min_cam_clock);

			if (status < 0) {
				return -EINVAL;
			}

			status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_CSI2_CLOCK_MAX_32R,
						 		AV_CAM_REG_SIZE, 
								AV_CAM_DATA_SIZE_32,
								(char *) &max_cam_clock);

			if (status < 0) {
				return -EINVAL;
			}

			if (ComputeCommonRange(	min_cam_clock, max_cam_clock, 1,
					    	min_host_clock, max_host_clock, 1,
					    	&min_clock, &max_clock, &inc_clock) == false) 
			{
				AV_ERR("Error: No common range for camera and host possible.");
				return -EINVAL;
			}

			if (clk_auto_conf) {
				AV_DEBUG("Auto negotiation clock settings...");
				AV_DEBUG(" Common clock range for camera and host is: Min=%d, Max=%d", min_clock, max_clock);
				AV_DEBUG(" Selecting maximum CSI clock: %d", max_clock);
				clock_val = clock_reg = max_clock;
			} else {
				AV_DEBUG("Manual clock settings from DTS...");
				AV_DEBUG(" Clock value from DTS is %d. Host supports: Min=%d, Max=%d", clk, min_host_clock, max_host_clock);

				if ((clk < min_host_clock) || (clk > max_host_clock)) {
					AV_ERR("Please use possible & supported clock values by camera & host in DTS file");
					return -EINVAL;
				}

				AV_DEBUG(" Supported CSI clock by host and device is %d", clk);
				clock_val = clock_reg = clk;
			}
			
			/* turn off camera DPHY */
			status = i2c_bcrm_write_byte_reg(cci_reg.bcrm_address + BCRM_PHY_RESET_REG_8RW, 1);

			/*Only reset MIPI CSI2 HW *once* at sensor initialize*/
#ifndef WANDBOARD_IMX6
			if (mode == av_cam_mode_INIT)
			{
				dphy_reg_val = compute_dphy_clock_reg(clock_val * 2); // DDR -> multiply by 2

				mipi_csi2_reset_with_dphy_freq(mipi_csi2_info, dphy_reg_val);
				AV_DEBUG(" DPHY clock reg. set to 0x%02x", dphy_reg_val);
			}
#endif			

			/* After mipi_csi2_reset_with_dphy_freq() or mipi_csi2_reset() is called, 
			   we *must* turn off camera output and turn it on again. Otherwise we will get
			   mipi_csi2_dphy_status always be 0x200 or 0x230, which means i.MX6 can't get the 
			   clock from camera.
			*/
		
			/* It is important to setup the local DPHY clock first before setting the camera CSI clock */
			swapbytes(&clock_reg, AV_CAM_DATA_SIZE_32);
			CLEAR(i2c_reg);
			i2c_reg.reg = cci_reg.bcrm_address + BCRM_CSI2_CLOCK_32RW;
			i2c_reg.reg_size = AV_CAM_REG_SIZE;
			i2c_reg.count = AV_CAM_DATA_SIZE_32;
			i2c_reg.buffer = (const char *) &clock_reg;
			/* Write clock value to camera */
			status = i2c_bcrm_write_reg(&i2c_reg);

			/* turn camera dphy on again */
			status = i2c_bcrm_write_byte_reg(cci_reg.bcrm_address + BCRM_PHY_RESET_REG_8RW, 0);

			/* Read back clock value from camera */
			status = i2c_generic_read_buffer(cci_reg.bcrm_address + BCRM_CSI2_CLOCK_32RW,
						 		AV_CAM_REG_SIZE, 
								AV_CAM_DATA_SIZE_32, 
								(char *) &cur_cam_clock);

			if (status < 0) {
				return -EINVAL;
			}

			if (cur_cam_clock != clock_val)
			{
				AV_WARNING("Camera did not accept clock value. Requested: %dHz. Camera value: %dHz", clock_val, cur_cam_clock);
			}

			AV_DEBUG("Supported CSI clock by host and camera is (after reading from the camera) %d", cur_cam_clock);

			/* ########################################### */
			/* Data type setting */
			/* ########################################### */
			AV_DEBUG("Setting up CSI-2 datatype...");
			/* Read the default pixelformat from allied vision camera */
			CLEAR(ct);
			ct.id = V4L2_AV_IMX_CSI2_PIXELFORMAT_R;
			retval = ioctl_send_command(&s, &ct);

			if (retval < 0)
				return -1;

			switch (ct.value0) {
			case V4L2_PIX_FMT_CUSTOM:
				mipi_csi2_datatye = MIPI_DT_CUSTOM;
				break;
			case V4L2_PIX_FMT_UYVY:
				mipi_csi2_datatye = MIPI_DT_YUV422;
				break;
			case V4L2_PIX_FMT_RGB565:
				mipi_csi2_datatye = MIPI_DT_RGB565;
				break;
			case V4L2_PIX_FMT_RGB24:
				mipi_csi2_datatye = MIPI_DT_RGB888;
				break;
			case V4L2_PIX_FMT_BGR24:
				mipi_csi2_datatye = MIPI_DT_RGB888;
				break;
			case V4L2_PIX_FMT_GREY:
				mipi_csi2_datatye = MIPI_DT_RAW8;
				break;
			case V4L2_PIX_FMT_SBGGR8:
				mipi_csi2_datatye = MIPI_DT_RAW8;
				break;
			case V4L2_PIX_FMT_SGBRG8:
				mipi_csi2_datatye = MIPI_DT_RAW8;
				break;
			case V4L2_PIX_FMT_SGRBG8:
				mipi_csi2_datatye = MIPI_DT_RAW8;
				break;
			case V4L2_PIX_FMT_SRGGB8:
				mipi_csi2_datatye = MIPI_DT_RAW8;
				break;
			case V4L2_PIX_FMT_Y10P:
				mipi_csi2_datatye = MIPI_DT_RAW10;
				break;
			case V4L2_PIX_FMT_SBGGR10P:
				mipi_csi2_datatye = MIPI_DT_RAW10;
				break;
			case V4L2_PIX_FMT_SGBRG10P:
				mipi_csi2_datatye = MIPI_DT_RAW10;
				break;
			case V4L2_PIX_FMT_SGRBG10P:
				mipi_csi2_datatye = MIPI_DT_RAW10;
				break;
			case V4L2_PIX_FMT_SRGGB10P:
				mipi_csi2_datatye = MIPI_DT_RAW10;
				break;
			case V4L2_PIX_FMT_GREY12P:
			case V4L2_PIX_FMT_Y12P:
				mipi_csi2_datatye = MIPI_DT_RAW12;
				break;
			case V4L2_PIX_FMT_SBGGR12P:
				mipi_csi2_datatye = MIPI_DT_RAW12;
				break;
			case V4L2_PIX_FMT_SGBRG12P:
				mipi_csi2_datatye = MIPI_DT_RAW12;
				break;
			case V4L2_PIX_FMT_SGRBG12P:
				mipi_csi2_datatye = MIPI_DT_RAW12;
				break;
			case V4L2_PIX_FMT_SRGGB12P:
				mipi_csi2_datatye = MIPI_DT_RAW12;
				break;
			default:
				AV_ERR("Unsupported pixelformat!");
				break;
			}
#ifndef WANDBOARD_IMX6
			mipi_csi2_set_datatype(mipi_csi2_info, mipi_csi2_datatye);
#else
			mipi_csi2_set_datatype(mipi_csi2_data, mipi_csi2_datatye);
#endif

		} else {
			AV_ERR("Unable to enable mipi csi2 driver!");
			return -1;
		}
	} else {
		AV_ERR("Failed to get mipi_csi2_info!");
		return -1;
	}

#ifndef WANDBOARD_IMX6
	if (mipi_csi2_info) {
#else
	if (mipi_csi2_data) {
#endif
		unsigned int i;

		i = 0;

		/* wait for mipi sensor ready */
		/* 0x230 still pass this check but means mipi clock not ready */
		do
		{
#ifndef WANDBOARD_IMX6
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
#else
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_data);
#endif
			AV_DEBUG("mipi_csi2_dphy_status=0x%08x", mipi_reg);
			i++;
			msleep(10);
		} while ((mipi_reg == 0x200) && (i < 10));

		dump_dphy_state_reg(mipi_reg);

		if (i >= 10) {
			AV_ERR("mipi csi2 can not receive sensor clk!");
			return -1;
		}

		if (!(mipi_reg & 0x100))	// check for phy_rxclkactivehs
		{
			AV_WARNING("Clock lane not receiving dphy DDR clock");
		}

		i = 0;
		/* wait for mipi stable */
		do {
#ifndef WANDBOARD_IMX6
			mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
#else
			mipi_reg = mipi_csi2_get_error1(mipi_csi2_data);
#endif
			AV_DEBUG("mipi_csi2_get_error1=0x%08x", mipi_reg);
			i++;
			msleep(10);
		} while ((mipi_reg != 0x0) && (i < 10));

		dump_dphy_errstate1_reg(mipi_reg);

		if (i >= 10) {
			AV_ERR("mipi csi2 can not reveive data correctly!");
			return -1;
		}
	}

	return retval;
}

static void dump_dphy_state_reg(uint32_t reg)
{
	AV_DEBUG("DPHY_State=0x%08x", reg);
	if (reg & 0x00000001)
		AV_DEBUG(" phy_rxulpsesc_0");
	if (reg & 0x00000002)
		AV_DEBUG(" phy_rxulpsesc_1");
	if (reg & 0x00000004)
		AV_DEBUG(" phy_rxulpsesc_2");
	if (reg & 0x00000008)
		AV_DEBUG(" phy_rxulpsesc_3");
	if (reg & 0x00000010)
		AV_DEBUG(" phy_stopstatedata_0");
	if (reg & 0x00000020)
		AV_DEBUG(" phy_stopstatedata_1");
	if (reg & 0x00000040)
		AV_DEBUG(" phy_stopstatedata_2");
	if (reg & 0x00000080)
		AV_DEBUG(" phy_stopstatedata_3");
	if (reg & 0x00000100)
		AV_DEBUG(" phy_rxclkactivehs");
	if (reg & 0x00000200)
		AV_DEBUG(" phy_rxulpsclknot");
	if (reg & 0x00000400)
		AV_DEBUG(" phy_stopstateclk");
	if (reg & 0x00000800)
		AV_DEBUG(" bypass_2ecc_tst");
}

static void dump_dphy_errstate1_reg(uint32_t reg)
{
	AV_DEBUG("DPHY_ErrState1=0x%08x", reg);
	if (reg & 0x00000001)
		AV_DEBUG(" mask_phy_errsotsynchs_0");
	if (reg & 0x00000002)
		AV_DEBUG(" mask_phy_errsotsynchs_1");
	if (reg & 0x00000004)
		AV_DEBUG(" mask_phy_errsotsynchs_2");
	if (reg & 0x00000008)
		AV_DEBUG(" mask_phy_errsotsynchs_3");
	if (reg & 0x00000010)
		AV_DEBUG(" mask_err_f_bndry_match_vc0");
	if (reg & 0x00000020)
		AV_DEBUG(" mask_err_f_bndry_match_vc1");
	if (reg & 0x00000040)
		AV_DEBUG(" mask_err_f_bndry_match_vc2");
	if (reg & 0x00000080)
		AV_DEBUG(" mask_err_f_bndry_match_vc3");
	if (reg & 0x00000100)
		AV_DEBUG(" mask_err_f_seq_vc0");
	if (reg & 0x00000200)
		AV_DEBUG(" mask_err_f_seq_vc1");
	if (reg & 0x00000400)
		AV_DEBUG(" mask_err_f_seq_vc2");
	if (reg & 0x00000800)
		AV_DEBUG(" mask_err_f_seq_vc3");
	if (reg & 0x00001000)
		AV_DEBUG(" mask_err_frame_data_vc0");
	if (reg & 0x00002000)
		AV_DEBUG(" mask_err_frame_data_vc1");
	if (reg & 0x00004000)
		AV_DEBUG(" mask_err_frame_data_vc2");
	if (reg & 0x00008000)
		AV_DEBUG(" mask_err_frame_data_vc3");
	if (reg & 0x00010000)
		AV_DEBUG(" mask_err_l_bndry_match_di0");
	if (reg & 0x00020000)
		AV_DEBUG(" mask_err_l_bndry_match_di1");
	if (reg & 0x00040000)
		AV_DEBUG(" mask_err_l_bndry_match_di2");
	if (reg & 0x00080000)
		AV_DEBUG(" mask_err_l_bndry_match_di3");
	if (reg & 0x00100000)
		AV_DEBUG(" mask_err_l_seq_di0");
	if (reg & 0x00200000)
		AV_DEBUG(" mask_err_l_seq_di1");
	if (reg & 0x00400000)
		AV_DEBUG(" mask_err_l_seq_di2");
	if (reg & 0x00800000)
		AV_DEBUG(" mask_err_l_seq_di3");
	if (reg & 0x01000000)
		AV_DEBUG(" mask_vc0_err_crc");
	if (reg & 0x02000000)
		AV_DEBUG(" mask_vc1_err_crc");
	if (reg & 0x04000000)
		AV_DEBUG(" mask_vc2_err_crc");
	if (reg & 0x08000000)
		AV_DEBUG(" mask_vc3_err_crc");
}

static void dump_dphy_errstate2_reg(uint32_t reg)
{
	AV_DEBUG("DPHY_ErrState2=0x%08x", reg);
	if (reg & 0x00000001)
		AV_DEBUG(" phy_erresc_0");
	if (reg & 0x00000002)
		AV_DEBUG(" phy_erresc_1");
	if (reg & 0x00000004)
		AV_DEBUG(" phy_erresc_2");
	if (reg & 0x00000008)
		AV_DEBUG(" phy_erresc_3");
	if (reg & 0x00000010)
		AV_DEBUG(" phy_errsoths_0");
	if (reg & 0x00000020)
		AV_DEBUG(" phy_errsoths_1");
	if (reg & 0x00000040)
		AV_DEBUG(" phy_errsoths_2");
	if (reg & 0x00000080)
		AV_DEBUG(" phy_errsoths_3");
	if (reg & 0x00000100)
		AV_DEBUG(" vc0_err_ecc_corrected");
	if (reg & 0x00000200)
		AV_DEBUG(" vc1_err_ecc_corrected");
	if (reg & 0x00000400)
		AV_DEBUG(" vc2_err_ecc_corrected");
	if (reg & 0x00000800)
		AV_DEBUG(" vc3_err_ecc_corrected");
	if (reg & 0x00001000)
		AV_DEBUG(" err_id_vc0");
	if (reg & 0x00002000)
		AV_DEBUG(" err_id_vc1");
	if (reg & 0x00004000)
		AV_DEBUG(" err_id_vc2");
	if (reg & 0x00008000)
		AV_DEBUG(" err_id_vc3");
	if (reg & 0x00010000)
		AV_DEBUG(" err_l_bndry_match_di4");
	if (reg & 0x00020000)
		AV_DEBUG(" err_l_bndry_match_di5");
	if (reg & 0x00040000)
		AV_DEBUG(" err_l_bndry_match_di6");
	if (reg & 0x00080000)
		AV_DEBUG(" err_l_bndry_match_di7");
	if (reg & 0x00100000)
		AV_DEBUG(" err_l_seq_di4");
	if (reg & 0x00200000)
		AV_DEBUG(" err_l_seq_di4");
	if (reg & 0x00400000)
		AV_DEBUG(" err_l_seq_di4");
	if (reg & 0x00800000)
		AV_DEBUG(" err_l_seq_di4");
}

struct dphy_clock_reg_struct{
	uint32_t min_clock_mhz;
	uint32_t max_clock_mhz;
	uint8_t dphy_reg_val;
};

static const struct dphy_clock_reg_struct dphy_clock_reg_table[] = {
	{950, 1000, 0x74},
	{900, 950, 0x54},
	{850, 900, 0x34},
	{800, 850, 0x14},
	{750, 800, 0x32},
	{700, 750, 0x12},
	{650, 700, 0x30},
	{600, 650, 0x10},
	{550, 600, 0x2e},
	{500, 550, 0x0e},
	{450, 500, 0x2c},
	{400, 450, 0x0c},
	{360, 400, 0x4a},
	{330, 360, 0x2a},
	{300, 330, 0x08},
	{270, 300, 0x28},
	{250, 270, 0x08},
	{240, 250, 0x46},
	{210, 240, 0x26},
	{200, 210, 0x06},
	{180, 200, 0x44},
	{160, 180, 0x04},
	{150, 160, 0x04},
	{140, 150, 0x42},
	{125, 140, 0x22},
	{110, 125, 0x02},
	{100, 110, 0x40},
	{90, 100, 0x20},
	{80, 90, 0x04},
};

/**
 * @brief Compute DPHY clock register value corresponding to a given Clock in Hz
 * 
 * @param camera_clock_hz : CLock vlaue in Hz
 * @return uint8_t : DPHY register value
 */
static uint8_t compute_dphy_clock_reg(uint32_t camera_clock_hz)
{
	int i;
	for (i = 0; i<sizeof(dphy_clock_reg_table)/sizeof(struct dphy_clock_reg_struct); i++)
	{
		if (	(camera_clock_hz >= dphy_clock_reg_table[i].min_clock_mhz*1000000ULL) &&
			(camera_clock_hz < dphy_clock_reg_table[i].max_clock_mhz*1000000ULL))
		{
			AV_DEBUG("Using DPHY DDR clock value 0x%02x (%dMHz <= clock < %dMHz)", dphy_clock_reg_table[i].dphy_reg_val, dphy_clock_reg_table[i].min_clock_mhz, dphy_clock_reg_table[i].max_clock_mhz);
			return dphy_clock_reg_table[i].dphy_reg_val;
		}
	}

	AV_ERR("Invalid DPHY clock request. (%dHz)", camera_clock_hz);
	return (camera_clock_hz >= 1000*1000000ULL) ? 0x74 : 0;
}


/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	int ret;

	struct sensor_data *sensor = s->priv;
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */

	void *mipi_csi2_info;

	av_cam_data.on = true;

	/* mclk */
	tgt_xclk = av_cam_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)AV_CAM_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)AV_CAM_XCLK_MIN);
	av_cam_data.mclk = tgt_xclk;

	AV_DEBUG("   Setting mclk to %d MHz", tgt_xclk / 1000000);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	mipi_csi2_info = mipi_csi2_get_info();

	/* enable mipi csi2 */
	if (mipi_csi2_info)
		mipi_csi2_enable(mipi_csi2_info);
	else {
		AV_ERR("Failed to get mipi_csi2_info!");
		return -EPERM;
	}

	ret = alliedvision_init_mode(av_cam_mode_INIT);

	return ret;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	void *mipi_csi2_info;

	mipi_csi2_info = mipi_csi2_get_info();

	/* disable mipi csi2 */
	if (mipi_csi2_info)
		if (mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_disable(mipi_csi2_info);
	return 0;
}

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		AV_ERR("No slave device set!");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = av_cam_data.mclk;
	AV_DEBUG("  clock_curr=mclk=%d", av_cam_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = AV_CAM_XCLK_MIN;
	p->u.bt656.clock_max = AV_CAM_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc av_cam_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*) ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_init_num, (v4l2_int_ioctl_func*) ioctl_init},

	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*) ioctl_g_ifparm},

	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},

	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},

	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
			(v4l2_int_ioctl_func *) ioctl_enum_frameintervals},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *) ioctl_g_chip_ident},
	{vidioc_int_query_menu_num,
				(v4l2_int_ioctl_func *) ioctl_query_menu},

	/* New IOCTL for BCRM Mode*/
	{vidioc_int_queryctrl_num,
				(v4l2_int_ioctl_func *)ioctl_queryctrl},
	{vidioc_int_ext_queryctrl_num,
				(v4l2_int_ioctl_func *)ioctl_ext_queryctrl},
	{vidioc_int_querycap_cap_num,
				(v4l2_int_ioctl_func *)ioctl_querycap},
	{vidioc_int_g_crop_num,
				(v4l2_int_ioctl_func *)ioctl_g_crop},
	{vidioc_int_s_crop_num,
				(v4l2_int_ioctl_func *)ioctl_s_crop},
	{vidioc_int_cropcap_num,
				(v4l2_int_ioctl_func *)ioctl_cropcap},
	{vidioc_int_try_fmt_num,
				(v4l2_int_ioctl_func *) ioctl_try_fmt},
	{vidioc_int_streamon_ex_num,
				(v4l2_int_ioctl_func *) ioctl_streamon_ex},

	{vidioc_int_g_ext_ctrl_num,
				(v4l2_int_ioctl_func *) ioctl_g_ext_ctrl},
	{vidioc_int_s_ext_ctrl_num,
				(v4l2_int_ioctl_func *) ioctl_s_ext_ctrl},
	{vidioc_int_try_ext_ctrl_num,
				(v4l2_int_ioctl_func *) ioctl_try_ext_ctrl},

	/* New IOCTL for GenCP Mode*/
	{vidioc_int_gencam_i2cread_reg_num,
				(v4l2_int_ioctl_func *) ioctl_gencam_i2cread_reg},
	{vidioc_int_gencam_i2cwrite_reg_num,
				(v4l2_int_ioctl_func *) ioctl_gencam_i2cwrite_reg},
	{vidioc_int_send_command_num,
				(v4l2_int_ioctl_func *) ioctl_send_command},
};

static struct v4l2_int_slave av_cam_slave = {
	.ioctls = av_cam_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(av_cam_ioctl_desc),
};

static struct v4l2_int_device av_cam_int_device = {
	.module = THIS_MODULE,
	.name = AV_QUERYCAP_CAM_NAME,
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &av_cam_slave,
	},
};

static ssize_t get_cci_register_layout_version(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", cci_reg.cci_register_layout_version);
}

static ssize_t get_device_capabilities(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%llu", cci_reg.device_capabilities);
}

static ssize_t get_device_guid(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", cci_reg.device_guid);
}

static ssize_t get_manufacturer_name(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", cci_reg.manufacturer_name);
}

static ssize_t get_model_name(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", cci_reg.model_name);
}

static ssize_t get_family_name(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", cci_reg.family_name);
}

static ssize_t get_device_version(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", cci_reg.device_version);
}

static ssize_t get_manufacturer_info(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", cci_reg.manufacturer_info);
}

static ssize_t get_serial_number(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", cci_reg.serial_number);
}

static ssize_t get_user_defined_name(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", cci_reg.user_defined_name);
}

static ssize_t av_cam_driver_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d:%d:%d:%d\n", MAJOR_DRV, MINOR_DRV, PATCH_DRV, BUILD_DRV);
}

static ssize_t show_debug_en(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", debug);
}

static ssize_t store_debug_en(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d\n", &debug);
	return count;
}

static ssize_t show_lane_count(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", lane_count);
}


static DEVICE_ATTR(lane_count, S_IRUGO, show_lane_count, NULL);
static DEVICE_ATTR(driver_version, (S_IRUGO), av_cam_driver_version, NULL);
static DEVICE_ATTR(cci_register_layout_version, S_IRUGO, get_cci_register_layout_version, NULL);
static DEVICE_ATTR(device_capabilities, S_IRUGO, get_device_capabilities, NULL);
static DEVICE_ATTR(device_guid, S_IRUGO, get_device_guid, NULL);
static DEVICE_ATTR(manufacturer_name, S_IRUGO, get_manufacturer_name, NULL);
static DEVICE_ATTR(model_name, S_IRUGO, get_model_name, NULL);
static DEVICE_ATTR(family_name, S_IRUGO, get_family_name, NULL);
static DEVICE_ATTR(device_version, S_IRUGO, get_device_version, NULL);
static DEVICE_ATTR(manufacturer_info, S_IRUGO, get_manufacturer_info, NULL);
static DEVICE_ATTR(serial_number, S_IRUGO, get_serial_number, NULL);
static DEVICE_ATTR(user_defined_name, S_IRUGO, get_user_defined_name, NULL);
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,14,52)
static DEVICE_ATTR(debug_en, 0660, show_debug_en, store_debug_en);
#else
static DEVICE_ATTR(debug_en, S_IRUGO | S_IWUGO, show_debug_en, store_debug_en);
#endif

/*!
 * I2C probe function
 *
 * @param adapter struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int av_cam_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int retval;
	struct v4l2_int_device s;
	struct v4l2_send_command_control ct;
	struct sensor_data *sensor;

	sensor = &av_cam_data;

	/* Set initial values for the sensor struct. */
	memset(&av_cam_data, 0, sizeof(av_cam_data));

#ifndef WANDBOARD_IMX6
	sensor->mipi_camera = 1;
#endif

	retval = of_property_read_u32(dev->of_node, "clk", &(clk));
	dev_err(dev, "clk=%d\n", clk);
	if (retval) {
		dev_err(dev, "clk missing or invalid\n");
		return -EINVAL;
	}
	AV_DEBUG("Property: clk=%d", clk);

	retval = of_property_read_u32(dev->of_node, "av_cam_i2c_clk", &(av_cam_i2c_clock));
	dev_err(dev, "av_cam_i2c_clock=%d\n", av_cam_i2c_clock);
	if (retval) {
		dev_err(dev, "av_cam_i2c_clock missing or invalid\n");
		return -EINVAL;
	}
	AV_DEBUG("Property: av_cam_i2c_clock=%d", av_cam_i2c_clock);
	av_cam_i2c_clock_frequency = av_cam_i2c_clock;

#ifndef WANDBOARD_IMX6
	retval = of_property_read_u32(dev->of_node, "ipu_id", &sensor->ipu_id);
	if (retval) {
		dev_err(dev, "ipu_id missing or invalid\n");
		return -EINVAL;
	}
	AV_DEBUG("Property: sensor->ipu_id=%d", sensor->ipu_id);
#endif

	retval = of_property_read_u32(dev->of_node, "csi_id", &(av_cam_data.csi));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return -EINVAL;
	}
	AV_DEBUG("Property: av_cam_data.csi=%d", av_cam_data.csi);

	retval = of_property_read_u32(dev->of_node, "lanes", &(lane_count));
	if (retval) {
		dev_err(dev, "lanes missing or invalid\n");
		return -EINVAL;
	}
	AV_DEBUG("Property: lane_count=%d", lane_count);

	retval = of_property_read_u32(dev->of_node, "lanes_auto_conf", &(lanes_auto_conf));
	if (retval) {
		dev_err(dev, "lanes_auto_conf missing or invalid\n");
		return -EINVAL;
	}
	AV_DEBUG("Property: lanes_auto_conf=%d", lanes_auto_conf);

	retval = of_property_read_u32(dev->of_node, "clk_auto_conf", &(clk_auto_conf));
	if (retval) {
		dev_err(dev, "clk_auto_conf missing or invalid\n");
		return -EINVAL;
	}
	AV_DEBUG("Property: clk_auto_conf=%d", clk_auto_conf);

	av_cam_data.io_init = av_cam_reset;
	av_cam_data.i2c_client = client;

	av_cam_data.streamcap.capability = V4L2_CAP_TIMEPERFRAME;
	av_cam_data.streamcap.capturemode = V4L2_MODE_HIGHQUALITY;
	av_cam_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	av_cam_data.streamcap.timeperframe.numerator = 1;

	sensor->virtual_channel = sensor->csi | (sensor->ipu_id << 1);
	av_cam_int_device.priv = &av_cam_data;

#ifndef WANDBOARD_IMX6
	AV_DEBUG("sensor->csi=%d, sensor->ipu_id=%d", sensor->csi, sensor->ipu_id);
	AV_DEBUG("sensor->virtual_channel=%d, av_cam_data.virtual_channel=%d", sensor->virtual_channel, av_cam_data.virtual_channel);
#endif

	retval = read_cci_registers();

	if (retval < 0) {
		AV_ERR("Failed to read_cci_registers. (Status=%d)", retval);
		goto err;
	} else {
		AV_DEBUG("Successfully read_cci_registers. (Status=%d)", retval);
	}

	retval = read_gencp_registers();
	if (retval < 0)	{
		AV_ERR("Failed to read_gencp_registers. (Status=%d)", retval);
		goto err;
	} else {
		AV_DEBUG("Successfully read_gencp_registers. (Status=%d)", retval);
	}

	if (((__u16) cci_reg.cci_register_layout_version) == CCI_REG_LAYOUT_MINOR_VER) {
		AV_DEBUG("Success, got the Minor version %d, cci_register_layout_version 0x%x", (__u16) cci_reg.cci_register_layout_version, cci_reg.cci_register_layout_version);
	} else {
		AV_ERR("Failed, Minor version didn't match. Minor version -> %d, cci_register_layout_version 0x%x ", (__u16) cci_reg.cci_register_layout_version, cci_reg.cci_register_layout_version);
		goto err;
	}

	if ((cci_reg.cci_register_layout_version >> 16) == CCI_REG_LAYOUT_MAJOR_VER) {
		AV_DEBUG("Success, got the Major version %d, cci_register_layout_version 0x%x",  (cci_reg.cci_register_layout_version >> 16), cci_reg.cci_register_layout_version);
	} else {
		AV_ERR("Failed, Major version didn't match. Major version -> %d, cci_register_layout_version 0x%x ", (cci_reg.cci_register_layout_version >> 16), cci_reg.cci_register_layout_version);
		goto err;
	}

	if (bcrm_version_check() <= 0) {
		AV_ERR("#################### This driver is not compatiable with connected camera for BCRM ####################");
		goto err;
	} else {
		AV_DEBUG( "This driver is compatible with connected camera for BCRM");
	}

	bcrm_get_device_fw_version();
	bcrm_get_write_handshake_availibility();

	bcrm_dump();

	if (gcprm_version_check() <= 0) {
		AV_ERR("#################### This driver is not compatiable with connected camera for GCPRM ####################");
		goto err;
	} else {
		AV_DEBUG( "This driver is compatible with connected camera for GCPRM");
	}

	retval = v4l2_int_device_register(&av_cam_int_device);

	if (retval < 0)
		AV_ERR("Unable to register the device. (Status=%d)", retval);
	else
		AV_DEBUG("Allied Vision camera driver is registered to capture driver. (Status=%d)", retval);


	if (device_create_file(dev, &dev_attr_lane_count))
		dev_err(dev, "%s: error creating lane_count entry\n", __func__);
	if (device_create_file(dev, &dev_attr_cci_register_layout_version))
		dev_err(dev, "%s: error creating manufacturer_name entry\n", __func__);
	if (device_create_file(dev, &dev_attr_device_capabilities))
		dev_err(dev, "%s: error creating device_capabilities entry\n", __func__);
	if (device_create_file(dev, &dev_attr_device_guid))
		dev_err(dev, "%s: error creating device_guid entry\n", __func__);
	if (device_create_file(dev, &dev_attr_manufacturer_name))
		dev_err(dev, "%s: error creating manufacturer_name entry\n", __func__);
	if (device_create_file(dev, &dev_attr_model_name))
		dev_err(dev, "%s: error creating model_name entry\n", __func__);
	if (device_create_file(dev, &dev_attr_family_name))
		dev_err(dev, "%s: error creating family_name entry\n", __func__);
	if (device_create_file(dev, &dev_attr_device_version))
		dev_err(dev, "%s: error creating device_version entry\n", __func__);
	if (device_create_file(dev, &dev_attr_manufacturer_info))
		dev_err(dev, "%s: error creating manufacturer_info entry\n", __func__);
	if (device_create_file(dev, &dev_attr_serial_number))
		dev_err(dev, "%s: error creating serial_number entry\n", __func__);
	if (device_create_file(dev, &dev_attr_user_defined_name))
		dev_err(dev, "%s: error creating manufacturer_info entry\n", __func__);
	if (device_create_file(dev, &dev_attr_debug_en))
		dev_err(dev, "%s: error creating debug_en entry\n", __func__);
	if (device_create_file(&client->dev, &dev_attr_driver_version))
		AV_ERR("error creating driver version entry");

/* Read the default width from allied vision camera */
	CLEAR(ct);
	ct.id = V4L2_AV_IMX_CSI2_WIDTH_R;
	retval = ioctl_send_command(&s, &ct);

	if (retval < 0)
		return -ENODEV;
	av_cam_data.pix.width = ct.value0;

/* Read the default height from allied vision camera */
	CLEAR(ct);
	ct.id = V4L2_AV_IMX_CSI2_HEIGHT_R;
	retval = ioctl_send_command(&s, &ct);

	if (retval < 0)
		return -ENODEV;

	av_cam_data.pix.height = ct.value0;

/* Read the default frame size from allied vision camera */
	CLEAR(ct);
	ct.id = V4L2_AV_IMX_CSI2_PAYLOADSIZE_R;
	retval = ioctl_send_command(&s, &ct);

	if (retval < 0)
		return -ENODEV;
	av_cam_data.pix.sizeimage = ct.value0;

#ifdef WANDBOARD_IMX6
/* Read the default pixelformat from allied vision camera */
	CLEAR(ct);
	ct.id = V4L2_AV_IMX_CSI2_PIXELFORMAT_R;
	retval = ioctl_send_command(&s, &ct);

	if (retval < 0)
		return -ENODEV;

	av_cam_data.pix.pixelformat = ct.value0;
#endif

	AV_DEBUG("Camera parameters:");
	AV_DEBUG("cam->v2f.fmt.pix.width=%d", av_cam_data.pix.width);
	AV_DEBUG("cam->v2f.fmt.pix.height=%d", av_cam_data.pix.height);
	AV_DEBUG("cam->v2f.fmt.pix.pixelformat=%d (0x%x)", av_cam_data.pix.pixelformat, av_cam_data.pix.pixelformat);
	AV_DEBUG("cam->v2f.fmt.pix.sizeimage=%d", av_cam_data.pix.sizeimage);

	retval = supported_pixformat();

	AV_DEBUG("supported_pixformat retval=%d, avail_pixformats=0x%x", retval, avail_pixformats);

	if (retval < 0) {
		AV_ERR("Pixelformat support is not available for the connected camera");
		goto err;
	}

	AV_DEBUG("Camera found");
	return retval;

err:
	AV_ERR("Camera not found");
	return -ENODEV;
}

/*!
 *  Allied Vision Camera I2C detach function
 *
 * @param client struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int av_cam_remove(struct i2c_client *client)
{
	device_remove_file(&client->dev, &dev_attr_lane_count);
	device_remove_file(&client->dev, &dev_attr_cci_register_layout_version);
	device_remove_file(&client->dev, &dev_attr_device_capabilities);
	device_remove_file(&client->dev, &dev_attr_device_guid);
	device_remove_file(&client->dev, &dev_attr_manufacturer_name);
	device_remove_file(&client->dev, &dev_attr_model_name);
	device_remove_file(&client->dev, &dev_attr_family_name);
	device_remove_file(&client->dev, &dev_attr_device_version);
	device_remove_file(&client->dev, &dev_attr_manufacturer_info);
	device_remove_file(&client->dev, &dev_attr_serial_number);
	device_remove_file(&client->dev, &dev_attr_user_defined_name);
	device_remove_file(&client->dev, &dev_attr_driver_version);
	device_remove_file(&client->dev, &dev_attr_debug_en);

	v4l2_int_device_unregister(&av_cam_int_device);

	return 0;
}

/*!
 * av_cam_init function
 *
 * @return  Error code indicating success or failure
 */
static __init int av_cam_init(void)
{
	u8 err;

	err = i2c_add_driver(&av_cam_i2c_driver);

	AV_DEBUG("AV debug=%d", debug);

	if (err != 0)
		AV_ERR("Driver registration failed. (Status=%d)", err);

	return err;
}

/*!
 * av_cam_clean function
 *
 * @return  Error code indicating success or failure
 */
static void __exit av_cam_clean(void)
{
	i2c_del_driver(&av_cam_i2c_driver);
}

module_init(av_cam_init);
module_exit(av_cam_clean);

MODULE_AUTHOR("Allied Vision Technologies GmbH");
MODULE_DESCRIPTION("Allied Vision i.MX6 MIPI-CSI2 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS("Allied Vision MIPI-CSI2 Driver");
