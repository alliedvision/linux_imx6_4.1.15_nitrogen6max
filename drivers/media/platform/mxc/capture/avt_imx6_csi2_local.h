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

#ifndef __AVT_IMX6_CSI2_LOCAL_H__
#define __AVT_IMX6_CSI2_LOCAL_H__

#include "avt_imx6_csi2_boardspec.h"

#define V4L2_AV_IMX_CSI2_BASE			0x1000
#define V4L2_AV_IMX_CSI2_WIDTH_R		(V4L2_AV_IMX_CSI2_BASE+0x0001)
#define V4L2_AV_IMX_CSI2_WIDTH_W		(V4L2_AV_IMX_CSI2_BASE+0x0002)
#define V4L2_AV_IMX_CSI2_WIDTH_MINVAL_R		(V4L2_AV_IMX_CSI2_BASE+0x0003)
#define V4L2_AV_IMX_CSI2_WIDTH_MAXVAL_R		(V4L2_AV_IMX_CSI2_BASE+0x0004)
#define V4L2_AV_IMX_CSI2_WIDTH_INCVAL_R		(V4L2_AV_IMX_CSI2_BASE+0x0005)
#define V4L2_AV_IMX_CSI2_HEIGHT_R		(V4L2_AV_IMX_CSI2_BASE+0x0006)
#define V4L2_AV_IMX_CSI2_HEIGHT_W		(V4L2_AV_IMX_CSI2_BASE+0x0007)
#define V4L2_AV_IMX_CSI2_HEIGHT_MINVAL_R	(V4L2_AV_IMX_CSI2_BASE+0x0008)
#define V4L2_AV_IMX_CSI2_HEIGHT_MAXVAL_R	(V4L2_AV_IMX_CSI2_BASE+0x0009)
#define V4L2_AV_IMX_CSI2_HEIGHT_INCVAL_R	(V4L2_AV_IMX_CSI2_BASE+0x000A)
#define V4L2_AV_IMX_CSI2_PIXELFORMAT_R		(V4L2_AV_IMX_CSI2_BASE+0x000B)
#define V4L2_AV_IMX_CSI2_PIXELFORMAT_W		(V4L2_AV_IMX_CSI2_BASE+0x000C)
#define V4L2_AV_IMX_CSI2_PAYLOADSIZE_R		(V4L2_AV_IMX_CSI2_BASE+0x000D)
#define V4L2_AV_IMX_CSI2_STREAMON_W		(V4L2_AV_IMX_CSI2_BASE+0x000E)
#define V4L2_AV_IMX_CSI2_STREAMOFF_W		(V4L2_AV_IMX_CSI2_BASE+0x000F)
#define V4L2_AV_IMX_CSI2_ABORT_W		(V4L2_AV_IMX_CSI2_BASE+0x0010)
#define V4L2_AV_IMX_CSI2_ACQ_STATUS_R		(V4L2_AV_IMX_CSI2_BASE+0x0011)
#define V4L2_AV_IMX_CSI2_HFLIP_R		(V4L2_AV_IMX_CSI2_BASE+0x0012)
#define V4L2_AV_IMX_CSI2_HFLIP_W		(V4L2_AV_IMX_CSI2_BASE+0x0013)
#define V4L2_AV_IMX_CSI2_VFLIP_R		(V4L2_AV_IMX_CSI2_BASE+0x0014)
#define V4L2_AV_IMX_CSI2_VFLIP_W		(V4L2_AV_IMX_CSI2_BASE+0x0015)
#define V4L2_AV_IMX_CSI2_OFFSET_X_W		(V4L2_AV_IMX_CSI2_BASE+0x0016)
#define V4L2_AV_IMX_CSI2_OFFSET_X_R		(V4L2_AV_IMX_CSI2_BASE+0x0017)
#define V4L2_AV_IMX_CSI2_OFFSET_X_MIN_R		(V4L2_AV_IMX_CSI2_BASE+0x0018)
#define V4L2_AV_IMX_CSI2_OFFSET_X_MAX_R		(V4L2_AV_IMX_CSI2_BASE+0x0019)
#define V4L2_AV_IMX_CSI2_OFFSET_X_INC_R		(V4L2_AV_IMX_CSI2_BASE+0x001A)
#define V4L2_AV_IMX_CSI2_OFFSET_Y_W		(V4L2_AV_IMX_CSI2_BASE+0x001B)
#define V4L2_AV_IMX_CSI2_OFFSET_Y_R		(V4L2_AV_IMX_CSI2_BASE+0x001C)
#define V4L2_AV_IMX_CSI2_OFFSET_Y_MIN_R		(V4L2_AV_IMX_CSI2_BASE+0x001D)
#define V4L2_AV_IMX_CSI2_OFFSET_Y_MAX_R		(V4L2_AV_IMX_CSI2_BASE+0x001E)
#define V4L2_AV_IMX_CSI2_OFFSET_Y_INC_R		(V4L2_AV_IMX_CSI2_BASE+0x001F)
#define V4L2_AV_IMX_CSI2_SENSOR_WIDTH_R		(V4L2_AV_IMX_CSI2_BASE+0x0020)
#define V4L2_AV_IMX_CSI2_SENSOR_HEIGHT_R	(V4L2_AV_IMX_CSI2_BASE+0x0021)
#define V4L2_AV_IMX_CSI2_MAX_WIDTH_R		(V4L2_AV_IMX_CSI2_BASE+0x0022)
#define V4L2_AV_IMX_CSI2_MAX_HEIGHT_R		(V4L2_AV_IMX_CSI2_BASE+0x0023)
#define V4L2_AV_IMX_CSI2_CURRENTMODE_R		(V4L2_AV_IMX_CSI2_BASE+0x0024)
#define V4L2_AV_IMX_CSI2_CHANGEMODE_W		(V4L2_AV_IMX_CSI2_BASE+0x0025)

#define V4L2_AV_IMX_CSI2_BCRM_MODE		0x00
#define V4L2_AV_IMX_CSI2_GENCP_MODE		0x01

#define MICRO_HZ		1000000UL
#define IMX_CAM_PORT_NO		1

#define IPU_X_VALID 		1
#define IPU_X_MIN 		16
#define IPU_X_MAX 		8192
#define IPU_X_INC 		16
#define IPU_Y_VALID 		1
#define IPU_Y_MIN 		16
#define IPU_Y_MAX 		4096
#define IPU_Y_INC 		16

#define V4L2_MIN_ANNOUNCED_FRAMES 3

#define LANE_RANGE_1_VALID	1
#define LANE_RANGE_1_MIN 	10000000UL
#define LANE_RANGE_1_MAX 	500000000UL
#define LANE_RANGE_2_VALID 	1
#define LANE_RANGE_2_MIN 	10000000UL
#define LANE_RANGE_2_MAX 	500000000UL
#define LANE_RANGE_3_VALID 	1
#define LANE_RANGE_3_MIN	10000000UL
#define LANE_RANGE_3_MAX	500000000UL
#define LANE_RANGE_4_VALID	1
#define LANE_RANGE_4_MIN	10000000UL
#define LANE_RANGE_4_MAX	395000000UL

/* imx6 Host supports 1, 2, 3, and 4 lanes configuration */
#define	CSI_HOST_SUPPORTED_LANE_COUNT	0xF

# define DATA_IDENTIFIER_INQ_1 	0x0002000000000000ull
# define DATA_IDENTIFIER_INQ_2	0x0
# define DATA_IDENTIFIER_INQ_3	0x0
# define DATA_IDENTIFIER_INQ_4	0x0

/* For reading CRC/Buffer Overrun */
#define VC0_ERR_ECC      	((uint32_t) 1 << 8)
#define VC0_ERR_HDRCRC   	((uint32_t) 1 << 28)
#define VC0_ERR_CRC      	((uint32_t) 1 << 24)
#define VC0_ERR2_CRC     	((uint32_t) 1 << 12)
#define VC0_ERR_FRM      	((uint32_t) 1 << 4)
#define VC0_ERR2_FRM     	((uint32_t) 1 << 8)
#define VC0_ERR_LIN     	((uint32_t) 1 << 20)
#define VC0_ERR2_LIN     	((uint32_t) 1 << 16)

#define VC1_ERR_CRC      	((uint32_t) 1 << 25)
#define VC2_ERR_CRC      	((uint32_t) 1 << 26)
#define VC3_ERR_CRC      	((uint32_t) 1 << 27)
#define ERR_F_BNDRY_MATCH_VC0	((uint32_t) 1 << 4)
#define ERR_F_BNDRY_MATCH_VC1	((uint32_t) 1 << 5)
#define ERR_F_BNDRY_MATCH_VC2	((uint32_t) 1 << 6)
#define ERR_F_BNDRY_MATCH_VC3	((uint32_t) 1 << 7)

#define AV_CAM_REG_SIZE		2
#define AV_CAM_DATA_SIZE_8	1
#define AV_CAM_DATA_SIZE_16	2
#define AV_CAM_DATA_SIZE_32	4
#define AV_CAM_DATA_SIZE_64	8

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

/* Driver release version */
#define	MAJOR_DRV		1
#define	MINOR_DRV		0
#define	PATCH_DRV		0
#define	BUILD_DRV		0
#define DRIVER_VERSION		STR(MAJOR_DRV) "." STR(MINOR_DRV) "." STR(PATCH_DRV) "." STR(BUILD_DRV)

#define AV_CAM_SYSFS_NAME	"avt_csi2_if1"
#define AV_QUERYCAP_CAM_NAME	"avt_imx6_csi2"
#define AV_QUERYCAP_BUSINFO	"csi2"

extern __u16 read_gencp_out_buffer_size;
extern __u16 read_gencp_in_buffer_size;
extern int av_cam_i2c_clock_frequency;
extern int lane_count;
extern bool is_gencp_mode;

/* imx6 Host supports 1, 2, 3, and 4 lanes configuration */
enum v4l2_lane_counts {
	V4L2_LANE_COUNT_1_LaneSupport  = 0x1,
	V4L2_LANE_COUNT_2_LaneSupport  = 0x2,
	V4L2_LANE_COUNT_3_LaneSupport  = 0x4,
	V4L2_LANE_COUNT_4_LaneSupport  = 0x8,
};

enum bayer_format {
	monochrome,/* 0 */
	bayer_gr,
	bayer_rg,
	bayer_gb,
	bayer_bg,
};

struct bcrm_to_v4l2 {
	int64_t min_bcrm;
	int64_t max_bcrm;
	int64_t step_bcrm;
	int32_t min_v4l2;
	int32_t max_v4l2;
	int32_t step_v4l2;
};

enum convert_type {
	min_enum,/* 0 */
	max_enum,
	step_enum,
};

#define INT32_MAX	0x7FFFFFFFU
#define INT32_MIN	0x80000000U


#endif				/* __ALLIEDVISION_IMX6_CSI2_LOCAL_H__ */
