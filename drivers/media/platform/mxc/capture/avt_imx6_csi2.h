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

#ifndef __AVT_IMX6_CSI2_H__
#define __AVT_IMX6_CSI2_H__

/* Lane count supported by the imx6 */
#define	CSI_HOST_MIN_LANE_COUNT			1
#define	CSI_HOST_MAX_LANE_COUNT			4
#define	CSI_HOST_STEP_LANE_COUNT		1

#define	CSI_HOST_SUPPORTED_LANE_COUNT_QUAD_PLUS	0xF
#define	CSI_HOST_SUPPORTED_LANE_COUNT_SOLO_DL	0x3

/* #define TOTAL_SUPP_PIXFORMATS	38 */
/* TODO */
/* host_map[] this needs to be updated if the 
* above line needs to be uncommented
*/
#define TOTAL_SUPP_PIXFORMATS			28
#define HOST_SUPPORTED_PIXFMTS			0x3FFF8005A0
#define MIPI_AVAIL_MASK				0x7F
#define BAYER_MASK				0x1F
#define RGB888_MASK				0xF00

#define RGB888_SHIFT	(uint64_t)RGB888_MASK) >> 8)
#define RGB888_PIX	9
#define RAW6_SHIFT	(uint64_t)MIPI_AVAIL_MASK) << 13
#define RAW6_PIX	(1 << 12)
#define RAW7_SHIFT	(uint64_t)MIPI_AVAIL_MASK) << 18
#define RAW7_PIX	(1 << 13)
#define RAW8_SHIFT	(uint64_t)MIPI_AVAIL_MASK) << 23
#define RAW8_PIX	(1 << 14)
#define RAW10_SHIFT	(uint64_t)MIPI_AVAIL_MASK) << 28
#define RAW10_PIX	(1 << 15)
#define RAW12_SHIFT	(uint64_t)MIPI_AVAIL_MASK) << 33
#define RAW12_PIX	(1 << 16)

#define IMX6_CSI_HOST_CLK_MIN_FREQ_HZ  		1000000ul
#define CSI_HOST_CLK_MAX_FREQ_HZ_1L    		500000000ul
#define CSI_HOST_CLK_MAX_FREQ_HZ_2L    		500000000ul
#define CSI_HOST_CLK_MAX_FREQ_HZ_3L    		500000000ul
#define CSI_HOST_CLK_MAX_FREQ_HZ_4L    		395000000ul

#define BCRM_DEVICE_VERSION			0x00010000ul
#define BCRM_MAJOR_VERSION			0x0001ul
#define BCRM_MINOR_VERSION			0x0000ul

#define GCPRM_DEVICE_VERSION			0x00010000ul
#define GCPRM_MAJOR_VERSION			0x0001ul
#define GCPRM_MINOR_VERSION			0x0000ul

#define CLEAR(x)	memset(&(x), 0, sizeof(x))

#define MICRO_HZ				1000000UL
#define EXP_ABS					100000UL

#define AV_SUCCESS				1
#define AV_CAM_ERR_SUCCESS			0
#define AV_CAM_ERR_CRC_FAIL  			-1
#define AV_CAM_ERR_I2CREAD_FAIL 		-2

#define CCI_REG_LAYOUT_MINOR_VER		0
#define CCI_REG_LAYOUT_MAJOR_VER		1

const int bsti = 1;  /* Byte swap test integer */
#define is_bigendian() ((*(char *)&bsti) == 0)

/* General Control Registers */
#define BCRM_VERSION_REG_32R  				0x0000
#define BCRM_FEATURE_INQUIRY_REG_64R 			0x0008
#define BCRM_DEVICE_FIRMWARE_VERSION_REG_64R 		0x0010
#define BCRM_WRITE_HANDSHAKE_REG_8RW			0x0018

/* Streaming Control Registers */
#define BCRM_SUPPORTED_CSI2_LANE_COUNTS_8R 		0x0040
#define BCRM_CSI2_LANE_COUNT_8RW 			0x0044
#define BCRM_CSI2_CLOCK_MIN_32R 			0x0048
#define BCRM_CSI2_CLOCK_MAX_32R 			0x004C
#define BCRM_CSI2_CLOCK_32RW 				0x0050
#define BCRM_BUFFER_SIZE_32R 				0x0054
#define BCRM_IPU_X_MIN_32W				0x0058
#define BCRM_IPU_X_MAX_32W				0x005C
#define BCRM_IPU_X_INC_32W				0x0060
#define BCRM_IPU_Y_MIN_32W				0x0064
#define BCRM_IPU_Y_MAX_32W				0x0068
#define BCRM_IPU_Y_INC_32W				0x006C
#define BCRM_IPU_X_32R					0x0070
#define BCRM_IPU_Y_32R					0x0074
#define BCRM_PHY_RESET_REG_8RW				0x0078

/* Acquisition Control Registers */
#define BCRM_ACQUISITION_START_8RW  			0x0080
#define BCRM_ACQUISITION_STOP_8RW 			0x0084
#define BCRM_ACQUISITION_ABORT_8RW 			0x0088
#define BCRM_ACQUISITION_STATUS_8R 			0x008C
#define BCRM_ACQUISITION_FRAME_RATE_64RW 		0x0090
#define BCRM_ACQUISITION_FRAME_RATE_MIN_64R 		0x0098
#define BCRM_ACQUISITION_FRAME_RATE_MAX_64R 		0x00A0
#define BCRM_ACQUISITION_FRAME_RATE_INC_64R 		0x00A8

#define BCRM_FRAME_RATE_ENABLE_8RW 			0x00B0
#define BCRM_FRAME_START_TRIGGER_MODE_8RW 		0x00B4
#define BCRM_FRAME_START_TRIGGER_SOURCE_8RW 		0x00B8
#define BCRM_FRAME_START_TRIGGER_ACTIVATION_8RW 	0x00BC
#define BCRM_TRIGGER_SOFTWARE_8W 			0x00C0

/* Image Format Control Registers */
#define BCRM_IMG_WIDTH_32RW  				0x0100
#define BCRM_IMG_WIDTH_MIN_32R 				0x0104
#define BCRM_IMG_WIDTH_MAX_32R 				0x0108
#define BCRM_IMG_WIDTH_INC_32R 				0x010C

#define BCRM_IMG_HEIGHT_32RW				0x0110
#define BCRM_IMG_HEIGHT_MIN_32R 			0x0114
#define BCRM_IMG_HEIGHT_MAX_32R 			0x0118
#define BCRM_IMG_HEIGHT_INC_32R				0x011C

#define BCRM_IMG_OFFSET_X_32RW				0x0120
#define BCRM_IMG_OFFSET_X_MIN_32R 			0x0124
#define BCRM_IMG_OFFSET_X_MAX_32R 			0x0128
#define BCRM_IMG_OFFSET_X_INC_32R 			0x012C

#define BCRM_IMG_OFFSET_Y_32RW				0x0130
#define BCRM_IMG_OFFSET_Y_MIN_32R 			0x0134
#define BCRM_IMG_OFFSET_Y_MAX_32R 			0x0138
#define BCRM_IMG_OFFSET_Y_INC_32R 			0x013C

#define BCRM_IMG_MIPI_DATA_FORMAT_32RW 			0x0140
#define BCRM_IMG_AVAILABLE_MIPI_DATA_FORMATS_64R  	0x0148

#define BCRM_IMG_BAYER_PATTERN_INQUIRY_8R 		0x0150
#define BCRM_IMG_BAYER_PATTERN_8RW 			0x0154

#define BCRM_IMG_REVERSE_X_8RW 				0x0158
#define BCRM_IMG_REVERSE_Y_8RW 				0x015C

#define BCRM_SENSOR_WIDTH_32R  				0x0160
#define BCRM_SENSOR_HEIGHT_32R 				0x0164

#define BCRM_WIDTH_MAX_32R 				0x0168
#define BCRM_HEIGHT_MAX_32R 				0x016C

/* Brightness Control Registers */
#define BCRM_EXPOSURE_TIME_64RW 			0x0180
#define BCRM_EXPOSURE_TIME_MIN_64R 			0x0188
#define BCRM_EXPOSURE_TIME_MAX_64R 			0x0190
#define BCRM_EXPOSURE_TIME_INC_64R 			0x0198
#define BCRM_EXPOSURE_AUTO_8RW 				0x01A0

#define BCRM_INTENSITY_AUTO_PRECEDENCE_8RW 		0x01A4
#define BCRM_INTENSITY_AUTO_PRECEDENCE_VALUE_32RW 	0x01A8
#define BCRM_INTENSITY_AUTO_PRECEDENCE_MIN_32R 		0x01AC
#define BCRM_INTENSITY_AUTO_PRECEDENCE_MAX_32R 		0x01B0
#define BCRM_INTENSITY_AUTO_PRECEDENCE_INC_32R 		0x01B4

#define BCRM_BLACK_LEVEL_32RW 				0x01B8
#define BCRM_BLACK_LEVEL_MIN_32R 			0x01BC
#define BCRM_BLACK_LEVEL_MAX_32R 			0x01C0
#define BCRM_BLACK_LEVEL_INC_32R 			0x01C4

#define BCRM_GAIN_64RW 					0x01C8
#define BCRM_GAIN_MIN_64R 				0x01D0
#define BCRM_GAIN_MAX_64R				0x01D8
#define BCRM_GAIN_INC_64R 				0x01E0
#define BCRM_GAIN_AUTO_8RW 				0x01E8

#define BCRM_GAMMA_64RW 				0x01F0
#define BCRM_GAMMA_MIN_64R 				0x01F8
#define BCRM_GAMMA_MAX_64R 				0x0200
#define BCRM_GAMMA_INC_64R 				0x0208

#define BCRM_CONTRAST_ENABLE_8RW 			0x0210
#define BCRM_CONTRAST_VALUE_32RW 			0x0214
#define BCRM_CONTRAST_VALUE_MIN_32R 			0x0218
#define BCRM_CONTRAST_VALUE_MAX_32R			0x021C
#define BCRM_CONTRAST_VALUE_INC_32R 			0x0220

/* Color Management Registers */
#define BCRM_SATURATION_32RW 				0x0240
#define BCRM_SATURATION_MIN_32R 			0x0244
#define BCRM_SATURATION_MAX_32R 			0x0248
#define BCRM_SATURATION_INC_32R				0x024C

#define BCRM_HUE_32RW 					0x0250
#define BCRM_HUE_MIN_32R 				0x0254
#define BCRM_HUE_MAX_32R 				0x0258
#define BCRM_HUE_INC_32R 				0x025C

#define BCRM_ALL_BALANCE_RATIO_64RW 			0x0260
#define BCRM_ALL_BALANCE_RATIO_MIN_64R 			0x0268
#define BCRM_ALL_BALANCE_RATIO_MAX_64R 			0x0270
#define BCRM_ALL_BALANCE_RATIO_INC_64R 			0x0278

#define BCRM_RED_BALANCE_RATIO_64RW 			0x0280
#define BCRM_RED_BALANCE_RATIO_MIN_64R 			0x0288
#define BCRM_RED_BALANCE_RATIO_MAX_64R 			0x0290
#define BCRM_RED_BALANCE_RATIO_INC_64R 			0x0298

#define BCRM_GREEN_BALANCE_RATIO_64RW 			0x02A0
#define BCRM_GREEN_BALANCE_RATIO_MIN_64R 		0x02A8
#define BCRM_GREEN_BALANCE_RATIO_MAX_64R 		0x02B0
#define BCRM_GREEN_BALANCE_RATIO_INC_64R 		0x02B8

#define BCRM_BLUE_BALANCE_RATIO_64RW 			0x02C0
#define BCRM_BLUE_BALANCE_RATIO_MIN_64R 		0x02C8
#define BCRM_BLUE_BALANCE_RATIO_MAX_64R 		0x02D0
#define BCRM_BLUE_BALANCE_RATIO_INC_64R 		0x02D8

#define BCRM_WHITE_BALANCE_AUTO_8RW 			0x02E0

/* Other Registers */
#define BCRM_SHARPNESS_32RW 				0x0300
#define BCRM_SHARPNESS_MIN_32R 				0x0304
#define BCRM_SHARPNESS_MAX_32R 				0x0308
#define BCRM_SHARPNESS_INC_32R 				0x030C

#define BCRM_DEVICE_TEMPERATURE_32R			0x0310
#define BCRM_ANALOG_GAIN_64RW				0x0314
#define BCRM_ANALOG_GAIN_MIN_64RW			0x031C
#define BCRM_ANALOG_GAIN_MAX_64RW			0x0324

#define _BCRM_LAST_ADDR					BCRM_ANALOG_GAIN_MAX_64RW

/* GenCP Registers */
#define GENCP_CHANGEMODE_8W 				0x021C
#define GENCP_CURRENTMODE_8R 				0x021D

/* BCRM exposure Auto values */
#define EXPOSURE_AUTO_OFF			0
#define EXPOSURE_AUTO_ONCE			1
#define EXPOSURE_AUTO_CONTINUOUS		2

#define GAIN_AUTO_OFF				0
#define GAIN_AUTO_ONCE				1
#define GAIN_AUTO_CONTINUOUS			2

#define WHITEBALANCE_AUTO_OFF			0
#define WHITEBALANCE_AUTO_ONCE			1
#define WHITEBALANCE_AUTO_CONTINUOUS		2

/* BCRM Acquisition status values */
#define ACQUISITION_STOPPED			0
#define ACQUISITION_RUNNING			1

/* CCI Device capability String encoding */
#define CCI_STRING_ENC_ASCII			0
#define CCI_STRING_ENC_UTF8			1
#define CCI_STRING_ENC_UTF16			2

#define OPERATION_MODE_BCRM			0
#define OPERATION_MODE_GENCP 			1

#define ACCESS_MODE_R				0x01
#define ACCESS_MODE_W				0x02

struct bcrm_reg_struct_struct{
	uint16_t reg;
	uint8_t access_mode;
};

static const struct bcrm_reg_struct_struct bcrm_reg_struct_table[] = {
	{BCRM_VERSION_REG_32R, 			ACCESS_MODE_R},
	{BCRM_FEATURE_INQUIRY_REG_64R, 		ACCESS_MODE_R},
	{BCRM_DEVICE_FIRMWARE_VERSION_REG_64R, 	ACCESS_MODE_R},
	{BCRM_WRITE_HANDSHAKE_REG_8RW, 		ACCESS_MODE_R | ACCESS_MODE_W},

	/* Streaming Control Registers */
	{BCRM_SUPPORTED_CSI2_LANE_COUNTS_8R, 	ACCESS_MODE_R},
	{BCRM_CSI2_LANE_COUNT_8RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_CSI2_CLOCK_MIN_32R, 		ACCESS_MODE_R},
	{BCRM_CSI2_CLOCK_MAX_32R, 		ACCESS_MODE_R},
	{BCRM_CSI2_CLOCK_32RW, 			ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_BUFFER_SIZE_32R, 			ACCESS_MODE_R},
	{BCRM_IPU_X_MIN_32W, 			ACCESS_MODE_W},
	{BCRM_IPU_X_MAX_32W, 			ACCESS_MODE_W},
	{BCRM_IPU_X_INC_32W, 			ACCESS_MODE_W},
	{BCRM_IPU_Y_MIN_32W, 			ACCESS_MODE_W},
	{BCRM_IPU_Y_MAX_32W, 			ACCESS_MODE_W},
	{BCRM_IPU_Y_INC_32W, 			ACCESS_MODE_W},
	{BCRM_IPU_X_32R, 			ACCESS_MODE_R},
	{BCRM_IPU_Y_32R, 			ACCESS_MODE_R},
	{BCRM_PHY_RESET_REG_8RW, 		ACCESS_MODE_R | ACCESS_MODE_W},

	/* Acquisition Control Registers */
	{BCRM_ACQUISITION_START_8RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_ACQUISITION_STOP_8RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_ACQUISITION_ABORT_8RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_ACQUISITION_STATUS_8R, 		ACCESS_MODE_R},
	{BCRM_ACQUISITION_FRAME_RATE_64RW, 	ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_ACQUISITION_FRAME_RATE_MIN_64R, 	ACCESS_MODE_R},
	{BCRM_ACQUISITION_FRAME_RATE_MAX_64R, 	ACCESS_MODE_R},
	{BCRM_ACQUISITION_FRAME_RATE_INC_64R, 	ACCESS_MODE_R},

	{BCRM_FRAME_RATE_ENABLE_8RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_FRAME_START_TRIGGER_MODE_8RW, 	ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_FRAME_START_TRIGGER_SOURCE_8RW, 	ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_FRAME_START_TRIGGER_ACTIVATION_8RW, ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_TRIGGER_SOFTWARE_8W, 		ACCESS_MODE_W},

	/* Image Format Control Registers */
	{BCRM_IMG_WIDTH_32RW, 			ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_IMG_WIDTH_MIN_32R, 		ACCESS_MODE_R},
	{BCRM_IMG_WIDTH_MAX_32R, 		ACCESS_MODE_R},
	{BCRM_IMG_WIDTH_INC_32R, 		ACCESS_MODE_R},

	{BCRM_IMG_HEIGHT_32RW, 			ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_IMG_HEIGHT_MIN_32R, 		ACCESS_MODE_R},
	{BCRM_IMG_HEIGHT_MAX_32R, 		ACCESS_MODE_R},
	{BCRM_IMG_HEIGHT_INC_32R, 		ACCESS_MODE_R},

	{BCRM_IMG_OFFSET_X_32RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_IMG_OFFSET_X_MIN_32R, 		ACCESS_MODE_R},
	{BCRM_IMG_OFFSET_X_MAX_32R, 		ACCESS_MODE_R},
	{BCRM_IMG_OFFSET_X_INC_32R,		ACCESS_MODE_R},

	{BCRM_IMG_OFFSET_Y_32RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_IMG_OFFSET_Y_MIN_32R, 		ACCESS_MODE_R},
	{BCRM_IMG_OFFSET_Y_MAX_32R, 		ACCESS_MODE_R},
	{BCRM_IMG_OFFSET_Y_INC_32R, 		ACCESS_MODE_R},

	{BCRM_IMG_MIPI_DATA_FORMAT_32RW, 	ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_IMG_AVAILABLE_MIPI_DATA_FORMATS_64R, ACCESS_MODE_R},

	{BCRM_IMG_BAYER_PATTERN_INQUIRY_8R, 	ACCESS_MODE_R},
	{BCRM_IMG_BAYER_PATTERN_8RW, 		ACCESS_MODE_R | ACCESS_MODE_W},

	{BCRM_IMG_REVERSE_X_8RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_IMG_REVERSE_Y_8RW, 		ACCESS_MODE_R | ACCESS_MODE_W},

	{BCRM_SENSOR_WIDTH_32R, 		ACCESS_MODE_R},
	{BCRM_SENSOR_HEIGHT_32R, 		ACCESS_MODE_R},

	{BCRM_WIDTH_MAX_32R,			ACCESS_MODE_R},
	{BCRM_HEIGHT_MAX_32R, 			ACCESS_MODE_R},

	/* Brightness Control Registers */
	{BCRM_EXPOSURE_TIME_64RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_EXPOSURE_TIME_MIN_64R,		ACCESS_MODE_R},
	{BCRM_EXPOSURE_TIME_MAX_64R,		ACCESS_MODE_R},
	{BCRM_EXPOSURE_TIME_INC_64R, 		ACCESS_MODE_R},
	{BCRM_EXPOSURE_AUTO_8RW, 		ACCESS_MODE_R | ACCESS_MODE_W},

	{BCRM_INTENSITY_AUTO_PRECEDENCE_8RW, 	ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_INTENSITY_AUTO_PRECEDENCE_VALUE_32RW, ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_INTENSITY_AUTO_PRECEDENCE_MIN_32R, ACCESS_MODE_R},
	{BCRM_INTENSITY_AUTO_PRECEDENCE_MAX_32R, ACCESS_MODE_R},
	{BCRM_INTENSITY_AUTO_PRECEDENCE_INC_32R, ACCESS_MODE_R},

	{BCRM_BLACK_LEVEL_32RW, 		ACCESS_MODE_R | ACCESS_MODE_W},	
	{BCRM_BLACK_LEVEL_MIN_32R, 		ACCESS_MODE_R},
	{BCRM_BLACK_LEVEL_MAX_32R, 		ACCESS_MODE_R},
	{BCRM_BLACK_LEVEL_INC_32R, 		ACCESS_MODE_R},

	{BCRM_GAIN_64RW, 			ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_GAIN_MIN_64R, 			ACCESS_MODE_R},
	{BCRM_GAIN_MAX_64R, 			ACCESS_MODE_R},
	{BCRM_GAIN_INC_64R, 			ACCESS_MODE_R},
	{BCRM_GAIN_AUTO_8RW, 			ACCESS_MODE_R | ACCESS_MODE_W},

	{BCRM_GAMMA_64RW, 			ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_GAMMA_MIN_64R, 			ACCESS_MODE_R},
	{BCRM_GAMMA_MAX_64R, 			ACCESS_MODE_R},
	{BCRM_GAMMA_INC_64R, 			ACCESS_MODE_R},

	{BCRM_CONTRAST_ENABLE_8RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_CONTRAST_VALUE_32RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_CONTRAST_VALUE_MIN_32R, 		ACCESS_MODE_R},
	{BCRM_CONTRAST_VALUE_MAX_32R, 		ACCESS_MODE_R},
	{BCRM_CONTRAST_VALUE_INC_32R, 		ACCESS_MODE_R},

	/* Color Management Registers */
	{BCRM_SATURATION_32RW, 			ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_SATURATION_MIN_32R, 		ACCESS_MODE_R},
	{BCRM_SATURATION_MAX_32R, 		ACCESS_MODE_R},
	{BCRM_SATURATION_INC_32R, 		ACCESS_MODE_R},

	{BCRM_HUE_32RW, 			ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_HUE_MIN_32R, 			ACCESS_MODE_R},
	{BCRM_HUE_MAX_32R, 			ACCESS_MODE_R},
	{BCRM_HUE_INC_32R, 			ACCESS_MODE_R},

	{BCRM_ALL_BALANCE_RATIO_64RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_ALL_BALANCE_RATIO_MIN_64R, 	ACCESS_MODE_R},
	{BCRM_ALL_BALANCE_RATIO_MAX_64R, 	ACCESS_MODE_R},
	{BCRM_ALL_BALANCE_RATIO_INC_64R, 	ACCESS_MODE_R},

	{BCRM_RED_BALANCE_RATIO_64RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_RED_BALANCE_RATIO_MIN_64R, 	ACCESS_MODE_R},
	{BCRM_RED_BALANCE_RATIO_MAX_64R, 	ACCESS_MODE_R},
	{BCRM_RED_BALANCE_RATIO_INC_64R, 	ACCESS_MODE_R},

	{BCRM_GREEN_BALANCE_RATIO_64RW, 	ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_GREEN_BALANCE_RATIO_MIN_64R, 	ACCESS_MODE_R},
	{BCRM_GREEN_BALANCE_RATIO_MAX_64R, 	ACCESS_MODE_R},
	{BCRM_GREEN_BALANCE_RATIO_INC_64R, 	ACCESS_MODE_R},

	{BCRM_BLUE_BALANCE_RATIO_64RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_BLUE_BALANCE_RATIO_MIN_64R,	ACCESS_MODE_R},
	{BCRM_BLUE_BALANCE_RATIO_MAX_64R, 	ACCESS_MODE_R},
	{BCRM_BLUE_BALANCE_RATIO_INC_64R, 	ACCESS_MODE_R},

	{BCRM_WHITE_BALANCE_AUTO_8RW, 		ACCESS_MODE_R | ACCESS_MODE_W},

	/* Other Registers */
	{BCRM_SHARPNESS_32RW, 			ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_SHARPNESS_MIN_32R, 		ACCESS_MODE_R},
	{BCRM_SHARPNESS_MAX_32R, 		ACCESS_MODE_R},
	{BCRM_SHARPNESS_INC_32R, 		ACCESS_MODE_R},

	{BCRM_DEVICE_TEMPERATURE_32R,		ACCESS_MODE_R},
	{BCRM_ANALOG_GAIN_64RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_ANALOG_GAIN_MIN_64RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
	{BCRM_ANALOG_GAIN_MAX_64RW, 		ACCESS_MODE_R | ACCESS_MODE_W},
};


enum CCI_REG_INFO {
	CCI_REGISTER_LAYOUT_VERSION = 0,
	RESERVED4BIT,
	DEVICE_CAPABILITIES,
	GCPRM_ADDRESS,
	RESERVED2BIT,
	BCRM_ADDRESS,
	RESERVED2BIT_2,
	DEVICE_GUID,
	MANUFACTURER_NAME,
	MODEL_NAME,
	FAMILY_NAME,
	DEVICE_VERSION,
	MANUFACTURER_INFO,
	SERIAL_NUMBER,
	USER_DEFINED_NAME,
	CHECKSUM,
	CHANGE_MODE,
	CURRENT_MODE,
	SOFT_RESET,
	MAX_CMD = SOFT_RESET
};

typedef struct cci_cmd_t {
	__u8 command_index; /* diagnostc test name */
	const __u32 address; /* NULL for no alias name */
	__u32 byte_count;
} cci_cmd_t;

static cci_cmd_t cci_cmd_tbl[MAX_CMD] = {
	/* command index		address  */
	{ CCI_REGISTER_LAYOUT_VERSION,	0x0000, 4 },
	{ DEVICE_CAPABILITIES,		0x0008, 8 },
	{ GCPRM_ADDRESS,		0x0010, 2 },
	{ BCRM_ADDRESS,			0x0014, 2 },
	{ DEVICE_GUID,			0x0018, 64 },
	{ MANUFACTURER_NAME,		0x0058, 64 },
	{ MODEL_NAME,			0x0098, 64 },
	{ FAMILY_NAME,			0x00D8, 64 },
	{ DEVICE_VERSION,		0x0118, 64 },
	{ MANUFACTURER_INFO,		0x0158, 64 },
	{ SERIAL_NUMBER,		0x0198, 64 },
	{ USER_DEFINED_NAME,		0x01D8, 64 },
	{ CHECKSUM,			0x0218, 4 },
	{ CHANGE_MODE,			0x021C, 1 },
	{ CURRENT_MODE,			0x021D, 1 },
	{ SOFT_RESET,			0x021E, 1 },
};

struct   __attribute__((__packed__))  cci_reg_t {
	__u32   cci_register_layout_version;
	__u32   reserved_4bit;
	__u64   device_capabilities;
	__u16   gcprm_address;
	__u16   reserved_2bit;
	__u16   bcrm_address;
	__u16   reserved_2bit_2;
	char    device_guid[64];
	char    manufacturer_name[64];
	char    model_name[64];
	char    family_name[64];
	char    device_version[64];
	char    manufacturer_info[64];
	char    serial_number[64];
	char    user_defined_name[64];
	__u32   checksum;
	__u8    change_mode;
	__u8    current_mode;
	__u8    soft_reset;
} cci_reg;

struct   __attribute__((__packed__))  gencp_reg_t {
	__u32   gcprm_layout_version;
	__u16   gencp_out_buffer_address;
	__u16   reserved_2byte;
	__u16   gencp_out_buffer_size;
	__u16   reserved_2byte_1;
	__u16	gencp_in_buffer_address;
	__u16   reserved_2byte_2;
	__u16   gencp_in_buffer_size;
} gencp_reg;

typedef union {
	struct {
	unsigned long long reverse_x_avail:1;
	unsigned long long reverse_y_avail:1;
	unsigned long long intensity_auto_prcedence_avail:1;
	unsigned long long black_level_avail:1;
	unsigned long long gain_avail:1;
	unsigned long long gamma_avail:1;
	unsigned long long contrast_avail:1;
	unsigned long long saturation_avail:1;
	unsigned long long hue_avail:1;
	unsigned long long white_balance_avail:1;
	unsigned long long sharpness_avail:1;
	unsigned long long exposure_auto:1;
	unsigned long long gain_auto:1;
	unsigned long long white_balance_auto_avail:1;
	unsigned long long device_temperature_avail:1;
	unsigned long long acquisition_abort_avail:1;
	unsigned long long acquisition_framerate_avail:1;
	unsigned long long frame_trigger:1;
	unsigned long long reserved:48;
	} feature_inq;

	unsigned long long value;
} bcrm_feature_reg_t;

typedef union {
	struct {
	unsigned long long yuv420_8_leg_avail:1;
	unsigned long long yuv420_8_avail:1;
	unsigned long long yuv420_10_avail:1;
	unsigned long long yuv420_8_csps_avail:1;
	unsigned long long yuv420_10_csps_avail:1;
	unsigned long long yuv422_8_avail:1;
	unsigned long long yuv422_10_avail:1;
	unsigned long long rgb888_avail:1;
	unsigned long long rgb666_avail:1;
	unsigned long long rgb565_avail:1;
	unsigned long long rgb555_avail:1;
	unsigned long long rgb444_avail:1;
	unsigned long long raw6_avail:1;
	unsigned long long raw7_avail:1;
	unsigned long long raw8_avail:1;
	unsigned long long raw10_avail:1;
	unsigned long long raw12_avail:1;
	unsigned long long raw14_avail:1;
	unsigned long long jpeg_avail:1;
	unsigned long long reserved:45;
	} avail_mipi;

	unsigned long long value;
} bcrm_availMipi_reg_t;

typedef union {
	struct {
	unsigned char monochrome_avail:1;
	unsigned char bayer_GR_avail:1;
	unsigned char bayer_RG_avail:1;
	unsigned char bayer_GB_avail:1;
	unsigned char bayer_BG_avail:1;
	unsigned char reserved:3;
	} bayer_pattern;

	unsigned char value;
} bcrm_bayerInquiry_reg_t;

typedef union {
	struct {
	unsigned char one_lane_avail:1;
	unsigned char two_lane_avail:1;
	unsigned char three_lane_avail:1;
	unsigned char four_lane_avail:1;
	unsigned char reserved:4;
	} lane_count;

	unsigned char value;
} bcrm_supported_lanecount_reg_t;

typedef union {
	struct {
	unsigned char one_lane_avail:1;
	unsigned char two_lane_avail:1;
	unsigned char three_lane_avail:1;
	unsigned char four_lane_avail:1;
	unsigned char reserved:4;
	} lane_count;

	unsigned char value;
} imx_supported_lanecount_reg_t;

struct avt_fmtdesc {
	__u32		    index;             /* Format number      */
	__u32               flags;
	__u8		    description[32];   /* Description string */
	__u32		    pixelformat;       /* Format fourcc      */
};

// Definition for feature names and corresponding bit mask offsets for "Feature Inquiry" BCRM register
#define AV_ATTR_EXPOSURE		{"Exposure", 	       0} // dummy
#define AV_ATTR_EXPOSURE_ABS		{"Exposure Absolute",  0} // dummy
#define AV_ATTR_REVERSE_X		{"Reverse X", 		0}
#define AV_ATTR_REVERSE_Y		{"Reverse Y", 		1}
#define AV_ATTR_INTENSITY_AUTO		{"Intensity Auto",	2}
#define AV_ATTR_BRIGHTNESS		{"Brightness", 		3}
#define AV_ATTR_GAIN			{"Gain", 		4}
#define AV_ATTR_GAMMA			{"Gamma", 		5}
#define AV_ATTR_CONTRAST		{"Contrast", 		6}
#define AV_ATTR_SATURATION		{"Saturation", 		7}
#define AV_ATTR_HUE			{"Hue", 		8}
#define AV_ATTR_WHITEBALANCE		{"White Balance",	9}
/* Red & Blue balance features are enabled by default since it doesn't have option in BCRM FEATURE REGISTER */
#define AV_ATTR_RED_BALANCE		{"Red Balance",         9}
#define AV_ATTR_BLUE_BALANCE		{"Blue Balance",        9}
#define AV_ATTR_SHARPNESS		{"Sharpnesss", 	       10}
#define AV_ATTR_EXPOSURE_AUTO		{"Exposure Auto",      11}
#define AV_ATTR_AUTOGAIN		{"Auto Gain",          12}
#define AV_ATTR_WHITEBALANCE_AUTO	{"Auto White Balance", 13}

struct avt_ctrl_mapping_t {
	u8  	reg_size;
	u8  	data_size;
	u16	min_offset;
	u16 	max_offset;
	u16 	reg_offset;
	u16	step_offset;
	u32  	id;
	u32 	type;
	u32 	flags;
	u32     menu_count;
	int 	(*get_default_val) (struct avt_ctrl_mapping_t *avt_ctrl_mapping, s64 *def_val);
	struct {
		s8 	*name;
		u8  	feature_avail;  
	} attr;
};

static int avt_get_int32_64_def_val (struct avt_ctrl_mapping_t *avt_ctrl_mapping, s64 *def_val);
static int avt_get_menu_def_val (struct avt_ctrl_mapping_t *avt_ctrl_mapping, s64 *def_val);
#endif
