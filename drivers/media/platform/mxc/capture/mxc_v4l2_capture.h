/*
 * Copyright 2004-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup MXC_V4L2_CAPTURE MXC V4L2 Video Capture Driver
 */
/*!
 * @file mxc_v4l2_capture.h
 *
 * @brief mxc V4L2 capture device API  Header file
 *
 * It include all the defines for frame operations, also three structure defines
 * use case ops structure, common v4l2 driver structure and frame structure.
 *
 * @ingroup MXC_V4L2_CAPTURE
 */
#ifndef __MXC_V4L2_CAPTURE_H__
#define __MXC_V4L2_CAPTURE_H__

#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/mxc_v4l2.h>
#include <linux/completion.h>
#include <linux/dmaengine.h>
#include <linux/pxp_dma.h>
#include <linux/ipu-v3.h>
#include <linux/platform_data/dma-imx.h>
#include <linux/mipi_csi2.h>
#include "avt_imx6_csi2_local.h"
#include "avt_debug.h"
#include <media/v4l2-dev.h>
#include "v4l2-int-device.h"

#define __FILENAME_H__ 			"mcx_v4l2_capture.h"

#define MAX_NUM_FRAMES 			20000
#define MAX_NUM_MXC_SENSORS		10
#define ACQUISITION_STOP_TIMEOUT	3000

#define CLEAR(x)                        memset(&(x), 0, sizeof(x))

enum imx_v4l2_devtype {
	IMX5_V4L2,
	IMX6_V4L2,
};

/*!
 * v4l2 frame structure.
 */
struct mxc_v4l_frame {
	u32 paddress;
	void *vaddress;
	int count;
	int width;
	int height;

	struct v4l2_buffer buffer;
	unsigned long userp;
	unsigned long length;
	struct list_head queue;
	int index;
	union {
		int ipu_buf_num;
		int csi_buf_num;
	};
};

/* Only for old version.  Will go away soon. */
typedef struct {
	u8 clk_mode;
	u8 ext_vsync;
	u8 Vsync_pol;
	u8 Hsync_pol;
	u8 pixclk_pol;
	u8 data_pol;
	u8 data_width;
	u8 pack_tight;
	u8 force_eof;
	u8 data_en_pol;
	u16 width;
	u16 height;
	u32 pixel_fmt;
	u32 mclk;
	u16 active_width;
	u16 active_height;
} sensor_interface;

/* Sensor control function */
/* Only for old version.  Will go away soon. */
struct camera_sensor {
	void (*set_color) (int bright, int saturation, int red, int green,
			   int blue);
	void (*get_color) (int *bright, int *saturation, int *red, int *green,
			   int *blue);
	void (*set_ae_mode) (int ae_mode);
	void (*get_ae_mode) (int *ae_mode);
	sensor_interface *(*config) (int *frame_rate, int high_quality);
	sensor_interface *(*reset) (void);
	void (*get_std) (v4l2_std_id *std);
	void (*set_std) (v4l2_std_id std);
	unsigned int csi;
};

/*!
 * common v4l2 driver structure.
 */
typedef struct _cam_data {
	struct video_device *video_dev;
	int device_type;

	/* semaphore guard against SMP multithreading */
	struct semaphore busy_lock;

	int open_count;
	struct delayed_work power_down_work;
	int power_on;

	__u32 timeout;

	struct delayed_work flush_work;
	wait_queue_head_t flush_queue;
	bool queued;
	struct delayed_work streamoff_work;
	spinlock_t flush_int_lock;
	spinlock_t streamoff_int_lock;

	/* params lock for this camera */
	struct semaphore param_lock;

	/* Encoder */
	struct list_head ready_q;
	struct list_head done_q;
	struct list_head working_q;
	int ping_pong_csi;
	spinlock_t queue_int_lock;
	spinlock_t dqueue_int_lock;
	struct mxc_v4l_frame frame[MAX_NUM_FRAMES];
	struct mxc_v4l_frame dummy_frame;
	struct mxc_v4l_frame userptr[MAX_NUM_FRAMES];
	wait_queue_head_t enc_queue;
	int enc_counter;
	dma_addr_t rot_enc_bufs[2];
	void *rot_enc_bufs_vaddr[2];
	int rot_enc_buf_size[2];
	enum v4l2_buf_type type;

	/* still image capture */
	wait_queue_head_t still_queue;
	int still_counter;
	dma_addr_t still_buf[2];
	void *still_buf_vaddr;

	/* overlay */
	struct v4l2_window win;
	struct v4l2_framebuffer v4l2_fb;
	dma_addr_t vf_bufs[2];
	void *vf_bufs_vaddr[2];
	int vf_bufs_size[2];
	dma_addr_t rot_vf_bufs[2];
	void *rot_vf_bufs_vaddr[2];
	int rot_vf_buf_size[2];
	bool overlay_active;
	int output;
	struct fb_info *overlay_fb;
	int fb_origin_std;
	struct work_struct csi_work_struct;

	/* v4l2 format */
	struct v4l2_format v2f;
	struct v4l2_format input_fmt;	/* camera in */
	bool bswapenable;
	int rotation;	/* for IPUv1 and IPUv3, this means encoder rotation */
	int vf_rotation; /* viewfinder rotation only for IPUv1 and IPUv3 */
	struct v4l2_mxc_offset offset;

	/* V4l2 control bit */
	int bright;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;
	unsigned int gain;
	unsigned int gain_auto;
	unsigned int expos;
	unsigned int expos_auto;


	/* standard */
	struct v4l2_streamparm streamparm;
	struct v4l2_standard standard;
	bool standard_autodetect;

	/* crop */
	struct v4l2_rect crop_bounds;
	struct v4l2_rect crop_defrect;
	struct v4l2_rect crop_current;

	int (*enc_update_eba) (void *private, dma_addr_t eba);
	int (*enc_enable) (void *private);
	int (*enc_disable) (void *private);
	int (*enc_enable_csi) (void *private);
	int (*enc_disable_csi) (void *private);
	void (*enc_callback) (u32 mask, void *dev);
	int (*vf_start_adc) (void *private);
	int (*vf_stop_adc) (void *private);
	int (*vf_start_sdc) (void *private);
	int (*vf_stop_sdc) (void *private);
	int (*vf_enable_csi) (void *private);
	int (*vf_disable_csi) (void *private);
	int (*csi_start) (void *private);
	int (*csi_stop) (void *private);

	/* misc status flag */
	bool overlay_on;
	bool capture_on;
	bool ipu_enable_csi_called;
	bool mipi_pixelclk_enabled;
	struct ipu_chan *ipu_chan;
	struct ipu_chan *ipu_chan_rot;
	int overlay_pid;
	int capture_pid;
	bool low_power;
	wait_queue_head_t power_queue;
	unsigned int ipu_id;
	unsigned int csi;
	unsigned mipi_camera;
	int csi_in_use;
	u8 mclk_source;
	bool mclk_on[2];	/* two mclk sources at most now */
	int current_input;

	int local_buf_num;

	/* camera sensor interface */
	struct camera_sensor *cam_sensor;	/* old version */
	struct v4l2_int_device *all_sensors[MAX_NUM_MXC_SENSORS];
	struct v4l2_int_device *sensor;
	struct v4l2_int_device *self;
	int sensor_index;
	void *ipu;
	void *csi_soc;
	enum imx_v4l2_devtype devtype;

	/* v4l2 buf elements related to PxP DMA */
	struct completion pxp_tx_cmpl;
	struct pxp_channel *pxp_chan;
	struct pxp_config_data pxp_conf;
	struct dma_async_tx_descriptor *txd;
	dma_cookie_t cookie;
	struct scatterlist sg[2];
} cam_data;

struct sensor_data {
	const struct ov5642_platform_data *platform_data;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
#ifndef WANDBOARD_IMX6
	struct v4l2_sensor_dimension spix;
#endif
	struct v4l2_captureparm streamcap;

	struct v4l2_rect crop_bounds;
	struct v4l2_rect crop_defrect;
	struct v4l2_rect crop_current;

	bool on;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;
#ifndef WANDBOARD_IMX6
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,00,00)
	int mirror;		/* For ov5640.c */
	int vflip;		/* For ov5640.c */
#endif
#endif

	unsigned int gain;
	unsigned int gain_auto;
	unsigned int expos;
	unsigned int expos_auto;

	u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;
	int ipu_id;
	int csi;
#ifndef WANDBOARD_IMX6
	int last_reg;
	unsigned mipi_camera;
#endif
	unsigned virtual_channel;	/* Used with mipi */

	void (*io_init)(void);
};

enum cam_cbf_status_t {
	cam_cbf_not_initiated = 0,
	cam_cbf_inprogress,
	cam_cbf_done
};

enum flush_status_t {
	flush_not_initiated = 0,
	flush_inprogress,
	flush_done
};

enum streamoff_status_t {
	streamoff_not_initiated = 0,
	streamoff_inprogress,
	streamoff_done
};

struct frames_count_t {
	u32 complete_frames_count;
	u32 incomplete_frames_count;
};

struct frames_error_count_t {
	u32 crc_error_count;
	u32 frames_overrun_error_count;
};

enum flush_frames_status_t {
	flush_frames_not_initiated = 0,
	flush_frames_unused,
	flush_frames_incomplete,
	flush_frames_unused_done,
	flush_frames_incomplete_done
};

enum streamoff_frames_status_t {
	streamoff_frames_not_initiated = 0,
	streamoff_frames_incomplete,
	streamoff_frames_incomplete_done,
	streamoff_frames_complete,
	streamoff_frames_complete_done
};

void set_mclk_rate(uint32_t *p_mclk_freq, uint32_t csi);
#ifndef WANDBOARD_IMX6
void mxc_camera_common_lock(void);
void mxc_camera_common_unlock(void);

static inline int cam_ipu_enable_csi(cam_data *cam)
{
	int ret = ipu_enable_csi(cam->ipu, cam->csi);
	if (!ret)
		cam->ipu_enable_csi_called = 1;
	return ret;
}

static inline int cam_ipu_disable_csi(cam_data *cam)
{
	if (!cam->ipu_enable_csi_called)
		return 0;
	cam->ipu_enable_csi_called = 0;
	return ipu_disable_csi(cam->ipu, cam->csi);
}

static inline int cam_mipi_csi2_enable(cam_data *cam, struct mipi_fields *mf)
{
#ifdef CONFIG_MXC_MIPI_CSI2
	void *mipi_csi2_info;
	struct sensor_data *sensor;

	if (!cam->sensor)
		return 0;
	sensor = cam->sensor->priv;
	if (!sensor)
		return 0;
	if (!sensor->mipi_camera)
		return 0;
	mipi_csi2_info = mipi_csi2_get_info();

	if (!mipi_csi2_info) {
		AV_ERR_H("Fail to get mipi_csi2_info!");
		//printk(KERN_ERR "%s() in %s: Fail to get mipi_csi2_info!\n", 
		//	__func__, __FILE__);
/*		return -EPERM; */
		return 0;
	}
	if (mipi_csi2_get_status(mipi_csi2_info)) {
		mf->en = true;
		mf->vc = sensor->virtual_channel;
		AV_INFO_H("sensor->virtual_channel=%d", sensor->virtual_channel);
		//printk(KERN_INFO "sensor->virtual_channel=%d\n", 
		//	sensor->virtual_channel);
		mf->id = mipi_csi2_get_datatype(mipi_csi2_info);
		if (!mipi_csi2_pixelclk_enable(mipi_csi2_info))
			cam->mipi_pixelclk_enabled = 1;
		return 0;
	}
	mf->en = false;
	mf->vc = 0;
	mf->id = 0;
#endif
	return 0;
}

static inline int cam_mipi_csi2_disable(cam_data *cam)
{
#ifdef CONFIG_MXC_MIPI_CSI2
	void *mipi_csi2_info;

	if (!cam->mipi_pixelclk_enabled)
		return 0;
	cam->mipi_pixelclk_enabled = 0;

	mipi_csi2_info = mipi_csi2_get_info();

	if (!mipi_csi2_info) {
		AV_ERR_H("Failed to get mipi_csi2_info!");
		//printk(KERN_ERR "Failed to get mipi_csi2_info!\n");
/*		return -EPERM; */
		return 0;
	}
	if (mipi_csi2_get_status(mipi_csi2_info))
		mipi_csi2_pixelclk_disable(mipi_csi2_info);
#endif
	return 0;
}
#endif // WANDBOARD_IMX6

#endif				/* __MXC_V4L2_CAPTURE_H__ */
