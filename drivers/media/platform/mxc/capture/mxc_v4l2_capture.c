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
 * @file drivers/media/video/mxc/capture/mxc_v4l2_capture.c
 *
 * @brief Mxc Video For Linux 2 driver
 *
 * @ingroup MXC_V4L2_CAPTURE
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/pagemap.h>
#include <linux/vmalloc.h>
#include <linux/types.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/mutex.h>
#include <linux/mxcfb.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>
#include "v4l2-int-device.h"
#include <linux/fsl_devices.h>
#include "mxc_v4l2_capture.h"
#include "ipu_prp_sw.h"
#include <linux/mipi_csi2.h>
#include <linux/pid.h>
#include <linux/sched.h>
#include <linux/fdtable.h>
#include <linux/rcupdate.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <media/videobuf2-vmalloc.h>
#include "avt_imx6_csi2_local.h"
#include "avt_imx6_csi2.h"
#include "avt_debug.h"
//#include <linux/mipi_csi2.h>

#ifdef WANDBOARD_IMX6
#pragma message("WANDBOARD_IMX6")
#endif

#define __FILENAME__ "mxc_v4l2_capture.c"

static int debug;
MODULE_PARM_DESC(debug, "Debug");
module_param(debug, int, 0600);/* S_IRUGO */

static int wait_for_camera_acqstop(cam_data *cam, unsigned long ms_timeout);
static int camera_acquisition_start(cam_data *cam);
static int camera_acquisition_stop(cam_data *cam);

static char av_cam_result[20];
static int av_cam_count[MAX_NUM_MXC_SENSORS];
static int i2c_address, i2c_busno;
static bool av_cam_open;
struct frames_count_t frames_count;
struct frames_error_count_t error_count; /* CRC error/ Buffer underrun reg read */
static enum flush_frames_status_t g_flush_frames_status = flush_frames_not_initiated;
static enum flush_status_t g_flush_status = flush_done;
static enum streamoff_status_t g_streamoff_status = streamoff_done;
static enum streamoff_frames_status_t g_streamoff_frames_status = streamoff_frames_not_initiated;
static enum cam_cbf_status_t g_cam_cbf_status = cam_cbf_not_initiated;
static unsigned int num_requested_buffers;
static __u32 g_bcrm_pixelformat;
static __u32 g_bcrm_width;
static __u32 g_bcrm_height;
static unsigned int video_val;
static int count_mem_allo;
static int count_mem_clear;
static struct timer_list timer;
static unsigned int count_init;
static unsigned int frame_count;
static unsigned int csi_fps;
static bool is_av_camera;
static bool irq_triggered;
static unsigned int first_frame = 1;
static unsigned int fps_count = 1;
static unsigned int fps_frame_num = 0;
struct timeval tstart;
s64 first_frame_time;
s64 second_frame_time;
s64 diff_time = 0;

//extern struct mipi_csi2_reg_data;

static __u64 cc_count;
static __u64 dq_count;

__u16 read_gencp_out_buffer_size;
EXPORT_SYMBOL(read_gencp_out_buffer_size);
__u16 read_gencp_in_buffer_size;
EXPORT_SYMBOL(read_gencp_in_buffer_size);
int av_cam_i2c_clock_frequency;
EXPORT_SYMBOL(av_cam_i2c_clock_frequency);
int lane_count;
EXPORT_SYMBOL(lane_count);
bool is_gencp_mode = false;/* BCRM mode in default */
EXPORT_SYMBOL(is_gencp_mode);

int access_user_addr(unsigned long addr, void *buf, int len, int write);
static long mxc_v4l_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

#define init_MUTEX(sem)	sema_init(sem, 1)
DEFINE_MUTEX(ioctl_mutex);

static struct platform_device_id imx_v4l2_devtype[] = {
	{
		.name = "v4l2-capture-imx5",
		.driver_data = IMX5_V4L2,
	}, {
		.name = "v4l2-capture-imx6",
		.driver_data = IMX6_V4L2,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, imx_v4l2_devtype);

static const struct of_device_id mxc_v4l2_dt_ids[] = {
	{
		.compatible = "fsl,imx6q-v4l2-capture",
		.data = &imx_v4l2_devtype[IMX6_V4L2],
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, mxc_v4l2_dt_ids);

static int video_nr = -1;

/*! This data is used for the output to the display. */
#define MXC_V4L2_CAPTURE_NUM_OUTPUTS	6
#define MXC_V4L2_CAPTURE_NUM_INPUTS	2
static struct v4l2_output mxc_capture_outputs[MXC_V4L2_CAPTURE_NUM_OUTPUTS] = {
	{
	 .index = 0,
	 .name = "DISP3 BG",
	 .type = V4L2_OUTPUT_TYPE_ANALOG,
	 .audioset = 0,
	 .modulator = 0,
	 .std = V4L2_STD_UNKNOWN,
	 },
	{
	 .index = 1,
	 .name = "DISP3 BG - DI1",
	 .type = V4L2_OUTPUT_TYPE_ANALOG,
	 .audioset = 0,
	 .modulator = 0,
	 .std = V4L2_STD_UNKNOWN,
	 },
	{
	 .index = 2,
	 .name = "DISP3 FG",
	 .type = V4L2_OUTPUT_TYPE_ANALOG,
	 .audioset = 0,
	 .modulator = 0,
	 .std = V4L2_STD_UNKNOWN,
	 },
	{
	 .index = 3,
	 .name = "DISP4 BG",
	 .type = V4L2_OUTPUT_TYPE_ANALOG,
	 .audioset = 0,
	 .modulator = 0,
	 .std = V4L2_STD_UNKNOWN,
	 },
	{
	 .index = 4,
	 .name = "DISP4 BG - DI1",
	 .type = V4L2_OUTPUT_TYPE_ANALOG,
	 .audioset = 0,
	 .modulator = 0,
	 .std = V4L2_STD_UNKNOWN,
	 },
	{
	 .index = 5,
	 .name = "DISP4 FG",
	 .type = V4L2_OUTPUT_TYPE_ANALOG,
	 .audioset = 0,
	 .modulator = 0,
	 .std = V4L2_STD_UNKNOWN,
	 },
};

static struct v4l2_input mxc_capture_inputs[MXC_V4L2_CAPTURE_NUM_INPUTS] = {
	{
	 .index = 0,
	 .name = "CSI IC MEM",
	 .type = V4L2_INPUT_TYPE_CAMERA,
	 .audioset = 0,
	 .tuner = 0,
	 .std = V4L2_STD_UNKNOWN,
	 .status = 0,
	 },
	{
	 .index = 1,
	 .name = "CSI MEM",
	 .type = V4L2_INPUT_TYPE_CAMERA,
	 .audioset = 0,
	 .tuner = 0,
	 .std = V4L2_STD_UNKNOWN,
	 .status = V4L2_IN_ST_NO_POWER,
	 },
};

/*! List of TV input video formats supported. The video formats is corresponding
 * to the v4l2_id in video_fmt_t.
 * Currently, only PAL and NTSC is supported. Needs to be expanded in the
 * future.
 */
typedef enum {
	TV_NTSC = 0,		/*!< Locked on (M) NTSC video signal. */
	TV_PAL,			/*!< (B, G, H, I, N)PAL video signal. */
	TV_NOT_LOCKED,		/*!< Not locked on a signal. */
} video_fmt_idx;

/*! Number of video standards supported (including 'not locked' signal). */
#define TV_STD_MAX		(TV_NOT_LOCKED + 1)

/*! Video format structure. */
typedef struct {
	int v4l2_id;		/*!< Video for linux ID. */
	char name[16];		/*!< Name (e.g., "NTSC", "PAL", etc.) */
	u16 raw_width;		/*!< Raw width. */
	u16 raw_height;		/*!< Raw height. */
	u16 active_width;	/*!< Active width. */
	u16 active_height;	/*!< Active height. */
	u16 active_top;		/*!< Active top. */
	u16 active_left;	/*!< Active left. */
} video_fmt_t;

/*!
 * Description of video formats supported.
 *
 *  PAL: raw=720x625, active=720x576.
 *  NTSC: raw=720x525, active=720x480.
 */
static video_fmt_t video_fmts[] = {
	{			/*! NTSC */
	 .v4l2_id = V4L2_STD_NTSC,
	 .name = "NTSC",
	 .raw_width = 720,		/* SENS_FRM_WIDTH */
	 .raw_height = 525,		/* SENS_FRM_HEIGHT */
	 .active_width = 720,		/* ACT_FRM_WIDTH */
	 .active_height = 480,		/* ACT_FRM_HEIGHT */
#ifdef WANDBOARD_IMX6
	 .active_top = 13,
#else
	 .active_top = 0,
#endif

	 .active_left = 0,
	 },
	{			/*! (B, G, H, I, N) PAL */
	 .v4l2_id = V4L2_STD_PAL,
	 .name = "PAL",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 .active_top = 0,
	 .active_left = 0,
	 },
	{			/*! Unlocked standard */
	 .v4l2_id = V4L2_STD_ALL,
	 .name = "Autodetect",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 .active_top = 0,
	 .active_left = 0,
	 },
};

/*!* Standard index of TV. */
static video_fmt_idx video_index = TV_NOT_LOCKED;

static int mxc_v4l2_master_attach(struct v4l2_int_device *slave);
static void mxc_v4l2_master_detach(struct v4l2_int_device *slave);
static int start_preview(cam_data *cam);
static int stop_preview(cam_data *cam);

/*! Information about this driver. */
static struct v4l2_int_master mxc_v4l2_master = {
	.attach = mxc_v4l2_master_attach,
	.detach = mxc_v4l2_master_detach,
};

/***************************************************************************
 * Functions for handling Frame buffers.
 **************************************************************************/

/*!
 * Free frame buffers
 *
 * @param cam      Structure cam_data *
 *
 * @return status  0 success.
 */
static int mxc_free_frame_buf(cam_data *cam)
{
	int i;

	AV_DEBUG("BCRM mode count_mem_allo=%d", count_mem_allo);

	for (i = 0; i < MAX_NUM_FRAMES; i++) {
		//AV_DEBUG(" mxc_free_frame_buf, FRAME_NO, i %d, cam->frame[%d].paddress %u", i , i, cam->frame[i].paddress);
		if (cam->frame[i].vaddress != 0) {
			dma_free_coherent(0, cam->frame[i].buffer.length,
					  cam->frame[i].vaddress,
					  cam->frame[i].paddress);
			cam->frame[i].vaddress = 0;
		}
	}

	return 0;
}

/*!
 * Free the frame buffers for gencp app
 *
 * @param cam      Structure cam_data *
 *
 * @return status  0 success.
 */
static int mxc_free_frame_buf_gencp(cam_data *cam)
{
	int i;

	AV_DEBUG("GENCP mode count_mem_allo=%d", count_mem_allo);

	for (i = 0; i < count_mem_allo; i++) {
		//AV_DEBUG(" mxc_free_frame_buf_gencp, FRAME_NO, i %d, cam->frame[%d].paddress %u", i, i, cam->frame[i].paddress);
		if (cam->frame[i].vaddress != 0) {
			dma_free_coherent(0, cam->frame[i].buffer.length,
					  cam->frame[i].vaddress,
					  cam->frame[i].paddress);
			cam->frame[i].vaddress = 0;
		}
	}

	return 0;
}

static void streamoff_work_handler(struct work_struct *w)
{
	struct mxc_v4l_frame *dummy_frame_buf, *dummy_frame_buf2;
	cam_data *cam = container_of(w, struct _cam_data, streamoff_work.work);
	static int incomplete_count;
	struct device videodev = cam->video_dev->dev;
	uint32_t irq = (cam->csi == 0) ?
	IPU_IRQ_CSI0_OUT_EOF : IPU_IRQ_CSI1_OUT_EOF;
	int err = 0;

	ipu_free_irq(cam->ipu, irq, cam);/* CSI IRQ0 */

	AV_DEBUG(" cam->ipu_id=%d", cam->ipu_id);
	AV_DEBUG(" cam->csi=%d", cam->csi);
	AV_DEBUG(" cam->capture_on=%d", cam->capture_on);
	AV_DEBUG(" mxc_capture_inputs[cam->current_input].name=%s", mxc_capture_inputs[cam->current_input].name);
/*
	if (cam->capture_on == false)
		return;
*/

	/* For both CSI--MEM and CSI--IC--MEM
	 * 1. wait for idmac eof
	 * 2. disable csi first
	 * 3. disable idmac
	 * 4. disable smfc (CSI--MEM channel)
	 */
	if (mxc_capture_inputs[cam->current_input].name != NULL) {
		if (cam->enc_disable_csi) {
			err = cam->enc_disable_csi(cam);
			if (err != 0)
				return;
		}
		if (cam->enc_disable) {
			err = cam->enc_disable(cam);
			if (err != 0)
				return;
		}
	}

	mxc_capture_inputs[cam->current_input].status |= V4L2_IN_ST_NO_POWER;
	cam->capture_on = false;
	AV_DEBUG("Cleared IPU interrupt and disabled CSI.");
	if (g_streamoff_status == streamoff_inprogress) {
		AV_DEBUG("STREAMOFF is already in progress...");
	}

	if (!list_empty(&cam->working_q)) {
		if (irq_triggered){
			AV_DEBUG("Waiting timeout for %d", cam->timeout);
			msleep(cam->timeout);
			irq_triggered = false;
		}
		AV_DEBUG("WORKING queue is not empty so there are partial filled frames.");
		incomplete_count++;
		list_for_each_entry_safe(dummy_frame_buf, dummy_frame_buf2, &cam->working_q, queue) {
		dummy_frame_buf->buffer.flags |= V4L2_BUF_FLAG_INCOMPLETE;
		AV_DEBUG("list_for_each_entry, dummy_frame_buf->paddress=0x%0x, dummy_frame_buf->buffer.m.offset=0x%x, dummy_frame_buf->buffer.index=0x%x, dummy_frame_buf->buffer.flags=0x%x",
			dummy_frame_buf->paddress, dummy_frame_buf->buffer.m.offset, dummy_frame_buf->buffer.index, dummy_frame_buf->buffer.flags);

		g_streamoff_frames_status = streamoff_frames_incomplete;
			if (list_empty(&cam->working_q)) {
				AV_DEBUG("incomplete frame processing done!");
				goto exit;
			}
		}
	}

exit:
	g_streamoff_frames_status = streamoff_frames_incomplete_done;
	g_streamoff_status = streamoff_done;
	AV_DEBUG("list_for_each_entry is done for cam->working_q and streamoff_done is set. Total num of incomplete frames: %d ",  incomplete_count);
	INIT_LIST_HEAD(&cam->working_q);/* Make sure that cleared the QUEUE */

	AV_DEBUG("sysfs_notify for 'streamoff'");
	sysfs_notify(&videodev.kobj, NULL, "streamoff");
}

static void flush_work_handler(struct work_struct *w)
{
	struct mxc_v4l_frame *dummy_frame_buf, *dummy_frame_buf2, *dummy_frame_buf3;
	cam_data *cam = container_of(w, struct _cam_data, flush_work.work);
	int err;
	static int incomplete_count;
	static int unused_count;
	struct device videodev = cam->video_dev->dev;

	if (list_empty(&cam->working_q) && list_empty(&cam->ready_q)) {
		AV_DEBUG("WORKING queue is empty -> No partial filled or incomplete frames");
		goto exit;
	}

	if (!list_empty(&cam->working_q)) {
		AV_DEBUG("WORKING queue is not empty -> Partial filled frames exist");
		incomplete_count++;
		list_for_each_entry_safe(dummy_frame_buf, dummy_frame_buf2, &cam->working_q, queue) {
		dummy_frame_buf->buffer.flags |= V4L2_BUF_FLAG_INCOMPLETE;
		AV_DEBUG("list_for_each_entry, dummy_frame_buf->paddress=0x%0x, dummy_frame_buf->buffer.m.offset=0x%x, dummy_frame_buf->buffer.index=0x%x, dummy_frame_buf->buffer.flags=0x%x", 
			dummy_frame_buf->paddress, dummy_frame_buf->buffer.m.offset, dummy_frame_buf->buffer.index, dummy_frame_buf->buffer.flags);
		cam->enc_counter = 1;
		g_flush_frames_status = flush_frames_incomplete;
			if (list_empty(&cam->working_q)) {
				AV_DEBUG("incomplete frame processing done!");
				goto exit;
			}
		}
	}

		g_flush_frames_status = flush_frames_incomplete_done;

	if (!list_empty(&cam->ready_q)) {
		AV_DEBUG("READY queue is not empty -> Unused frames exist");
		unused_count++;
		list_for_each_entry_safe(dummy_frame_buf, dummy_frame_buf3, &cam->ready_q, queue) {
		dummy_frame_buf->buffer.flags |= V4L2_BUF_FLAG_UNUSED;
		AV_DEBUG("list_for_each_entry, dummy_frame_buf->paddress=0x%x, dummy_frame_buf->buffer.m.offset=0x%x, dummy_frame_buf->buffer.index=0x%x, dummy_frame_buf->buffer.flags=0x%x", 
			dummy_frame_buf->paddress, dummy_frame_buf->buffer.m.offset, dummy_frame_buf->buffer.index, dummy_frame_buf->buffer.flags);
		cam->enc_counter = 1;

		g_flush_frames_status = flush_frames_unused;
			if (list_empty(&cam->ready_q)) {
				AV_DEBUG("unused frame processing done!");
				goto exit;
			}
		}
	}

exit:
	g_flush_frames_status = flush_frames_incomplete_done;/* need to set for exit case. */
	g_flush_frames_status = flush_frames_unused_done;

	AV_DEBUG("list_for_each_entry is done forcam->working_q & cam->ready_q. Total num of incomplete frames: %d, Total num of unused frames: %d", incomplete_count, unused_count);

	g_flush_status = flush_done;

	INIT_LIST_HEAD(&cam->working_q);
	INIT_LIST_HEAD(&cam->ready_q);

	if (cam->enc_enable_csi) {
		AV_DEBUG("cam->enc_enable_csi called again after flush.");
		err = cam->enc_enable_csi(cam);
		if (err != 0) {
			AV_ERR("cam->enc_enable_csi called again after flush.");
			return;
		}
	}

	AV_DEBUG("sysfs_notify from %s for 'flush'", __func__);
	sysfs_notify(&videodev.kobj, NULL, "flush");
}


static int mxc_v4l2_send_command(cam_data *cam,	struct v4l2_send_command_control *c)
{
	int ret = 0;
    
	if (vidioc_int_send_command(cam->sensor, c)) {
		ret = -EINVAL;
	}
	return ret;
}


static int v4l2_dma_alloc_mem(cam_data *cam, struct v4l2_dma_mem *dma_mem)
{
	if (count_mem_clear == 0) {
		count_mem_clear++;
		AV_DEBUG("vidioc_int_dma_alloc_mem: Freeing frames for only one time count_mem_clear %d", count_mem_clear);
		mxc_free_frame_buf_gencp(cam);
	}

 	AV_DEBUG("cam->v2f.fmt.pix.sizeimage=%d", cam->v2f.fmt.pix.sizeimage); 

	cam->frame[dma_mem->index].vaddress =
	    dma_alloc_coherent(0,
			       PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage),
			       &cam->frame[dma_mem->index].paddress,
			       GFP_DMA | GFP_KERNEL);


	if (cam->frame[dma_mem->index].vaddress == 0) {
		AV_ERR("failed, memory is not sufficient");
		mxc_free_frame_buf_gencp(cam);
		return -ENOBUFS;
	}

	cam->frame[dma_mem->index].buffer.index = dma_mem->index;
	cam->frame[dma_mem->index].buffer.flags = V4L2_BUF_FLAG_MAPPED;
	cam->frame[dma_mem->index].buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cam->frame[dma_mem->index].buffer.length = PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage);
	cam->frame[dma_mem->index].buffer.memory = V4L2_MEMORY_MMAP;
	cam->frame[dma_mem->index].buffer.m.offset = cam->frame[dma_mem->index].paddress;
	cam->frame[dma_mem->index].index = dma_mem->index;

/* 	AV_DEBUG("VIDIOC_MEM_ALLOC count_mem_allo is incremented to %d , cam->frame[%d].paddress %u  dma_mem->index  %d  ",count_mem_allo,count_mem_allo,cam->frame[dma_mem->index].paddress,dma_mem->index); */

	count_mem_allo++;

	return 0;
}

static int v4l2_dma_alloc_mem_free(cam_data *cam, struct v4l2_dma_mem *dma_mem)
{
	if (dma_mem->index < 0) {
		AV_ERR("VIDIOC_MEM_FREE: Please do make sure the frame no which you want to free! ");
		return -EINVAL;
	}

		if (cam->frame[dma_mem->index].vaddress != 0) {
/* 		AV_DEBUG("VIDIOC_MEM_FREE: FRAME_NO %d freed, cam->frame[%d].paddress %u", dma_mem->index, dma_mem->index, cam->frame[dma_mem->index].paddress); */

			dma_free_coherent(0, cam->frame[dma_mem->index].buffer.length,
					  cam->frame[dma_mem->index].vaddress,
					  cam->frame[dma_mem->index].paddress);
			cam->frame[dma_mem->index].vaddress = 0;
		}
	return 0;
}

/*!
 * Allocate frame buffers
 *
 * @param cam      Structure cam_data*
 * @param count    int number of buffer need to allocated
 *
 * @return status  -0 Successfully allocated a buffer, -ENOBUFS	failed.
 */
static int mxc_allocate_frame_buf(cam_data *cam, int count)
{
	int i;

	AV_DEBUG("cam->v2f.fmt.pix.sizeimage=%d", cam->v2f.fmt.pix.sizeimage);

	for (i = 0; i < count; i++) {
		cam->frame[i].vaddress =
		    dma_alloc_coherent(0,
				       PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage),
				       &cam->frame[i].paddress,
				       GFP_DMA | GFP_KERNEL);
		if (cam->frame[i].vaddress == 0) {
			AV_ERR("allocate frame buffer failed.");
			mxc_free_frame_buf(cam);
			return -ENOBUFS;
		}

		cam->frame[i].buffer.index = i;
		cam->frame[i].buffer.flags = V4L2_BUF_FLAG_MAPPED;
		cam->frame[i].buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cam->frame[i].buffer.length =
		    PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage);
		cam->frame[i].buffer.memory = V4L2_MEMORY_MMAP;
		cam->frame[i].buffer.m.offset = cam->frame[i].paddress;
		cam->frame[i].index = i;

		AV_DEBUG("VIDIOC_REQBUFS count is incremented to %d, cam->frame[%d].paddress=0x%x", i, i, cam->frame[i].paddress);
	}
	return 0;
}

/*!
 * Allocate frame buffers for USERPTR
 *
 * @param cam      Structure cam_data*
 * @param count    int number of buffer need to allocated
 *
 * @return status  -0 Successfully allocated a buffer, -ENOBUFS	failed.
 */
static int v4l2_dma_alloc_mem_userp(cam_data *cam, struct v4l2_dma_mem *dma_mem)
{
	if (count_mem_clear == 0) {
		count_mem_clear++;
		AV_DEBUG("vidioc_int_dma_alloc_mem_userp: Freeing frames for only one time count_mem_clear %d", count_mem_clear);
		mxc_free_frame_buf_gencp(cam);
	}

	AV_DEBUG("cam->v2f.fmt.pix.sizeimage=%d", cam->v2f.fmt.pix.sizeimage);

	cam->frame[dma_mem->index].vaddress =
	    dma_alloc_coherent(0,
			       PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage),
			       &cam->frame[dma_mem->index].paddress,
			       GFP_DMA | GFP_KERNEL);


	if (cam->frame[dma_mem->index].vaddress == 0) {
		AV_ERR("allocate dma memory failed.");
		mxc_free_frame_buf_gencp(cam);
		return -ENOBUFS;
	}

	cam->frame[dma_mem->index].buffer.index = dma_mem->index;
	cam->frame[dma_mem->index].buffer.flags = V4L2_BUF_FLAG_MAPPED;
	cam->frame[dma_mem->index].buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cam->frame[dma_mem->index].buffer.length = PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage);
	cam->frame[dma_mem->index].buffer.memory = V4L2_MEMORY_USERPTR;
	cam->frame[dma_mem->index].buffer.m.offset = cam->frame[dma_mem->index].paddress;
	cam->frame[dma_mem->index].index = dma_mem->index;

	AV_DEBUG("VIDIOC_MEM_ALLOC  USERPTR count_mem_allo is incremented to %d, cam->frame[%d].paddress=0x%x, dma_mem->index=%d",
		count_mem_allo, count_mem_allo, cam->frame[dma_mem->index].paddress, dma_mem->index);

	count_mem_allo++;

	return 0;

}


/*!
 * Allocate frame buffers for USERPTR
 *
 * @param cam      Structure cam_data*
 * @param count    int number of buffer need to allocated
 *
 * @return status  -0 Successfully allocated a buffer, -ENOBUFS	failed.
 */
static int mxc_allocate_frame_buf_userp(cam_data *cam, int count)
{
	int i;

	AV_DEBUG("cam->v2f.fmt.pix.sizeimage=%d", cam->v2f.fmt.pix.sizeimage);

	for (i = 0; i < count; i++) {
		cam->frame[i].vaddress =
		    dma_alloc_coherent(0,
				       PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage),
				       &cam->frame[i].paddress,
				       GFP_DMA | GFP_KERNEL);
		if (cam->frame[i].vaddress == 0) {
			AV_ERR("failed");
			mxc_free_frame_buf(cam);
			return -ENOBUFS;
		}

		cam->frame[i].buffer.index = i;
		cam->frame[i].buffer.flags = V4L2_BUF_FLAG_MAPPED;
		cam->frame[i].buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cam->frame[i].buffer.length =
		    PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage);
		cam->frame[i].buffer.memory = V4L2_MEMORY_USERPTR;
		cam->frame[i].buffer.m.offset = cam->frame[i].paddress;
		cam->frame[i].index = i;

		AV_DEBUG("VIDIOC_REQBUFS count is incremented to %d ,cam->frame[%d].paddress=0x%x FOR USERPTR", i, i, cam->frame[i].paddress);

	}
	return 0;
}

/*!
 * Free frame buffers status
 *
 * @param cam    Structure cam_data *
 *
 * @return none
 */
static void mxc_free_frames(cam_data *cam)
{
	int i;

	if (!is_gencp_mode) {
		AV_DEBUG("BCRM mode");
		for (i = 0; i < MAX_NUM_FRAMES; i++)
			cam->frame[i].buffer.flags = 0;
	} else {
		AV_DEBUG("GENCP mode");
		for (i = 0; i < count_mem_allo; i++)
			cam->frame[i].buffer.flags = 0;
	}

	cam->enc_counter = 0;
	INIT_LIST_HEAD(&cam->ready_q);
	INIT_LIST_HEAD(&cam->working_q);
	INIT_LIST_HEAD(&cam->done_q);
}

/*!
 * Return the buffer status
 *
 * @param cam	   Structure cam_data *
 * @param buf	   Structure v4l2_buffer *
 *
 * @return status  0 success, EINVAL failed.
 */
static int mxc_v4l2_buffer_status(cam_data *cam, struct v4l2_buffer *buf)
{
	if (buf->index < 0 || buf->index >= MAX_NUM_FRAMES) {
		AV_ERR("v4l2 capture: mxc_v4l2_buffer_status buffers not allocated");
		return -EINVAL;
	}

	memcpy(buf, &(cam->frame[buf->index].buffer), sizeof(*buf));
	return 0;
}

/*!
 * Return the buffer status
 *
 * @param cam	   Structure cam_data *
 * @param buf	   Structure v4l2_buffer *
 *
 * @return status  0 success, EINVAL failed.
 */
static int mxc_v4l2_buffer_status_gencp(cam_data *cam, struct v4l2_buffer *buf)
{
	if (buf->index < 0) {
		AV_ERR("v4l2 capture: mxc_v4l2_buffer_status buffers not allocated");
		return -EINVAL;
	}

	memcpy(buf, &(cam->frame[buf->index].buffer), sizeof(*buf));
	return 0;
}


/***************************************************************************
 * Functions for handling the video stream.
 **************************************************************************/

/*!
 * Indicates whether the palette is supported.
 *
 * @param palette V4L2_PIX_FMT_RGB565, V4L2_PIX_FMT_BGR24 or V4L2_PIX_FMT_BGR32
 *
 * @return 0 if failed
 */
static inline int valid_mode(u32 palette)
{
	return (palette == V4L2_PIX_FMT_RGB565) ||
		(palette == V4L2_PIX_FMT_BGR24) ||
		(palette == V4L2_PIX_FMT_RGB24) ||

		(palette == V4L2_PIX_FMT_CUSTOM) ||

		(palette == V4L2_PIX_FMT_GREY)	 ||
		(palette == V4L2_PIX_FMT_SBGGR8) ||
		(palette == V4L2_PIX_FMT_SGBRG8) ||
		(palette == V4L2_PIX_FMT_SGRBG8) ||
		(palette == V4L2_PIX_FMT_SRGGB8) ||

		(palette == V4L2_PIX_FMT_Y10P) ||
		(palette == V4L2_PIX_FMT_SBGGR10P) ||
		(palette == V4L2_PIX_FMT_SGBRG10P) ||
		(palette == V4L2_PIX_FMT_SGRBG10P) ||
		(palette == V4L2_PIX_FMT_SRGGB10P) ||

		(palette == V4L2_PIX_FMT_Y12P) ||
		(palette == V4L2_PIX_FMT_SBGGR12P) ||
		(palette == V4L2_PIX_FMT_SGBRG12P) ||
		(palette == V4L2_PIX_FMT_SGRBG12P) ||
		(palette == V4L2_PIX_FMT_SRGGB12P) ||
		(palette == V4L2_PIX_FMT_GREY12P)  ||


		(palette == V4L2_PIX_FMT_RGB666) ||
		(palette == V4L2_PIX_FMT_RGB555) ||
		(palette == V4L2_PIX_FMT_RGB444) ||

		(palette == V4L2_PIX_FMT_BGR32) ||
		(palette == V4L2_PIX_FMT_RGB32) ||
		(palette == V4L2_PIX_FMT_YUV422P) ||
		(palette == V4L2_PIX_FMT_UYVY) ||
		(palette == V4L2_PIX_FMT_YUYV) ||
		(palette == V4L2_PIX_FMT_YUV420) ||
		(palette == V4L2_PIX_FMT_YVU420) ||
		(palette == V4L2_PIX_FMT_NV12);
}

/*!
 * Start the encoder job
 *
 * @param cam      structure cam_data *
 *
 * @return status  0 Success
 */
static int mxc_streamon_ex(cam_data *cam)
{
	struct mxc_v4l_frame *frame;
	unsigned long lock_flags;
	int err = 0;

	if (NULL == cam) {
		AV_ERR("cam == NULL");
		return -1;
	}

	if (cam->capture_on) {
		AV_ERR("v4l2 capture: Capture stream has been turned on");
		return -1;
	}

	cam->queued = true;

	if (list_empty(&cam->ready_q)) {
		/* handle STREAMON called before QUEUE in genCP */
		if (!is_gencp_mode) {
			AV_ERR("v4l2 capture: mxc_streamon buffer has not been queued yet");
			err = -EINVAL;
			cam->queued = false;
			goto exit;
		} else {
			cam->queued = false;
			cam->capture_on = true;
			return 0;
		}
	}

	if (cam->enc_update_eba && cam->ready_q.prev == cam->ready_q.next) {
		/* handle STREAMON called before QUEUE in genCP*/
		if (is_gencp_mode)
			return 0;

		AV_ERR("v4l2 capture: mxc_streamon buffer need ping pong at least two buffers");
		return -EINVAL;
	}

	cam->capture_pid = current->pid;

	if (cam->overlay_on == true)
		stop_preview(cam);

	if (cam->enc_enable) {
		err = cam->enc_enable(cam);
		if (err != 0)
			return err;
	}

	spin_lock_irqsave(&cam->queue_int_lock, lock_flags);
	cam->ping_pong_csi = 0;
	cam->local_buf_num = 0;
	if (cam->enc_update_eba) {
		frame =	list_entry(cam->ready_q.next, struct mxc_v4l_frame, queue);
		list_del(cam->ready_q.next);
		list_add_tail(&frame->queue, &cam->working_q);
		frame->ipu_buf_num = cam->ping_pong_csi;
		err = cam->enc_update_eba(cam, frame->buffer.m.offset);

		frame =	list_entry(cam->ready_q.next, struct mxc_v4l_frame, queue);
		list_del(cam->ready_q.next);
		list_add_tail(&frame->queue, &cam->working_q);
		frame->ipu_buf_num = cam->ping_pong_csi;
		err |= cam->enc_update_eba(cam, frame->buffer.m.offset);
		spin_unlock_irqrestore(&cam->queue_int_lock, lock_flags);
	} else {
		spin_unlock_irqrestore(&cam->queue_int_lock, lock_flags);
		return -EINVAL;
	}

	if (cam->overlay_on == true)
		start_preview(cam);

	if (cam->enc_enable_csi) {
		err = cam->enc_enable_csi(cam);
		if (err != 0)
			return err;
	}

	cam->capture_on = true;

exit:
	return err;
}


static int camera_acquisition_start(cam_data *cam)
{
	struct v4l2_send_command_control ctrl;
	int retval = 0;

	if (NULL == cam) 
	{
		AV_ERR("cam == NULL");
		return -EINVAL;
	}

	if (is_av_camera)
	{
		/* Read the acquisition status of Alliedvision camera */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_ACQ_STATUS_R;
		retval = mxc_v4l2_send_command(cam, &ctrl);

		if (retval < 0)
		{
			AV_ERR("ERROR while reading acquisition status register");
			return -EINVAL;
		}
			
		AV_DEBUG("Current camera acquisition status=%d", ctrl.value0);
		/* Start the acquisition if not started */
		if (ctrl.value0 == ACQUISITION_STOPPED) {
			AV_DEBUG("Starting camera acquisition...");
			CLEAR(ctrl);
			ctrl.id = V4L2_AV_IMX_CSI2_STREAMON_W;
			ctrl.value0 = 1;
			retval = mxc_v4l2_send_command(cam, &ctrl);

			if (retval < 0)
			{
				AV_ERR("ERROR while writing acquisition status register");
				return -EINVAL;
			}

			CLEAR(ctrl);
			ctrl.id = V4L2_AV_IMX_CSI2_ACQ_STATUS_R;
			retval = mxc_v4l2_send_command(cam, &ctrl);
			AV_DEBUG("New camera acquisition status=%d", ctrl.value0);
		}
	}

	return 0;
}

static int camera_acquisition_stop(cam_data *cam)
{
	struct v4l2_send_command_control ctrl;
	int retval = 0;
	int err = 0;

	if (NULL == cam) 
	{
		AV_ERR("cam == NULL");
		return -EINVAL;
	}

	if (is_av_camera)
	{
		/* Read acquisition status of the camera */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_ACQ_STATUS_R;
		retval = mxc_v4l2_send_command(cam, &ctrl);

		if (retval < 0){
			AV_ERR("ERROR while reading acquisition status register");
			return -EINVAL;
		}

		AV_DEBUG("Current camera acquisition status=%d (%s)", ctrl.value0, (ctrl.value0 == ACQUISITION_STOPPED) ? "ACQUISITION_STOPPED" : "ACQUISITION_RUNNING");
		/* STOP the acquisition */
		if (ctrl.value0 == ACQUISITION_RUNNING) {
			AV_DEBUG("Stopping acquisition...");
			CLEAR(ctrl);
			ctrl.id = V4L2_AV_IMX_CSI2_STREAMOFF_W;
			ctrl.value0 = 1;
			retval = mxc_v4l2_send_command(cam, &ctrl);

			if (retval < 0){
				AV_ERR("Error while stopping acquisition");
				return -EINVAL;
			}
		}
		else
		{
			AV_DEBUG("Acquisition already stopped");
		}

		/* We need to poll the register because the camera needs some time
		*  until the stopping process is really finished. 
		*  Measured: ~1sec
		*/
		err = wait_for_camera_acqstop(cam, ACQUISITION_STOP_TIMEOUT);

		if (err != 0)
		{
			/* Waiting failed: Try acquisition ABORT */
			AV_DEBUG(" Aborting acquisition...");
			CLEAR(ctrl);
			ctrl.id = V4L2_AV_IMX_CSI2_ABORT_W;
			ctrl.value0 = 1;
			retval = mxc_v4l2_send_command(cam, &ctrl);

			if (retval < 0)
			{
				AV_ERR(" Error while aborting acquisition");
				return -EINVAL;
			}

			return wait_for_camera_acqstop(cam, ACQUISITION_STOP_TIMEOUT);
		}
	}

	return 0;
}

/*!
 * Start the encoder job
 *
 * @param cam      structure cam_data *
 *
 * @return status  0 Success
 */
static int mxc_streamon(cam_data *cam)
{
	struct mxc_v4l_frame *frame;
	unsigned long lock_flags;
	int err = 0;
	struct v4l2_send_command_control ctrl1;
	int retval = 0;

	AV_DEBUG("mxc_streamon START --->");
	
	/*
	void *info;
	info = mipi_csi2_get_info();
	mipi_csi2_reg_dump(info);
	*/

	if (NULL == cam) {
		AV_ERR("cam == NULL");
		return -EINVAL;
	}

	if (cam->capture_on) {
		AV_ERR("v4l2 capture: Capture stream has been turned on");
		return -EIO;
	}

	cam->queued = true;

	if (list_empty(&cam->ready_q)) {
		AV_ERR("v4l2 capture: mxc_streamon buffer has not been queued yet");
		cam->queued = false;
		return -EINVAL;
	}

	if (cam->enc_update_eba &&
		cam->ready_q.prev == cam->ready_q.next) {
		AV_ERR("v4l2 capture: mxc_streamon buffer need ping pong at least two buffers");
		return -EINVAL;
	}

	cam->capture_pid = current->pid;

	if (cam->overlay_on == true)
		stop_preview(cam);

	if (cam->enc_enable) {
		err = cam->enc_enable(cam);
		if (err != 0)
		{
			AV_ERR("ERROR while calling cam->enc_enable");
			return err;
		}
	}

	spin_lock_irqsave(&cam->queue_int_lock, lock_flags);
	cam->ping_pong_csi = 0;
	cam->local_buf_num = 0;
	if (cam->enc_update_eba) {
		frame =	list_entry(cam->ready_q.next, struct mxc_v4l_frame, queue);
		list_del(cam->ready_q.next);
		list_add_tail(&frame->queue, &cam->working_q);
		frame->ipu_buf_num = cam->ping_pong_csi;
		err = cam->enc_update_eba(cam, frame->buffer.m.offset);

		frame =	list_entry(cam->ready_q.next, struct mxc_v4l_frame, queue);
		list_del(cam->ready_q.next);
		list_add_tail(&frame->queue, &cam->working_q);
		frame->ipu_buf_num = cam->ping_pong_csi;
		err |= cam->enc_update_eba(cam, frame->buffer.m.offset);
		spin_unlock_irqrestore(&cam->queue_int_lock, lock_flags);
	} else {
		spin_unlock_irqrestore(&cam->queue_int_lock, lock_flags);
		AV_ERR("ERROR: cam->enc_update_eba");
		return -EINVAL;
	}

	if (cam->overlay_on == true)
		start_preview(cam);

	if (cam->enc_enable_csi) {
		err = cam->enc_enable_csi(cam);
		if (err != 0)
		{
			AV_ERR("ERROR: cam->enc_enable_csi");
			return err;
		}
	}

	cam->capture_on = true;

	err = camera_acquisition_start(cam);

	if (!err)
	{
		AV_DEBUG("mxc_streamon SUCCESS <---");
	}
	
	/*
	info = mipi_csi2_get_info();
	mipi_csi2_reg_dump(info);
	*/

	return err;
}

/*!
 * Shut down the encoder job
 *
 * @param cam      structure cam_data *
 *
 * @return status  0 Success
 */
static int mxc_streamoff(cam_data *cam)
{
	int err = 0;
	int retval = 0;
	struct v4l2_send_command_control ctrl;

	AV_DEBUG("mxc_streamoff START --->");
	
	if (NULL == cam) {
		AV_ERR("cam = NULL");
		return -1;
	}

	err = camera_acquisition_stop(cam);
	
	AV_DEBUG(" cam->ipu_id=%d", cam->ipu_id);
	AV_DEBUG(" cam->csi=%d", cam->csi);
	AV_DEBUG(" capture_on=%d", cam->capture_on);
	AV_DEBUG(" mxc_capture_inputs[cam->current_input].name=%s", mxc_capture_inputs[cam->current_input].name);

	if (cam->capture_on == false)
		return 0;

	/* For both CSI--MEM and CSI--IC--MEM
	 * 1. wait for idmac eof
	 * 2. disable csi first
	 * 3. disable idmac
	 * 4. disable smfc (CSI--MEM channel)
	 */

	if (mxc_capture_inputs[cam->current_input].name != NULL) {
		if (cam->enc_disable_csi) {
			err = cam->enc_disable_csi(cam);
			if (err != 0)
			{
				AV_ERR("ERROR: cam->enc_disable_csi");
				return err;
			}
		}
		if (cam->enc_disable) {
			err = cam->enc_disable(cam);
			if (err != 0)
			{
				AV_ERR("ERROR: cam->enc_disable");
				return err;
			}
		}
	}

	mxc_free_frames(cam);
	mxc_capture_inputs[cam->current_input].status |= V4L2_IN_ST_NO_POWER;
	cam->capture_on = false;

	if (!err)
		AV_DEBUG("mxc_streamoff SUCCESS <---");	

	return err;
}

static int wait_for_camera_acqstop(cam_data *cam, unsigned long ms_timeout)
{
	int retval = 0;
	bool success = false;
	struct timeval tstart;
	struct timeval tend;
	struct v4l2_send_command_control ctrl;	
	uint64_t nStart;
	uint64_t nEnd;

	AV_DEBUG(" Waiting for camera acquisition stop...");
	do_gettimeofday(&tstart);
	nStart = (tstart.tv_sec * (uint64_t)1000) + (tstart.tv_usec / 1000);

	do{
		/* Read the camera acquisition status */
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_ACQ_STATUS_R;
		retval = mxc_v4l2_send_command(cam, &ctrl);

		if (retval < 0){
			AV_ERR(" Failed to query acquisition status register");
			break;
		}
		do_gettimeofday(&tend);
		nEnd = (tend.tv_sec * (uint64_t)1000) + (tend.tv_usec / 1000);

		if (nEnd - nStart > (uint64_t)ms_timeout)
			break;

AV_DEBUG("ACQ_STATUS=%d", ctrl.value0);
		if (ctrl.value0 == ACQUISITION_RUNNING) {
			/* Still running -> Wait */
			msleep(50);
		}
		else
		{
			/* Camera has stopped -> exit */
			success = true;					
		}
	}while(success == 0);

	if (success)
		AV_DEBUG(" Acq. stop success. Duration: %lldms", nEnd - nStart);
	else
		AV_ERR(" Acq. stop FAILED. Duration: %lldms", nEnd - nStart);

	return (success == false) ? -EIO : 0;
}

/*!
 * Valid and adjust the overlay window size, position
 *
 * @param cam      structure cam_data *
 * @param win      struct v4l2_window  *
 *
 * @return 0
 */
static int verify_preview(cam_data *cam, struct v4l2_window *win)
{
	int i = 0, width_bound = 0, height_bound = 0;
	int *width, *height;
	unsigned int ipu_ch = CHAN_NONE;
	struct fb_info *bg_fbi = NULL, *fbi = NULL;
	bool foregound_fb = false;
	mm_segment_t old_fs;

	do {
		fbi = (struct fb_info *)registered_fb[i];
		if (fbi == NULL) {
			AV_ERR("verify_preview frame buffer NULL.");
			return -1;
		}

		/* Which DI supports 2 layers? */
		if (((strncmp(fbi->fix.id, "DISP3 BG", 8) == 0) &&
					(cam->output < 3)) ||
		    ((strncmp(fbi->fix.id, "DISP4 BG", 8) == 0) &&
					(cam->output >= 3))) {
			if (fbi->fbops->fb_ioctl) {
				old_fs = get_fs();
				set_fs(KERNEL_DS);
				fbi->fbops->fb_ioctl(fbi, MXCFB_GET_FB_IPU_CHAN,
						(unsigned long)&ipu_ch);
				set_fs(old_fs);
			}
			if (ipu_ch == MEM_BG_SYNC) {
				bg_fbi = fbi;
				AV_DEBUG("Found background frame buffer.");
			}
		}

		/* Found the frame buffer to preview on. */
		if (strcmp(fbi->fix.id,
			    mxc_capture_outputs[cam->output].name) == 0) {
			if (((strcmp(fbi->fix.id, "DISP3 FG") == 0) &&
						(cam->output < 3)) ||
			    ((strcmp(fbi->fix.id, "DISP4 FG") == 0) &&
						(cam->output >= 3)))
				foregound_fb = true;

			cam->overlay_fb = fbi;
			break;
		}
	} while (++i < FB_MAX);

	if (foregound_fb) {
		width_bound = bg_fbi->var.xres;
		height_bound = bg_fbi->var.yres;

		if (win->w.width + win->w.left > bg_fbi->var.xres ||
		    win->w.height + win->w.top > bg_fbi->var.yres) {
			AV_ERR("FG window position exceeds.");
			return -1;
		}
	} else {
		/* 4 bytes alignment for BG */
		width_bound = cam->overlay_fb->var.xres;
		height_bound = cam->overlay_fb->var.yres;

		if (cam->overlay_fb->var.bits_per_pixel == 24)
			win->w.left -= win->w.left % 4;
		else if (cam->overlay_fb->var.bits_per_pixel == 16)
			win->w.left -= win->w.left % 2;

		if (win->w.width + win->w.left > cam->overlay_fb->var.xres)
			win->w.width = cam->overlay_fb->var.xres - win->w.left;
		if (win->w.height + win->w.top > cam->overlay_fb->var.yres)
			win->w.height = cam->overlay_fb->var.yres - win->w.top;
	}

	/* stride line limitation */
	win->w.height -= win->w.height % 8;
	win->w.width -= win->w.width % 8;

	if (cam->rotation >= IPU_ROTATE_90_RIGHT) {
		height = &win->w.width;
		width = &win->w.height;
	} else {
		width = &win->w.width;
		height = &win->w.height;
	}

	if (*width == 0 || *height == 0) {
		AV_ERR("v4l2 capture: width or height too small.");
		return -EINVAL;
	}

	if ((cam->crop_bounds.width / *width > 8) ||
	    ((cam->crop_bounds.width / *width == 8) &&
	     (cam->crop_bounds.width % *width))) {
		*width = cam->crop_bounds.width / 8;
		if (*width % 8)
			*width += 8 - *width % 8;
		if (*width + win->w.left > width_bound) {
			AV_ERR("v4l2 capture: width exceeds resize limit.");
			return -1;
		}
		AV_ERR("v4l2 capture: width exceeds limit. Resize to %d.", *width);
	}

	if ((cam->crop_bounds.height / *height > 8) ||
	    ((cam->crop_bounds.height / *height == 8) &&
	     (cam->crop_bounds.height % *height))) {
		*height = cam->crop_bounds.height / 8;
		if (*height % 8)
			*height += 8 - *height % 8;
		if (*height + win->w.top > height_bound) {
			AV_ERR("v4l2 capture: height exceeds resize limit.");
			return -1;
		}
		AV_ERR("v4l2 capture: height exceeds limit resize to %d.", *height);
	}

	return 0;
}

/*!
 * start the viewfinder job
 *
 * @param cam      structure cam_data *
 *
 * @return status  0 Success
 */
static int start_preview(cam_data *cam)
{
	int err = 0;

	if (cam->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY)
	#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
		err = prp_vf_sdc_select(cam);
	#else
		err = foreground_sdc_select(cam);
	#endif
	else if (cam->v4l2_fb.flags == V4L2_FBUF_FLAG_PRIMARY)
	#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
		err = prp_vf_sdc_select_bg(cam);
	#else
		err = bg_overlay_sdc_select(cam);
	#endif
	if (err != 0)
		return err;

	if (cam->vf_start_sdc) {
		err = cam->vf_start_sdc(cam);
		if (err != 0)
			return err;
	}

	if (cam->vf_enable_csi)
		err = cam->vf_enable_csi(cam);

	AV_DEBUG("End of v2f pix w=%d, h=%d",
		cam->v2f.fmt.pix.width, cam->v2f.fmt.pix.height);
	AV_DEBUG("End of crop_bounds w=%d, h=%d",
		cam->crop_bounds.width, cam->crop_bounds.height);
	AV_DEBUG("End of crop_defrect w=%d, h=%d",
		cam->crop_defrect.width, cam->crop_defrect.height);
	AV_DEBUG("End of crop_current w=%d, h=%d",
		cam->crop_current.width, cam->crop_current.height);

	return err;
}

/*!
 * shut down the viewfinder job
 *
 * @param cam      structure cam_data *
 *
 * @return status  0 Success
 */
static int stop_preview(cam_data *cam)
{
	int err = 0;
    
	if (cam->vf_disable_csi) {
		err = cam->vf_disable_csi(cam);
		if (err != 0)
			return err;
	}

	if (cam->vf_stop_sdc) {
		err = cam->vf_stop_sdc(cam);
		if (err != 0)
			return err;
	}

	if (cam->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY)
	#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
		err = prp_vf_sdc_deselect(cam);
	#else
		err = foreground_sdc_deselect(cam);
	#endif
	else if (cam->v4l2_fb.flags == V4L2_FBUF_FLAG_PRIMARY)
	#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
		err = prp_vf_sdc_deselect_bg(cam);
	#else
		err = bg_overlay_sdc_deselect(cam);
	#endif

	return err;
}

/***************************************************************************
 * VIDIOC Functions.
 **************************************************************************/

/*!
 * V4L2 - mxc_v4l2_g_fmt function
 *
 * @param cam	 structure cam_data *
 *
 * @param f	   structure v4l2_format *
 *
 * @return  status    0 success, EINVAL failed
 */
static int mxc_v4l2_g_fmt(cam_data *cam, struct v4l2_format *f)
{
	int retval = 0;
	struct v4l2_send_command_control ct;

 	AV_DEBUG("type=%d", f->type);

	if (is_av_camera) {

		if (is_gencp_mode) {
			AV_DEBUG("GENCP mode");
			goto gencp_l1;
		}

		/* Read the default width from alliedvision camera */
		CLEAR(ct);
		ct.id = V4L2_AV_IMX_CSI2_WIDTH_R;
		retval = mxc_v4l2_send_command(cam, &ct);

		if (retval < 0)
			return -EINVAL;

		cam->v2f.fmt.pix.width = ct.value0;

		/* Read the default height from alliedvision camera */
		CLEAR(ct);
		ct.id = V4L2_AV_IMX_CSI2_HEIGHT_R;
		retval = mxc_v4l2_send_command(cam, &ct);

		if (retval < 0)
			return -EINVAL;

		cam->v2f.fmt.pix.height = ct.value0;

		/* Read the default frame size from alliedvision camera */
		CLEAR(ct);
		ct.id = V4L2_AV_IMX_CSI2_PAYLOADSIZE_R;
		retval = mxc_v4l2_send_command(cam, &ct);

		if (retval < 0)
			return -EINVAL;

		cam->v2f.fmt.pix.sizeimage = ct.value0;


		/* Read the default pixelformat from alliedvision camera */
		CLEAR(ct);
		ct.id = V4L2_AV_IMX_CSI2_PIXELFORMAT_R;
		retval = mxc_v4l2_send_command(cam, &ct);

		if (retval < 0)
			return -EINVAL;
		/*
		cam->v2f.fmt.pix.pixelformat = ct.value0;
		*/

		if (ct.value0 == V4L2_PIX_FMT_Y10P || ct.value0 == V4L2_PIX_FMT_SBGGR10P || \
			 ct.value0 == V4L2_PIX_FMT_SGBRG10P || ct.value0 == V4L2_PIX_FMT_SGRBG10P ||
			 ct.value0 == V4L2_PIX_FMT_SRGGB10P) {
			/* Recalculate the payload size for RAW10 since IPU is configured to 16bpp */
			cam->v2f.fmt.pix.sizeimage = cam->v2f.fmt.pix.height * cam->v2f.fmt.pix.width * 2;
		}

		AV_DEBUG("Camera values:");
		AV_DEBUG(" cam->v2f.fmt.pix.width=%d", cam->v2f.fmt.pix.width);
		AV_DEBUG(" cam->v2f.fmt.pix.height=%d ", cam->v2f.fmt.pix.height);
		AV_DEBUG(" cam->v2f.fmt.pix.pixelformat=%d (0x%x)", cam->v2f.fmt.pix.pixelformat, cam->v2f.fmt.pix.pixelformat);
		AV_DEBUG(" cam->v2f.fmt.pix.sizeimage=%d", cam->v2f.fmt.pix.sizeimage);
	}

gencp_l1:

	AV_DEBUG("v2f pix w=%d, h=%d, sizeimage=%d",
		 cam->v2f.fmt.pix.width, cam->v2f.fmt.pix.height, cam->v2f.fmt.pix.sizeimage);

	AV_DEBUG("pix w=%d, h=%d, sizeimage=%d",
		 f->fmt.pix.width, f->fmt.pix.height, f->fmt.pix.sizeimage);

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		AV_DEBUG("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE");
		f->fmt.pix = cam->v2f.fmt.pix;
		break;
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		AV_DEBUG("   type is V4L2_BUF_TYPE_VIDEO_OVERLAY");
		f->fmt.win = cam->win;
		break;
	default:
		AV_DEBUG("   type is invalid");
		retval = -EINVAL;
	}

	AV_DEBUG("v2f pix w=%d, h=%d, sizeimage=%d, pixelformat=%d (0x%x)",
		 cam->v2f.fmt.pix.width, cam->v2f.fmt.pix.height, cam->v2f.fmt.pix.sizeimage, cam->v2f.fmt.pix.pixelformat, cam->v2f.fmt.pix.pixelformat);
	AV_DEBUG("crop_bounds w=%d, h=%d",
		 cam->crop_bounds.width, cam->crop_bounds.height);
	AV_DEBUG("crop_defrect w=%d, h=%d",
		 cam->crop_defrect.width, cam->crop_defrect.height);
	AV_DEBUG("crop_current w=%d, h=%d",
		cam->crop_current.width, cam->crop_current.height);

	return retval;
}


/*!
 * V4L2 - mxc_v4l2_s_fmt function
 *
 * @param cam	 structure cam_data *
 *
 * @param f	   structure v4l2_format *
 *
 * @return  status    0 success, EINVAL failed
 */
static int mxc_v4l2_s_fmt(cam_data *cam, struct v4l2_format *f)
{
	int retval = 0;
	uint32_t size = 0;
	uint32_t size_av = 0;
	int bytesperline = 0;
	int *width, *height;
	struct v4l2_send_command_control c;

	AV_DEBUG("w=%d, h=%d, pixel_fmt=%d, v4l2_buffer_type=%d", f->fmt.pix.width, f->fmt.pix.height, f->fmt.pix.pixelformat, f->type);

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		AV_DEBUG("   v4l2_buffer_type=V4L2_BUF_TYPE_VIDEO_CAPTURE");
		if (!valid_mode(f->fmt.pix.pixelformat)) {
			AV_ERR("v4l2 capture: mxc_v4l2_s_fmt: format not supported");
			return -EINVAL;
		}

#if 0
		/*
		 * Force the capture window resolution to be crop bounds
		 * for CSI MEM input mode.
		 */
		if (strcmp(mxc_capture_inputs[cam->current_input].name,
			   "CSI MEM") == 0) {
			f->fmt.pix.width = cam->crop_current.width;
			f->fmt.pix.height = cam->crop_current.height;
		}
#endif

		if (is_av_camera) {
			if (is_gencp_mode) {
				AV_DEBUG("GENCP mode (sizeimage=%d)", f->fmt.pix.sizeimage);
				size = f->fmt.pix.sizeimage;
				/* size = f->fmt.pix.sizeimage = f->fmt.pix.width * f->fmt.pix.height; */
				goto gencp_l3;
			}
		}

		if (cam->rotation >= IPU_ROTATE_90_RIGHT) {
			height = &f->fmt.pix.width;
			width = &f->fmt.pix.height;
		} else {
			width = &f->fmt.pix.width;
			height = &f->fmt.pix.height;
		}

		/* stride line limitation */
		if((*width & 7) > 0)
        {
            int width_new = *width & ~7;
            AV_DEBUG("Width (value = %d) will be adjusted because it is not divisible by 8! New value: %d", *width, width_new);
            *width = width_new;
        }
 
        if((*height & 7) > 0)
        {
            int height_new = *height & ~7;
            AV_DEBUG("Height (value = %d) will be adjusted because it is not divisible by 8! New value: %d", *height, height_new);
            *height = height_new;
        }

		if (*width == 0 || *height == 0) {
			AV_ERR("v4l2 capture: width or height too small.");
			return -EINVAL;
		}

		if ((cam->crop_current.width / *width > 8) ||
		    ((cam->crop_current.width / *width == 8) &&
		     (cam->crop_current.width % *width))) {
			*width = cam->crop_current.width / 8;
			if (*width % 8)
				*width += 8 - *width % 8;
			AV_ERR("v4l2 capture: width exceeds limit resize to %d.", *width);
		}

		if ((cam->crop_current.height / *height > 8) ||
		    ((cam->crop_current.height / *height == 8) &&
		     (cam->crop_current.height % *height))) {
			*height = cam->crop_current.height / 8;
			if (*height % 8)
				*height += 8 - *height % 8;
			AV_ERR("v4l2 capture: height exceeds limit resize to %d.", *height);
		}


		if (is_av_camera) {

			AV_DEBUG("BCRM mode (sizeimage=%d)", f->fmt.pix.sizeimage);

			/* Read the Width Max value */
			CLEAR(c);
			c.id = V4L2_AV_IMX_CSI2_WIDTH_MAXVAL_R;
			retval = mxc_v4l2_send_command(cam, &c);

			if (retval < 0)
				return -EINVAL;

			if (f->fmt.pix.width > c.value0) {
				AV_ERR("Width exceeds camera limitation!");
				f->fmt.pix.width = c.value0;
			}

			/* Read the Width Increment value */
			CLEAR(c);
			c.id = V4L2_AV_IMX_CSI2_WIDTH_INCVAL_R;
			retval = mxc_v4l2_send_command(cam, &c);

			if (retval < 0)
				return -EINVAL;

			if (!((f->fmt.pix.width % c.value0) == 0)) {
				AV_ERR("Width INCREMENT not supported.");
				return -EINVAL;
			}

			/* Write the Width value */
			CLEAR(c);
			c.id = V4L2_AV_IMX_CSI2_WIDTH_W;
			c.value0 = f->fmt.pix.width;
			retval = mxc_v4l2_send_command(cam, &c);

			if (retval < 0)
				return -EINVAL;


			/* Read the Height Max value */
			CLEAR(c);
			c.id = V4L2_AV_IMX_CSI2_HEIGHT_MAXVAL_R;
			retval = mxc_v4l2_send_command(cam, &c);

			if (retval < 0)
				return -EINVAL;

			if (f->fmt.pix.height > c.value0) {
				AV_ERR("Height exceeds camera limitation!");
				f->fmt.pix.height = c.value0;
			}

			/* Read the Height Increment value */
			CLEAR(c);
			c.id = V4L2_AV_IMX_CSI2_HEIGHT_INCVAL_R;
			retval = mxc_v4l2_send_command(cam, &c);

			if (retval < 0)
				return -EINVAL;

			if (!((f->fmt.pix.height % c.value0) == 0)) {
				AV_ERR("Height INCREMENT not supported.");
				return -EINVAL;
			}


			/* Write the Height value */
			CLEAR(c);
			c.id = V4L2_AV_IMX_CSI2_HEIGHT_W;
			c.value0 = f->fmt.pix.height;
			retval = mxc_v4l2_send_command(cam, &c);

			if (retval < 0)
				return -EINVAL;

			/* Write the pixelformat value */
			CLEAR(c);
			c.id = V4L2_AV_IMX_CSI2_PIXELFORMAT_W;
			c.value0 = f->fmt.pix.pixelformat;
			retval = mxc_v4l2_send_command(cam, &c);

			if (retval < 0)
				return -EINVAL;

			/* Reading the frame size from device to Set in driver. */
			CLEAR(c);
			c.id = V4L2_AV_IMX_CSI2_PAYLOADSIZE_R;
			retval = mxc_v4l2_send_command(cam, &c);

			if (retval < 0)
				return -EINVAL;

			size_av = c.value0;
			AV_DEBUG("Camera payloadsize=%d", size_av);

			/* Read the pixelformat from alliedvision camera */
			CLEAR(c);
			c.id = V4L2_AV_IMX_CSI2_PIXELFORMAT_R;
			retval = mxc_v4l2_send_command(cam, &c);

			if (retval < 0)
				return -EINVAL;

			if (c.value0 == V4L2_PIX_FMT_Y10P || c.value0 == V4L2_PIX_FMT_SBGGR10P
				 || c.value0 == V4L2_PIX_FMT_SGBRG10P
				 || c.value0 == V4L2_PIX_FMT_SGRBG10P || c.value0 == V4L2_PIX_FMT_SRGGB10P) {
				/* Recalculate the payload size for RAW10 since IPU is configured to 16bpp */
				size_av = f->fmt.pix.width * f->fmt.pix.height * 2;
			}
		}
gencp_l3:

		/* Print fourcc */
		AV_DEBUG("\"%c%c%c%c\"", 
			(f->fmt.pix.pixelformat >> 0) & 0xff,
			(f->fmt.pix.pixelformat >> 8) & 0xff,
			(f->fmt.pix.pixelformat >> 16) & 0xff,
			(f->fmt.pix.pixelformat >> 24) & 0xff);

		switch (f->fmt.pix.pixelformat) {
		case V4L2_PIX_FMT_GREY:
			size = f->fmt.pix.width * f->fmt.pix.height * 1;
			bytesperline = f->fmt.pix.width * 1;
			break;
		case V4L2_PIX_FMT_SBGGR8:
			size = f->fmt.pix.width * f->fmt.pix.height * 1;
			bytesperline = f->fmt.pix.width * 1;
			break;
		case V4L2_PIX_FMT_SGBRG8:
			size = f->fmt.pix.width * f->fmt.pix.height * 1;
			bytesperline = f->fmt.pix.width * 1;
			break;
		case V4L2_PIX_FMT_SGRBG8:
			size = f->fmt.pix.width * f->fmt.pix.height * 1;
			bytesperline = f->fmt.pix.width * 1;
			break;
		case V4L2_PIX_FMT_SRGGB8:
			size = f->fmt.pix.width * f->fmt.pix.height * 1;
			bytesperline = f->fmt.pix.width * 1;
			break;
		case V4L2_PIX_FMT_Y10P:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width * 2;/* 10 / 8; */
			break;
		case V4L2_PIX_FMT_SBGGR10P:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width * 2;/* 10 / 8; */
			break;
		case V4L2_PIX_FMT_SGBRG10P:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width * 2;/* 10 / 8; */
			break;
		case V4L2_PIX_FMT_SGRBG10P:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width * 2;/* 10 / 8; */
			break;
		case V4L2_PIX_FMT_SRGGB10P:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width * 2;/* 10 / 8; */
			break;
		case V4L2_PIX_FMT_Y12P:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width * 12 / 8;
			break;
		case V4L2_PIX_FMT_SBGGR12P:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width * 12 / 8;
			break;
		case V4L2_PIX_FMT_SGBRG12P:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width * 12 / 8;
			break;
		case V4L2_PIX_FMT_SGRBG12P:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width * 12 / 8;
			break;
		case V4L2_PIX_FMT_SRGGB12P:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width * 12 / 8;
			break;
		case V4L2_PIX_FMT_GREY12P:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width * 12 / 8;
			break;
		case V4L2_PIX_FMT_RGB565:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width * 2;
			break;
		case V4L2_PIX_FMT_BGR24:
			size = f->fmt.pix.width * f->fmt.pix.height * 3;
			bytesperline = f->fmt.pix.width * 3;
			break;
		case V4L2_PIX_FMT_RGB24:
			size = f->fmt.pix.width * f->fmt.pix.height * 3;
			bytesperline = f->fmt.pix.width * 3;
			break;
		case V4L2_PIX_FMT_CUSTOM:
			size = size_av = f->fmt.pix.sizeimage;
			bytesperline = f->fmt.pix.width;
			break;
		case V4L2_PIX_FMT_RGB666:
			size = f->fmt.pix.width * f->fmt.pix.height * 9 / 4;
			bytesperline = f->fmt.pix.width * 2;
			break;
		case V4L2_PIX_FMT_RGB555:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width * 2;
			break;
		case V4L2_PIX_FMT_RGB444:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width * 2;
			break;
		case V4L2_PIX_FMT_BGR32:
			size = f->fmt.pix.width * f->fmt.pix.height * 4;
			bytesperline = f->fmt.pix.width * 4;
			break;
		case V4L2_PIX_FMT_RGB32:
			size = f->fmt.pix.width * f->fmt.pix.height * 4;
			bytesperline = f->fmt.pix.width * 4;
			break;
		case V4L2_PIX_FMT_YUV422P:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width;
			break;
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_YUYV:
			size = f->fmt.pix.width * f->fmt.pix.height * 2;
			bytesperline = f->fmt.pix.width * 2;
			break;
		case V4L2_PIX_FMT_YUV420:
		case V4L2_PIX_FMT_YVU420:
			size = f->fmt.pix.width * f->fmt.pix.height * 3 / 2;
			bytesperline = f->fmt.pix.width;
			break;
		case V4L2_PIX_FMT_NV12:
			size = f->fmt.pix.width * f->fmt.pix.height * 3 / 2;
			bytesperline = f->fmt.pix.width;
			break;
		default:
			break;
		}

		if (is_av_camera) {
			f->fmt.pix.bytesperline = bytesperline;
			f->fmt.pix.sizeimage = size_av;
		} else {
			if (f->fmt.pix.bytesperline < bytesperline)
				f->fmt.pix.bytesperline = bytesperline;
			else
				bytesperline = f->fmt.pix.bytesperline;

			if (f->fmt.pix.sizeimage < size)
				f->fmt.pix.sizeimage = size;
			else
				size = f->fmt.pix.sizeimage;
		}

		cam->v2f.fmt.pix = f->fmt.pix;

		if (cam->v2f.fmt.pix.priv != 0) {
			if (copy_from_user(&cam->offset,
					   (void *)cam->v2f.fmt.pix.priv,
					   sizeof(cam->offset))) {
				retval = -EFAULT;
				break;
			}
		}

		AV_DEBUG("f->fmt.pix.bytesperline=%d, f->fmt.pix.sizeimage=%d", f->fmt.pix.bytesperline, f->fmt.pix.sizeimage);

		break;
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		AV_DEBUG("   type=V4L2_BUF_TYPE_VIDEO_OVERLAY");
		retval = verify_preview(cam, &f->fmt.win);
		cam->win = f->fmt.win;
		break;
	default:
		AV_ERR("case default");
		retval = -EINVAL;
	}

	AV_DEBUG("v2f pix w=%d, h=%d, sizeimage=%d",
		 cam->v2f.fmt.pix.width, cam->v2f.fmt.pix.height, cam->v2f.fmt.pix.sizeimage);
	AV_DEBUG("crop_bounds w=%d, h=%d",
		 cam->crop_bounds.width, cam->crop_bounds.height);
	AV_DEBUG("crop_defrect w=%d, h=%d",
		 cam->crop_defrect.width, cam->crop_defrect.height);
	AV_DEBUG("crop_current w=%d, h=%d",
		cam->crop_current.width, cam->crop_current.height);

	AV_DEBUG("EXIT");
	return retval;
}

/*!
 * get control param
 *
 * @param cam	 structure cam_data *
 *
 * @param c	   structure v4l2_control *
 *
 * @return  status    0 success, EINVAL failed
 */
static int mxc_v4l2_g_ctrl(cam_data *cam, struct v4l2_control *c)
{
	int status = 0;
	struct v4l2_send_command_control ct;

	/* probably don't need to store the values that can be retrieved,
	 * locally, but they are for now. */
	switch (c->id) {
	case V4L2_CID_HFLIP:
		/* This is handled in the ipu. */
		if (is_av_camera) {
			CLEAR(ct);
			ct.id = V4L2_AV_IMX_CSI2_HFLIP_R;
			status = mxc_v4l2_send_command(cam, &ct);

			if (status < 0) {
				AV_ERR("mxc_v4l2_send_command is failed.");
				status = -EINVAL;
				return status;
			}
			c->value = ct.value0;
		} else {
			if (cam->rotation == IPU_ROTATE_HORIZ_FLIP)
				c->value = 1;
		}
		break;
	case V4L2_CID_VFLIP:
		/* This is handled in the ipu. */
		if (is_av_camera) {
			CLEAR(ct);
			ct.id = V4L2_AV_IMX_CSI2_VFLIP_R;
			status = mxc_v4l2_send_command(cam, &ct);

			if (status < 0) {
				AV_ERR("mxc_v4l2_send_command is failed.");
				status = -EINVAL;
				return status;
			}
			c->value = ct.value0;
		} else {
			if (cam->rotation == IPU_ROTATE_VERT_FLIP)
				c->value = 1;
		}

		break;
	case V4L2_CID_MXC_ROT:
		/* This is handled in the ipu. */
		c->value = cam->rotation;
		break;
	case V4L2_CID_BRIGHTNESS:
		if (cam->sensor) {
			c->value = cam->bright;
			status = vidioc_int_g_ctrl(cam->sensor, c);
			cam->bright = c->value;
		} else {
			AV_ERR("v4l2 capture: slave not found!");
			status = -ENODEV;
		}
		break;
	case V4L2_CID_HUE:
		if (cam->sensor) {
			c->value = cam->hue;
			status = vidioc_int_g_ctrl(cam->sensor, c);
			cam->hue = c->value;
		} else {
			AV_ERR("v4l2 capture: slave not found!");
			status = -ENODEV;
		}
		break;
	case V4L2_CID_CONTRAST:
		if (cam->sensor) {
			c->value = cam->contrast;
			status = vidioc_int_g_ctrl(cam->sensor, c);
			cam->contrast = c->value;
		} else {
			AV_ERR("v4l2 capture: slave not found!");
			status = -ENODEV;
		}
		break;
	case V4L2_CID_SATURATION:
		if (cam->sensor) {
			c->value = cam->saturation;
			status = vidioc_int_g_ctrl(cam->sensor, c);
			cam->saturation = c->value;
		} else {
			AV_ERR("v4l2 capture: slave not found!");
			status = -ENODEV;
		}
		break;
	case V4L2_CID_RED_BALANCE:
		if (cam->sensor) {
			c->value = cam->red;
			status = vidioc_int_g_ctrl(cam->sensor, c);
			cam->red = c->value;
		} else {
			AV_ERR("v4l2 capture: slave not found!");
			status = -ENODEV;
		}
		break;
	case V4L2_CID_BLUE_BALANCE:
		if (cam->sensor) {
			c->value = cam->blue;
			status = vidioc_int_g_ctrl(cam->sensor, c);
			cam->blue = c->value;
		} else {
			AV_ERR("v4l2 capture: slave not found!");
			status = -ENODEV;
		}
		break;
	case V4L2_CID_BLACK_LEVEL:
		if (cam->sensor) {
			c->value = cam->ae_mode;
			status = vidioc_int_g_ctrl(cam->sensor, c);
			cam->ae_mode = c->value;
		} else {
			AV_ERR("v4l2 capture: slave not found!");
			status = -ENODEV;
		}
		break;
	default:
		if (is_av_camera) {
			if (cam->sensor) {
				AV_DEBUG("c->value=%d", c->value);
				status = vidioc_int_g_ctrl(cam->sensor, c);
				AV_DEBUG("c->value=%d", c->value);
				cam->gain = c->value;
			} else {
				AV_ERR("v4l2 capture: slave not found!");
				status = -ENODEV;
			}
		} else {
			AV_DEBUG("default case");
			status = -EINVAL;
		}
			break;
	}

	return status;
}


static int mxc_v4l2_query_ctrl(cam_data *cam,
		struct v4l2_queryctrl *qctrl)
{
	int ret = 0;

	ret = vidioc_int_queryctrl(cam->sensor, qctrl);
	AV_DEBUG("vidioc_int_queryctrl: Status=%d", ret);
	return ret < 0 ? -EINVAL : 0;
}


static int mxc_v4l2_ext_query_ctrl(cam_data *cam,
		struct v4l2_query_ext_ctrl *ext_qctrl)
{
	int ret = 0;

	ret = vidioc_int_ext_queryctrl(cam->sensor, ext_qctrl);
	AV_DEBUG("err %d \n", ret);

	return ret < 0 ? -EINVAL : 0;
}


/*!
 * V4L2 - set_control function
 *	  V4L2_CID_PRIVATE_BASE is the extention for IPU preprocessing.
 *	  0 for normal operation
 *	  1 for vertical flip
 *	  2 for horizontal flip
 *	  3 for horizontal and vertical flip
 *	  4 for 90 degree rotation
 * @param cam	 structure cam_data *
 *
 * @param c	   structure v4l2_control *
 *
 * @return  status    0 success, EINVAL failed
 */
static int mxc_v4l2_s_ctrl(cam_data *cam, struct v4l2_control *c)
{
	int i, ret = 0;
	int tmp_rotation = IPU_ROTATE_NONE;
	struct sensor_data *sensor_data;
	struct v4l2_send_command_control ct;

	AV_DEBUG(" ID is 0x%x", c->id);

	if (c->id == 0)
		return -EINVAL;

	switch (c->id) {
	case V4L2_CID_HFLIP:
		/* This is done by the IPU */
		if (is_av_camera) {
			CLEAR(ct);
			ct.id = V4L2_AV_IMX_CSI2_HFLIP_W;
			ct.value0 = c->value;
			ret = mxc_v4l2_send_command(cam, &ct);

			if (ret < 0) {
				AV_ERR("mxc_v4l2_send_command failed.");
				ret = -EINVAL;
				return ret;
			}
			break;
		}
		if (c->value == 1) {
			if ((cam->rotation != IPU_ROTATE_VERT_FLIP) &&
			    (cam->rotation != IPU_ROTATE_180))
				cam->rotation = IPU_ROTATE_HORIZ_FLIP;
			else
				cam->rotation = IPU_ROTATE_180;
		} else {
			if (cam->rotation == IPU_ROTATE_HORIZ_FLIP)
				cam->rotation = IPU_ROTATE_NONE;
			if (cam->rotation == IPU_ROTATE_180)
				cam->rotation = IPU_ROTATE_VERT_FLIP;
		}

		break;
	case V4L2_CID_VFLIP:
		/* This is done by the IPU */
		if (is_av_camera) {
			CLEAR(ct);
			ct.id = V4L2_AV_IMX_CSI2_VFLIP_W;
			ct.value0 = c->value;
			ret = mxc_v4l2_send_command(cam, &ct);

			if (ret < 0) {
				AV_ERR("mxc_v4l2_send_command failed.");
				ret = -EINVAL;
				return ret;
			}
			break;
		}
		if (c->value == 1) {
			if ((cam->rotation != IPU_ROTATE_HORIZ_FLIP) &&
			    (cam->rotation != IPU_ROTATE_180))
				cam->rotation = IPU_ROTATE_VERT_FLIP;
			else
				cam->rotation = IPU_ROTATE_180;
		} else {
			if (cam->rotation == IPU_ROTATE_VERT_FLIP)
				cam->rotation = IPU_ROTATE_NONE;
			if (cam->rotation == IPU_ROTATE_180)
				cam->rotation = IPU_ROTATE_HORIZ_FLIP;
		}
		break;
	case V4L2_CID_MXC_ROT:
	case V4L2_CID_MXC_VF_ROT:
		/* This is done by the IPU */
		switch (c->value) {
		case V4L2_MXC_ROTATE_NONE:
			tmp_rotation = IPU_ROTATE_NONE;
			break;
		case V4L2_MXC_ROTATE_VERT_FLIP:
			tmp_rotation = IPU_ROTATE_VERT_FLIP;
			break;
		case V4L2_MXC_ROTATE_HORIZ_FLIP:
			tmp_rotation = IPU_ROTATE_HORIZ_FLIP;
			break;
		case V4L2_MXC_ROTATE_180:
			tmp_rotation = IPU_ROTATE_180;
			break;
		case V4L2_MXC_ROTATE_90_RIGHT:
			tmp_rotation = IPU_ROTATE_90_RIGHT;
			break;
		case V4L2_MXC_ROTATE_90_RIGHT_VFLIP:
			tmp_rotation = IPU_ROTATE_90_RIGHT_VFLIP;
			break;
		case V4L2_MXC_ROTATE_90_RIGHT_HFLIP:
			tmp_rotation = IPU_ROTATE_90_RIGHT_HFLIP;
			break;
		case V4L2_MXC_ROTATE_90_LEFT:
			tmp_rotation = IPU_ROTATE_90_LEFT;
			break;
		case V4L2_MXC_CAM_ROTATE_NONE:
			if (vidioc_int_s_ctrl(cam->sensor, c)) {
				ret = -EINVAL;
			}
			break;
		case V4L2_MXC_CAM_ROTATE_VERT_FLIP:
			if (vidioc_int_s_ctrl(cam->sensor, c)) {
				ret = -EINVAL;
			}
			break;
		case V4L2_MXC_CAM_ROTATE_HORIZ_FLIP:
			if (vidioc_int_s_ctrl(cam->sensor, c)) {
				ret = -EINVAL;
			}
			break;
		case V4L2_MXC_CAM_ROTATE_180:
			if (vidioc_int_s_ctrl(cam->sensor, c)) {
				ret = -EINVAL;
			}
			break;
		default:
			ret = -EINVAL;
		}
		#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
		if (c->id == V4L2_CID_MXC_VF_ROT)
			cam->vf_rotation = tmp_rotation;
		else
			cam->rotation = tmp_rotation;
		#else
			cam->rotation = tmp_rotation;
		#endif

		break;
	case V4L2_CID_HUE:
		if (cam->sensor) {
			cam->hue = c->value;
			ret = vidioc_int_s_ctrl(cam->sensor, c);
		} else {
			AV_ERR("v4l2 capture: slave not found!");
			ret = -ENODEV;
		}
		break;
	case V4L2_CID_CONTRAST:
		if (cam->sensor) {
			cam->contrast = c->value;
			ret = vidioc_int_s_ctrl(cam->sensor, c);
		} else {
			AV_ERR("v4l2 capture: slave not found!");
			ret = -ENODEV;
		}
		break;
	case V4L2_CID_BRIGHTNESS:
		if (cam->sensor) {
			cam->bright = c->value;
			ret = vidioc_int_s_ctrl(cam->sensor, c);
		} else {
			AV_ERR("v4l2 capture: slave not found!");
			ret = -ENODEV;
		}
		break;
	case V4L2_CID_SATURATION:
		if (cam->sensor) {
			cam->saturation = c->value;
			ret = vidioc_int_s_ctrl(cam->sensor, c);
		} else {
			AV_ERR("v4l2 capture: slave not found!");
			ret = -ENODEV;
		}
		break;
	case V4L2_CID_RED_BALANCE:
		if (cam->sensor) {
			cam->red = c->value;
			ret = vidioc_int_s_ctrl(cam->sensor, c);
		} else {
			AV_ERR("v4l2 capture: slave not found!");
			ret = -ENODEV;
		}
		break;
	case V4L2_CID_BLUE_BALANCE:
		if (cam->sensor) {
			cam->blue = c->value;
			ret = vidioc_int_s_ctrl(cam->sensor, c);
		} else {
			AV_ERR("v4l2 capture: slave not found!");
			ret = -ENODEV;
		}
		break;
	case V4L2_CID_EXPOSURE:
		if (cam->sensor) {
			cam->ae_mode = c->value;
			cam->expos = c->value;
			ret = vidioc_int_s_ctrl(cam->sensor, c);
		} else {
			AV_ERR("v4l2 capture: slave not found!");
			ret = -ENODEV;
		}
		break;

	case V4L2_CID_MXC_FLASH:
#ifdef CONFIG_MXC_IPU_V1
		ipu_csi_flash_strobe(true);
#endif
		break;

	case V4L2_CID_AUTO_FOCUS_START: {
		ret = vidioc_int_s_ctrl(cam->sensor, c);
		break;
	}

	case V4L2_CID_AUTO_FOCUS_STOP: {
		if (vidioc_int_s_ctrl(cam->sensor, c)) {
			ret = -EINVAL;
		}
		break;
	}

	case V4L2_CID_MXC_SWITCH_CAM:
		if (cam->sensor == cam->all_sensors[c->value])
			break;

		/* power down other cameraes before enable new one */
		for (i = 0; i < cam->sensor_index; i++) {
			if (i != c->value) {
				vidioc_int_dev_exit(cam->all_sensors[i]);
				vidioc_int_s_power(cam->all_sensors[i], 0);
				if (cam->mclk_on[cam->mclk_source]) {
					ipu_csi_enable_mclk_if (cam->ipu,
							CSI_MCLK_I2C,
							cam->mclk_source,
							false, false);
					cam->mclk_on[cam->mclk_source] =
								false;
				}
			}
		}
		sensor_data = cam->all_sensors[c->value]->priv;
		if (sensor_data->io_init)
			sensor_data->io_init();
		cam->sensor = cam->all_sensors[c->value];
		cam->mclk_source = sensor_data->mclk_source;
		ipu_csi_enable_mclk_if (cam->ipu, CSI_MCLK_I2C,
				       cam->mclk_source, true, true);
		cam->mclk_on[cam->mclk_source] = true;
		vidioc_int_s_power(cam->sensor, 1);
		ret = vidioc_int_dev_init(cam->sensor);

		if (ret < 0)
			ret = -EINVAL;

		break;
	default:
		if (is_av_camera) {
			if (cam->sensor) {
				ret = vidioc_int_s_ctrl(cam->sensor, c);
			} else {
				AV_ERR("v4l2 capture: slave not found!");
				ret = -ENODEV;
			}
		} else {
			AV_DEBUG("default case");
			ret = -EINVAL;
		}
		break;
	}
	return ret;
}

void setup_ifparm(cam_data *cam, int init_defrect)
{
	struct v4l2_format cam_fmt;
	ipu_csi_signal_cfg_t csi_param;
	struct v4l2_ifparm ifparm;
	int swidth, sheight;
	int sleft, stop;
	struct v4l2_send_command_control ct;
	int retval;

	vidioc_int_g_ifparm(cam->sensor, &ifparm);
	memset(&csi_param, 0, sizeof(csi_param));
	csi_param.csi = cam->csi;
	csi_param.mclk = ifparm.u.bt656.clock_curr;

	if (is_gencp_mode) {
		AV_DEBUG("GENCP mode");
		goto gencp_l2;
	}

	/* Need to read the width and height from camera */
	if (init_defrect) {
		if (is_av_camera) {
		/* Read the default width from alliedvision camera */
			CLEAR(ct);
			ct.id = V4L2_AV_IMX_CSI2_WIDTH_R;
			retval = mxc_v4l2_send_command(cam, &ct);

			if (retval < 0) {
				AV_ERR("Reading width failed. (Status=%d)", retval);
				return;
			}

			cam->v2f.fmt.pix.width = ct.value0;

			/* Read the default height from alliedvision camera */
			CLEAR(ct);
			ct.id = V4L2_AV_IMX_CSI2_HEIGHT_R;
			retval = mxc_v4l2_send_command(cam, &ct);

			if (retval < 0) {
				AV_ERR("Reading height failed. (Status=%d)", retval);
				return;
			}

			cam->v2f.fmt.pix.height = ct.value0;

			/* Read the default frame size from alliedvision camera */
			CLEAR(ct);
			ct.id = V4L2_AV_IMX_CSI2_PAYLOADSIZE_R;
			retval = mxc_v4l2_send_command(cam, &ct);

			if (retval < 0) {
				AV_ERR("Reading payloadsize failed. (Status=%d)", retval);
				return;
			}

			cam->v2f.fmt.pix.sizeimage = ct.value0;

			/* Read the default pixelformat from alliedvision camera */
			CLEAR(ct);
			ct.id = V4L2_AV_IMX_CSI2_PIXELFORMAT_R;
			retval = mxc_v4l2_send_command(cam, &ct);

			if (retval < 0)
				return;
			/*
			cam->v2f.fmt.pix.pixelformat = ct.value0;
			*/
			if (ct.value0 == V4L2_PIX_FMT_Y10P || ct.value0 == V4L2_PIX_FMT_SBGGR10P || \
				 ct.value0 == V4L2_PIX_FMT_SGBRG10P || ct.value0 == V4L2_PIX_FMT_SGRBG10P ||
				 ct.value0 == V4L2_PIX_FMT_SRGGB10P) {
				/* Recalculate the payload size for RAW10 since IPU is configured to 16bpp */
				cam->v2f.fmt.pix.sizeimage = cam->v2f.fmt.pix.height * cam->v2f.fmt.pix.width * 2;
			}

			AV_DEBUG("Camera values:");
			AV_DEBUG(" cam->v2f.fmt.pix.width=%d", cam->v2f.fmt.pix.width);
			AV_DEBUG(" cam->v2f.fmt.pix.height=%d", cam->v2f.fmt.pix.height);
			AV_DEBUG(" cam->v2f.fmt.pix.pixelformat=%d (0x%x)", cam->v2f.fmt.pix.pixelformat, cam->v2f.fmt.pix.pixelformat);
			AV_DEBUG(" cam->v2f.fmt.pix.sizeimage=%d", cam->v2f.fmt.pix.sizeimage);

			cam->v2f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			retval = mxc_v4l2_s_fmt(cam, &cam->v2f);

			if (retval < 0) {
				AV_ERR("setformat failed. (Status=%d)", retval);
				return;
			}
		}
	}

gencp_l2:

	AV_DEBUG("   clock_curr=mclk=%d", ifparm.u.bt656.clock_curr);

#ifndef WANDBOARD_IMX6
	switch (ifparm.if_type) {
	case V4L2_IF_TYPE_BT1120_PROGRESSIVE_DDR:
		csi_param.clk_mode = IPU_CSI_CLK_MODE_CCIR1120_PROGRESSIVE_DDR;
		break;
	case V4L2_IF_TYPE_BT1120_PROGRESSIVE_SDR:
		csi_param.clk_mode = IPU_CSI_CLK_MODE_CCIR1120_PROGRESSIVE_SDR;
		break;
	case V4L2_IF_TYPE_BT1120_INTERLACED_DDR:
		csi_param.clk_mode = IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_DDR;
		break;
	case V4L2_IF_TYPE_BT1120_INTERLACED_SDR:
		csi_param.clk_mode = IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_SDR;
		break;
	case V4L2_IF_TYPE_BT656_PROGRESSIVE:
		csi_param.clk_mode = IPU_CSI_CLK_MODE_CCIR656_PROGRESSIVE;
		break;
	case V4L2_IF_TYPE_BT656_INTERLACED:
		csi_param.clk_mode = IPU_CSI_CLK_MODE_CCIR656_INTERLACED;
		break;
	case V4L2_IF_TYPE_BT656:
/* 		csi_param.clk_mode = IPU_CSI_CLK_MODE_CCIR656_PROGRESSIVE; */
/* 		break; */
	default:
		csi_param.clk_mode = (ifparm.u.bt656.clock_curr == 0) ?
				IPU_CSI_CLK_MODE_CCIR656_INTERLACED :
				IPU_CSI_CLK_MODE_GATED_CLK;
	}

	csi_param.pixclk_pol = ifparm.u.bt656.latch_clk_inv;

	csi_param.data_width =
		(ifparm.u.bt656.mode == V4L2_IF_TYPE_BT656_MODE_NOBT_10BIT) ||
		(ifparm.u.bt656.mode == V4L2_IF_TYPE_BT656_MODE_BT_10BIT) ?
		IPU_CSI_DATA_WIDTH_10 : IPU_CSI_DATA_WIDTH_8;

	csi_param.pack_tight = (csi_param.data_width == IPU_CSI_DATA_WIDTH_10) ? 1 : 0;
#else
	if (ifparm.u.bt656.clock_curr == 0)
		csi_param.clk_mode = IPU_CSI_CLK_MODE_CCIR656_INTERLACED;
	else
		csi_param.clk_mode = IPU_CSI_CLK_MODE_GATED_CLK;

	csi_param.pixclk_pol = ifparm.u.bt656.latch_clk_inv;

	if (ifparm.u.bt656.mode == V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT) {
		csi_param.data_width = IPU_CSI_DATA_WIDTH_8;
	} else if (ifparm.u.bt656.mode
				== V4L2_IF_TYPE_BT656_MODE_NOBT_10BIT) {
		csi_param.data_width = IPU_CSI_DATA_WIDTH_10;
	} else {
		csi_param.data_width = IPU_CSI_DATA_WIDTH_8;
	}
#endif

	csi_param.Vsync_pol = ifparm.u.bt656.nobt_vs_inv;
	csi_param.Hsync_pol = ifparm.u.bt656.nobt_hs_inv;
	csi_param.ext_vsync = ifparm.u.bt656.bt_sync_correct;
	AV_DEBUG("vsync_pol(%d), hsync_pol(%d), ext_vsync(%d)", csi_param.Vsync_pol, csi_param.Hsync_pol, csi_param.ext_vsync);

	/* if the capturemode changed, the size bounds will have changed. */
	cam_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vidioc_int_g_fmt_cap(cam->sensor, &cam_fmt);
	AV_DEBUG("   g_fmt_cap returns width x height of input as %d x %d",
			cam_fmt.fmt.pix.width, cam_fmt.fmt.pix.height);

	csi_param.data_fmt = cam->v2f.fmt.pix.pixelformat;

	cam->crop_bounds.top = cam->crop_bounds.left = 0;

/*	cam->crop_bounds.width = cam->v2f.fmt.pix.width;
	cam->crop_bounds.height = cam->v2f.fmt.pix.height;
*/
	/*
	 * Set the default current cropped resolution to be the same with
	 * the cropping boundary(except for tvin module).
	 */
	if (cam->device_type != 1) {
		cam->crop_current.width = cam->crop_bounds.width;
		cam->crop_current.height = cam->crop_bounds.height;
	}

	if (init_defrect) {
		/* This also is the max crop size for this device. */
		cam->crop_defrect.top = cam->crop_defrect.left = 0;

		cam->crop_defrect.width = cam->v2f.fmt.pix.width;
		cam->crop_defrect.height = cam->v2f.fmt.pix.height;

		/* At this point, this is also the current image size. */
		cam->crop_current.top = cam->crop_current.left = 0;

		cam->crop_current.width = cam->v2f.fmt.pix.width;
		cam->crop_current.height = cam->v2f.fmt.pix.height;
		AV_DEBUG("On Open: Input to ipu size is %d x %d",
			cam->v2f.fmt.pix.width, cam->v2f.fmt.pix.height);


		AV_DEBUG("v2f pix widthxheight %d x %d",
			cam->v2f.fmt.pix.width, cam->v2f.fmt.pix.height);
		AV_DEBUG("crop_bounds widthxheight %d x %d",
			cam->crop_bounds.width, cam->crop_bounds.height);
		AV_DEBUG("crop_defrect widthxheight %d x %d",
			cam->crop_defrect.width, cam->crop_defrect.height);
		AV_DEBUG("crop_current widthxheight %d x %d",
			cam->crop_current.width, cam->crop_current.height);
	}

	cam->crop_current.width = cam->v2f.fmt.pix.width;
	cam->crop_current.height = cam->v2f.fmt.pix.height;
	cam->crop_bounds.width = cam->v2f.fmt.pix.width;
	cam->crop_bounds.height = cam->v2f.fmt.pix.height;
	cam->crop_defrect.width = cam->v2f.fmt.pix.width;
	cam->crop_defrect.height = cam->v2f.fmt.pix.height;
	cam->crop_current.top = cam->crop_current.left = 0;
	cam->crop_defrect.top = cam->crop_defrect.left = 0;
	cam->crop_bounds.top =  cam->crop_bounds.left = 0;

	swidth = cam->crop_current.width;
	sheight = cam->crop_current.height;
	sleft = 0;
	stop = 0;

#ifndef WANDBOARD_IMX6
	cam_fmt.type = V4L2_BUF_TYPE_SENSOR;
	cam_fmt.fmt.spix.swidth = 0;
	vidioc_int_g_fmt_cap(cam->sensor, &cam_fmt);
	if (cam_fmt.fmt.spix.swidth) {
		swidth = cam_fmt.fmt.spix.swidth;
		sheight = cam_fmt.fmt.spix.sheight;
		sleft =  cam_fmt.fmt.spix.left;
		stop =  cam_fmt.fmt.spix.top;
	}
	/* This essentially loses the data at the left and bottom of the image
	 * giving a digital zoom image, if crop_current is less than the full
	 * size of the image. */
	ipu_csi_window_size_crop(cam->ipu,
			swidth, sheight,
			cam->crop_current.width, cam->crop_current.height, csi_param.data_fmt,
			sleft + cam->crop_current.left, stop + cam->crop_current.top,
			cam->csi);

	ipu_csi_init_interface(cam->ipu, cam->crop_bounds.width,
			       cam->crop_bounds.height,
			       csi_param.data_fmt, csi_param);
#else
	/* This essentially loses the data at the left and bottom of the image
	 * giving a digital zoom image, if crop_current is less than the full
	 * size of the image. */
	ipu_csi_set_window_size(cam->ipu, cam->crop_current.width,
				cam->crop_current.height, cam->csi);
	ipu_csi_set_window_pos(cam->ipu, cam->crop_current.left,
			       cam->crop_current.top,
			       cam->csi);
	ipu_csi_init_interface(cam->ipu, cam->crop_bounds.width,
			       cam->crop_bounds.height,
			       cam_fmt.fmt.pix.pixelformat, csi_param);
#endif
}

/*!
 * V4L2 - mxc_v4l2_s_param function
 * Allows setting of capturemode and frame rate.
 *
 * @param cam	 structure cam_data *
 * @param parm	structure v4l2_streamparm *
 *
 * @return  status    0 success, EINVAL failed
 */
static int mxc_v4l2_s_param(cam_data *cam, struct v4l2_streamparm *parm)
{
	struct v4l2_streamparm currentparm;
#ifdef WANDBOARD_IMX6
	struct v4l2_ifparm ifparm;
	struct v4l2_format cam_fmt;
	ipu_csi_signal_cfg_t csi_param;
#endif
	u32 current_fps, parm_fps;
	int err = 0;

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		AV_ERR( "mxc_v4l2_s_param invalid type");
		return -EINVAL;
	}

    /*  if timeperframe is 0, return the current */
	if (parm->parm.capture.timeperframe.numerator == 0 || parm->parm.capture.timeperframe.denominator == 0)
	{
		return vidioc_int_g_parm(cam->sensor, parm);
	}

	/* Stop the viewfinder */
	if (cam->overlay_on == true)
		stop_preview(cam);

	currentparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	/* First check that this device can support the changes requested. */
	err = vidioc_int_g_parm(cam->sensor, &currentparm);
	if (err) {
		AV_ERR("vidioc_int_g_parm returned an error %d",
			err);
		goto exit;
	}

	current_fps = currentparm.parm.capture.timeperframe.denominator
			/ currentparm.parm.capture.timeperframe.numerator;
	parm_fps = parm->parm.capture.timeperframe.denominator
			/ parm->parm.capture.timeperframe.numerator;

	AV_DEBUG("   Current capabilities: 0x%x", currentparm.parm.capture.capability);
	AV_DEBUG("   Current capturemode: %d. Change to %d",	currentparm.parm.capture.capturemode, parm->parm.capture.capturemode);
	AV_DEBUG("   Current framerate: is %d. Change to %d", current_fps, parm_fps);

	/* This will change any camera settings needed. */
	err = vidioc_int_s_parm(cam->sensor, parm);
	if (err) {
		AV_ERR("vidioc_int_s_parm error. (Status=%d)", err);
		goto exit;
	}

	/* If resolution changed, need to re-program the CSI */
	/* Get new values. */
#ifndef WANDBOARD_IMX6
	setup_ifparm(cam, 0);
#else
	vidioc_int_g_ifparm(cam->sensor, &ifparm);

	csi_param.data_width = 0;
	csi_param.clk_mode = 0;
	csi_param.ext_vsync = 0;
	csi_param.Vsync_pol = 0;
	csi_param.Hsync_pol = 0;
	csi_param.pixclk_pol = 0;
	csi_param.data_pol = 0;
	csi_param.sens_clksrc = 0;
	csi_param.pack_tight = 0;
	csi_param.force_eof = 0;
	csi_param.data_en_pol = 0;
	csi_param.data_fmt = 0;
	csi_param.csi = cam->csi;
	csi_param.mclk = 0;

	pr_debug("   clock_curr=mclk=%d\n", ifparm.u.bt656.clock_curr);
	if (ifparm.u.bt656.clock_curr == 0)
		csi_param.clk_mode = IPU_CSI_CLK_MODE_CCIR656_INTERLACED;
	else
		csi_param.clk_mode = IPU_CSI_CLK_MODE_GATED_CLK;

	csi_param.pixclk_pol = ifparm.u.bt656.latch_clk_inv;

	if (ifparm.u.bt656.mode == V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT) {
		csi_param.data_width = IPU_CSI_DATA_WIDTH_8;
	} else if (ifparm.u.bt656.mode
				== V4L2_IF_TYPE_BT656_MODE_NOBT_10BIT) {
		csi_param.data_width = IPU_CSI_DATA_WIDTH_10;
	} else {
		csi_param.data_width = IPU_CSI_DATA_WIDTH_8;
	}

	csi_param.Vsync_pol = ifparm.u.bt656.nobt_vs_inv;
	csi_param.Hsync_pol = ifparm.u.bt656.nobt_hs_inv;
	csi_param.ext_vsync = ifparm.u.bt656.bt_sync_correct;

	/* if the capturemode changed, the size bounds will have changed. */
	cam_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vidioc_int_g_fmt_cap(cam->sensor, &cam_fmt);
	pr_debug("   g_fmt_cap returns widthxheight of input as %d x %d\n",
			cam_fmt.fmt.pix.width, cam_fmt.fmt.pix.height);

	csi_param.data_fmt = cam_fmt.fmt.pix.pixelformat;

	cam->crop_bounds.top = cam->crop_bounds.left = 0;
	cam->crop_bounds.width = cam_fmt.fmt.pix.width;
	cam->crop_bounds.height = cam_fmt.fmt.pix.height;

	/*
	 * Set the default current cropped resolution to be the same with
	 * the cropping boundary(except for tvin module).
	 */
	if (cam->device_type != 1) {
		cam->crop_current.width = cam->crop_bounds.width;
		cam->crop_current.height = cam->crop_bounds.height;
	}

	/* This essentially loses the data at the left and bottom of the image
	 * giving a digital zoom image, if crop_current is less than the full
	 * size of the image. */
	ipu_csi_set_window_size(cam->ipu, cam->crop_current.width,
				cam->crop_current.height, cam->csi);
	ipu_csi_set_window_pos(cam->ipu, cam->crop_current.left,
			       cam->crop_current.top,
			       cam->csi);
	ipu_csi_init_interface(cam->ipu, cam->crop_bounds.width,
			       cam->crop_bounds.height,
			       cam_fmt.fmt.pix.pixelformat, csi_param);
#endif


exit:
	if (cam->overlay_on == true)
		start_preview(cam);

	return err;
}

/*!
 * V4L2 - mxc_v4l2_s_std function
 *
 * Sets the TV standard to be used.
 *
 * @param cam	      structure cam_data *
 * @param parm	      structure v4l2_streamparm *
 *
 * @return  status    0 success, EINVAL failed
 */
static int mxc_v4l2_s_std(cam_data *cam, v4l2_std_id e)
{
	AV_DEBUG("%Lx", e);

	if (e == V4L2_STD_PAL) {
		AV_DEBUG("   Setting standard to PAL 0x%Lx", V4L2_STD_PAL);
		cam->standard.id = V4L2_STD_PAL;
		video_index = TV_PAL;
	} else if (e == V4L2_STD_NTSC) {
		AV_DEBUG("   Setting standard to NTSC 0x%Lx",
				V4L2_STD_NTSC);
		/* Get rid of the white dot line in NTSC signal input */
		cam->standard.id = V4L2_STD_NTSC;
		video_index = TV_NTSC;
	} else {
		cam->standard.id = V4L2_STD_ALL;
		video_index = TV_NOT_LOCKED;
		AV_ERR("unrecognized std! 0x%Lx (PAL=0x%Lx, NTSC=0x%Lx", e, V4L2_STD_PAL, V4L2_STD_NTSC);
	}

	cam->standard.index = video_index;
	strcpy(cam->standard.name, video_fmts[video_index].name);
	cam->crop_bounds.width = video_fmts[video_index].raw_width;
	cam->crop_bounds.height = video_fmts[video_index].raw_height;
	cam->crop_current.width = video_fmts[video_index].active_width;
	cam->crop_current.height = video_fmts[video_index].active_height;
	cam->crop_current.top = video_fmts[video_index].active_top;
	cam->crop_current.left = video_fmts[video_index].active_left;

	return 0;
}

/*!
 * V4L2 - mxc_v4l2_g_std function
 *
 * Gets the TV standard from the TV input device.
 *
 * @param cam	      structure cam_data *
 *
 * @param e	      structure v4l2_streamparm *
 *
 * @return  status    0 success, EINVAL failed
 */
static int mxc_v4l2_g_std(cam_data *cam, v4l2_std_id *e)
{
	struct v4l2_format tv_fmt;

	if (cam->device_type == 1) {
		/* Use this function to get what the TV-In device detects the
		 * format to be. pixelformat is used to return the std value
		 * since the interface has no vidioc_g_std.*/
		tv_fmt.type = V4L2_BUF_TYPE_PRIVATE;
		vidioc_int_g_fmt_cap(cam->sensor, &tv_fmt);

		/* If the TV-in automatically detects the standard, then if it
		 * changes, the settings need to change. */
		if (cam->standard_autodetect) {
			if (cam->standard.id != tv_fmt.fmt.pix.pixelformat) {
				AV_DEBUG("MVC: mxc_v4l2_g_std: Changing standard");
				mxc_v4l2_s_std(cam, tv_fmt.fmt.pix.pixelformat);
			}
		}

		*e = tv_fmt.fmt.pix.pixelformat;
	}

	return 0;
}

static int mxc_v4l_flush_frames(cam_data *cam, struct v4l2_buffer *buf)
{
	if ((list_empty(&cam->ready_q))  && (list_empty(&cam->working_q))) {
		AV_DEBUG("cam->working_q & cam->ready_q are empty, thus exiting.");
		g_flush_status = flush_done;
		g_streamoff_status = streamoff_done;
		return 0;
	}
 	
	AV_DEBUG("schedule_delayed_work: flush_work");
	schedule_delayed_work(&cam->flush_work, msecs_to_jiffies(1));

	return 0;
}



int access_user_addr(unsigned long addr, void *buf, int len, int write)
{
	struct page *page;
	void *old_buf = buf;
    
	down_read(&current->mm->mmap_sem);
	/* ignore errors, just check how much was sucessfully transfered */
	while (len) {
		int bytes, ret, offset;
		void *maddr;

		ret = get_user_pages(current, current->mm, addr, 1,
				 write, 0, &page, NULL);

		if (ret <= 0)
			break;
		bytes = len;
		offset = addr & (PAGE_SIZE-1);
		if (bytes > PAGE_SIZE-offset)
			bytes = PAGE_SIZE-offset;

		maddr = kmap(page);
		if (write) {
			memcpy(maddr + offset, buf, bytes);
		} else {
			memcpy(buf, maddr + offset, bytes);
		}

		kunmap(page);
		put_page(page);
		len -= bytes;
		buf += bytes;
		addr += bytes;

	}
	up_read(&current->mm->mmap_sem);
	return buf - old_buf;
}

/*!
 * Dequeue one V4L capture buffer
 *
 * @param cam	 structure cam_data *
 * @param buf	 structure v4l2_buffer *
 *
 * @return  status    0 success, EINVAL invalid frame number,
 *		    ETIME timeout, ERESTARTSYS interrupted by user
 */
static int mxc_v4l_dqueue(cam_data *cam, struct v4l2_buffer *buf)
{
	int retval = 0;
	struct mxc_v4l_frame *frame = NULL;
	unsigned long lock_flags;
	int ret = 0;

	//AV_DEBUG("timeout=%d jiffies. HZ=%d", 10 * HZ, HZ);    
	if (!wait_event_interruptible_timeout(cam->enc_queue,
					      cam->enc_counter != 0,
					      10 * HZ)) {
		AV_ERR("v4l2 capture: mxc_v4l_dqueue timeout or no frames to DQUEUE enc_counter 0x%x", cam->enc_counter);
		return -ETIME;
	} else if (signal_pending(current)) {
		AV_ERR("v4l2 capture: mxc_v4l_dqueue() interrupt received");
		return -ERESTARTSYS;
	}

	dq_count++;

	if (down_interruptible(&cam->busy_lock))
		return -EBUSY;

	if (count_init == 0) {
		AV_DEBUG("Starting timer to fire in 1Sec (%ld)", jiffies);
		ret = mod_timer(&timer, jiffies + msecs_to_jiffies(1000));
		if (ret)
			AV_ERR("Error in mod_timer");
		count_init++;
		frame_count = 0;
	}

	spin_lock_irqsave(&cam->dqueue_int_lock, lock_flags);

	cam->enc_counter --;

	AV_DEBUG("g_flush_status=%d, g_streamoff_status=%d, g_flush_frames_status=%d, g_streamoff_frames_status=%d",
		g_flush_status, g_streamoff_status, g_flush_frames_status, g_streamoff_frames_status);

	if (g_streamoff_frames_status == streamoff_frames_complete) {
		AV_DEBUG_STREAM("DQUEUEing the complete frame (NA)");
		frames_count.complete_frames_count++;
		/* We should set this for final streamoff incomplete frames (DQUEUE) */
		g_streamoff_frames_status = streamoff_frames_complete_done;
		frame = list_entry(cam->done_q.next, struct mxc_v4l_frame, queue);
		list_del(cam->done_q.next);
	} else if (g_flush_status == flush_inprogress) {

		if (g_flush_frames_status == flush_frames_unused) {
			AV_DEBUG_STREAM("DQUEUEing the unused frame, flush");
			frame = list_entry(cam->ready_q.next, struct mxc_v4l_frame, queue);
			list_del(cam->ready_q.next);
			list_add_tail(&frame->queue, &cam->done_q);
		}

		if (g_flush_frames_status == flush_frames_incomplete) {
			AV_DEBUG_STREAM("g_flush_frames_status the unused frame, flush");
	/* 		frames_count.incomplete_frames_count++; */
			frame = list_entry(cam->working_q.next, struct mxc_v4l_frame, queue);
			list_del(cam->working_q.next);
			list_add_tail(&frame->queue, &cam->done_q);
		}

	} else if (g_streamoff_status == streamoff_inprogress) {
	/* 		frames_count.incomplete_frames_count++; */
			AV_DEBUG_STREAM("DQUEUEing the incomplete frame, streamoff");
			frame = list_entry(cam->working_q.next, struct mxc_v4l_frame, queue);
			list_del(cam->working_q.next);
			list_add_tail(&frame->queue, &cam->done_q);
		} else {
		frames_count.complete_frames_count++;
		frame_count++;/* FPS */
		AV_DEBUG_STREAM("DQUEUEing the complete frame");
		frame = list_entry(cam->done_q.next, struct mxc_v4l_frame, queue);
		list_del(cam->done_q.next);
	}

	if (frame->buffer.flags & V4L2_BUF_FLAG_DONE) {
		AV_DEBUG_STREAM("Good frame received frame->index=%d", frame->index);
		frame->buffer.flags &= ~V4L2_BUF_FLAG_DONE;
		frame->buffer.flags |= V4L2_BUF_FLAG_VALID;
	} else if (frame->buffer.flags & V4L2_BUF_FLAG_QUEUED) {
		AV_ERR("v4l2 capture: VIDIOC_DQBUF: Buffer not filled. frame->index=%d", frame->index);
		frame->buffer.flags &= ~V4L2_BUF_FLAG_QUEUED;
		frame->buffer.flags |= V4L2_BUF_FLAG_INCOMPLETE;
		retval = -EINVAL;
	} else if ((frame->buffer.flags & 0x7) == V4L2_BUF_FLAG_MAPPED) {
		AV_ERR("v4l2 capture: VIDIOC_DQBUF: Buffer not queued. frame->index=%d", frame->index);
		frame->buffer.flags |= V4L2_BUF_FLAG_UNUSED;
		retval = -EINVAL;
	}

	if (error_count.crc_error_count)
	{
		frame->buffer.flags &= ~V4L2_BUF_FLAG_VALID;
		frame->buffer.flags |= V4L2_BUF_FLAG_INVALID;
	}

	cam->frame[frame->index].buffer.field = cam->device_type ?
				V4L2_FIELD_INTERLACED : V4L2_FIELD_NONE;

	frame->buffer.flags |= V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
#ifdef V4L2_BUF_FLAG_TSTAMP_SRC_EOF
	frame->buffer.flags |= V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
#endif

	buf->bytesused = cam->v2f.fmt.pix.sizeimage;
	buf->index = frame->index;
	buf->flags = frame->buffer.flags;
	buf->sequence = frame_count;					/* Sequence = Number of received frames */

	if (buf->memory & V4L2_MEMORY_MMAP) {
		buf->m = cam->frame[frame->index].buffer.m;
		buf->length = cam->frame[frame->index].buffer.length;
	}
	else
	if (buf->memory & V4L2_MEMORY_USERPTR) {
		buf->m.userptr = cam->userptr[frame->index].userp;
		buf->length = cam->userptr[frame->index].length;
	}

	buf->timestamp = cam->frame[frame->index].buffer.timestamp;
	buf->field = cam->frame[frame->index].buffer.field;
	spin_unlock_irqrestore(&cam->dqueue_int_lock, lock_flags);
	up(&cam->busy_lock);

	return retval;
}


static void power_down_callback(struct work_struct *work)
{
	cam_data *cam;
	cam = container_of(work, struct _cam_data, power_down_work.work);

	down(&cam->busy_lock);
	if (!cam->open_count) {
		AV_DEBUG("ipu%d/csi%d", cam->ipu_id, cam->csi);
		vidioc_int_s_power(cam->sensor, 0);
		cam->power_on = 0;
	}
	up(&cam->busy_lock);
}

/* cam->busy_lock is held */
void power_up_camera(cam_data *cam)
{
	int ret = 0;

	if (cam->power_on) {
		AV_DEBUG("cancel_delayed_work: power_down_work");
		cancel_delayed_work(&cam->power_down_work);
		return;
	}
	vidioc_int_s_power(cam->sensor, 1);
	vidioc_int_init(cam->sensor);
	ret = vidioc_int_dev_init(cam->sensor);

	if (ret == -EINVAL) {
		AV_ERR("Init failed!");
		cam->power_on = 0;
		return;
	}

	cam->power_on = 1;
}


void power_off_camera(cam_data *cam)
{
	AV_DEBUG("schedule_delayed_work: power_down_work");
	schedule_delayed_work(&cam->power_down_work, (HZ * 2));
}

int mxc_cam_select_input(cam_data *cam, int index)
{
	int retval = -EINVAL;

/*  Always use "CSI<->MEM" channel. */
	if (strcmp(mxc_capture_inputs[index].name, "CSI MEM") == 0)
		retval = csi_enc_select(cam);
	else if (strcmp(mxc_capture_inputs[index].name, "CSI IC MEM") == 0)
		retval = csi_enc_select(cam);

	if (retval) {
		AV_ERR("error(%d) setting input=%d", retval, index);
		return retval;
	}
	mxc_capture_inputs[index].status &= ~V4L2_IN_ST_NO_POWER;
	cam->current_input = index;
	AV_DEBUG("input(%d) %s", index, mxc_capture_inputs[index].name);
	return 0;
}

static int get_bcrm_info(cam_data *cam)
{
	if (is_av_camera) {
		int retval = 0;
		struct v4l2_send_command_control ct;

		/* Read the default width from alliedvision camera */
		CLEAR(ct);
		ct.id = V4L2_AV_IMX_CSI2_WIDTH_R;
		retval = mxc_v4l2_send_command(cam, &ct);

		if (retval < 0)
			return -EINVAL;

		g_bcrm_width = ct.value0;

		/* Read the default height from alliedvision camera */
		CLEAR(ct);
		ct.id = V4L2_AV_IMX_CSI2_HEIGHT_R;
		retval = mxc_v4l2_send_command(cam, &ct);

		if (retval < 0)
			return -EINVAL;

		g_bcrm_height = ct.value0;

		/* Read the default pixelformat from alliedvision camera */
		CLEAR(ct);
		ct.id = V4L2_AV_IMX_CSI2_PIXELFORMAT_R;
		retval = mxc_v4l2_send_command(cam, &ct);

		if (retval < 0)
			return -EINVAL;

		g_bcrm_pixelformat = ct.value0;
	}
	return 0;
}

static int update_bcrm_info(cam_data *cam)
{
	int retval = 0;
    
	if (is_av_camera) {
		struct v4l2_send_command_control c;
		struct v4l2_format f;
		char changed;

		/* Write the Width value */
		CLEAR(c);
		c.id = V4L2_AV_IMX_CSI2_WIDTH_W;
		c.value0 = g_bcrm_width;
		retval = mxc_v4l2_send_command(cam, &c);

		if (retval < 0)
			return retval;

		/* Write the Height value */
		CLEAR(c);
		c.id = V4L2_AV_IMX_CSI2_HEIGHT_W;
		c.value0 = g_bcrm_height;
		retval = mxc_v4l2_send_command(cam, &c);

		if (retval < 0)
			return retval;

		/* Write the pixelformat value */
		CLEAR(c);
		c.id = V4L2_AV_IMX_CSI2_PIXELFORMAT_W;
		c.value0 = g_bcrm_pixelformat;
		retval = mxc_v4l2_send_command(cam, &c);

		if (retval < 0)
			return retval;

		f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		f.fmt.pix = cam->v2f.fmt.pix;
		f.fmt.pix.width = g_bcrm_width;
		f.fmt.pix.height = g_bcrm_height;
		f.fmt.pix.pixelformat = cam->v2f.fmt.pix.pixelformat = g_bcrm_pixelformat;

		AV_DEBUG("\"%c%c%c%c\"", 
			(f.fmt.pix.pixelformat >> 0) & 0xff,
			(f.fmt.pix.pixelformat >> 8) & 0xff,
			(f.fmt.pix.pixelformat >> 16) & 0xff,
			(f.fmt.pix.pixelformat >> 24) & 0xff);

		retval = mxc_v4l2_s_fmt(cam, &f);

		/* If resolution changed, need to re-program the CSI */
		/* Get new values. */
		if ((g_bcrm_width != cam->v2f.fmt.pix.width) || (g_bcrm_height != cam->v2f.fmt.pix.height))
			changed = 1;

		if (changed) {
			AV_DEBUG("Calling setup_ifparm function as new resolution is requested!");
			setup_ifparm(cam, 0);
		}
	}
	return retval;
}

unsigned long csi_in_use;

void timer_callback(unsigned long data)
{
	AV_DEBUG("timer callback called (%ld). Total Interrupts %d. FPS %d", jiffies, frame_count, frame_count/(1));
	csi_fps = frame_count/1;
	count_init = 0;/* Restart the timer. */
}

/*!
 * V4L interface - open function
 *
 * @param file	 structure file *
 *
 * @return  status    0 success, ENODEV invalid device instance,
 *	    ENODEV timeout, ERESTARTSYS interrupted by user
 */
static int mxc_v4l_open(struct file *file)
{
	struct video_device *dev = video_devdata(file);
	cam_data *cam = video_get_drvdata(dev);
	int err = 0;
	struct sensor_data *sensor;
	int csi_bit;
	int i;
	struct device videodev = dev->dev;
	struct regmap *gpr = NULL;
    
	if (!cam) {
		AV_ERR("%s cam_data not found!", dev->name);
		return -EBADF;
	}
	if (!cam->sensor) {
		AV_DEBUG("%s no sensor ipu%d/csi%d", dev->name, cam->ipu_id, cam->csi);
		return -EAGAIN;
	}
	if (cam->sensor->type != v4l2_int_type_slave) {
		AV_DEBUG("%s wrong type ipu%d/csi%d, type=%d/%d", dev->name, cam->ipu_id, cam->csi, cam->sensor->type, v4l2_int_type_slave);
		return -EAGAIN;
	}

/* Init */
	for (i = 0; i < MAX_NUM_MXC_SENSORS; i++) {
		if (av_cam_count[i] != 1)
			av_cam_count[i] = -1;
	}

	for (i = 0; i < MAX_NUM_MXC_SENSORS; i++) {
		if (av_cam_count[i] == 1) {
			AV_ERR("Camera is busy with other application!");
			return -EBUSY;
		}
	}

	sensor = cam->sensor->priv;
	if (!sensor) {
		AV_ERR("sensor_data is not found!");
		return -EBADF;
	}
	AV_DEBUG("dev->name=%s ipu%d/csi%d", dev->name, cam->ipu_id, cam->csi);

	down(&cam->busy_lock);

	err = 0;
	if (signal_pending(current))
		goto oops;

	if (cam->open_count++ == 0) {
		csi_bit = (cam->ipu_id << 1) | cam->csi;
		if (test_and_set_bit(csi_bit, &csi_in_use)) {
			AV_ERR("%s CSI already in use", dev->name);
			err = -EBUSY;
			cam->open_count = 0;
			goto oops;
		}
		cam->csi_in_use = 1;

		err = get_bcrm_info(cam);

		if (err < 0) {
			AV_ERR("get_bcrm_info is failed! err %d", err);
			goto oops;
		}

		AV_DEBUG("Calling sysfs_notify for 'availability'");
		sysfs_notify(&videodev.kobj, NULL, "availability");

#ifndef WANDBOARD_IMX6
		gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
		if (!IS_ERR(gpr)) {
			if (of_machine_is_compatible("fsl,imx6q")) {
				if (cam->ipu_id == cam->csi) {
					unsigned shift = 19 + cam->csi;
					unsigned mask = 1 << shift;
					unsigned val = (cam->mipi_camera ? 0 : 1) << shift;

					regmap_update_bits(gpr, IOMUXC_GPR1, mask, val);
				}
			} else if (of_machine_is_compatible("fsl,imx6dl")) {
				unsigned shift = cam->csi * 3;
				unsigned mask = 7 << shift;
				unsigned val = (cam->mipi_camera ? csi_bit : 4) << shift;

				regmap_update_bits(gpr, IOMUXC_GPR13, mask, val);
			}
		} else {
			AV_ERR("failed to find fsl,imx6q-iomux-gpr regmap");
		}
#endif

		wait_event_interruptible(cam->power_queue,
					 cam->low_power == false);

		err = mxc_cam_select_input(cam, cam->current_input);
		if (err)
			err = mxc_cam_select_input(cam, cam->current_input ^ 1);

		cam->enc_counter = 0;
		INIT_LIST_HEAD(&cam->ready_q);
		INIT_LIST_HEAD(&cam->working_q);
		INIT_LIST_HEAD(&cam->done_q);

		setup_timer(&timer, timer_callback, 0);

		frames_count.complete_frames_count = 0;
		frames_count.incomplete_frames_count = 0;
		error_count.crc_error_count = 0;
		error_count.frames_overrun_error_count = 0;

		setup_ifparm(cam, 1);

		num_requested_buffers = 0;

#ifndef WANDBOARD_IMX6
		if (!IS_ERR(sensor->sensor_clk))
			clk_prepare_enable(sensor->sensor_clk);
#endif

		power_up_camera(cam);
	}

	if (!cam->power_on) {
		err = -EINVAL;
		goto oops;
	}

	file->private_data = dev;

	av_cam_count[video_val] = 1;

oops:
	up(&cam->busy_lock);
	return err;
}

/*!
 * V4L interface - close function
 *
 * @param file     struct file *
 *
 * @return	 0 success
 */
static int mxc_v4l_close(struct file *file)
{
	struct video_device *dev = video_devdata(file);
	int err = 0;
	cam_data *cam = video_get_drvdata(dev);
	struct sensor_data *sensor;
	struct v4l2_send_command_control ctrl;
	int i;
	struct device videodev = dev->dev;
  
	if (!cam) {
		AV_ERR("cam_data not found!");
		return -EBADF;
	}

	if (!cam->sensor) {
		AV_ERR("Camera not found!");
		return -EBADF;
	}

	sensor = cam->sensor->priv;
	if (!sensor) {
		AV_ERR("sensor_data not found!");
		return -EBADF;
	}

	down(&cam->busy_lock);

	/* For both GENCP & BCRM modes */
	if (is_av_camera) {
		CLEAR(ctrl);
		ctrl.id = V4L2_AV_IMX_CSI2_CURRENTMODE_R;
		err = mxc_v4l2_send_command(cam, &ctrl);

		if (err < 0)
			AV_ERR("mxc_v4l_close: Reading current mode (GenCP) reg is failed.");

		if (ctrl.value0 == V4L2_AV_IMX_CSI2_BCRM_MODE) {
			AV_DEBUG("Camera is already in BCRM mode.");

			if (cam->capture_on) {
				AV_DEBUG("mxc_v4l_close: Allied Vision STREAMOFF");

				CLEAR(ctrl);
				ctrl.id = V4L2_AV_IMX_CSI2_STREAMOFF_W;
				ctrl.value0 = 1;
				err = mxc_v4l2_send_command(cam, &ctrl);

				if (err < 0)
					AV_ERR("mxc_v4l_close: Allied Vision STREAMOFF failed.");
			}

		} else {
			AV_DEBUG("Camera is in GenCP mode and set back to BCRM mode now");
			CLEAR(ctrl);
			ctrl.id = V4L2_AV_IMX_CSI2_CHANGEMODE_W;
			ctrl.value0 = V4L2_AV_IMX_CSI2_BCRM_MODE;
			err = mxc_v4l2_send_command(cam, &ctrl);
			if (err < 0)
				AV_ERR("mxc_v4l_close: Set back to BCRM mode is failed.");
		}
	}

	/* for the case somebody hit the ctrl C */
	if (cam->overlay_pid == current->pid && cam->overlay_on) {
		err = stop_preview(cam);
		cam->overlay_on = false;
	}

#ifndef WANDBOARD_IMX6
	if (--cam->open_count == 0) {
		err |= mxc_streamoff(cam);
		wake_up_interruptible(&cam->enc_queue);
		if (!IS_ERR(sensor->sensor_clk))
			clk_disable_unprepare(sensor->sensor_clk);
#else
	if (cam->capture_pid == current->pid) {
		err |= mxc_streamoff(cam);
		wake_up_interruptible(&cam->enc_queue);
	}
	if (--cam->open_count == 0) {
		vidioc_int_s_power(cam->sensor, 0);
		clk_disable_unprepare(sensor->sensor_clk);
#endif
		wait_event_interruptible(cam->power_queue,
					 cam->low_power == false);
		AV_DEBUG("mxc_v4l_close: release resource");

		/* Always use CSI<->MEM channel */
		/* Also changed in mxc_cam_select_input() function */
		if (strcmp(mxc_capture_inputs[cam->current_input].name, "CSI MEM") == 0)
			err |= csi_enc_deselect(cam);
		else if (strcmp(mxc_capture_inputs[cam->current_input].name, "CSI IC MEM") == 0)
			err |= csi_enc_deselect(cam);

		if (is_gencp_mode) {
			mxc_free_frame_buf_gencp(cam);
			err = update_bcrm_info(cam);
			AV_DEBUG("GENCP mode (Status=%d)", err);
		} else {
			AV_DEBUG("BCRM mode");
			mxc_free_frame_buf(cam);
		}

		file->private_data = NULL;

		/* capture off */
		wake_up_interruptible(&cam->enc_queue);

		count_mem_allo = 0;
		count_mem_clear = 0;
		g_flush_frames_status = flush_frames_not_initiated;
		g_streamoff_frames_status = streamoff_frames_not_initiated;
		frames_count.complete_frames_count = 0;
		frames_count.incomplete_frames_count = 0;
		error_count.crc_error_count = 0;
		error_count.frames_overrun_error_count = 0;
		AV_DEBUG("cancel_delayed_work: flush_work");
		cancel_delayed_work(&cam->flush_work);
		AV_DEBUG("cancel_delayed_work: streamoff_work");
		cancel_delayed_work(&cam->streamoff_work);

		del_timer(&timer);
		frame_count = count_init = csi_fps = 0;

		for (i = 0; i < MAX_NUM_MXC_SENSORS; i++) {
			if (av_cam_count[i] == 1)
				av_cam_count[i] = -1;
		}

		mxc_free_frames(cam);
		cam->enc_counter++;
		power_off_camera(cam);
		num_requested_buffers = 0;

		cc_count = 0;
		dq_count = 0;

		g_flush_frames_status = flush_frames_not_initiated;
		g_flush_status = flush_done;
		g_streamoff_status = streamoff_done;
		g_streamoff_frames_status = streamoff_frames_not_initiated;
		g_cam_cbf_status = cam_cbf_not_initiated;

		if (cam->csi_in_use) {

			int csi_bit = (cam->ipu_id << 1) | cam->csi;

			clear_bit(csi_bit, &csi_in_use);
			cam->csi_in_use = 0;
		}
	}

	AV_DEBUG("Calling sysfs_notify for 'availability'");
	sysfs_notify(&videodev.kobj, NULL, "availability");

	is_gencp_mode = false;/* BCRM */

	up(&cam->busy_lock);
	return err;
}

#if defined(CONFIG_MXC_IPU_PRP_ENC) || defined(CONFIG_MXC_IPU_CSI_ENC) || \
    defined(CONFIG_MXC_IPU_PRP_ENC_MODULE) || \
    defined(CONFIG_MXC_IPU_CSI_ENC_MODULE)
/*
 * V4L interface - read function
 *
 * @param file       struct file *
 * @param read buf   char *
 * @param count      size_t
 * @param ppos       structure loff_t *
 *
 * @return	   bytes read
 */
static ssize_t mxc_v4l_read(struct file *file, char *buf, size_t count,
			    loff_t *ppos)
{
	int err = 0;
	u8 *v_address[2];
	struct video_device *dev = video_devdata(file);
	cam_data *cam = video_get_drvdata(dev);

	if (down_interruptible(&cam->busy_lock))
		return -EINTR;

	/* Stop the viewfinder */
	if (cam->overlay_on == true)
		stop_preview(cam);

	v_address[0] = dma_alloc_coherent(0,
				       PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage),
				       &cam->still_buf[0],
				       GFP_DMA | GFP_KERNEL);

	v_address[1] = dma_alloc_coherent(0,
				       PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage),
				       &cam->still_buf[1],
				       GFP_DMA | GFP_KERNEL);

	if (!v_address[0] || !v_address[1]) {
		err = -ENOBUFS;
		goto exit0;
	}

	err = prp_still_select(cam);
	if (err != 0) {
		err = -EIO;
		goto exit0;
	}

	cam->still_counter = 0;
	err = cam->csi_start(cam);
	if (err != 0) {
		err = -EIO;
		goto exit1;
	}

	if (!wait_event_interruptible_timeout(cam->still_queue,
					      cam->still_counter != 0,
					      10 * HZ)) {
		AV_ERR("v4l2 capture: mxc_v4l_read timeout counter 0x%x",
		       cam->still_counter);
		err = -ETIME;
		goto exit1;
	}
	err = copy_to_user(buf, v_address[1], cam->v2f.fmt.pix.sizeimage);

exit1:
	prp_still_deselect(cam);

exit0:
	if (v_address[0] != 0)
		dma_free_coherent(0, cam->v2f.fmt.pix.sizeimage, v_address[0],
				  cam->still_buf[0]);
	if (v_address[1] != 0)
		dma_free_coherent(0, cam->v2f.fmt.pix.sizeimage, v_address[1],
				  cam->still_buf[1]);

	cam->still_buf[0] = cam->still_buf[1] = 0;

	if (cam->overlay_on == true)
		start_preview(cam);

	up(&cam->busy_lock);
	if (err < 0) {
		return 1;/* return err; */
	}

	return cam->v2f.fmt.pix.sizeimage - err;
}
#endif

static int mxc_v4l_stream_stat(cam_data *cam, struct v4ls_stats_t *stream_stat)
{
	if (cam->capture_on){
		stream_stat->current_frame_count = fps_count;
		stream_stat->current_frame_interval = diff_time;
	}
	else {
		stream_stat->current_frame_count = 0;
		stream_stat->current_frame_interval = diff_time;
	}

	stream_stat->frames_count = frames_count.complete_frames_count;
	stream_stat->frames_incomplete = frames_count.incomplete_frames_count;
	stream_stat->packet_crc_err = error_count.crc_error_count;
	stream_stat->frames_underrun = error_count.frames_overrun_error_count = (cc_count - dq_count);

	AV_DEBUG("csi_fps=%u", csi_fps);
	AV_DEBUG("stream_stat->current_frame_count=%llu, stream_stat->current_frame_interval=%llu",
		stream_stat->current_frame_count, stream_stat->current_frame_interval);
	AV_DEBUG("stream_stat->frames_count=%llu, stream_stat->frames_incomplete=%llu",
		stream_stat->frames_count, stream_stat->frames_incomplete);
	AV_DEBUG("stream_stat->packet_crc_err=%llu, stream_stat->frames_underrun is %llu", 
		stream_stat->packet_crc_err, stream_stat->frames_underrun);
	return 0;
}

/*!
 * V4L interface - ioctl function
 *
 * @param file       struct file*
 *
 * @param ioctlnr    unsigned int
 *
 * @param arg	void*
 *
 * @return	   0 success, ENODEV for invalid device instance,
 *	   -1 for other errors.
 */
static long mxc_v4l_do_ioctl(struct file *file,
			    unsigned int ioctlnr, void *arg)
{
	struct video_device *dev = video_devdata(file);
	cam_data *cam = video_get_drvdata(dev);
	int retval = 0;

	AV_DEBUG("ioctlnr=0x%x ipu%d/csi%d", ioctlnr, cam->ipu_id, cam->csi);

	wait_event_interruptible(cam->power_queue, cam->low_power == false);
	/* make this _really_ smp-safe */
	if (ioctlnr != VIDIOC_DQBUF)
		if (down_interruptible(&cam->busy_lock))
			return -EBUSY;

	switch (ioctlnr) {


	/*!
	 * V4l2 VIDIOC_R_I2C ioctl
	 */

	case VIDIOC_R_I2C: {
		struct v4l2_i2c *i2c_reg = arg;
		//is_gencp_mode = true;/* Set GenCP mode */
		AV_DEBUG("   case VIDIOC_R_I2C");
		retval = vidioc_int_gencam_i2cread_reg(cam->sensor, i2c_reg);
		break;
	}


	/*!
	 * V4l2 VIDIOC_W_I2C ioctl
	 */

	case VIDIOC_W_I2C: {
		struct v4l2_i2c *i2c_reg = arg;
		//is_gencp_mode = true;/* Set GenCP mode */
		AV_DEBUG("   case VIDIOC_W_I2C");

		retval = vidioc_int_gencam_i2cwrite_reg(cam->sensor, i2c_reg);

		if (i2c_reg->reg == GENCP_CHANGEMODE_8W)
		{
			is_gencp_mode = (i2c_reg->buffer[0] == OPERATION_MODE_BCRM) ? false : true; 
			AV_DEBUG("Switched to %s", is_gencp_mode ? "GenCP Mode" : "BCRM Mode");
		}

		break;
	}


	/*!
	 * V4l2 VIDIOC_MEM_ALLOC ioctl
	 */

	case VIDIOC_MEM_ALLOC: {
		struct v4l2_dma_mem *dma_mem = arg;
		//is_gencp_mode = true;/* Set GenCP mode */
		AV_DEBUG("   case VIDIOC_MEM_ALLOC");
		AV_DEBUG("cam->v2f.fmt.pix.sizeimage=%d", cam->v2f.fmt.pix.sizeimage);

		if ((dma_mem->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)) {
			AV_ERR("v4l2 capture: VIDIOC_MEM_ALLOC: wrong buffer type");
			retval = -EINVAL;
			break;
		}

		if (count_mem_allo > MAX_NUM_FRAMES) {
			AV_ERR("v4l2 capture: VIDIOC_MEM_ALLOC: exceeded memory allocation");
			retval = -EINVAL;
			break;
		}

		if (dma_mem->memory & V4L2_MEMORY_USERPTR) {
			AV_DEBUG("case V4L2_MEMORY_USERPTR");
			retval = v4l2_dma_alloc_mem_userp(cam, dma_mem);
		}

		if (dma_mem->memory & V4L2_MEMORY_MMAP) {
			AV_DEBUG("case V4L2_MEMORY_MMAP");
			retval = v4l2_dma_alloc_mem(cam, dma_mem);
		}

		break;
	}

	/*!
	 * V4l2 VIDIOC_MEM_FREE ioctl
	 */

	case VIDIOC_MEM_FREE: {
		struct v4l2_dma_mem *dma_mem = arg;
		AV_DEBUG("   case VIDIOC_MEM_FREE");


		if ((dma_mem->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)) {
			AV_ERR("v4l2 capture: VIDIOC_MEM_FREE: wrong buffer type");
			retval = -EINVAL;
			break;
		}

		if (dma_mem->memory & V4L2_MEMORY_MMAP)
			retval = v4l2_dma_alloc_mem_free(cam, dma_mem);
		break;
	}

	/*!
	 * V4l2 VIDIOC_FLUSH_FRAMES ioctl
	 */

	case VIDIOC_FLUSH_FRAMES: {

		int err;
		struct v4l2_buffer *buf;

		uint32_t irq = (cam->csi == 0) ?
		IPU_IRQ_CSI0_OUT_EOF : IPU_IRQ_CSI1_OUT_EOF;
		ipu_free_irq(cam->ipu, irq, cam);/* CSI IRQ0 */

		AV_DEBUG("  case VIDIOC_FLUSH_FRAMES");

		buf = arg;

		if (g_flush_status == flush_inprogress) {
			AV_DEBUG("VIDIOC_FLUSH_FRAMES ioctl is in progress...");
			break;
		}

		g_flush_status = flush_inprogress;

		AV_DEBUG("ipu%d/csi%d capture_on=%d %s", cam->ipu_id,
				cam->csi, cam->capture_on,
				mxc_capture_inputs[cam->current_input].name);

		AV_DEBUG("Cleared IPU interrupt, disable the DMA & CSI");

#ifndef WANDBOARD_IMX6
		/* For both CSI--MEM and CSI--IC--MEM
		 * 1. wait for idmac eof
		 * 2. disable csi first
		 * 3. disable idmac
		 * 4. disable smfc (CSI--MEM channel)
		 */
		if (mxc_capture_inputs[cam->current_input].name != NULL) {
			if (cam->enc_disable_csi) {
				err = cam->enc_disable_csi(cam);
				if (err != 0)
					return err;
			}
			if (cam->enc_disable) {
				err = cam->enc_disable(cam);
				if (err != 0)
					return err;
			}
		}
#endif

		if ((buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)) {
			AV_ERR("v4l2 capture: VIDIOC_FLUSH_FRAMES: wrong buffer type");
			retval = -EINVAL;
			break;

		}

		mxc_v4l_flush_frames(cam, buf);
		break;
	}


	/*!
	 * V4l2 VIDIOC_STREAM_STAT ioctl
	 */

	case VIDIOC_STREAM_STAT: {
		struct v4ls_stats_t *stream_statistics = arg;
		AV_DEBUG("   case VIDIOC_STREAM_STAT");
		retval = mxc_v4l_stream_stat(cam, stream_statistics);
		AV_DEBUG("  case VIDIOC_STREAM_STAT: Return value %d, stream_statistics->current_frame_count %llu, stream_statistics->current_frame_interval %llu", retval, stream_statistics->current_frame_count, stream_statistics->current_frame_interval);
		AV_DEBUG("stream_stat->packet_crc_err is %llu ,stream_stat->frames_underrun is %llu, stream_stat->frames_count %llu", stream_statistics->packet_crc_err, stream_statistics->frames_underrun, stream_statistics->frames_count);

		break;
	}

	/*!
	 * V4l2 VIDIOC_QUERYCAP ioctl
	 */
	case VIDIOC_QUERYCAP: {
		struct v4l2_capability *cap = arg;
		AV_DEBUG("  case VIDIOC_QUERYCAP ");


		if (is_av_camera) {
			/* Read Alliedvision camera's capability */
			retval = vidioc_int_querycap_cap(cam->sensor, cap);
			cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
					    V4L2_CAP_STREAMING;
		} else {
			strcpy(cap->driver, "mxc_v4l2");
			cap->version = KERNEL_VERSION(0, 1, 11);
			cap->card[0] = '\0';
			cap->bus_info[0] = '\0';
			cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
				    V4L2_CAP_VIDEO_OVERLAY |
				    V4L2_CAP_STREAMING |
				    V4L2_CAP_READWRITE;
		}
		break;
	}

	/*!
	 * V4l2 VIDIOC_G_FMT ioctl
	 */
	case VIDIOC_G_FMT: {
		struct v4l2_format *gf = arg;
		AV_DEBUG("   case VIDIOC_G_FMT");
		retval = mxc_v4l2_g_fmt(cam, gf);
		break;
	}

	/*!
	 * V4l2 VIDIOC_TRY_FMT ioctl
	 */
	case VIDIOC_TRY_FMT: {
		struct v4l2_format *sf = arg;
		AV_DEBUG("   case VIDIOC_TRY_FMT");
		retval = vidioc_int_try_fmt(cam->sensor, sf);
		break;
	}

	/*!
	 * V4l2 VIDIOC_S_FMT ioctl
	 */
	case VIDIOC_S_FMT: {
		struct v4l2_format *sf = arg;
		int changed = 0;
		AV_DEBUG("   case VIDIOC_S_FMT");

		if (cam->capture_on) {
			AV_ERR("Can't set the format while camera is streaming");
			retval = -EBUSY;
			break;
		}

		AV_DEBUG("    cam->v2f.fmt.pix.width=%d", cam->v2f.fmt.pix.width);
		AV_DEBUG("    cam->v2f.fmt.pix.height=%d", cam->v2f.fmt.pix.height);
		AV_DEBUG("    sf->fmt.pix.width=%d", sf->fmt.pix.width);
		AV_DEBUG("    sf->fmt.pix.height=%d",sf->fmt.pix.height);
		AV_DEBUG("    sf->fmt.pix.pixelformat=%c%c%c%c", 
			(sf->fmt.pix.pixelformat >> 0) & 0xff,
			(sf->fmt.pix.pixelformat >> 8) & 0xff,
			(sf->fmt.pix.pixelformat >> 16) & 0xff,
			(sf->fmt.pix.pixelformat >> 24) & 0xff);

		AV_DEBUG("    cam->crop_current.width=%d", cam->crop_current.width);
		AV_DEBUG("    cam->crop_current.height=%d", cam->crop_current.height);
		AV_DEBUG("    cam->crop_bounds.width=%d", cam->crop_bounds.width);
		AV_DEBUG("    cam->crop_bounds.height=%d", cam->crop_bounds.height);
		AV_DEBUG("    retval=%d", retval);

#if 0
/* Check the CROP cap */
		if (!is_gencp_mode) {
			if (sf->fmt.pix.width != cam->crop_current.width)
				sf->fmt.pix.width = cam->crop_current.width;

			if (sf->fmt.pix.height != cam->crop_current.height)
				sf->fmt.pix.height = cam->crop_current.height;

			if (sf->fmt.pix.width > cam->crop_bounds.width)
				sf->fmt.pix.width = cam->crop_bounds.width;

			if (sf->fmt.pix.height > cam->crop_bounds.height)
				sf->fmt.pix.height = cam->crop_bounds.height;
		}
#endif

		/* Resolution change via VIDIOC_S_FMT is not allowed according to CRS */
		if ( 	!is_gencp_mode &&
			((sf->fmt.pix.width != cam->v2f.fmt.pix.width) | (sf->fmt.pix.height != cam->v2f.fmt.pix.height)) ) 
		{
			AV_ERR("New Resolution request not allowed via VIDIOC_S_FMT.");
			retval = -EINVAL;
		}
		else
		{
			AV_DEBUG("cam->v2f.fmt.pix.sizeimage=%d",cam->v2f.fmt.pix.sizeimage);
			AV_DEBUG("sf->fmt.pix.sizeimage=%d",sf->fmt.pix.sizeimage);
			cam->v2f.fmt.pix.sizeimage = sf->fmt.pix.sizeimage;
			retval = mxc_v4l2_s_fmt(cam, sf);
			setup_ifparm(cam, 0);
			AV_DEBUG("cam->v2f.fmt.pix.sizeimage=%d",cam->v2f.fmt.pix.sizeimage);
			AV_DEBUG("sf->fmt.pix.sizeimage=%d",sf->fmt.pix.sizeimage);

		}

		/* If resolution changed, need to re-program the CSI */
		/* Get new values. */
/*		if ((sf->fmt.pix.width != cam->v2f.fmt.pix.width) | (sf->fmt.pix.height != cam->v2f.fmt.pix.height)) {
			AV_DEBUG("New Resolution is requested!");
			changed = 1;
		} else
			AV_DEBUG("New Resolution is not requested!");

		retval = mxc_v4l2_s_fmt(cam, sf);
		setup_ifparm(cam, 0);
*/
		break;
	}

	/*!
	 * V4l2 VIDIOC_REQBUFS ioctl
	 */
	case VIDIOC_REQBUFS: {
		struct v4l2_requestbuffers *req = arg;

		AV_DEBUG("   case VIDIOC_REQBUFS req->count=%d", req->count);

		if (req->memory <= 0 || req->type <= 0) {
			retval = -EINVAL;
			break;
		}

		if (req->count == 0) { /* As per spec, need to free the memory if count is 0 */
			mxc_streamoff(cam);
			mxc_free_frame_buf(cam);
			retval = 0;
			break;
		} else if (req->count <= 2) {
			AV_WARNING("v4l2 capture: VIDIOC_REQBUFS: 1 or 2 buffers might lead to lost frames. Try at least 3 buffers.");
		}

		if (req->count > MAX_NUM_FRAMES) {
			AV_WARNING("v4l2 capture: VIDIOC_REQBUFS: Requested too many buffers. Maximum is %d", MAX_NUM_FRAMES);
			req->count = MAX_NUM_FRAMES;
		}

		if ((req->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)) {
			AV_ERR("v4l2 capture: VIDIOC_REQBUFS: wrong buffer type (0x%x)", req->type);
			retval = -EINVAL;
			break;
		}

		mxc_streamoff(cam);

		num_requested_buffers = req->count;
		if (req->memory & V4L2_MEMORY_MMAP) {

			mxc_free_frame_buf(cam);

			retval = mxc_allocate_frame_buf(cam, req->count);
		}


		if (req->memory & V4L2_MEMORY_USERPTR) {
			mxc_free_frame_buf(cam);
			retval = mxc_allocate_frame_buf_userp(cam, req->count);
		}
		break;
	}

	/*!
	 * V4l2 VIDIOC_QUERYBUF ioctl
	 */
	case VIDIOC_QUERYBUF: {
		struct v4l2_buffer *buf = arg;
		int index = buf->index;
		AV_DEBUG("   case VIDIOC_QUERYBUF");

		if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			AV_ERR("v4l2 capture: VIDIOC_QUERYBUFS: wrong buffer type (0x%x)", buf->type);
			retval = -EINVAL;
			break;
		}

		if (!is_gencp_mode)
		{
			if (buf->index >= num_requested_buffers)
			{
				AV_ERR("v4l2 capture: VIDIOC_QUERYBUFS: wrong buffer index. buf->index=%d, num_requested_buffers=%d", buf->index, num_requested_buffers);
				retval = -EINVAL;
				break;	
			}
		}
		
		if (buf->memory & V4L2_MEMORY_MMAP) {
			memset(buf, 0, sizeof(buf));
			buf->index = index;
		}

		down(&cam->param_lock);

		if (buf->memory & V4L2_MEMORY_MMAP) {

			if (is_gencp_mode) {
				AV_DEBUG("GENCP mode");
				retval = mxc_v4l2_buffer_status_gencp(cam, buf);
			} else
				retval = mxc_v4l2_buffer_status(cam, buf);
		}

		up(&cam->param_lock);
		break;
	}

	/*!
	 * V4l2 VIDIOC_QBUF ioctl
	 */
	case VIDIOC_QBUF: {
		struct v4l2_buffer *buf = arg;
		int index = buf->index;
		int err = 0;
		struct mxc_v4l_frame *frame;
		unsigned long lock_flags;

		AV_DEBUG_STREAM(" case VIDIOC_QBUF, length=%d, cam->capture_on=%d, cam->queued=%d", buf->length, cam->capture_on, cam->queued);
		AV_DEBUG_STREAM("Dropped frames %lld", (cc_count - dq_count));

		if (buf->memory & V4L2_MEMORY_USERPTR) {
			cam->userptr[buf->index].userp = buf->m.userptr;
			cam->userptr[buf->index].index = buf->index;
			cam->userptr[buf->index].length = buf->length;
		}

		spin_lock_irqsave(&cam->queue_int_lock, lock_flags);
		if ((cam->frame[index].buffer.flags & 0x7) ==
		    V4L2_BUF_FLAG_MAPPED) {
			cam->frame[index].buffer.flags |=
			    V4L2_BUF_FLAG_QUEUED;
			list_add_tail(&cam->frame[index].queue,
				      &cam->ready_q);
		} else if (cam->frame[index].buffer.
			   flags & V4L2_BUF_FLAG_QUEUED) {
			AV_ERR("v4l2 capture: VIDIOC_QBUF: buffer already queued");
			retval = -EINVAL;
		} else if (cam->frame[index].buffer.
			   flags & V4L2_BUF_FLAG_DONE) {
			AV_ERR("v4l2 capture: VIDIOC_QBUF: overwrite done buffer.");
			cam->frame[index].buffer.flags &=
			    ~V4L2_BUF_FLAG_DONE;
			cam->frame[index].buffer.flags |=
			    V4L2_BUF_FLAG_QUEUED;
			retval = -EINVAL;
		}

		buf->flags = cam->frame[index].buffer.flags;
		spin_unlock_irqrestore(&cam->queue_int_lock, lock_flags);

		/* STREAMON called before QBUF */
		if (is_gencp_mode) { /* gencp mode */
			AV_DEBUG_STREAM("GENCP mode");
			if (cam->capture_on == true) {
				if (!cam->ipu_enable_csi_called) {/* if STREAMON remaining part is not completed */
					if (cam->enc_update_eba &&
						cam->ready_q.prev == cam->ready_q.next) {
						AV_DEBUG_STREAM("Seems QBUF is called after STREAMON");
						retval = 0;
						goto exit;
					}

					cam->capture_pid = current->pid;

					if (cam->overlay_on == true)
						stop_preview(cam);

					if (cam->enc_enable) {
						err = cam->enc_enable(cam);
						if (err != 0)
							return err;
					}

					spin_lock_irqsave(&cam->queue_int_lock, lock_flags);
					cam->ping_pong_csi = 0;
					cam->local_buf_num = 0;
					if (cam->enc_update_eba) {
						frame =
						    list_entry(cam->ready_q.next, struct mxc_v4l_frame, queue);
						list_del(cam->ready_q.next);
						list_add_tail(&frame->queue, &cam->working_q);
						frame->ipu_buf_num = cam->ping_pong_csi;
						err = cam->enc_update_eba(cam, frame->buffer.m.offset);

						frame =
						    list_entry(cam->ready_q.next, struct mxc_v4l_frame, queue);
						list_del(cam->ready_q.next);
						list_add_tail(&frame->queue, &cam->working_q);
						frame->ipu_buf_num = cam->ping_pong_csi;
						err |= cam->enc_update_eba(cam, frame->buffer.m.offset);
						spin_unlock_irqrestore(&cam->queue_int_lock, lock_flags);
					} else {
						spin_unlock_irqrestore(&cam->queue_int_lock, lock_flags);
						return -EINVAL;
					}

					if (cam->overlay_on == true)
						start_preview(cam);

					if (cam->enc_enable_csi) {
						err = cam->enc_enable_csi(cam);
						if (err != 0)
							return err;
					}

					cam->capture_on = true;
				}
			}
		}
exit:
		AV_DEBUG_STREAM(" case VIDIOC_QBUF done.");
		break;
	}

	/*!
	 * V4l2 VIDIOC_DQBUF ioctl
	 */
	case VIDIOC_DQBUF: {
		struct v4l2_buffer *buf = arg;
		int ret = 0;
		AV_DEBUG_STREAM("   case VIDIOC_DQBUF");		/* Use TRACE here or else we produce just too much log entries */
		if ((cam->enc_counter == 0) &&
			(file->f_flags & O_NONBLOCK)) {
			retval = -EAGAIN;
			break;
		}
		retval = mxc_v4l_dqueue(cam, buf);

		if (buf->memory & V4L2_MEMORY_USERPTR) {
			ret = access_user_addr(cam->userptr[buf->index].userp,
			cam->frame[buf->index].vaddress, cam->userptr[buf->index].length, 1);
			AV_DEBUG_STREAM("Frame size -> %d", ret);
		}

		if (fps_frame_num % fps_count == 0) {
			do_gettimeofday(&tstart);
			first_frame_time = (tstart.tv_sec + tstart.tv_usec);
			if (first_frame) {
				first_frame = 0;
				second_frame_time = first_frame_time;
			} else {
				diff_time = second_frame_time - first_frame_time;
				diff_time = (diff_time >= 0 ? diff_time : - diff_time);
				second_frame_time = first_frame_time;
			}
		}

		fps_frame_num ++;

		break;
	}

	/*!
	 * V4l2 VIDIOC_G_IPU_RESTRICTIONS ioctl
	 */
	case VIDIOC_G_IPU_RESTRICTIONS: {
		struct v4l2_ipu_restrictions *ipu_restrictions_user = arg;
		AV_DEBUG("   case VIDIOC_G_IPU_RESTRICTIONS");
		ipu_restrictions_user->ipu_x.nValid = IPU_X_VALID;
		ipu_restrictions_user->ipu_x.nMin = IPU_X_MIN;
		ipu_restrictions_user->ipu_x.nMax = IPU_X_MAX;
		ipu_restrictions_user->ipu_x.nInc = IPU_X_INC;
		ipu_restrictions_user->ipu_y.nValid = IPU_Y_VALID;
		ipu_restrictions_user->ipu_y.nMin = IPU_Y_MIN;
		ipu_restrictions_user->ipu_y.nMax = IPU_Y_MAX;
		ipu_restrictions_user->ipu_y.nInc = IPU_Y_INC;
		break;
	}

	/*!
	 * V4l2 VIDIOC_G_MIN_ANNOUNCED_FRAMES ioctl
	 */
	case VIDIOC_G_MIN_ANNOUNCED_FRAMES: {
		struct v4l2_min_announced_frames *min_announced_frames = arg;
		AV_DEBUG("   case VIDIOC_G_MIN_ANNOUNCED_FRAMES");
		min_announced_frames->nMinAnnouncedFrames = V4L2_MIN_ANNOUNCED_FRAMES;
		break;
	}

	/*!
	 * V4l2 VIDIOC_G_CSI_HOST_CLK_FREQ ioctl
	 */
	case VIDIOC_G_CSI_HOST_CLK_FREQ: {
		struct v4l2_csi_host_clock_freq_ranges *host_clock_freq_ranges = arg;
		AV_DEBUG("   case VIDIOC_G_CSI_HOST_CLK_FREQ");
		host_clock_freq_ranges->lane_range_1.nValid = LANE_RANGE_1_VALID;
		host_clock_freq_ranges->lane_range_1.nMin = LANE_RANGE_1_MIN;
		host_clock_freq_ranges->lane_range_1.nMax = LANE_RANGE_1_MAX;
		host_clock_freq_ranges->lane_range_2.nValid = LANE_RANGE_2_VALID;
		host_clock_freq_ranges->lane_range_2.nMin = LANE_RANGE_2_MIN;
		host_clock_freq_ranges->lane_range_2.nMax = LANE_RANGE_2_MAX;
		host_clock_freq_ranges->lane_range_3.nValid = LANE_RANGE_3_VALID;
		host_clock_freq_ranges->lane_range_3.nMin = LANE_RANGE_3_MIN;
		host_clock_freq_ranges->lane_range_3.nMax = LANE_RANGE_3_MAX;
		host_clock_freq_ranges->lane_range_4.nValid = LANE_RANGE_4_VALID;
		host_clock_freq_ranges->lane_range_4.nMin = LANE_RANGE_4_MIN;
		host_clock_freq_ranges->lane_range_4.nMax = LANE_RANGE_4_MAX;
		break;
	}

	/*!
	 * V4l2 VIDIOC_G_SUPPORTED_LANE_COUNTS ioctl
	 */
	case VIDIOC_G_SUPPORTED_LANE_COUNTS: {
		struct v4l2_supported_lane_counts *supported_lane_counts = arg;
		AV_DEBUG("   case VIDIOC_G_SUPPORTED_LANE_COUNTS");
/* 		supported_lane_counts->nSupportedLaneCounts = CSI_HOST_SUPPORTED_LANE_COUNT; */
		supported_lane_counts->nSupportedLaneCounts = V4L2_LANE_COUNT_1_LaneSupport | V4L2_LANE_COUNT_2_LaneSupport
				| V4L2_LANE_COUNT_3_LaneSupport | V4L2_LANE_COUNT_4_LaneSupport;
		break;
	}

	/*!
	 * V4l2 VIDIOC_G_GENCP_BUFFER_SIZES ioctl
	 */
	case VIDIOC_G_GENCP_BUFFER_SIZES: {
		struct v4l2_gencp_buffer_sizes *gencp_buffer_sizes = arg;
		AV_DEBUG("   case VIDIOC_G_GENCP_BUFFER_SIZES");
		gencp_buffer_sizes->nGenCPOutBufferSize = read_gencp_out_buffer_size;
		gencp_buffer_sizes->nGenCPInBufferSize = read_gencp_in_buffer_size;
		break;
	}

	/*!
	 * V4l2 VIDIOC_I2C_CLOCK_FREQ ioctl
	 */
	case VIDIOC_G_I2C_CLOCK_FREQ: {
		int *i2c_clk_freq = arg;
		AV_DEBUG("   case VIDIOC_G_I2C_CLOCK_FREQ");
		*i2c_clk_freq = av_cam_i2c_clock_frequency;
		break;
	}

	/*!
	 * VIDIOC_RESET_STREAMSTAT ioctl
	 */
	case VIDIOC_RESET_STREAMSTAT: {
		AV_DEBUG("   case VIDIOC_RESET_STREAMSTAT");

		frames_count.complete_frames_count = 0;
		frames_count.incomplete_frames_count = 0;
		error_count.crc_error_count = 0;
		error_count.frames_overrun_error_count = 0;
		csi_fps = 0;
		cc_count = dq_count = 0;
		break;
	}

	/*!
	 * V4l2 VIDIOC_G_SUPPORTED_DATA_IDENTIFIERS ioctl
	 */
	case VIDIOC_G_SUPPORTED_DATA_IDENTIFIERS: {
		struct v4l2_csi_data_identifiers_inq *data_identifiers_inq = arg;
		AV_DEBUG("   case VIDIOC_G_SUPPORTED_DATA_IDENTIFIERS");
		data_identifiers_inq->nDataIdentifiersInq1 = DATA_IDENTIFIER_INQ_1;
		data_identifiers_inq->nDataIdentifiersInq2 = DATA_IDENTIFIER_INQ_2;
		data_identifiers_inq->nDataIdentifiersInq3 = DATA_IDENTIFIER_INQ_3;
		data_identifiers_inq->nDataIdentifiersInq4 = DATA_IDENTIFIER_INQ_4;
		break;
	}

	/*!
	 * V4l2 VIDIOC_G_STATISTICS_CAPABILITIES ioctl
	 */
	case VIDIOC_G_STATISTICS_CAPABILITIES: {
		struct v4l2_statistics_capabilities *statistics_capabilities = arg;
		AV_DEBUG("   case VIDIOC_G_STATISTICS_CAPABILITIES");
		statistics_capabilities->nStatisticsCapability = V4L2_STATISTICS_CAPABILITY_FrameCount |
				V4L2_STATISTICS_CAPABILITY_FramesIncomplete |
				V4L2_STATISTICS_CAPABILITY_PacketCRCError |
				V4L2_STATISTICS_CAPABILITY_CurrentFrameCount |
				V4L2_STATISTICS_CAPABILITY_CurrentFrameInterval |
				V4L2_STATISTICS_CAPABILITY_FramesUnderrun;
		break;
	}

	/*!
	 * V4l2 VIDIOC_STREAMON_EX ioctl
	 */
	case VIDIOC_STREAMON_EX: {
		struct v4l2_streamon_ex *streamon_ex = arg;
		struct v4l2_format f;
		struct streamon_ex streamon;
		int changed = 0;

		AV_DEBUG("   case VIDIOC_STREAMON_EX");

		//is_gencp_mode = true;/* Set GenCP mode */

		AV_DEBUG("streamon_ex->type %d,streamon_ex->nDataIdentifier 0x%x  streamon_ex->nLaneCount %d", streamon_ex->type, streamon_ex->nDataIdentifier, streamon_ex->nLaneCount);
		AV_DEBUG("streamon_ex->nIPU_X %d, streamon_ex->nIPU_Y %d, streamon_ex->nLaneClockFrequency %d", streamon_ex->nIPU_X, streamon_ex->nIPU_Y, streamon_ex->nLaneClockFrequency);
/*
		f.type = streamon_ex->type;
		f.fmt.pix.width = streamon_ex->nIPU_X;
		f.fmt.pix.height = streamon_ex->nIPU_Y;
		f.fmt.pix.pixelformat = cam->v2f.fmt.pix.pixelformat;
		f.fmt.pix.sizeimage = streamon_ex->nTotalDataSize;
		f.fmt.pix.field = V4L2_FIELD_ANY;
*/

		f.fmt.pix = cam->v2f.fmt.pix;/* Complete copy */
		f.type = streamon_ex->type;
		f.fmt.pix.width = streamon_ex->nIPU_X;
		f.fmt.pix.height = streamon_ex->nIPU_Y;
		f.fmt.pix.sizeimage = streamon_ex->nTotalDataSize;


		lane_count = streamon_ex->nLaneCount;

		streamon.nLaneCount = streamon_ex->nLaneCount;
		/*  Data identifier according to MIPI spec (Bit 0..5=DataType. Bit 6..7=VirtualChannel) */
		streamon.virtualChannel = streamon_ex->nDataIdentifier >> 6;
		/*  Data identifier according to MIPI spec (Bit 0..5=DataType. Bit 6..7=VirtualChannel) */
		streamon.dataType = streamon_ex->nDataIdentifier & 0x3F;/* TODO : Need to use this data type to set the pixelformat */

		AV_DEBUG("streamon.dataType=0x%x, streamon.nLaneCount=%d, streamon.virtualChannel=%d", 
			streamon.dataType, streamon.nLaneCount, streamon.virtualChannel);

		retval = vidioc_int_streamon_ex(cam->sensor, &streamon);

		AV_DEBUG("cam->v2f.fmt.pix.width=%d", cam->v2f.fmt.pix.width);
		AV_DEBUG("cam->v2f.fmt.pix.height=%d", cam->v2f.fmt.pix.height);
		AV_DEBUG("f.fmt.pix.width=%d", f.fmt.pix.width);
		AV_DEBUG("f.fmt.pix.height=%d", f.fmt.pix.height);
		AV_DEBUG("f.fmt.pix.sizeimage=%d", f.fmt.pix.sizeimage);
		AV_DEBUG("f.fmt.pix.pixelformat=%d", f.fmt.pix.pixelformat);

		AV_DEBUG("\"%c%c%c%c\"", 
			(f.fmt.pix.pixelformat >> 0) & 0xff, 
			(f.fmt.pix.pixelformat >> 8) & 0xff, 
			(f.fmt.pix.pixelformat >> 16) & 0xff, 
			(f.fmt.pix.pixelformat >> 24) & 0xff);

		/* If resolution changed, need to re-program the CSI */
		/* Get new values. */
		if ((f.fmt.pix.width != cam->v2f.fmt.pix.width) || (f.fmt.pix.height != cam->v2f.fmt.pix.height)) {
			AV_DEBUG("New Resolution is requested!");
			changed = 1;
		} else
			AV_DEBUG("New Resolution is not requested!");


		retval = mxc_v4l2_s_fmt(cam, &f);

		if (retval < 0) {
			AV_ERR("setting the format failed. (Status=%d)", retval);
			break;
		}

		if (changed) {
			AV_DEBUG("Calling setup_ifparm function as new resolution is requested!");
			setup_ifparm(cam, 0);
		}

		retval = mxc_streamon_ex(cam);

		g_streamoff_status = streamoff_not_initiated;
		g_streamoff_frames_status = streamoff_frames_not_initiated;
		break;
	}

	/*!
	 * V4l2 VIDIOC_STREAMON ioctl
	 */
	case VIDIOC_STREAMON: {
		AV_DEBUG("   case VIDIOC_STREAMON");

		retval = mxc_streamon(cam);
		break;
	}

	/*!
	 * V4l2 VIDIOC_STREAMOFF ioctl
	 */
	case VIDIOC_STREAMOFF: {
		AV_DEBUG("   case VIDIOC_STREAMOFF");

		retval = mxc_streamoff(cam);
		break;
	}

	/*!
	 * V4l2 Customized VIDIOC_STREAMOFF_EX ioctl
	 */
	case VIDIOC_STREAMOFF_EX: {
		struct v4l2_streamoff_ex *streamoff = arg;

#if 0
		uint32_t irq = (cam->csi == 0) ?
		IPU_IRQ_CSI0_OUT_EOF : IPU_IRQ_CSI1_OUT_EOF;

		AV_DEBUG("case VIDIOC_STREAMOFF_EX, cam->enc_counter %d", cam->enc_counter);
		if (!list_empty(&cam->working_q))
			msleep(streamoff->timeout);

		ipu_free_irq(cam->ipu, irq, cam);/* CSI IRQ0 */

		AV_DEBUG("ipu%d/csi%d capture_on=%d %s", cam->ipu_id,
				cam->csi, cam->capture_on,
				mxc_capture_inputs[cam->current_input].name);
		if (cam->capture_on == false)
			return 0;

		/* For both CSI--MEM and CSI--IC--MEM
		 * 1. wait for idmac eof
		 * 2. disable csi first
		 * 3. disable idmac
		 * 4. disable smfc (CSI--MEM channel)
		 */
		if (mxc_capture_inputs[cam->current_input].name != NULL) {
			if (cam->enc_disable_csi) {
				err = cam->enc_disable_csi(cam);
				if (err != 0)
					return err;
			}
			if (cam->enc_disable) {
				err = cam->enc_disable(cam);
				if (err != 0)
					return err;
			}
		}

		mxc_capture_inputs[cam->current_input].status |= V4L2_IN_ST_NO_POWER;
		cam->capture_on = false;
		AV_DEBUG("Cleared the IPU interrupt and disabled the CSI.");
		if (g_streamoff_status == streamoff_inprogress) {
			AV_DEBUG("STREAMOFF is already in progress...");
			break;
		}
#endif

		cam->timeout = streamoff->timeout;
		g_streamoff_status = streamoff_inprogress;

		/* When multiple interrupts called (even we requested single frame) */
		if (cam->enc_counter != 0) {
			AV_DEBUG("Interrupt was raised and DQUEUE is not done, so there is complete frame in done_q which is not DQUEUEd yet.");
			g_streamoff_frames_status = streamoff_frames_complete;
		}

/* TODO: Check if acq_off polling is here necessary too */

		AV_DEBUG("schedule_delayed_work: streamoff_work");
		schedule_delayed_work(&cam->streamoff_work, msecs_to_jiffies(1));

		g_flush_status = flush_done;/* Its for when the app will close immediately after STREAMOFF */
		cam->capture_on = false;
		break;

	}

	/*!
	 * V4l2 VIDIOC_G_CTRL ioctl
	 */
	case VIDIOC_G_CTRL: {
		AV_DEBUG("   case VIDIOC_G_CTRL");
		retval = mxc_v4l2_g_ctrl(cam, arg);
		break;
	}

	/*!
	 * V4l2 VIDIOC_S_CTRL ioctl
	 */
	case VIDIOC_S_CTRL: {
		AV_DEBUG("   case VIDIOC_S_CTRL");
		retval = mxc_v4l2_s_ctrl(cam, arg);
		break;
	}

	/*!
	 * V4l2 VIDIOC_CROPCAP ioctl
	 */
	case VIDIOC_CROPCAP: {
		struct sensor_data *sensor;
		struct v4l2_cropcap *cap = arg;
		AV_DEBUG("   case VIDIOC_CROPCAP");
		if (cap->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
		    cap->type != V4L2_BUF_TYPE_VIDEO_OVERLAY) {
			retval = -EINVAL;
			break;
		}

		if (is_av_camera) {
			if (!cam->sensor) {
				AV_ERR("Internal error, camera not found!");
				return -EBADF;
			}

			sensor = cam->sensor->priv;
			if (!sensor) {
				AV_ERR("Internal error, sensor_data not found!");
				return -EBADF;
			}

			retval = vidioc_int_cropcap(cam->sensor, cap);

			cap->bounds = cam->crop_bounds = sensor->crop_bounds;
			cap->defrect = cam->crop_defrect = sensor->crop_defrect;

			cam->standard.frameperiod.denominator = 1;
			cam->standard.frameperiod.numerator = 1;
			cam->streamparm.parm.capture.timeperframe = cam->standard.frameperiod;

			cap->pixelaspect = cam->standard.frameperiod;
			
			AV_DEBUG("cam->crop_defrect.width=%u", cam->crop_defrect.width);
			AV_DEBUG("cam->crop_defrect.height=%u", cam->crop_defrect.height);
			AV_DEBUG("cam->crop_bounds.width=%u", cam->crop_bounds.width);
			AV_DEBUG("cam->crop_bounds.height=%u", cam->crop_bounds.height);
		} else {
			cap->bounds = cam->crop_bounds;
			cap->defrect = cam->crop_defrect;
		}

		break;
	}

	/*!
	 * V4l2 VIDIOC_G_CROP ioctl
	 */
	case VIDIOC_G_CROP: {
		struct sensor_data *sensor;
		struct v4l2_crop *crop = arg;

		if (!cam->sensor)
			return 0;
		sensor = cam->sensor->priv;
		if (!sensor)
			return 0;

		AV_DEBUG("  case VIDIOC_G_CROP");

		if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
		    crop->type != V4L2_BUF_TYPE_VIDEO_OVERLAY) {
			retval = -EINVAL;
			break;
		}

		if (is_av_camera) {
			
			/* Read Width, Height, Left, Top from camera */
			retval = vidioc_int_g_crop(cam->sensor, crop);

			cam->crop_current = crop->c;
			sensor->crop_current.left = crop->c.left;
			sensor->crop_current.top = crop->c.top;

			break;
		}

		crop->c = cam->crop_current;
		crop->c.left = sensor->crop_current.left;
		crop->c.top = sensor->crop_current.top;

		break;
	}

	/*!
	 * V4l2 VIDIOC_S_CROP ioctl
	 */
	case VIDIOC_S_CROP: {
		struct v4l2_rect *crop_current;
		struct sensor_data *sensor;
		struct v4l2_rect *b = &cam->crop_bounds;
		struct v4l2_crop *crop = arg;
		struct v4l2_format sf;
		struct v4l2_cropcap cap;
		int changed = 0;

		AV_DEBUG("  case VIDIOC_S_CROP");

		if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
		    crop->type != V4L2_BUF_TYPE_VIDEO_OVERLAY) {
			retval = -EINVAL;
			break;
		}

		if (is_av_camera) {
			if (cam->capture_on) {
				AV_ERR("Can't CROP as camera is streaming");
				retval = -EBUSY;
				break;
			}

			if (!cam->sensor)
				return 0;
			sensor = cam->sensor->priv;
			if (!sensor)
				return 0;

			crop_current = &cam->crop_current;
			sensor->crop_current = cam->crop_current;

			// dummy call to VIDIOC_CROPCAP to set the crop bounds
			vidioc_int_cropcap(cam->sensor, &cap);

			cam->crop_bounds = sensor->crop_bounds;

			AV_DEBUG("crop_current->width=%u, crop_current->height=%u", crop_current->width, crop_current->height);

			AV_DEBUG("crop->c.top=%u", crop->c.top);
			AV_DEBUG("crop->c.left=%u", crop->c.left);
			AV_DEBUG("crop->c.width=%u", crop->c.width);
			AV_DEBUG("crop->c.height=%u", crop->c.height);
			AV_DEBUG("b->width=%u", b->width);
			AV_DEBUG("b->height=%u", b->height);  

			if (crop->c.width > cam->crop_bounds.width)
				crop->c.width = cam->crop_bounds.width;
			if (crop->c.height > cam->crop_bounds.height)
				crop->c.height = cam->crop_bounds.height;

			retval = vidioc_int_s_crop(cam->sensor, crop);

			sensor->crop_current = cam->crop_current = crop->c;

			CLEAR(sf);
			sf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			sf.fmt.pix.height = crop->c.height;
			sf.fmt.pix.width = crop->c.width;
			sf.fmt.pix.pixelformat = cam->v2f.fmt.pix.pixelformat;
			sf.fmt.pix.field = V4L2_FIELD_ANY;

			AV_DEBUG("cam->v2f.fmt.pix.width=%d", cam->v2f.fmt.pix.width);
			AV_DEBUG("cam->v2f.fmt.pix.height=%d", cam->v2f.fmt.pix.height);
			AV_DEBUG("retval=%d", retval);
			AV_DEBUG("sf->fmt.pix.width=%d", sf.fmt.pix.width);
			AV_DEBUG("sf->fmt.pix.height=%d", sf.fmt.pix.height);
			AV_DEBUG("cam->crop_current.width=%d", cam->crop_current.width);
			AV_DEBUG("cam->crop_current.height=%d", cam->crop_current.height);
			AV_DEBUG("cam->crop_bounds.width=%d", cam->crop_bounds.width);
			AV_DEBUG("cam->crop_bounds.height=%d", cam->crop_bounds.height);
			AV_DEBUG("cam->crop_current.left=%d", cam->crop_current.left);
			AV_DEBUG("cam->crop_current.top=%d", cam->crop_current.top);

			/* Check the CROP cap */
			if (!is_gencp_mode) {
				if (sf.fmt.pix.width != cam->crop_current.width)
					sf.fmt.pix.width = cam->crop_current.width;

				if (sf.fmt.pix.height != cam->crop_current.height)
					sf.fmt.pix.height = cam->crop_current.height;

				if (sf.fmt.pix.width > cam->crop_bounds.width)
					sf.fmt.pix.width = cam->crop_bounds.width;

				if (sf.fmt.pix.height > cam->crop_bounds.height)
					sf.fmt.pix.height = cam->crop_bounds.height;
			}

			/* If resolution changed, need to re-program the CSI */
			/* Get new values. */
			if ((sf.fmt.pix.width != cam->v2f.fmt.pix.width) | (sf.fmt.pix.height != cam->v2f.fmt.pix.height)) {
				AV_DEBUG("New Resolution is requested!");
				changed = 1;
			} else
				AV_DEBUG("New Resolution is not requested!");

			retval = mxc_v4l2_s_fmt(cam, &sf);
			setup_ifparm(cam, 0);

			AV_DEBUG("crop->c.top=%u", crop->c.top);
			AV_DEBUG("crop->c.left=%u", crop->c.left);
			AV_DEBUG("crop->c.width=%u", crop->c.width);
			AV_DEBUG("crop->c.height=%u",crop->c.height);

		} else {
			crop->c.top = (crop->c.top < b->top) ? b->top
					: crop->c.top;
			if (crop->c.top > b->top + b->height)
				crop->c.top = b->top + b->height - 1;
			if (crop->c.height > b->top + b->height - crop->c.top)
				crop->c.height =
					b->top + b->height - crop->c.top;

			crop->c.left = (crop->c.left < b->left) ? b->left
				: crop->c.left;
			if (crop->c.left > b->left + b->width)
				crop->c.left = b->left + b->width - 1;
			if (crop->c.width > b->left - crop->c.left + b->width)
				crop->c.width =
					b->left - crop->c.left + b->width;

			crop->c.width -= crop->c.width % 8;
			crop->c.left -= crop->c.left % 4;
			cam->crop_current = crop->c;

			AV_DEBUG("Cropping Input to ipu size %d x %d", cam->crop_current.width, cam->crop_current.height);
			ipu_csi_set_window_size(cam->ipu, 
						cam->crop_current.width,
						cam->crop_current.height,
						cam->csi);
			ipu_csi_set_window_pos(cam->ipu, 
						cam->crop_current.left,
						cam->crop_current.top,
						cam->csi);
		}
		break;
	}

	/*!
	 * V4l2 VIDIOC_OVERLAY ioctl
	 */
	case VIDIOC_OVERLAY: {
		int *on = arg;
		AV_DEBUG("   VIDIOC_OVERLAY on=%d", *on);
		if (*on) {
			cam->overlay_on = true;
			cam->overlay_pid = current->pid;
			retval = start_preview(cam);
		}
		if (!*on) {
			retval = stop_preview(cam);
			cam->overlay_on = false;
		}
		break;
	}

	/*!
	 * V4l2 VIDIOC_G_FBUF ioctl
	 */
	case VIDIOC_G_FBUF: {
		struct v4l2_framebuffer *fb = arg;
		struct sensor_data *sensor;
		AV_DEBUG("   case VIDIOC_G_FBUF");

		if (is_av_camera) {
			AV_DEBUG("   case default or not supported");
			retval = -ENOTTY;
			break;	
		}

		sensor = cam->sensor->priv;
		if (!sensor) {
			AV_ERR("Internal error, sensor_data is not found!");
			return -EBADF;
		}

		*fb = cam->v4l2_fb;
		fb->capability = V4L2_FBUF_CAP_EXTERNOVERLAY;

#if LINUX_VERSION_CODE > KERNEL_VERSION(3,14,52)
		fb->fmt.width = sensor->pix.width;
		fb->fmt.height = sensor->pix.height;
		fb->fmt.pixelformat = sensor->pix.pixelformat;
		fb->fmt.bytesperline = sensor->pix.bytesperline;
		fb->fmt.sizeimage = sensor->pix.sizeimage;
		fb->fmt.field = sensor->pix.field;
		fb->fmt.colorspace = sensor->pix.colorspace;
		fb->fmt.priv = sensor->pix.priv;
#else
		fb->fmt = sensor->pix;
#endif
		break;
	}

	/*!
	 * V4l2 VIDIOC_S_FBUF ioctl
	 */
	case VIDIOC_S_FBUF: {
		struct v4l2_framebuffer *fb = arg;
		AV_DEBUG("   case VIDIOC_S_FBUF");
		cam->v4l2_fb = *fb;
		break;
	}

	case VIDIOC_G_PARM: {
		struct v4l2_streamparm *parm = arg;
		AV_DEBUG("   case VIDIOC_G_PARM");
		if (cam->sensor)
			retval = vidioc_int_g_parm(cam->sensor, parm);
		else {
			AV_ERR("v4l2 capture: slave not found!");
			retval = -ENODEV;
		}
		break;
	}

	case VIDIOC_S_PARM:  {
		struct v4l2_streamparm *parm = arg;
		AV_DEBUG("   case VIDIOC_S_PARM");

		if (cam->capture_on) {
			AV_ERR("Can't set frame rate as camera is busy");
			retval = -EBUSY;
			break;
		}

		if (cam->sensor)
			retval = mxc_v4l2_s_param(cam, parm);
		else {
			AV_ERR("v4l2 capture: slave not found!");
			retval = -ENODEV;
		}
		break;
	}

	/* linux v4l2 bug, kernel c0485619 user c0405619 */
	case VIDIOC_ENUMSTD: {
		struct v4l2_standard *e = arg;
		
		if (is_av_camera) {
			AV_DEBUG("VIDIOC_ENUMSTD not supported");
			retval = -ENOTTY;
			break;	
		}

		if (e->index > 0) {
			retval = -EINVAL;
			break;
		}
		*e = cam->standard;
		break;
	}

	case VIDIOC_G_STD: {
		v4l2_std_id *e = arg;
		AV_DEBUG("   case VIDIOC_G_STD");

		if (is_av_camera) {
			AV_DEBUG("VIDIOC_G_STD not supported");
			retval = -ENOTTY;
			break;	
		}

		if (cam->sensor)
			retval = mxc_v4l2_g_std(cam, e);
		else {
			AV_ERR("v4l2 capture: slave not found!");
			retval = -ENODEV;
		}
		break;
	}

	case VIDIOC_S_STD: {
		v4l2_std_id *e = arg;
		
		if (is_av_camera) {
			AV_DEBUG("VIDIOC_G_STD not supported");
			retval = -ENOTTY;
			break;	
		}

		retval = mxc_v4l2_s_std(cam, *e);

		break;
	}

	case VIDIOC_ENUMOUTPUT: {
		struct v4l2_output *output = arg;

		if (is_av_camera) {
			AV_DEBUG("VIDIOC_ENUMOUTPUT not supported");
			retval = -ENOTTY;
			break;	
		}	

		if (output->index >= MXC_V4L2_CAPTURE_NUM_OUTPUTS) {
			retval = -EINVAL;
			break;
		}
		*output = mxc_capture_outputs[output->index];

		break;
	}
	case VIDIOC_G_OUTPUT: {
		int *p_output_num = arg;

		if (is_av_camera) {
			AV_DEBUG("VIDIOC_G_OUTPUT not supported");
			retval = -ENOTTY;
			break;	
		}

		*p_output_num = cam->output;
		break;
	}

	case VIDIOC_S_OUTPUT: {
		int *p_output_num = arg;
		
		if(is_av_camera)
		{			
			AV_DEBUG("VIDIOC_S_OUTPUT not supported");
			retval = -ENOTTY;
			break;
		}
		
		if (*p_output_num >= MXC_V4L2_CAPTURE_NUM_OUTPUTS) {
			retval = -EINVAL;
			break;
		}

		cam->output = *p_output_num;
		break;
	}

	case VIDIOC_ENUMINPUT: {
		struct v4l2_input *input = arg;
		AV_DEBUG("   case VIDIOC_ENUMINPUT");

		if (input->index >= MXC_V4L2_CAPTURE_NUM_INPUTS) {
			retval = -EINVAL;
			break;
		}
		*input = mxc_capture_inputs[input->index];
		break;
	}

	case VIDIOC_G_INPUT: {
		int *index = arg;
		AV_DEBUG("   case VIDIOC_G_INPUT");
		*index = cam->current_input;
		break;
	}

	case VIDIOC_S_INPUT: {
		int index = *(int *)arg;
		AV_DEBUG("   case VIDIOC_S_INPUT(%d)", index);
		if (index >= MXC_V4L2_CAPTURE_NUM_INPUTS) {
			retval = -EINVAL;
			break;
		}

		if (index == cam->current_input)
			break;

		if ((mxc_capture_inputs[cam->current_input].status &
		    V4L2_IN_ST_NO_POWER) == 0) {
			retval = mxc_streamoff(cam);
			if (retval)
				break;
			mxc_capture_inputs[cam->current_input].status |=
							V4L2_IN_ST_NO_POWER;
		}

		retval = mxc_cam_select_input(cam, index);
		break;
	}
	case VIDIOC_ENUM_FMT: {
		struct v4l2_fmtdesc *f = arg;
		AV_DEBUG("   case VIDIOC_ENUM_FMT");

		if (f->type == 0) {
			retval = -EINVAL;
			break;
		}

		if(is_av_camera)
		{
			if(f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			{
				retval = -EINVAL;
				break;
			}
		}

		if (cam->sensor)
			retval = vidioc_int_enum_fmt_cap(cam->sensor, f);
		else {
			AV_ERR("v4l2 capture: slave not found!");
			retval = -ENODEV;
		}
		break;
	}
	case VIDIOC_ENUM_FRAMESIZES: {
		struct v4l2_frmsizeenum *fsize = arg;
		AV_DEBUG("   case VIDIOC_ENUM_FRAMESIZES");
		if (cam->sensor)
			retval = vidioc_int_enum_framesizes(cam->sensor, fsize);
		else {
			AV_ERR("v4l2 capture: slave not found!");
			retval = -ENODEV;
		}
		break;
	}
	case VIDIOC_ENUM_FRAMEINTERVALS: {
		struct v4l2_frmivalenum *fival = arg;
		AV_DEBUG("   case VIDIOC_ENUM_FRAMEINTERVALS");
		if (cam->sensor) {
			retval = vidioc_int_enum_frameintervals(cam->sensor,
								fival);
		} else {
			AV_ERR("v4l2 capture: slave not found!");
			retval = -ENODEV;
		}
		break;
	}
	case VIDIOC_DBG_G_CHIP_IDENT: {
		struct v4l2_dbg_chip_ident *p = arg;
		p->ident = V4L2_IDENT_NONE;
		p->revision = 0;
		AV_DEBUG("   case VIDIOC_DBG_G_CHIP_IDENT");
		if (cam->sensor)
			retval = vidioc_int_g_chip_ident(cam->sensor, (int *)p);
		else {
			AV_ERR("v4l2 capture: slave not found!");
			retval = -ENODEV;
		}
		break;
	}

	case VIDIOC_SEND_COMMAND: {
		AV_DEBUG("   case VIDIOC_SEND_COMMAND");
		retval = mxc_v4l2_send_command(cam, arg);
		break;
	}

	case VIDIOC_QUERYCTRL: {
		struct v4l2_queryctrl *qctrl = arg;
		AV_DEBUG("   case VIDIOC_QUERYCTRL");
		retval = mxc_v4l2_query_ctrl(cam, qctrl);

		AV_DEBUG("case VIDIOC_QUERYCTRL, retval=%d, qctrl->minimum=0x%x, qctrl->maximum=0x%x, qctrl->step=0x%x", retval, qctrl->minimum, qctrl->maximum, qctrl->step);
		break;
	}

	case VIDIOC_QUERY_EXT_CTRL: {
		struct v4l2_query_ext_ctrl *ext_qctrl = arg;
		AV_DEBUG("   case VIDIOC_QUERY_EXT_CTRL\n");

		retval = mxc_v4l2_ext_query_ctrl(cam, ext_qctrl);

		AV_DEBUG("case VIDIOC_QUERY_EXT_CTRL, retval %d, ext_qctrl->minimum 0x%llx, ext_qctrl->maximum 0x%llx, \
			ext_qctrl->step 0x%llx, ext_qctrl->type %d\n", retval, ext_qctrl->minimum, ext_qctrl->maximum, ext_qctrl->step, ext_qctrl->type);
		break;
	}


	case VIDIOC_G_EXT_CTRLS: {
		struct v4l2_ext_controls *g_ext_ctrl = arg;
		AV_DEBUG("   case VIDIOC_G_EXT_CTRLS");

		if ((vidioc_int_g_ext_ctrl(cam->sensor, g_ext_ctrl)) < 0) {
			retval = -EINVAL;
		}

		break;
	}

	case VIDIOC_S_EXT_CTRLS: {
		struct v4l2_ext_controls *s_ext_ctrl = arg;
		AV_DEBUG("   case VIDIOC_S_EXT_CTRLS");

		if ((vidioc_int_s_ext_ctrl(cam->sensor, s_ext_ctrl)) < 0) {
			retval = -EINVAL;
		}

		break;
	}

	case VIDIOC_TRY_EXT_CTRLS: {
		struct v4l2_ext_controls *try_ext_ctrl = arg;
		AV_DEBUG("   case VIDIOC_TRY_EXT_CTRLS");

		if ((vidioc_int_try_ext_ctrl(cam->sensor, try_ext_ctrl)) < 0) {
			retval = -EINVAL;
		}

		break;
	}

	case VIDIOC_QUERYMENU: {
		struct v4l2_querymenu *query_menu = arg;
		AV_DEBUG("   case VIDIOC_QUERYMENU");

		if ((vidioc_int_query_menu(cam->sensor, query_menu)) < 0) {
			retval = -EINVAL;
		}
		break;
	}

	case VIDIOC_G_TUNER:
	case VIDIOC_S_TUNER:
	case VIDIOC_G_FREQUENCY:
	case VIDIOC_S_FREQUENCY:
	default:
		AV_DEBUG("   case default or not supported");
/*		retval = -EINVAL; */
		retval = -ENOTTY;
		break;
	}

	if (ioctlnr != VIDIOC_DQBUF)
		up(&cam->busy_lock);

	return retval;
}

/*
 * V4L interface - ioctl function
 *
 * @return  None
 */
static long mxc_v4l_ioctl(struct file *file, unsigned int cmd,
			 unsigned long arg)
{
	int lock;
	int val = 0;

	lock = mutex_lock_interruptible(&ioctl_mutex);

	if (lock)
		return -EINVAL;

	val = video_usercopy(file, cmd, arg, mxc_v4l_do_ioctl);

	mutex_unlock(&ioctl_mutex);

	return val;
}

/*!
 * V4L interface - mmap function
 *
 * @param file	structure file *
 *
 * @param vma	 structure vm_area_struct *
 *
 * @return status     0 Success, EINTR busy lock error, ENOBUFS remap_page error
 */
static int mxc_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct video_device *dev = video_devdata(file);
	unsigned long size;
	int res = 0;
	cam_data *cam = video_get_drvdata(dev);

  	AV_DEBUG("pgoff=0x%lx, start=0x%lx, end=0x%lx", vma->vm_pgoff, vma->vm_start, vma->vm_end);

	/* make this _really_ smp-safe */
	if (down_interruptible(&cam->busy_lock))
		return -EINTR;

	size = vma->vm_end - vma->vm_start;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start,
			    vma->vm_pgoff, size, vma->vm_page_prot)) {
		AV_ERR("v4l2 capture: mxc_mmap: remap_pfn_range failed");
		res = -ENOBUFS;
		goto mxc_mmap_exit;
	}

	vma->vm_flags &= ~VM_IO;	/* using shared anonymous pages */

mxc_mmap_exit:
	up(&cam->busy_lock);
	return res;
}

DEFINE_MUTEX(poll_mutex);

/*!
 * V4L interface - poll function
 *
 * @param file       structure file *
 *
 * @param wait       structure poll_table_struct *
 *
 * @return  status   POLLIN | POLLRDNORM
 */
static unsigned int mxc_poll(struct file *file, struct poll_table_struct *wait)
{
	struct video_device *dev = video_devdata(file);
	cam_data *cam = video_get_drvdata(dev);
	wait_queue_head_t *queue = NULL;
	int res = 0;
    
	if (down_interruptible(&cam->busy_lock))
		return -EINTR;

	queue = &cam->enc_queue;
	poll_wait(file, queue, wait);

	if (list_empty(&cam->done_q)) {
		up(&cam->busy_lock);
		return res;
	}

	up(&cam->busy_lock);
	return res | POLLIN | POLLRDNORM;

#if 0
	struct video_device *dev = video_devdata(file);
	cam_data *cam = video_get_drvdata(dev);
	wait_queue_head_t *queue = NULL;
	struct mxc_v4l_frame *done_frame;
	int res = 0;

	if (down_interruptible(&cam->busy_lock))
		return -EINTR;

	queue = &cam->enc_queue;
	poll_wait(file, queue, wait);


/* Needed for 'select' and 'poll' */
	if (!list_empty(&cam->done_q))
		done_frame = list_entry(&cam->done_q, struct mxc_v4l_frame, queue);
	else {
		up(&cam->busy_lock);
		return res;
	}

	if (cam->enc_counter > 0) {
		up(&cam->busy_lock);
		return POLLIN | POLLRDNORM;
	}

	up(&cam->busy_lock);
	return res;
#endif
}

/*!
 * This structure defines the functions to be called in this driver.
 */
static struct v4l2_file_operations mxc_v4l_fops = {
	.owner = THIS_MODULE,
	.open = mxc_v4l_open,
	.release = mxc_v4l_close,
	.read = mxc_v4l_read,
#ifdef WANDBOARD_IMX6
	.unlocked_ioctl = mxc_v4l_ioctl,
#else
	.ioctl = mxc_v4l_ioctl,
#endif
	.mmap = mxc_mmap,
	.poll = mxc_poll,
};

static struct video_device mxc_v4l_template = {
	.name = "Mxc Camera",
	.fops = &mxc_v4l_fops,
	.release = video_device_release,
};

/*!
 * This function can be used to release any platform data on closing.
 */
static void camera_platform_release(struct device *device)
{
}

/*!
 * Camera V4l2 callback function.
 *
 * @param mask      u32
 *
 * @param dev       void device structure
 *
 * @return status
 */
static void camera_callback(u32 mask, void *dev)
{
	struct mxc_v4l_frame *done_frame;
	struct mxc_v4l_frame *ready_frame;
	struct timeval cur_time;
	cam_data *cam;
	void *mipi_csi2_info;
	u32 mipi_reg = 0;
	u32 mipi_reg2 = 0;
	u32 mipi_phy_state = 0;

	cc_count++;
	cam = (cam_data *) dev;
	if (cam == NULL)
		return;

	irq_triggered = true;
	AV_DEBUG_STREAM(" camera_callback called!, cam->enc_counter=%d", cam->enc_counter);

	spin_lock(&cam->flush_int_lock);
	spin_lock(&cam->streamoff_int_lock);
	spin_lock(&cam->queue_int_lock);
	spin_lock(&cam->dqueue_int_lock);

	g_cam_cbf_status = cam_cbf_inprogress;

	mipi_csi2_info = mipi_csi2_get_info();
	AV_DEBUG_STREAM("*mipi_csi2_info=0x%p", mipi_csi2_info);

	if (!mipi_csi2_info)
		AV_ERR("Fail to get mipi_csi2_info!");

	mipi_reg = mipi_csi2_read(mipi_csi2_info, MIPI_CSI2_ERR1);
	mipi_reg2 = mipi_csi2_read(mipi_csi2_info, MIPI_CSI2_ERR2);
	mipi_phy_state = mipi_csi2_dphy_status(mipi_csi2_info);

	AV_DEBUG_STREAM("MIPI_CSI2 ERR register status: mipi_ERR1_reg=0x%x. mipi_ERR2_reg2=0x%x", mipi_reg, mipi_reg2);
	AV_DEBUG_STREAM("MIPI_CSI2_PHY_STATE=0x%08x", mipi_phy_state);

	/* Enabled for VirtualChannel 0 */
	if ((mipi_reg & VC0_ERR_CRC) | (mipi_reg2 & VC0_ERR_ECC) | (mipi_reg2 & VC0_ERR_HDRCRC))
		error_count.crc_error_count++;
	else if (mipi_reg & VC0_ERR2_CRC)
		error_count.crc_error_count++;
	if ((mipi_reg & VC0_ERR_FRM) | (mipi_reg & VC0_ERR2_FRM) | (mipi_reg & VC0_ERR_LIN) | (mipi_reg & VC0_ERR2_LIN))
		frames_count.incomplete_frames_count++;

	/* Other virtual channels 1 to 3 */
	/*
		if (mipi_reg & VC1_ERR_CRC)
			error_count.crc_error_count++;
		if (mipi_reg & VC2_ERR_CRC)
			error_count.crc_error_count++;
		if (mipi_reg & VC3_ERR_CRC)
			error_count.crc_error_count++;
	*/
	if (!list_empty(&cam->working_q)) {
		do_gettimeofday(&cur_time);

		done_frame = list_entry(cam->working_q.next,
					struct mxc_v4l_frame,
					queue);

		if (done_frame->ipu_buf_num != cam->local_buf_num)
			goto next;

		/*
		 * Set the current time to done frame buffer's
		 * timestamp. Users can use this information to judge
		 * the frame's usage.
		 */
		done_frame->buffer.timestamp = cur_time;

		if (done_frame->buffer.flags & V4L2_BUF_FLAG_QUEUED) {
			done_frame->buffer.flags |= V4L2_BUF_FLAG_DONE;
			done_frame->buffer.flags &= ~V4L2_BUF_FLAG_QUEUED;

			/* Added to the done queue */
			list_del(cam->working_q.next);
			list_add_tail(&done_frame->queue, &cam->done_q);

			/* Wake up the queue */
			cam->enc_counter ++;
			wake_up_interruptible(&cam->enc_queue);
		} else {
			AV_ERR("v4l2 capture: camera_callback: buffer not queued");
		}
	}

next:

	if (!list_empty(&cam->ready_q)) {
		ready_frame = list_entry(cam->ready_q.next,
					 struct mxc_v4l_frame,
					 queue);
		if (cam->enc_update_eba)
			if (cam->enc_update_eba(
				cam,
				ready_frame->buffer.m.offset) == 0) {
				list_del(cam->ready_q.next);
				list_add_tail(&ready_frame->queue,
					      &cam->working_q);
				ready_frame->ipu_buf_num = cam->local_buf_num;
			}
	} else {
		if (cam->enc_update_eba)
			cam->enc_update_eba(
				cam, cam->dummy_frame.buffer.m.offset);
	}

	cam->local_buf_num = (cam->local_buf_num == 0) ? 1 : 0;

	g_cam_cbf_status = cam_cbf_done;

	spin_unlock(&cam->dqueue_int_lock);
	spin_unlock(&cam->queue_int_lock);
	spin_unlock(&cam->streamoff_int_lock);
	spin_unlock(&cam->flush_int_lock);
	return;
}

/*!
 * initialize cam_data structure
 *
 * @param cam      structure cam_data *
 *
 * @return status  0 Success
 */
static int init_camera_struct(cam_data *cam, struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(mxc_v4l2_dt_ids, &pdev->dev);
	struct device_node *np = pdev->dev.of_node;
	int ipu_id, csi_id, mclk_source;
#ifndef WANDBOARD_IMX6
	int mipi_camera;
	static int camera_id;
#endif
	int ret = 0;
	struct v4l2_device *v4l2_dev;

	ret = of_property_read_u32(np, "ipu_id", &ipu_id);
	if (ret) {
		dev_err(&pdev->dev, "ipu_id missing or invalid\n");
		return ret;
	}

	ret = of_property_read_u32(np, "csi_id", &csi_id);
	if (ret) {
		dev_err(&pdev->dev, "csi_id missing or invalid\n");
		return ret;
	}

	ret = of_property_read_u32(np, "mclk_source", &mclk_source);
	if (ret) {
		dev_err(&pdev->dev, "sensor mclk missing or invalid\n");
		return ret;
	}
#ifndef WANDBOARD_IMX6
	ret = of_property_read_u32(np, "mipi_camera", &mipi_camera);
	if (ret)
		mipi_camera = 0;
#endif

	/* Default everything to 0 */
	memset(cam, 0, sizeof(cam_data));

	/* get devtype to distinguish if the cpu is imx5 or imx6
	 * IMX5_V4L2 specify the cpu is imx5
	 * IMX6_V4L2 specify the cpu is imx6q or imx6sdl
	 */
	if (of_id)
		pdev->id_entry = of_id->data;
	cam->devtype = pdev->id_entry->driver_data;

	cam->ipu = ipu_get_soc(ipu_id);
	if (cam->ipu == NULL) {
		AV_ERR("v4l2 capture: failed to get ipu");
		return -EINVAL;
	} else if (cam->ipu == ERR_PTR(-ENODEV)) {
		AV_ERR("v4l2 capture: get invalid ipu");
		return -ENODEV;
	}

	init_MUTEX(&cam->param_lock);
	init_MUTEX(&cam->busy_lock);

	//AV_DEBUG("INIT_DELAYED_WORK: power_down_work");
	INIT_DELAYED_WORK(&cam->power_down_work, power_down_callback);
	//AV_DEBUG("INIT_DELAYED_WORK: flush_work");
	INIT_DELAYED_WORK(&cam->flush_work, flush_work_handler);
	//AV_DEBUG("INIT_DELAYED_WORK: streamoff_work");
	INIT_DELAYED_WORK(&cam->streamoff_work, streamoff_work_handler);

	cam->video_dev = video_device_alloc();
	if (cam->video_dev == NULL)
		return -ENODEV;

	*(cam->video_dev) = mxc_v4l_template;

	video_set_drvdata(cam->video_dev, cam);
	dev_set_drvdata(&pdev->dev, (void *)cam);
	cam->video_dev->minor = -1;

	v4l2_dev = kzalloc(sizeof(*v4l2_dev), GFP_KERNEL);
	if (!v4l2_dev) {
		dev_err(&pdev->dev, "failed to allocate v4l2_dev structure\n");
		video_device_release(cam->video_dev);
		return -ENOMEM;
	}

	if (v4l2_device_register(&pdev->dev, v4l2_dev) < 0) {
		dev_err(&pdev->dev, "register v4l2 device failed\n");
		video_device_release(cam->video_dev);
		kfree(v4l2_dev);
		return -ENODEV;
	}
	cam->video_dev->v4l2_dev = v4l2_dev;

	init_waitqueue_head(&cam->enc_queue);
	init_waitqueue_head(&cam->still_queue);
	init_waitqueue_head(&cam->flush_queue);

	/* setup cropping */
	cam->crop_bounds.left = 0;
	cam->crop_bounds.width = 640;
	cam->crop_bounds.top = 0;
	cam->crop_bounds.height = 480;
	cam->crop_current = cam->crop_defrect = cam->crop_bounds;
	ipu_csi_set_window_size(cam->ipu, cam->crop_current.width,
				cam->crop_current.height, cam->csi);
	ipu_csi_set_window_pos(cam->ipu, cam->crop_current.left,
				cam->crop_current.top, cam->csi);
	cam->streamparm.parm.capture.capturemode = 0;

	cam->standard.index = 0;
	cam->standard.id = V4L2_STD_UNKNOWN;
	cam->standard.frameperiod.denominator = 30;/* 30fps */
	cam->standard.frameperiod.numerator = 1;
	cam->standard.framelines = 480;
	cam->standard_autodetect = true;
	cam->streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cam->streamparm.parm.capture.timeperframe = cam->standard.frameperiod;
	cam->streamparm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	cam->overlay_on = false;
	cam->capture_on = false;
	cam->v4l2_fb.flags = V4L2_FBUF_FLAG_OVERLAY;

	cam->v2f.fmt.pix.width = 640;
	cam->v2f.fmt.pix.height = 480;
	cam->v2f.fmt.pix.sizeimage = cam->v2f.fmt.pix.width * cam->v2f.fmt.pix.height * 2;
	cam->v2f.fmt.pix.bytesperline = cam->v2f.fmt.pix.width * 2;

	if (is_av_camera) {
		cam->v2f.fmt.pix.width = 1296;
		cam->v2f.fmt.pix.height = 968;
		cam->v2f.fmt.pix.sizeimage = cam->v2f.fmt.pix.width * cam->v2f.fmt.pix.height * 3;
		cam->v2f.fmt.pix.bytesperline = cam->v2f.fmt.pix.width * 3;
		cam->v2f.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
	} else {
		cam->v2f.fmt.pix.width = 640;
		cam->v2f.fmt.pix.height = 480;
		cam->v2f.fmt.pix.sizeimage = cam->v2f.fmt.pix.width * cam->v2f.fmt.pix.height * 2;
		cam->v2f.fmt.pix.bytesperline = cam->v2f.fmt.pix.width * 2;
		cam->v2f.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	}

	cam->win.w.width = 160;
	cam->win.w.height = 160;
	cam->win.w.left = 0;
	cam->win.w.top = 0;

	cam->ipu_id = ipu_id;
	cam->csi = csi_id;
#ifndef WANDBOARD_IMX6
	cam->mipi_camera = mipi_camera;
#endif
	cam->mclk_source = mclk_source;
	cam->mclk_on[cam->mclk_source] = false;

	cam->enc_callback = camera_callback;
	init_waitqueue_head(&cam->power_queue);
	spin_lock_init(&cam->queue_int_lock);
	spin_lock_init(&cam->dqueue_int_lock);
	spin_lock_init(&cam->flush_int_lock);
	spin_lock_init(&cam->streamoff_int_lock);

#ifndef WANDBOARD_IMX6
	cam->dummy_frame.vaddress = dma_alloc_coherent(0,
			       SZ_8M, &cam->dummy_frame.paddress,
			       GFP_DMA | GFP_KERNEL);
	if (cam->dummy_frame.vaddress == 0)
		AV_ERR("v4l2 capture: Allocate dummy frame failed.");
	cam->dummy_frame.buffer.length = SZ_8M;
#endif

	cam->self = kmalloc(sizeof(struct v4l2_int_device), GFP_KERNEL);
	cam->self->module = THIS_MODULE;
#ifndef WANDBOARD_IMX6
	sprintf(cam->self->name, "mxc_v4l2_cap%d", camera_id++);
#else
	sprintf(cam->self->name, "mxc_v4l2_cap%d", cam->csi);
#endif
	cam->self->type = v4l2_int_type_master;
	cam->self->u.master = &mxc_v4l2_master;

	return 0;
}

static ssize_t show_streaming(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct video_device *video_dev = container_of(dev,
						struct video_device, dev);
	cam_data *cam = video_get_drvdata(video_dev);
    
	if (cam->capture_on)
		return sprintf(buf, "stream on\n");
	else
		return sprintf(buf, "stream off\n");
}

static ssize_t show_overlay(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct video_device *video_dev = container_of(dev,
						struct video_device, dev);
	cam_data *cam = video_get_drvdata(video_dev);
    
	if (cam->overlay_on)
		return sprintf(buf, "overlay on\n");
	else
		return sprintf(buf, "overlay off\n");
}

static ssize_t show_csi(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct video_device *video_dev = container_of(dev,
						struct video_device, dev);
	cam_data *cam = video_get_drvdata(video_dev);
    
	if (strcmp(cam->self->name, av_cam_result) == 0)
		return sprintf(buf, "ipu%d_csi%d\n", cam->ipu_id, cam->csi);
	else
		return 0;

}

static ssize_t show_flush(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct video_device *video_dev = container_of(dev,
						struct video_device, dev);
	cam_data *cam = video_get_drvdata(video_dev);
    
	if (strcmp(cam->self->name, av_cam_result) == 0)
		return sprintf(buf, "%d\n", g_flush_status);
	else
		return 0;
}

static ssize_t show_streamoff(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct video_device *video_dev = container_of(dev,
						struct video_device, dev);
	cam_data *cam = video_get_drvdata(video_dev);
    
	if (strcmp(cam->self->name, av_cam_result) == 0)
		return sprintf(buf, "%d\n", g_streamoff_status);
	else
		return 0;
}

static ssize_t availability_func(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct video_device *video_dev = container_of(dev,
						struct video_device, dev);
	cam_data *cam = video_get_drvdata(video_dev);
    
	if (cam->open_count > 0)
		av_cam_open = false;
	else
		av_cam_open = true;

	if (strcmp(cam->self->name, av_cam_result) == 0)
		return sprintf(buf, "%d\n", av_cam_open);
	else
		return 0;

}

static ssize_t av_cam_if_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct video_device *video_dev = container_of(dev,
						struct video_device, dev);
	cam_data *cam = video_get_drvdata(video_dev);
    
	if (strcmp(cam->self->name, av_cam_result) == 0)
		return sprintf(buf, "%s\n", AV_CAM_SYSFS_NAME);
	else
		return 0;
}

static ssize_t av_bus_info(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct video_device *video_dev = container_of(dev,
						struct video_device, dev);
	cam_data *cam = video_get_drvdata(video_dev);
    
	if (strcmp(cam->self->name, av_cam_result) == 0)
		return sprintf(buf, "%d:%d:0x%x\n", IMX_CAM_PORT_NO, i2c_busno, i2c_address);
	else
		return 0;
}

static ssize_t show_debug_en(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct video_device *video_dev = container_of(dev,
						struct video_device, dev);
	cam_data *cam = video_get_drvdata(video_dev);
    
	if (strcmp(cam->self->name, av_cam_result) == 0)
		return sprintf(buf, "%d\n", debug);
	else
		return 0;
}

static ssize_t store_debug_en(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct video_device *video_dev = container_of(dev,
						struct video_device, dev);
	cam_data *cam = video_get_drvdata(video_dev);
    
	if (strcmp(cam->self->name, av_cam_result) == 0) {
		sscanf(buf, "%d\n", &debug);
		return count;
	}
	else
		return 0;
}

static DEVICE_ATTR(fsl_v4l2_capture_property, S_IRUGO, show_streaming, NULL);
static DEVICE_ATTR(fsl_v4l2_overlay_property, S_IRUGO, show_overlay, NULL);
static DEVICE_ATTR(if_name, S_IRUGO, av_cam_if_name, NULL);
static DEVICE_ATTR(bus_info, S_IRUGO, av_bus_info, NULL);
static DEVICE_ATTR(fsl_csi_property, S_IRUGO, show_csi, NULL);
static DEVICE_ATTR(availability, S_IRUGO, availability_func, NULL);
static DEVICE_ATTR(flush, S_IRUGO, show_flush, NULL);
static DEVICE_ATTR(streamoff, S_IRUGO, show_streamoff, NULL);
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,14,52)
static DEVICE_ATTR(debug_en, 0660, show_debug_en, store_debug_en);
#else
static DEVICE_ATTR(debug_en, S_IRUGO | S_IWUGO, show_debug_en, store_debug_en);
#endif


/*!
 * This function is called to probe the devices if registered.
 *
 * @param   pdev  the device structure used to give information on which device
 *		to probe
 *
 * @return  The function returns 0 on success and -1 on failure.
 */
static int mxc_v4l2_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int device_id = -1;
	int ret;
	cam_data *cam = NULL;

	/* Create cam and initialize it. */
	cam = kmalloc(sizeof(cam_data), GFP_KERNEL);
	if (cam == NULL) {
		AV_ERR("v4l2 capture: failed to register camera");
		return -1;
	}

	init_camera_struct(cam, pdev);
	pdev->dev.release = camera_platform_release;

	/* Set up the v4l2 device and register it*/
	cam->self->priv = cam;
	v4l2_int_device_register(cam->self);

#ifndef WANDBOARD_IMX6
	ret = of_property_read_u32(np, "device_id", &device_id);
	if (ret)
		device_id = -1;

	/* register v4l video device */
	if (video_register_device(cam->video_dev, VFL_TYPE_GRABBER,
			(device_id >= 0) ? device_id : video_nr) < 0) {
#else
	if (video_register_device(cam->video_dev, VFL_TYPE_GRABBER, video_nr)
		< 0) {
#endif
		kfree(cam);
		cam = NULL;
		AV_ERR("v4l2 capture: video_register_device failed");
		return -1;
	}
	AV_DEBUG("   Video device registered: %s #%d",
		 cam->video_dev->name, cam->video_dev->minor);

	if (device_create_file(&cam->video_dev->dev,
			&dev_attr_fsl_v4l2_capture_property))
		dev_err(&pdev->dev, "Error on creating sysfs file"
			" for capture\n");

	if (device_create_file(&cam->video_dev->dev,
			&dev_attr_fsl_v4l2_overlay_property))
		dev_err(&pdev->dev, "Error on creating sysfs file"
			" for overlay\n");

	if (device_create_file(&cam->video_dev->dev,
			&dev_attr_fsl_csi_property))
		dev_err(&pdev->dev, "Error on creating sysfs file"
			" for csi number\n");

	if (device_create_file(&cam->video_dev->dev,
			&dev_attr_availability))
		dev_err(&pdev->dev, "Error on creating sysfs file"
			" for csi number\n");

	if (device_create_file(&cam->video_dev->dev,
			&dev_attr_if_name))
		AV_ERR("Error on creating sysfs file"
			" for capture");

	if (device_create_file(&cam->video_dev->dev,
			&dev_attr_bus_info))
		AV_ERR("Error on creating sysfs file"
			" for capture");

	if (device_create_file(&cam->video_dev->dev,
			&dev_attr_flush))
		AV_ERR("Error on creating sysfs file"
			" for Flush");

	if (device_create_file(&cam->video_dev->dev,
			&dev_attr_streamoff))
		AV_ERR("Error on creating sysfs file"
			" for STREAMOFF");

	if (device_create_file(&cam->video_dev->dev,
			&dev_attr_debug_en))
		AV_ERR("Error on creating sysfs file"
			" for debug_en");

	return 0;
}

/*!
 * This function is called to remove the devices when device unregistered.
 *
 * @param   pdev  the device structure used to give information on which device
 *		to remove
 *
 * @return  The function returns 0 on success and -1 on failure.
 */
static int mxc_v4l2_remove(struct platform_device *pdev)
{
	cam_data *cam = (cam_data *)platform_get_drvdata(pdev);

	if (cam->open_count) {
		AV_ERR("v4l2 capture:camera open - setting ops to NULL");
		return -EBUSY;
	} else {
		struct v4l2_device *v4l2_dev = cam->video_dev->v4l2_dev;
		device_remove_file(&cam->video_dev->dev,
			&dev_attr_fsl_v4l2_capture_property);
		device_remove_file(&cam->video_dev->dev,
			&dev_attr_fsl_v4l2_overlay_property);
		device_remove_file(&cam->video_dev->dev,
			&dev_attr_fsl_csi_property);
		device_remove_file(&cam->video_dev->dev,
			&dev_attr_availability);
		device_remove_file(&cam->video_dev->dev,
			&dev_attr_if_name);
		device_remove_file(&cam->video_dev->dev,
			&dev_attr_bus_info);
		device_remove_file(&cam->video_dev->dev,
			&dev_attr_flush);
		device_remove_file(&cam->video_dev->dev,
			&dev_attr_streamoff);
		device_remove_file(&cam->video_dev->dev,
			&dev_attr_debug_en);

		AV_DEBUG("V4L2 freeing image input device");
		v4l2_int_device_unregister(cam->self);
		video_unregister_device(cam->video_dev);

#ifndef WANDBOARD_IMX6
		if (cam->dummy_frame.vaddress != 0) {
			dma_free_coherent(0, cam->dummy_frame.buffer.length,
					  cam->dummy_frame.vaddress,
					  cam->dummy_frame.paddress);
			cam->dummy_frame.vaddress = 0;
		}
#endif

		if (is_gencp_mode) {
			AV_DEBUG("GENCP mode");
			mxc_free_frame_buf_gencp(cam);
		} else {
			AV_DEBUG("BCRM mode");
			mxc_free_frame_buf(cam);
		}

		kfree(cam);

		v4l2_device_unregister(v4l2_dev);
		kfree(v4l2_dev);
	}

	AV_DEBUG("V4L2 unregistering video");
	return 0;
}

/*!
 * This function is called to put the sensor in a low power state.
 * Refer to the document driver-model/driver.txt in the kernel source tree
 * for more information.
 *
 * @param   pdev  the device structure used to give information on which I2C
 *		to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function returns 0 on success and -1 on failure.
 */
static int mxc_v4l2_suspend(struct platform_device *pdev, pm_message_t state)
{
	cam_data *cam = platform_get_drvdata(pdev);

	if (cam == NULL)
		return -1;

	down(&cam->busy_lock);

	cam->low_power = true;

	if (cam->overlay_on == true)
		stop_preview(cam);
	if (cam->capture_on == true) {
		if (cam->enc_disable_csi)
			cam->enc_disable_csi(cam);

		if (cam->enc_disable)
			cam->enc_disable(cam);
	}

	if (cam->sensor && cam->open_count) {
		if (cam->mclk_on[cam->mclk_source]) {
			ipu_csi_enable_mclk_if (cam->ipu, CSI_MCLK_I2C,
					       cam->mclk_source,
					       false, false);
			cam->mclk_on[cam->mclk_source] = false;
		}
		vidioc_int_s_power(cam->sensor, 0);
	}

	up(&cam->busy_lock);

	return 0;
}

/*!
 * This function is called to bring the sensor back from a low power state.
 * Refer to the document driver-model/driver.txt in the kernel source tree
 * for more information.
 *
 * @param   pdev   the device structure
 *
 * @return  The function returns 0 on success and -1 on failure
 */
static int mxc_v4l2_resume(struct platform_device *pdev)
{
	cam_data *cam = platform_get_drvdata(pdev);

	if (cam == NULL)
		return -1;

	down(&cam->busy_lock);

	cam->low_power = false;
	wake_up_interruptible(&cam->power_queue);

	if (cam->sensor && cam->open_count) {
		if ((cam->overlay_on == true) || (cam->capture_on == true))
			vidioc_int_s_power(cam->sensor, 1);

		if (!cam->mclk_on[cam->mclk_source]) {
			ipu_csi_enable_mclk_if (cam->ipu, CSI_MCLK_I2C,
					       cam->mclk_source,
					       true, true);
			cam->mclk_on[cam->mclk_source] = true;
		}
	}

	if (cam->overlay_on == true)
		start_preview(cam);
	if (cam->capture_on == true) {
		if (cam->enc_enable)
			cam->enc_enable(cam);

		if (cam->enc_enable_csi)
			cam->enc_enable_csi(cam);
	}

	up(&cam->busy_lock);

	return 0;
}

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxc_v4l2_driver = {
	.driver = {
		   .name = "mxc_v4l2_capture",
		   .owner = THIS_MODULE,
		   .of_match_table = mxc_v4l2_dt_ids,
		   },
	.id_table = imx_v4l2_devtype,
	.probe = mxc_v4l2_probe,
	.remove = mxc_v4l2_remove,
	.suspend = mxc_v4l2_suspend,
	.resume = mxc_v4l2_resume,
	.shutdown = NULL,
};

/*!
 * Initializes the camera driver.
 */
static int mxc_v4l2_master_attach(struct v4l2_int_device *slave)
{
	cam_data *cam = slave->u.slave->master->priv;
	struct v4l2_format cam_fmt;
	int i;
	struct sensor_data *sdata = slave->priv;

	AV_DEBUG("slave.name=%s, master.name=%s",
		slave->name, slave->u.slave->master->name);

	if (slave == NULL) {
		AV_ERR("v4l2 capture: slave parameter not valid.");
		return -1;
	}

#ifndef WANDBOARD_IMX6
	if ((sdata->ipu_id != cam->ipu_id) || (sdata->csi != cam->csi) || (sdata->mipi_camera != cam->mipi_camera)) {
		AV_DEBUG("ipu(%d:%d)/csi(%d:%d)/mipi(%d:%d) doesn't match",
			sdata->ipu_id, cam->ipu_id, sdata->csi, cam->csi, sdata->mipi_camera, cam->mipi_camera);
		AV_ERR("v4l2 capture: slave parameter doesn't match with Virtual Channel");
		return -EINVAL;
	}
#else
	if (sdata->csi != cam->csi) {
		AV_ERR("csi doesn't match\n");
		return -1;
	}
#endif

	cam->sensor = slave;

	if (cam->sensor_index < MAX_NUM_MXC_SENSORS) {
		cam->all_sensors[cam->sensor_index] = slave;
		cam->sensor_index++;
	} else {
		AV_ERR("v4l2 capture: slave number exceeds the maximum.");
		return -1;
	}

	for (i = 0; i < cam->sensor_index; i++) {
		//AV_ERR("attached failed: 0x%x", i);
		vidioc_int_dev_exit(cam->all_sensors[i]);
		vidioc_int_s_power(cam->all_sensors[i], 0);

		video_val = slave->u.slave->master->name[(strlen(slave->u.slave->master->name)-1)] - '0';

		if ((strcmp(slave->name, AV_QUERYCAP_CAM_NAME)) == 0) {
			is_av_camera = true;
			sprintf(av_cam_result, "mxc_v4l2_cap%d", video_val);
			AV_DEBUG("Allied Vision camera detected! Camera sysfs name: %s, slave->name: %s, MXC is attached to /dev/video%d and av_cam_result=%s", 
				AV_CAM_SYSFS_NAME, slave->name, video_val, av_cam_result);
			cam->v2f.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
		}
	}

	cam_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vidioc_int_g_fmt_cap(cam->sensor, &cam_fmt);

	/* Used to detect TV in (type 1) vs. camera (type 0)*/
	cam->device_type = cam_fmt.fmt.pix.priv;

	/* Set the input size to the ipu for this device */
	cam->crop_bounds.top = cam->crop_bounds.left = 0;
	cam->crop_bounds.width = cam_fmt.fmt.pix.width;
	cam->crop_bounds.height = cam_fmt.fmt.pix.height;

	/* This also is the max crop size for this device. */
	cam->crop_defrect.top = cam->crop_defrect.left = 0;
	cam->crop_defrect.width = cam_fmt.fmt.pix.width;
	cam->crop_defrect.height = cam_fmt.fmt.pix.height;

	/* At this point, this is also the current image size. */
	cam->crop_current.top = cam->crop_current.left = 0;
	cam->crop_current.width = cam_fmt.fmt.pix.width;
	cam->crop_current.height = cam_fmt.fmt.pix.height;

	AV_DEBUG("v2f pix w=%d, h=%d",
		 cam->v2f.fmt.pix.width, cam->v2f.fmt.pix.height);
	AV_DEBUG("crop_bounds w=%d, h=%d",
		 cam->crop_bounds.width, cam->crop_bounds.height);
	AV_DEBUG("crop_defrect w=%d, h=%d",
		 cam->crop_defrect.width, cam->crop_defrect.height);
	AV_DEBUG("crop_current w=%d, h=%d",
		cam->crop_current.width, cam->crop_current.height);

	AV_DEBUG("ipu%d:/csi%d %s attached %s:%s",
		cam->ipu_id, cam->csi, cam->mipi_camera ? "mipi" : "parallel",
		slave->name, slave->u.slave->master->name);

	i2c_address = sdata->i2c_client->addr;
	i2c_busno = sdata->i2c_client->adapter->nr;

	AV_DEBUG("Allied Vision Camera: i2c_address=0x%x, i2c_busno=0x%x ", i2c_address, i2c_busno);

	if (is_av_camera) {
		power_up_camera(cam);
	}

	AV_DEBUG("###### ATTACHED ######");

	return 0;
}

/*!
 * Disconnects the camera driver.
 */
static void mxc_v4l2_master_detach(struct v4l2_int_device *slave)
{
	unsigned int i;
	cam_data *cam = slave->u.slave->master->priv;

	if (cam->sensor_index > 1) {
		for (i = 0; i < cam->sensor_index; i++) {
			if (cam->all_sensors[i] != slave)
				continue;
			/* Move all the sensors behind this
			 * sensor one step forward
			 */
			for (; i <= MAX_NUM_MXC_SENSORS - 2; i++)
				cam->all_sensors[i] = cam->all_sensors[i+1];
			break;
		}
		/* Point current sensor to the last one */
		cam->sensor = cam->all_sensors[cam->sensor_index - 2];
	} else
		cam->sensor = NULL;

	AV_DEBUG("###### DETACHED ######");

	cam->sensor_index--;
	vidioc_int_dev_exit(slave);
}

#ifndef WANDBOARD_IMX6
DEFINE_MUTEX(camera_common_mutex);

void mxc_camera_common_lock(void)
{
	mutex_lock(&camera_common_mutex);
}
EXPORT_SYMBOL(mxc_camera_common_lock);

void mxc_camera_common_unlock(void)
{
	mutex_unlock(&camera_common_mutex);
}
EXPORT_SYMBOL(mxc_camera_common_unlock);
#endif

/*!
 * Entry point for the V4L2
 *
 * @return  Error code indicating success or failure
 */
static __init int camera_init(void)
{
	u8 err = 0;

	AV_DEBUG("NX debug=%d", debug);

	/* Register the device driver structure. */
	err = platform_driver_register(&mxc_v4l2_driver);
	if (err != 0) {
		AV_ERR("v4l2 capture:camera_init: platform_driver_register failed.");
		return err;
	}
	return err;
}

/*!
 * Exit and cleanup for the V4L2
 */
static void __exit camera_exit(void)
{
	platform_driver_unregister(&mxc_v4l2_driver);
}

module_init(camera_init);
module_exit(camera_exit);

module_param(video_nr, int, 0444);
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("V4L2 capture driver for Mxc based cameras");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("video");
