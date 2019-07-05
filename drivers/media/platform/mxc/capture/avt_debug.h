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

#ifndef _AVT_DEBUG_H
#define _AVT_DEBUG_H

#define AV_EMERG( fmt, args ... ) do { pr_emerg( "#AV(emerg): %s:%s(),line:%04d #" fmt "\n", __FILENAME__, __func__, __LINE__, ##args); } while (0)
#define AV_ALERT( fmt, args ... ) do { pr_alert( "#AV(alert): %s:%s(),line:%04d #" fmt "\n", __FILENAME__, __func__, __LINE__, ##args); } while (0)
#define AV_CRIT( fmt, args ... ) do { pr_crit( "#AV(crit): %s:%s(),line:%04d #" fmt "\n", __FILENAME__, __func__, __LINE__, ##args); } while (0)
#define AV_ERR( fmt, args ... ) do { pr_err( "#AV(err): %s:%s(),line:%04d #" fmt "\n", __FILENAME__, __func__, __LINE__, ##args); } while (0)
#define AV_WARNING( fmt, args ... ) do { pr_warning( "#AV(warning): %s:%s(),line:%04d #" fmt "\n", __FILENAME__, __func__, __LINE__, ##args); } while (0)
#define AV_NOTICE( fmt, args ... ) do { pr_notice( "#AV(notice): %s:%s(),line:%04d #" fmt "\n", __FILENAME__, __func__, __LINE__, ##args); } while (0)
#define AV_INFO( fmt, args ... ) do { pr_info( "#AV(info): %s:%s(),line:%04d #" fmt "\n", __FILENAME__, __func__, __LINE__, ##args); } while (0)
#define AV_DEBUG( fmt, args ... ) do { pr_debug( "#AV(debug): %s:%s(),line:%04d #" fmt "\n", __FILENAME__, __func__, __LINE__, ##args); } while (0)
#define AV_DEBUG_STREAM( fmt, args ... ) do { pr_debug_ratelimited( "#AV(debug): %s:%s(),line:%04d #" fmt "\n", __FILENAME__, __func__, __LINE__, ##args); } while (0)

#define AV_EMERG_H( fmt, args ... ) do { pr_emerg( "#AV(emerg): %s:%s(),line:%04d #" fmt "\n", __FILENAME_H__, __func__, __LINE__, ##args); } while (0)
#define AV_ALERT_H( fmt, args ... ) do { pr_alert( "#AV(alert): %s:%s(),line:%04d #" fmt "\n", __FILENAME_H__, __func__, __LINE__, ##args); } while (0)
#define AV_CRIT_H( fmt, args ... ) do { pr_crit( "#AV(crit): %s:%s(),line:%04d #" fmt "\n", __FILENAME_H__, __func__, __LINE__, ##args); } while (0)
#define AV_ERR_H( fmt, args ... ) do { pr_err( "#AV(err): %s:%s(),line:%04d #" fmt "\n", __FILENAME_H__, __func__, __LINE__, ##args); } while (0)
#define AV_WARNING_H( fmt, args ... ) do { pr_warning( "#AV(warning): %s:%s(),line:%04d #" fmt "\n", __FILENAME_H__, __func__, __LINE__, ##args); } while (0)
#define AV_NOTICE_H( fmt, args ... ) do { pr_notice( "#AV(notice): %s:%s(),line:%04d #" fmt "\n", __FILENAME_H__, __func__, __LINE__, ##args); } while (0)
#define AV_INFO_H( fmt, args ... ) do { pr_info( "#AV(info): %s:%s(),line:%04d #" fmt "\n", __FILENAME_H__, __func__, __LINE__, ##args); } while (0)
#define AV_DEBUG_H( fmt, args ... ) do { pr_debug( "#AV(debug): %s:%s(),line:%04d #" fmt "\n", __FILENAME_H__, __func__, __LINE__, ##args); } while (0)
#define AV_DEBUG_STREAM_H( fmt, args ... ) do { pr_debug_ratelimited( "#AV(debug): %s:%s(),line:%04d #" fmt "\n", __FILENAME_H__, __func__, __LINE__, ##args); } while (0)

#endif
