/* include/linux/logger.h
 *
 * Copyright (C) 2007-2008 Google, Inc.
 * Author: Robert Love <rlove@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_LOGGER_H
#define _LINUX_LOGGER_H

#include <linux/types.h>
#include <linux/ioctl.h>

/**
 * struct logger_entry - defines a single entry that is given to a logger
 * @len:	The length of the payload
 * @hdr_size:	sizeof(struct logger_entry_v2)
 * @pid:	The generating process' process ID
 * @tid:	The generating process' thread ID
 * @sec:	The number of seconds that have elapsed since the Epoch
 * @nsec:	The number of nanoseconds that have elapsed since @sec
 * @euid:	Effective UID of logger
 * @msg:	The message that is to be logged
 *
 * The structure for version 2 of the logger_entry ABI.
 * This structure is returned to userspace if ioctl(LOGGER_SET_VERSION)
 * is called with version >= 2
 */
struct logger_entry {
	__u16		len;
	__u16		hdr_size;
	__s32		pid;
	__u32		tid;
	__u32		sec;
	__u32		nsec;
	__u32       lid;
	kuid_t      uid;
};

/* Header Structure to logd */
typedef struct __attribute__((__packed__)) {
  uint8_t id;
  uint16_t tid;
  struct {
	__u32 tv_sec;
	__u32 tv_nsec;
  } realtime;
} android_log_header_t;

#define LOG_ID_APP  8

#define LOGGER_LOG_DROP	    "log_drop"	    /* droped log messages */
#define LOGGER_LOG_COMMON	"log_common"	/* normal log messages */

#define LOGGER_ENTRY_MAX_PAYLOAD   4076

#endif /* _LINUX_LOGGER_H */
