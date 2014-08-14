/*
 * include/linux/goodix_touch.h
 *
 * Copyright (C) 2011 Goodix, Inc.
 *
 * Author: Felix
 * Date: 2011.04.28
 */

#ifndef 	_LINUX_GOODIX_TOUCH_H
#define		_LINUX_GOODIX_TOUCH_H

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>

#define GOODIX_I2C_NAME "gt828"
#define TOUCH_MAX_HEIGHT 	768
#define TOUCH_MAX_WIDTH	    1024

#define PIO_BASE_ADDRESS             (0x01c20800)
#define PIO_RANGE_SIZE               (0x400)

#define IRQ_EINT21                   (21)
#define IRQ_EINT29                   (29)

#define PIO_INT_STAT_OFFSET          (0x214)
#define PIO_INT_CTRL_OFFSET          (0x210)

#define SHUTDOWN_PORT                ()
#define INT_PORT                     (SW_INT_IRQNO_PIO)

#define TPD_CHIP_VERSION_C_FIRMWARE_BASE  0x5A
#define TPD_CHIP_VERSION_D1_FIRMWARE_BASE 0x7A
#define TPD_CHIP_VERSION_E_FIRMWARE_BASE  0x9A
#define TPD_CHIP_VERSION_D2_FIRMWARE_BASE 0xBA

//set GT801 PLUS trigger mode,只能设置0或1
#define INT_TRIGGER		1	   // 1=rising 0=falling
#define POLL_TIME		10	//actual query spacing interval:POLL_TIME+6

#define GOODIX_MULTI_TOUCH
#ifdef GOODIX_MULTI_TOUCH
	#define MAX_FINGER_NUM	5
#else
	#define MAX_FINGER_NUM	1
#endif

#if defined(INT_PORT)
	#if MAX_FINGER_NUM <= 3
	#define READ_BYTES_NUM 1+2+MAX_FINGER_NUM*5
	#elif MAX_FINGER_NUM == 4
	#define READ_BYTES_NUM 1+28
	#elif MAX_FINGER_NUM == 5
	#define READ_BYTES_NUM 1+34
	#endif
#else
	#define READ_BYTES_NUM 1+34
#endif

/* Notice: This definition used by platform_data.
 * It should be move this struct info to platform head file such as plat/ts.h.
 * If not used in client, it will be NULL in function of goodix_ts_probe.
 */
struct goodix_i2c_platform_data {
	uint32_t gpio_irq;			//IRQ port, use macro such as "gpio_to_irq" to get Interrupt Number.
	uint32_t irq_cfg;			//IRQ port config, must refer to master's Datasheet.
	uint32_t gpio_shutdown;		        //Shutdown port number
	uint32_t shutdown_cfg;		        //Shutdown port config
	uint32_t screen_width;		        //screen width
	uint32_t screen_height;		        //screen height
};

#define READ_TOUCH_ADDR_H 0x0F
#define READ_TOUCH_ADDR_L 0x40

struct goodix_ts_data {
	uint16_t addr;
	uint8_t bad_data;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_reset;		//use RESET flag
	int use_irq;		//use EINT flag
    int gpio_irq;
	int read_mode;		//read moudle mode,20110221 by andrew
	struct hrtimer timer;
	struct work_struct  work;
	char phys[32];
	int retry;
       unsigned int version;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int (*power)(struct goodix_ts_data * ts, int on);
};


//*************************Touchkey Surpport Part*****************************
//#define HAVE_TOUCH_KEY
#ifdef HAVE_TOUCH_KEY
	const uint16_t touch_key_array[]={
									  KEY_MENU,				//MENU
									  KEY_BACK,				//HOME
									  KEY_SEND				//CALL
									 };
	#define MAX_KEY_NUM	 (sizeof(touch_key_array)/sizeof(touch_key_array[0]))
#endif

struct goodix_i2c_rmi_platform_data {
	uint32_t version;	/* Use this entry for panels with */
	//reservation
};

#endif /* _LINUX_GOODIX_TOUCH_H */
