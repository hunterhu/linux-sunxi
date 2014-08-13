#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <asm/io.h>
#include <linux/platform_device.h>
//#include <mach/gpio.h>
#include <linux/irq.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <linux/syscalls.h>
#include <linux/reboot.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/completion.h>
#include <asm/uaccess.h>
#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <plat/sys_config.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/errno.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
    #include <linux/pm.h>
    #include <linux/earlysuspend.h>
#endif

#include "gt828.h"
#include "ctp_platform_ops.h"

#define FOR_TSLIB_TEST
#define PRINT_INT_INFO
#define PRINT_POINT_INFO
#define PRINT_SUSPEND_INFO
#define TEST_I2C_TRANSFER
//#define DEBUG

static int reg_val;
const char *f3x_ts_name="gt828";
static struct workqueue_struct *goodix_wq;
//static uint8_t read_chip_value[3] = {0x0f,0x7d,0};


static short  goodix_read_version(struct goodix_ts_data *ts);


#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len);
static int i2c_write_bytes(struct i2c_client *client,uint8_t *data,int len);

#ifdef DEBUG
int sum = 0;
int access_count = 0;
int int_count = 0;
#endif

//specific tp related macro: need be configured for specific tp
#define CTP_IRQ_NO			(gpio_int_info[0].port_num)

#define CTP_IRQ_MODE			(POSITIVE_EDGE)
#define CTP_NAME			    GOODIX_I2C_NAME
#define TS_RESET_LOW_PERIOD		(15)
#define TS_INITIAL_HIGH_PERIOD  (15)
#define TS_WAKEUP_LOW_PERIOD	(100)
#define TS_WAKEUP_HIGH_PERIOD	(100)
#define SCREEN_MAX_HEIGHT		(screen_max_x)
#define SCREEN_MAX_WIDTH		(screen_max_y)

static void* __iomem gpio_addr = NULL;
static int gpio_int_hdle = 0;
static int gpio_wakeup_hdle = 0;
static int gpio_reset_hdle = 0;
static int gpio_wakeup_enable = 1;
static int gpio_reset_enable = 1;
static user_gpio_set_t  gpio_int_info[1];

static int screen_max_x = 1024;
static int screen_max_y = 768;
static int revert_x_flag = 0;
static int revert_y_flag = 0;
static int exchange_x_y_flag = 0;
static __u32 twi_addr = 0;
static __u32 twi_id = 0;
static int	int_cfg_addr[]={PIO_INT_CFG0_OFFSET,PIO_INT_CFG1_OFFSET,
			PIO_INT_CFG2_OFFSET, PIO_INT_CFG3_OFFSET};


/* Addresses to scan */
union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};

#ifdef PRINT_POINT_INFO
#define print_point_info(fmt, args...)   \
        do{                              \
                printk(fmt, ##args);     \
        }while(0)
#else
#define print_point_info(fmt, args...)   //
#endif

#ifdef PRINT_INT_INFO
#define print_int_info(fmt, args...)     \
        do{                              \
                printk(fmt, ##args);     \
        }while(0)
#else
#define print_int_info(fmt, args...)   //
#endif

/*
 * ctp_get_pendown_state  : get the int_line data state,
 *
 * return value:
 *             return PRESS_DOWN: if down
 *             return FREE_UP: if up,
 *             return 0: do not need process, equal free up.
 */
static int ctp_get_pendown_state(void)
{
	unsigned int reg_val;
	static int state = FREE_UP;

	//get the input port state
	reg_val = readl(gpio_addr + PIOH_DATA);
	//printk("reg_val = %x\n",reg_val);
	if(!(reg_val & (1<<CTP_IRQ_NO))){
		state = PRESS_DOWN;
		print_int_info("pen down. \n");
	}else{ //touch panel is free up
		state = FREE_UP;
		print_int_info("free up. \n");
	}
	return state;
}

/**
 * ctp_clear_penirq - clear int pending
 *
 */
static void ctp_clear_penirq(void)
{
	int reg_val;
	//clear the IRQ_EINT29 interrupt pending
	//printk("clear pend irq pending\n");
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	//writel(reg_val,gpio_addr + PIO_INT_STAT_OFFSET);
	//writel(reg_val&(1<<(CTP_IRQ_NO)),gpio_addr + PIO_INT_STAT_OFFSET);
	if((reg_val = (reg_val&(1<<(CTP_IRQ_NO))))){
		print_int_info("%s: %d. ==CTP_IRQ_NO=\n", __func__, __LINE__);
		writel(reg_val,gpio_addr + PIO_INT_STAT_OFFSET);
	}
	return;
}

/**
 * ctp_set_irq_mode - according sysconfig's subkey "ctp_int_port" to config int port.
 *
 * return value:
 *              0:      success;
 *              others: fail;
 */
static int ctp_set_irq_mode(char *major_key , char *subkey, ext_int_mode int_mode)
{
	int ret = 0;
	__u32 reg_num = 0;
	__u32 reg_addr = 0;
	__u32 reg_val = 0;
	//config gpio to int mode
	pr_info("%s: config gpio to int mode. \n", __func__);
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	gpio_int_hdle = gpio_request_ex(major_key, subkey);
	if(!gpio_int_hdle){
		pr_info("request tp_int_port failed. \n");
		ret = -1;
		goto request_tp_int_port_failed;
	}
	gpio_get_one_pin_status(gpio_int_hdle, gpio_int_info, subkey, 1);
	pr_info("%s, %d: gpio_int_info, port = %d, port_num = %d. \n", __func__, __LINE__, \
		gpio_int_info[0].port, gpio_int_info[0].port_num);
#endif

#ifdef AW_GPIO_INT_API_ENABLE
#else
	pr_info(" INTERRUPT CONFIG\n");
	reg_num = (gpio_int_info[0].port_num)%8;
	reg_addr = (gpio_int_info[0].port_num)/8;
	reg_val = readl(gpio_addr + int_cfg_addr[reg_addr]);
	reg_val &= (~(7 << (reg_num * 4)));
	reg_val |= (int_mode << (reg_num * 4));
	writel(reg_val,gpio_addr+int_cfg_addr[reg_addr]);

	ctp_clear_penirq();

	reg_val = readl(gpio_addr+PIO_INT_CTRL_OFFSET);
	reg_val |= (1 << (gpio_int_info[0].port_num));
	writel(reg_val,gpio_addr+PIO_INT_CTRL_OFFSET);

	udelay(1);
#endif

request_tp_int_port_failed:
	return ret;
}

/**
 * ctp_set_gpio_mode - according sysconfig's subkey "ctp_io_port" to config io port.
 *
 * return value:
 *              0:      success;
 *              others: fail;
 */
static int ctp_set_gpio_mode(void)
{
	//int reg_val;
	int ret = 0;
	//config gpio to io mode
	printk("%s: config gpio to io mode. \n", __func__);
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	gpio_int_hdle = gpio_request_ex("ctp_para", "ctp_io_port");
	if(!gpio_int_hdle){
		printk("request ctp_io_port failed. \n");
		ret = -1;
		goto request_tp_io_port_failed;
	}
#endif
	return ret;

request_tp_io_port_failed:
	return ret;
}

/**
 * ctp_judge_int_occur - whether interrupt occur.
 *
 * return value:
 *              0:      int occur;
 *              others: no int occur;
 */
static int ctp_judge_int_occur(void)
{
	//int reg_val[3];
	int reg_val;
	int ret = -1;

	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	if(reg_val&(1<<(CTP_IRQ_NO))){
		ret = 0;
	}
	return ret;
}

/**
 * ctp_free_platform_resource - corresponding with ctp_init_platform_resource
 *
 */
static void ctp_free_platform_resource(void)
{
	printk("=======%s=========.\n", __func__);
	if(gpio_addr){
		iounmap(gpio_addr);
	}

	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}

	if(gpio_wakeup_hdle){
		gpio_release(gpio_wakeup_hdle, 2);
	}

	if(gpio_reset_hdle){
		gpio_release(gpio_reset_hdle, 2);
	}

	return;
}


/**
 * ctp_init_platform_resource - initialize platform related resource
 * return value: 0 : success
 *               -EIO :  i/o err.
 *
 */
static int ctp_init_platform_resource(void)
{
	int ret = 0;

	gpio_addr = ioremap(PIO_BASE_ADDRESS, PIO_RANGE_SIZE);
	//printk("%s, gpio_addr = 0x%x. \n", __func__, gpio_addr);
	if(!gpio_addr) {
		ret = -EIO;
		goto exit_ioremap_failed;
	}
	//    gpio_wakeup_enable = 1;
	gpio_wakeup_hdle = gpio_request_ex("ctp_para", "ctp_wakeup");
	if(!gpio_wakeup_hdle) {
		pr_warning("%s: tp_wakeup request gpio fail!\n", __func__);
		gpio_wakeup_enable = 0;
	}

	gpio_reset_hdle = gpio_request_ex("ctp_para", "ctp_reset");
	if(!gpio_reset_hdle) {
		pr_warning("%s: tp_reset request gpio fail!\n", __func__);
		gpio_reset_enable = 0;
	}

	return ret;

exit_ioremap_failed:
	ctp_free_platform_resource();
	return ret;
}


/**
 * ctp_fetch_sysconfig_para - get config info from sysconfig.fex file.
 * return value:
 *                    = 0; success;
 *                    < 0; err
 */
static int ctp_fetch_sysconfig_para(void)
{
	int ret = -1;
	int ctp_used = -1;
	char name[I2C_NAME_SIZE];
	script_parser_value_type_t type = SCRIPT_PARSER_VALUE_TYPE_STRING;

	printk("%s. \n", __func__);
	memset(name, 0, I2C_NAME_SIZE);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_used", &ctp_used, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	if(1 != ctp_used){
		pr_err("%s: ctp_unused. \n",  __func__);
		//ret = 1;
		return ret;
	}

	if(SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para", "ctp_name", (int *)(&name), &type, sizeof(name)/sizeof(int))){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	if(strcmp(CTP_NAME, name)){
		pr_err("%s: name %s does not match CTP_NAME. \n", __func__, name);
		pr_err(CTP_NAME);
		//ret = 1;
		return ret;
	}

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err. \n", name);
		goto script_parser_fetch_err;
	}
	//big-endian or small-endian?
	//printk("%s: before: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);
	u_i2c_addr.dirty_addr_buf[0] = twi_addr;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
	printk("%s: after: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);
	//printk("%s: after: ctp_twi_addr is 0x%x, u32_dirty_addr_buf: 0x%hx. u32_dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u32_dirty_addr_buf[0],u32_dirty_addr_buf[1]);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_id", &twi_id, sizeof(twi_id)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err. \n", name);
		goto script_parser_fetch_err;
	}
	printk("%s: ctp_twi_id is %d. \n", __func__, twi_id);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_x", &screen_max_x, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: screen_max_x = %d. \n", __func__, screen_max_x);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_y", &screen_max_y, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: screen_max_y = %d. \n", __func__, screen_max_y);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_x_flag", &revert_x_flag, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: revert_x_flag = %d. \n", __func__, revert_x_flag);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_y_flag", &revert_y_flag, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: revert_y_flag = %d. \n", __func__, revert_y_flag);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_exchange_x_y_flag", &exchange_x_y_flag, 1)){
		pr_err("ft5x_ts: script_parser_fetch err. \n");
		goto script_parser_fetch_err;
	}
	pr_info("%s: exchange_x_y_flag = %d. \n", __func__, exchange_x_y_flag);

	return 0;

script_parser_fetch_err:
	pr_notice("=========script_parser_fetch_err============\n");
	return ret;
}

/**
 * ctp_reset - function
 *
 */
static void ctp_reset(void)
{
	printk("%s. \n", __func__);
	if(gpio_reset_enable){
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_reset_hdle, 0, "ctp_reset")){
			printk("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_RESET_LOW_PERIOD);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_reset_hdle, 1, "ctp_reset")){
			printk("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_INITIAL_HIGH_PERIOD);
	}
}

/**
 * ctp_wakeup - function
 *
 */
static void ctp_wakeup(void)
{
	printk("%s. \n", __func__);
	if(1 == gpio_wakeup_enable){
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_wakeup_hdle, 1, "ctp_wakeup")){
			printk("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_WAKEUP_LOW_PERIOD);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_wakeup_hdle, 0, "ctp_wakeup")){
			printk("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_WAKEUP_HIGH_PERIOD);

	}
	return;
}
/**
 * ctp_detect - Device detection callback for automatic device creation
 * return value:
 *                    = 0; success;
 *                    < 0; err
 */
int ctp_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)){
		printk("====== I2c Check Failed =====\n");
		return -ENODEV;
	}

	if(twi_id == adapter->nr)
	{
		pr_info("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, CTP_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, CTP_NAME, I2C_NAME_SIZE);
		return 0;
	}else{
		return -ENODEV;
	}

#if 0
	if(twi_id == adapter->nr){
		i2c_read_bytes(client,read_chip_value,3);
		pr_info("addr:0x%x,chip_id_value:0x%x\n",client->addr,read_chip_value[2]);

		/* FIXME: Only compare GT828 PID */
		if(read_chip_value[2] == 0x28){
			strlcpy(info->type, CTP_NAME, I2C_NAME_SIZE);
			return 0;
		}
		printk("%s:I2C connection might be something wrong ! \n",__func__);
		return -ENODEV;
	}else{
		return -ENODEV;
	}
#endif
}

static struct ctp_platform_ops ctp_ops = {
	.get_pendown_state = ctp_get_pendown_state,
	.clear_penirq	   = ctp_clear_penirq,
	.set_irq_mode      = ctp_set_irq_mode,
	.set_gpio_mode     = ctp_set_gpio_mode,
	.judge_int_occur   = ctp_judge_int_occur,
	.init_platform_resource = ctp_init_platform_resource,
	.free_platform_resource = ctp_free_platform_resource,
	.fetch_sysconfig_para = ctp_fetch_sysconfig_para,
	.ts_reset =          ctp_reset,
	.ts_wakeup =         ctp_wakeup,
	.ts_detect = ctp_detect,
};

static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret=-1;

	/* address */
	msgs[0].flags=!I2C_M_RD;
	msgs[0].addr=client->addr;
	msgs[0].len=2;
	msgs[0].buf=&buf[0];

	/* buffer */
	msgs[1].flags=I2C_M_RD;
	msgs[1].addr=client->addr;
	msgs[1].len=len-2;
	msgs[1].buf=&buf[2];

	ret=i2c_transfer(client->adapter,msgs, 2);
	return ret;
}

static int i2c_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
	struct i2c_msg msg;
	int ret=-1;

	msg.flags=!I2C_M_RD;
	msg.addr=client->addr;
	msg.len=len;
	msg.buf=data;

	ret=i2c_transfer(client->adapter,&msg, 1);
	return ret;
}

static int i2c_end_cmd(struct goodix_ts_data *ts)
{
	int ret;
	uint8_t end_cmd_data[2]={0};
	end_cmd_data[0]=0x80;
	end_cmd_data[1]=0x00;
	ret=i2c_write_bytes(ts->client,end_cmd_data,2);
//	msleep(2);
	return ret;
}

/*******************************************************
notice: init panel need to be complete within 200ms.
	so, i2c transfer clk more faster more better
	     and do not add too much print info when debug.
*******************************************************/
static int goodix_init_panel(struct goodix_ts_data *ts)
{
	/* FIXME: I have put both array for 1024x768 resolution */
	short ret=-1;
	int i = 0;
	uint8_t info_1024x768[] = {0x04, 0x00, 0x03, 0x00};
	static uint8_t config_info1[114];
	uint8_t config_info_d[] = {
		0x0F,0x80, // 0,1
		0x00,0x0F,0x01,0x10,0x02,0x11,0x03,0x12, // 2,9
		0x04,0x13,0x05,0x14,0x06,0x15,0x07,0x16, // 10,17
		0x08,0x17,0x09,0x18,0x0A,0x19,0x0B,0x1A, // 18,25
		0x0C,0x1B,0x0D,0x1C,0x0E,0x1D,0x13,0x09, // 26,33
		0x12,0x08,0x11,0x07,0x10,0x06,0x0F,0x05, // 34,41
		0x0E,0x04,0x0D,0x03,0x0C,0x02,0x0B,0x01, // 42,49
		0x0A,0x00,0x1F,0x03,0x68,0x10,0x10,0x25, // 50,57
		0x00,0x00,0x0A,0x00,0x00,0x0A,0x50,0x3C, // 58,65
		0x35,0x03,0x00,0x05,0x00,0x03,0x00,0x04, // 66,73
		0x00,0x3A,0x3C,0x3D,0x3E,0x28,0x00,0x06, // 74,81
		0x19,0x02,0x14,0x10,0x00,0x04,0x00,0x00, // 82,89
		0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x20, // 90,97
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // 98,105
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01  // 106,113
	};
	config_info_d[71] = info_1024x768[0]; /* 0xFC5 */
	config_info_d[72] = info_1024x768[1]; /* 72-2=70=0x46+0xF80=0xFC6 */
	config_info_d[73] = info_1024x768[2]; /* 0xFC7 */
	config_info_d[74] = info_1024x768[3]; /* 0xFC8 */

    ret = goodix_read_version(ts);
    if (ret < 0)
		return ret;

	dev_info(&ts->client->dev,"Init Panel...\n");
	ret=i2c_write_bytes(ts->client,config_info_d, (sizeof(config_info_d)/sizeof(config_info_d[0])));

	if (ret < 0)
	{
		return ret;
		dev_info(&ts->client->dev,"Init Panel Fail...\n");
	}

	/* read back the configurations */
	config_info1[0] = 0x0F;
	config_info1[1] = 0x80;
	ret=i2c_read_bytes(ts->client,config_info1,114);
	for ( i = 0; i<114; i++) {
		 if ( 2 == i )
			printk(KERN_DEBUG "\n" );

	     printk(KERN_DEBUG "config_info1[%i] = %x \n ",i, config_info1[i]);

		 if ( i > 2 && 0 == (i-2) % 8 )
			printk (KERN_DEBUG "\n");
	}
	msleep(10);
	return 0;
}

/*******************************************************

*******************************************************/
static short  goodix_read_version(struct goodix_ts_data *ts)
{
	short ret;
	u8 buf[8];
	/* GT828 0xF7D, 0xF7E, 0xF7F contains PID and VID */
	buf[0] = 0x0f;
	buf[1] = 0x7d;

	ret = i2c_read_bytes(ts->client, buf, 5);
	i2c_end_cmd(ts);
	if (ret < 0)
		return ret;

	dev_info(&ts->client->dev,"PID:%02x, VID:%02x%02x\n", buf[2], buf[3], buf[4]);
    ts->version = (buf[3]<<8)+buf[4];
	return ret;
}

/*******************************************************
Function:
	Touch down report function.
Input:
	ts:private data.
	id:tracking id.
	x:input x.
	y:input y.
	w:input weight.
Output:
	None.
*******************************************************/
static void goodix_touch_down(struct goodix_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
	pr_info("source data:ID:%d, X:%d, Y:%d, W:%d\n", id, x, y, w);
	if(1 == exchange_x_y_flag){
		swap(x, y);
	}
	if(1 == revert_x_flag){
		x = SCREEN_MAX_HEIGHT - x;
	}
	if(1 == revert_y_flag){
		y = SCREEN_MAX_WIDTH - y;
	}
	pr_info("report data:ID:%d, X:%d, Y:%d, W:%d\n", id, x, y, w);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(ts->input_dev);
}
/*******************************************************
Function:
	Touch up report function.
Input:
	ts:private data.
Output:
	None.
*******************************************************/
static void goodix_touch_up(struct goodix_ts_data* ts)
{
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
	input_mt_sync(ts->input_dev);
}

/*******************************************************

********************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
	u8* coor_data = NULL;
	u8  point_data[2 + 2 + 5 * MAX_FINGER_NUM + 1]={READ_TOUCH_ADDR_H,READ_TOUCH_ADDR_L};
	u8  check_sum = 0;
	u8  touch_num = 0;
	u8  finger = 0;
	u8  key_value = 0;
	s32 input_x = 0;
	s32 input_y = 0;
	s32 input_w = 0;
	s32 idx = 0;
	s32 ret = -1;
	s32 pos = 0;
	s32 i = 0;

	struct goodix_ts_data *ts = NULL;

    pr_info("%s: %s, %d. \n", __FILE__, __func__, __LINE__);

	ts = container_of(work, struct goodix_ts_data, work);

	/*Read the first 10 first, if touch_num > 1, read more, see line 712 */
	ret = i2c_read_bytes(ts->client, point_data, 10);
	if (ret <= 0){
		printk("%s:I2C read error!",__func__);
		goto exit_work_func;
	} else {
		/* print point_data read from GT828 */
		pr_info("point_data[0-9]: ");
		for ( i = 0; i < 10; i++ )
		{
			printk("%d  ", point_data[i]);
		}
	}

	finger = point_data[2];
	touch_num = (finger & 0x01) + !!(finger & 0x02) + !!(finger & 0x04) + !!(finger & 0x08) + !!(finger & 0x10);
    pr_info("%s: %s, %d: touch_num=%d \n", __FILE__, __func__, __LINE__, touch_num);
	if (touch_num > 1){
		u8 buf[25];
		buf[1] = READ_TOUCH_ADDR_L + 8;
		ret = i2c_read_bytes(ts->client, buf, 5 * (touch_num - 1) + 2);
		memcpy(&point_data[10], &buf[2], 5 * (touch_num - 1));
	}
	i2c_end_cmd(ts);

	if((finger & 0xC0) != 0x80){
		pr_info("%s: %s, %d. Data not ready \n", __FILE__, __func__, __LINE__);
		goto exit_work_func;
	}

	key_value = point_data[3]&0x0f; // 1, 2, 4, 8
	if ((key_value & 0x0f) == 0x0f){
		if (!goodix_init_panel(ts)){
			printk("%s:Reload config failed!\n",__func__);
		}
		goto exit_work_func;
	}

	coor_data = &point_data[4];
	check_sum = 0;
	for ( idx = 0; idx < 5 * touch_num; idx++){
		check_sum += coor_data[idx];
	}
	if (check_sum != coor_data[5 * touch_num]){
		printk("%s:Check sum error!",__func__);
		goto exit_work_func;
	}

	if (touch_num){
		for (idx = 0; idx < MAX_FINGER_NUM; idx++){
			if (!(finger & (0x01 << idx))){
			        continue;
			}
			input_x  = coor_data[pos] << 8;
			input_x |= coor_data[pos + 1];

			input_y  = coor_data[pos + 2] << 8;
			input_y |= coor_data[pos + 3];

			input_w  = coor_data[pos + 4];

			pos += 5;

			pr_info("%s: %s, %d. Touch Down\n", __FILE__, __func__, __LINE__);
			goodix_touch_down(ts, idx, input_x, input_y, input_w);
		}
	}else{
		pr_info("%s: %s, %d. Touch Up\n", __FILE__, __func__, __LINE__);
		goodix_touch_up(ts);
	}
	/* Until support touch keys */
    //input_report_key(ts->input_dev, BTN_TOUCH, (touch_num || key_value));
	input_sync(ts->input_dev);

exit_work_func:
	return;
}

static enum hrtimer_restart goodix_ts_timer_func(struct hrtimer *timer)
{
	struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);
	queue_work(goodix_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, (POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	if(reg_val&(1<<(CTP_IRQ_NO)))
	{
		print_int_info("%s: %d. ==CTP_IRQ_NO=\n", __func__, __LINE__);
		//clear the CTP_IRQ_NO interrupt pending
		writel(reg_val&(1<<(CTP_IRQ_NO)),gpio_addr + PIO_INT_STAT_OFFSET);
		queue_work(goodix_wq, &ts->work);
	}
	else
	{
	    print_int_info("Other Interrupt\n");
	    return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int goodix_ts_power(struct goodix_ts_data * ts, int on)
{
	s32 ret = -1;
	s32 success = 1;
	u8 i2c_control_buf1[3] = {0x0F,0xF2,0xc0};  /* suspend cmd */
	u8 i2c_control_buf2[3] = {0x0F,0xF2,0x00};

	switch (on)
	{
	case 0:
		ret = i2c_write_bytes(ts->client, i2c_control_buf1, 3);
		i2c_end_cmd(ts);
		return ret;
	case 1:
		ctp_wakeup();
		ret = goodix_init_panel(ts);
		if( ret != 1){
			printk("init panel fail!\n");
			return -1;
		}
		ret = i2c_write_bytes(ts->client, i2c_control_buf2, 3);
		msleep(10);
		return success;

	 default:
	        printk("%s: Cant't support this command.",f3x_ts_name );
	        return -EINVAL;
	}

}

static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int err = 0;
	int retry=0;

#if defined(NO_DEFAULT_ID) && defined(INT_PORT)
	uint8_t goodix_id[3] = {0,0xff,0};
#endif
	char test_data = 1;
	struct goodix_ts_data *ts = NULL;

	dev_dbg(&client->dev,"Installing GT828 Touchscreen Driver.\n");

	printk("====== GT828 Probe======\n");

	gpio_wakeup_hdle = gpio_request_ex("ctp_para", "ctp_wakeup");
	if(!gpio_wakeup_hdle) {
		pr_warning("touch panel tp_wakeup request gpio fail!\n");
		goto exit_gpio_wakeup_request_failed;
	}

	/* Check I2C function */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		dev_err(&client->dev, "Must have I2C_FUNC_I2C.\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	gpio_addr = ioremap(PIO_BASE_ADDRESS, PIO_RANGE_SIZE);
	if(!gpio_addr) {
	    err = -EIO;
	    goto exit_ioremap_failed;
	}

	ts->gpio_irq = INT_PORT;

#ifdef	INT_PORT
	gpio_set_one_pin_io_status(gpio_int_hdle, 0, "ctp_int_port");
	gpio_set_one_pin_pull(gpio_int_hdle, 0, "ctp_int_port");
#endif
	for(retry=0;retry < 3; retry++)
	{
		gpio_set_one_pin_io_status(gpio_wakeup_hdle, 1, "ctp_wakeup");
		gpio_write_one_pin_value(gpio_wakeup_hdle, 0, "ctp_wakeup");
		msleep(50);
		gpio_set_one_pin_io_status(gpio_wakeup_hdle, 0, "ctp_wakeup");
		msleep(200);
		ret =i2c_write_bytes(client, &test_data, 1);	//Test I2C connection.
		if (ret > 0)
			break;
	}

	/* if still not successful, something is wrong */
	if(ret <= 0)
	{
		dev_err(&client->dev, "Warnning: I2C communication might be ERROR!\n");
		goto err_i2c_failed;
	}

	ts->power = goodix_ts_power;
	INIT_WORK(&ts->work, goodix_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_dbg(&client->dev,"goodix_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);

#ifdef HAVE_TOUCH_KEY
	for(retry = 0; retry < MAX_KEY_NUM; retry++)
	{
		input_set_capability(ts->input_dev,EV_KEY,touch_key_array[retry]);
	}
#endif
	input_set_abs_params(ts->input_dev, ABS_X, 0, TOUCH_MAX_HEIGHT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, TOUCH_MAX_WIDTH, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);

#ifdef GOODIX_MULTI_TOUCH
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TOUCH_MAX_HEIGHT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TOUCH_MAX_WIDTH, 0, 0);
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = f3x_ts_name;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 0x1105;

	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev,"Probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	ts->bad_data = 0;

#ifdef INT_PORT
	client->irq=INT_PORT;
	if (client->irq)
	{
		#if INT_TRIGGER==1
			#define GT828_IRQ_TYPE IRQ_TYPE_EDGE_RISING
		#elif INT_TRIGGER==0
			#define GT828_IRQ_TYPE IRQ_TYPE_EDGE_FALLING
		#endif

		err = ctp_ops.set_irq_mode("ctp_para", "ctp_int_port", CTP_IRQ_MODE);
		if(0 != err){
			printk("%s:ctp_ops.set_irq_mode err. \n", __func__);
			goto exit_set_irq_mode;
		}

		err =  request_irq(SW_INT_IRQNO_PIO, goodix_ts_irq_handler, GT828_IRQ_TYPE|IRQF_SHARED, client->name, ts);
		if (err < 0) {
			pr_info( "goodix_probe: request irq failed\n");
			goto exit_irq_request_failed;
		}
		ts->use_irq = 1;
		printk("======Request IRQ Succeeded!==== \n");
		dev_dbg(&client->dev,"Reques IRQ %d succeeded on GPIO:%d\n",INT_PORT,INT_PORT);
	}
#endif

	if (!ts->use_irq)
	{
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	goodix_read_version(ts);

	/* init panel */
	for(retry=0; retry<3; retry++)
	{
		ret=goodix_init_panel(ts);
		dev_info(&client->dev,"the config ret is :%d\n",ret);
		msleep(100);
		if(ret != 0)
		{
			dev_info(&client->dev,"Init_panel failed, retrying ... :%d\n", ret);
			continue;
		}
		else
		{
			break;
		}
	}
	if(ret != 0) {
		ts->bad_data=1;
		goto err_init_godix_ts;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	i2c_end_cmd(ts);
	dev_info(&client->dev,"Start %s in %s mode\n",
		ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	return 0;

err_init_godix_ts:
	i2c_end_cmd(ts);
	if(ts->use_irq)
	{
		ts->use_irq = 0;
	#ifdef INT_PORT
		gpio_set_one_pin_io_status(gpio_int_hdle, 0, "ctp_int_port");
	#endif
	}
	else
		hrtimer_cancel(&ts->timer);

exit_set_irq_mode:
exit_irq_request_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	i2c_set_clientdata(client, NULL);
exit_gpio_wakeup_request_failed:
exit_ioremap_failed:
	if(gpio_addr){
		iounmap(gpio_addr);
	}
err_i2c_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	if (ts && ts->use_irq)
	{
	#ifdef INT_PORT
		gpio_set_one_pin_io_status(gpio_int_hdle, 0, "ctp_int_port");

	#endif
	}
	else if(ts)
		hrtimer_cancel(&ts->timer);

	dev_notice(&client->dev,"The driver is removing...\n");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
static int goodix_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq){
		reg_val = readl(gpio_addr + PIO_INT_CTRL_OFFSET);
		reg_val &=~(1<<CTP_IRQ_NO);
		writel(reg_val,gpio_addr + PIO_INT_CTRL_OFFSET);
	}
	else
		hrtimer_cancel(&ts->timer);
	if (ts->power) {
		ret = ts->power(ts, 0);
		if (ret < 0)
			printk(KERN_ERR "goodix_ts_suspend power off failed\n");
		else
			printk(KERN_ERR "goodix_ts_suspend power off success\n");
	}
	return 0;
}

static int goodix_ts_resume(struct i2c_client *client)
{
	int ret;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts->power) {
		ret = ts->power(ts, 1);
		if (ret < 0)
			printk(KERN_ERR "goodix_ts_resume power on failed\n");
		else
			printk(KERN_ERR "goodix_ts_resume power on success\n");
	}

	if (ts->use_irq){
		reg_val = readl(gpio_addr + PIO_INT_CTRL_OFFSET);
		reg_val |=(1<<CTP_IRQ_NO);
		writel(reg_val,gpio_addr + PIO_INT_CTRL_OFFSET);
	}
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	ts = container_of(h, struct goodix_ts_data, early_suspend);
	goodix_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void goodix_ts_late_resume(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	ts = container_of(h, struct goodix_ts_data, early_suspend);
	goodix_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id goodix_ts_id[] = {
	{ GOODIX_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver goodix_ts_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		= goodix_ts_probe,
	.remove		= goodix_ts_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
#else
#ifdef CONFIG_PM
	.suspend	= goodix_ts_suspend,
	.resume		= goodix_ts_resume,
#endif
#endif
	.id_table	= goodix_ts_id,
	.driver = {
		.name	= GOODIX_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.address_list	= u_i2c_addr.normal_i2c,
};

static int __devinit goodix_ts_init(void)
{
	int ret = -1;
	int err = -1;

	printk("===========================%s=====================\n", __func__);

	if (ctp_ops.fetch_sysconfig_para)
	{
		if(ctp_ops.fetch_sysconfig_para()){
			printk("%s: err.\n", __func__);
			return -1;
		}
	}
	printk("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
	__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

	err = ctp_ops.init_platform_resource();
	if(0 != err){
		printk("%s:ctp_ops.init_platform_resource err. \n", __func__);
	}
	ctp_set_gpio_mode();

	goodix_wq = create_singlethread_workqueue("goodix_wq");
	if (!goodix_wq) {
		printk(KERN_ALERT "Creat %s workqueue failed.\n", f3x_ts_name);
		return -ENOMEM;
	}
	ctp_ops.ts_reset();
	ctp_ops.ts_wakeup();
	goodix_ts_driver.detect = ctp_ops.ts_detect;
	ret = i2c_add_driver(&goodix_ts_driver);
	return ret;
}

static void __exit goodix_ts_exit(void)
{
	printk(KERN_ALERT "Touchscreen driver of guitar exited.\n");
	i2c_del_driver(&goodix_ts_driver);
	if (goodix_wq)
		destroy_workqueue(goodix_wq);
}

late_initcall(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("Goodix GT828 Touchscreen Driver");
MODULE_LICENSE("GPL");
