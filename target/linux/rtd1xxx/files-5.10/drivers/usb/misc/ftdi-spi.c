// SPDX-License-Identifier: GPL-2.0
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/usb.h>

#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/device.h>

#include <linux/kernel.h>

#define FTDI_VID	0x0403	/* Vendor Id */

/*** "original" FTDI device PIDs ***/
#define FTDI_2232H_PID	0x6010 /* Dual channel device */
#define FTDI_4232H_PID	0x6011 /* Quad channel hi-speed device */
#define FTDI_232H_PID	0x6014 /* Single channel hi-speed device */

/* USB LANG IDs (https://github.com/wireshark/wireshark/blob/7370516d210bb211ba668365085004dcb76eb297/epan/dissectors/packet-usb.c#L442)*/
#define USB_LANG_NO	0x0000
#define USB_LANG_EN	0x0409

/* USB bmRequestType values */
#define USB_REQUEST_TYPE(direction, type, recipient) \
	((direction) | (type) | (recipient))
#define USB_CONTROL \
	 USB_REQUEST_TYPE(USB_DIR_OUT, \
	 USB_TYPE_VENDOR, \
	 USB_RECIP_DEVICE)
#define USB_DESCRIPTOR \
	 USB_REQUEST_TYPE(USB_DIR_IN, \
	 USB_TYPE_STANDARD, \
	 USB_RECIP_DEVICE)

/* FTDI request lists */
#define REQUEST_RESET           0x00
#define REQUEST_MODEM_CTRL      0x01
#define REQUEST_SET_FLOW_CTRL   0x02
#define REQUEST_SET_BAUD_RATE   0x03
#define REQUEST_SET_DATA        0x04
#define REQUEST_GET_MODEM_STAT  0x05
#define REQUEST_SET_EVENT_CHAR  0x06
#define REQUEST_SET_ERROR_CHAR  0x07
#define REQUEST_SET_LAT_TIMER   0x09
#define REQUEST_GET_LAT_TIMER   0x0A
#define REQUEST_SET_BITMODE     0x0B

/* FTDI descriptor define */
#define DESC_GETLANG	0x00
#define DESC_GETVENDOR	0x01
#define DESC_GETDEVICE	0x02
#define DESC_STRING	0x0300

/* FTDI purge RX/TX define */
#define PURGE_BOTH	0x00
#define PURGE_RX	0x01
#define PURGE_TX	0x02

/* FTDI bitmode lists */
#define BITMODE_RESET   0x00
#define BITMODE_BITBANG 0x01
#define BITMODE_MPSSE   0x02
#define BITMODE_SYNCBB  0x04
#define BITMODE_MCU     0x08
#define BITMODE_OPTO    0x10
#define BITMODE_CBUS    0x20
#define BITMODE_SYNCFF  0x40
#define BITMODE_FT1284  0x80

/* FTDI Set clk divisor */
#define CMD_CLOCK_SET_DIVISOR_DIS_CLK_BY_5	0x8A	/* Disable Clk Divide by 5*/
#define CMD_CLOCK_SET_DIVISOR_EN_CLK_BY_5	0x8B	/* Enable Clk Divide by 5 */
#define CMD_CLOCK_SET_DIVISOR_EN_3_PHASE_DC	0x8C	/* Enable 3 Phase Data Clocking */
#define CMD_CLOCK_SET_DIVISOR_DIS_3_PHASE_DC	0x8D	/* Disable 3 Phase Data Clocking */
#define CMD_CLOCK_SET_DIVISOR_N_BITS_NO_DT	0x8E	/* Clock For n bits with no data transfer */
#define CMD_CLOCK_SET_DIVISOR_8N_BITS_NO_DT	0x8F	/* Clock For n x 8 bits with no data transfer */
#define CMD_CLOCK_SET_DIVISOR_CLK_CW_HIGH	0x94	/* Clk continuously and Wait On I/O High */
#define CMD_CLOCK_SET_DIVISOR_CLK_CW_LOW	0x95	/* Clk continuously and Wait On I/O Low */
#define CMD_CLOCK_SET_DIVISOR_TURN_ON_ADAP_CLK	0x96	/* Turn On Adaptive clocking */
#define CMD_CLOCK_SET_DIVISOR_TURN_OFF_ADAP_CLK	0x97	/* Turn Off Adaptive clocking */
#define CMD_CLOCK_SET_DIVISOR_CLK_8N_GPIO_HIGH	0x9C	/* Clock For n x 8 bits with no data transfer or Until GPIOL1 is High */
#define CMD_CLOCK_SET_DIVISOR_CLK_8N_GPIO_LOW	0x9D	/* Clock For n x 8 bits with no data transfer or Until GPIOL1 is Low */

#define CMD_SET_DATA_BITS_LOW_BYTE	0x80
#define CMD_READ_DATA_BITS_LOW_BYTE	0x81
#define CMD_SET_DATA_BITS_HIGH_BYTE	0x82
#define CMD_READ_DATA_BITS_HIGH_BYTE	0x83
#define CMD_CLOCK_SET_DIVISOR		0x86
#define CMD_CPUMODE_READ_SHORT_ADDR	0x90
#define CMD_CPUMODE_READ_EXT_ADDR	0x91
#define CMD_CPUMODE_WRITE_SHORT_ADDR	0x92
#define CMD_CPUMODE_WRITE_EXT_ADDR	0x93

/* FTDI clk divisor list */
#define TCK_MAX_30M_6M		0x0000	/* Divisor: 0x0000, TCK Max: 30 MHz (60 MHz master clock) or 6 MHz (12 MHz master clock) */
#define TCK_MAX_15M_3M		0x0001	/* Divisor: 0x0000, TCK Max: 15 MHz (60 MHz master clock) or 3 MHz (12 MHz master clock) */
#define TCK_MAX_7_5M_1_5M	0x0002	/* Divisor: 0x0000, TCK Max: 7.5 MHz (60 MHz master clock) or 1.5 MHz (12 MHz master clock) */
#define TCK_MAX_6M_1_2M		0x0003	/* Divisor: 0x0000, TCK Max: 6 MHz (60 MHz master clock) or 1.2 MHz (12 MHz master clock) */
#define TCK_MAX_5M_1M		0x0004	/* Divisor: 0x0000, TCK Max: 5 MHz (60 MHz master clock) or 1 MHz (12 MHz master clock) */

/* FTDI usb setting */
#define FTDI_IO_TIMEOUT	500
#define BUFFER_SIZE	1024

/* SPI setting */
#define FTDI_SPI_CTRL_BUFFER_SIZE 254
#define FTDI_SPI_BULK_BUFFER_SIZE 512
#define FTDI_SPI_FRAME_SIZE 6

/* Cdev setting */
#define FTDI_SPI_MAX_DEVICES	(1U << MINORBITS)

/* Struct defined */
struct ftdi_usb {
	struct cdev cdev;
	dev_t dev;
	char *buffer;
	struct usb_device *udev;
	struct usb_interface *interface;
	int io_timeout;
};

/* Function defined */
/* File OP Part */
static int ftdi_spi_open(struct inode *inode, struct file *file);
static ssize_t
ftdi_spi_read(struct file *filp, char __user *buf, size_t len, loff_t *off);
/* SPI Part */
static int spi_snd_data(struct ftdi_usb *ftdi, char *buf, int *act_len);
static int spi_rcv_data(struct ftdi_usb *ftdi, char *buf, int *act_len);
static int spi_exchange_data(struct ftdi_usb *ftdi, char *buf, int *act_len);
static int ftdi_info_get(struct ftdi_usb *ftdi);
static int ftdi_spi_setbitmode(struct ftdi_usb *ftdi, int mask, int bitmode);
static int ftdi_spi_setlattimer(struct ftdi_usb *ftdi, int latency);
static int ftdi_spi_seteventchar(struct ftdi_usb *ftdi);
static int ftdi_spi_seterrorchar(struct ftdi_usb *ftdi);
static int ftdi_spi_reset(struct ftdi_usb *ftdi);
static int ftdi_spi_setup(struct ftdi_usb *ftdi);
static int ftdi_spi_get_data(struct ftdi_usb *ftdi);
/* Driver Part */
static int ftdi_usb_probe(struct usb_interface *interface,
			  const struct usb_device_id *id);
static void ftdi_usb_disconnect(struct usb_interface *interface);

/* Cdev varible */
dev_t dev_node;
static struct class *ftdi_spi_class;

static const struct usb_device_id ftdi_id_table[] = {
	{ USB_DEVICE(FTDI_VID, FTDI_2232H_PID) },
	{ USB_DEVICE(FTDI_VID, FTDI_4232H_PID) },
	{ USB_DEVICE(FTDI_VID, FTDI_232H_PID) },
	{ }
};

MODULE_DEVICE_TABLE(usb, ftdi_id_table);

static const struct file_operations ftdi_spi_fops = {
	.owner		= THIS_MODULE,
	.read		= ftdi_spi_read,
	.open		= ftdi_spi_open
};

static int ftdi_spi_open(struct inode *inode, struct file *file)
{
	struct ftdi_usb *ftdi;

	ftdi = container_of(inode->i_cdev, struct ftdi_usb, cdev);
	file->private_data = ftdi;

	pr_info("Device File Opened...!!!\n");
	return 0;
}

static ssize_t
ftdi_spi_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
	struct ftdi_usb *ftdi;
	int ret;
	int cha, chb;
	char *msg;
	static char flag;
	size_t len_tmp;


	ftdi = (struct ftdi_usb *) filp->private_data;
	ret = ftdi_spi_get_data(ftdi);
	if (ret < 0)
		return -EFAULT;

	if (len > FTDI_SPI_FRAME_SIZE) {
		if (flag > 0) {
			flag = 0;
			return 0;
		}
		msg = kzalloc(BUFFER_SIZE, GFP_KERNEL);

		/* cha = 0x00 00 00 01 FF FE 00 00 */
		cha = (((0x01 & ftdi->buffer[1]) << 16) |
			 ((0xff & ftdi->buffer[2]) << 8) |
			 (0xfe & ftdi->buffer[3])) >> 1;
		/* chb = 0x00 00 00 00 00 01 FF FE */
		chb = (((0x01 & ftdi->buffer[3]) << 16) |
			 ((0xff & ftdi->buffer[4]) << 8) |
			 (0xfe & ftdi->buffer[5])) >> 1;

		/* Calc value */
		/* value / 65535 * 5 * 1000 = output */
		cha = cha + (cha << 2);	/* cha *= 5; */
		cha = (cha << 10) - (cha << 4) - (cha << 3);	/* cha *= 1000 */
		cha = cha >> 16;	/* cha /= 65535 */
		chb = chb + (chb << 2);	/* chb *= 5; */
		chb = (chb << 10) - (chb << 4) - (chb << 3);	/* chb *= 1000 */
		chb = chb >> 16;	/* chb /= 65535 */

		len_tmp = snprintf(msg, BUFFER_SIZE,
					 "CH_A: %d m\nCH_B: %d m\n", cha, chb);
		pr_alert("res: %u, msg_size: %zu, len: %zu\n",
				 ret, len_tmp, len);

		ret = copy_to_user(buf, msg, len_tmp);
		if (ret < 0)
			return -EFAULT;
		flag++;
		kfree(msg);
		return len_tmp;
	}

	ret = copy_to_user(buf, ftdi->buffer, len);
	pr_alert("res: %u, msg_size: %u, len: %zu\n",
			 ret, FTDI_SPI_FRAME_SIZE, len);
	if (ret < 0)
		return -EFAULT;

	pr_info("Read Function\n");
	return len;
}

/* SPI Part */
static int spi_snd_data(struct ftdi_usb *ftdi, char *buf, int *act_len)
{
	int ret;
	int curr_intf;
	int snd_endpoint;

	curr_intf = ftdi->interface->cur_altsetting->desc.bInterfaceNumber;

	snd_endpoint = curr_intf * 2 + 2;

	ret = usb_bulk_msg(
		ftdi->udev, usb_sndbulkpipe(ftdi->udev, snd_endpoint),
		/* data = */buf,
		/* size = */*act_len,
		/* act_size = */act_len,
		ftdi->io_timeout);
	if (ret < 0 && *act_len <= 0)
		goto err;

	return ret;
err:
	pr_alert("%s(%u)-Err code: %d", __func__, __LINE__, ret);
	pr_alert("ACT Len: %d", *act_len);
	return ret;
}

static int spi_rcv_data(struct ftdi_usb *ftdi, char *buf, int *act_len)
{
	int ret;
	int curr_intf;
	int rcv_endpoint;

	curr_intf = ftdi->interface->cur_altsetting->desc.bInterfaceNumber;

	rcv_endpoint = curr_intf * 2 + 1;

	ret = usb_bulk_msg(
		ftdi->udev, usb_rcvbulkpipe(ftdi->udev, rcv_endpoint),
		/* data = */buf,
		/* size = */FTDI_SPI_BULK_BUFFER_SIZE,
		/* act_size = */act_len,
		ftdi->io_timeout);
	if (ret < 0 && *act_len <= 0)
		goto err;

	return ret;
err:
	pr_alert("%s(%u)-Err code: %d", __func__, __LINE__, ret);
	pr_alert("ACT Len: %d", *act_len);
	return ret;
}

static int spi_exchange_data(struct ftdi_usb *ftdi, char *buf, int *act_len)
{
	int ret;

	ret = spi_snd_data(ftdi, buf, act_len);
	if (ret < 0 && *act_len <= 0)
		goto err;

	ret = spi_rcv_data(ftdi, buf, act_len);
	if (ret < 0 && *act_len <= 0)
		goto err;

	return ret;
err:
	pr_alert("%s(%u)-Err code: %d", __func__, __LINE__, ret);
	pr_alert("ACT Len: %d", *act_len);
	return ret;
}

static int ftdi_info_get(struct ftdi_usb *ftdi)
{
	int ret;
	char *buf;
	int wvalue;

	buf = kzalloc(BUFFER_SIZE, GFP_KERNEL);

	wvalue = DESC_STRING | DESC_GETLANG;
	ret = usb_control_msg(
		ftdi->udev, usb_rcvctrlpipe(ftdi->udev, 0),
		/* bRequest = */REQUEST_SET_EVENT_CHAR,
		/* bRequestType = */USB_DESCRIPTOR,
		/* wValue = */wvalue,
		/* wIndex = */USB_LANG_NO,
		/* data = */buf,
		/* size = */FTDI_SPI_CTRL_BUFFER_SIZE,
		ftdi->io_timeout);
	print_hex_dump(KERN_ALERT, "Ret: ", DUMP_PREFIX_ADDRESS,
			16, 1, buf, buf[0], 1);
	if (ret < 0)
		goto err;

	wvalue = DESC_STRING | DESC_GETDEVICE;
	ret = usb_control_msg(
		ftdi->udev, usb_rcvctrlpipe(ftdi->udev, 0),
		/* bRequest = */REQUEST_SET_EVENT_CHAR,
		/* bRequestType = */USB_DESCRIPTOR,
		/* wValue = */wvalue,
		/* wIndex = */USB_LANG_EN,
		/* data = */buf,
		/* size = */FTDI_SPI_CTRL_BUFFER_SIZE,
		ftdi->io_timeout);
	print_hex_dump(KERN_ALERT, "Ret: ", DUMP_PREFIX_ADDRESS,
			16, 1, buf, buf[0], 1);
	if (ret < 0)
		goto err;

	return ret;
err:
	pr_alert("%s(%u)-Err code: %d", __func__, __LINE__, ret);
	return ret;

}

static int ftdi_spi_setbitmode(struct ftdi_usb *ftdi, int mask, int bitmode)
{
	int ret;
	char *buf;
	int curr_intf;
	int target_port;
	int wValue;

	buf = kzalloc(BUFFER_SIZE, GFP_KERNEL);
	curr_intf = ftdi->interface->cur_altsetting->desc.bInterfaceNumber;
	target_port = curr_intf + 1;
	wValue = bitmode << 8 | mask;

	ret = usb_control_msg(
		ftdi->udev, usb_sndctrlpipe(ftdi->udev, 0),
		/* bRequest = */REQUEST_SET_BITMODE,
		/* bRequestType = */USB_CONTROL,
		/* wValue = */wValue,
		/* wIndex = */target_port,
		/* data = */buf,
		/* size = */0,
		ftdi->io_timeout);
	if (ret < 0)
		goto err;

	kfree(buf);
	return ret;
err:
	pr_alert("%s(%u)-Err code: %d", __func__, __LINE__, ret);
	kfree(buf);
	return ret;
}

static int ftdi_spi_setlattimer(struct ftdi_usb *ftdi, int latency)
{
	int ret;
	char *buf;
	int curr_intf;
	int target_port;

	buf = kzalloc(BUFFER_SIZE, GFP_KERNEL);
	curr_intf = ftdi->interface->cur_altsetting->desc.bInterfaceNumber;
	target_port = curr_intf + 1;

	ret = usb_control_msg(
		ftdi->udev, usb_sndctrlpipe(ftdi->udev, 0),
		/* bRequest = */REQUEST_SET_LAT_TIMER,
		/* bRequestType = */USB_CONTROL,
		/* wValue = */latency,
		/* wIndex = */target_port,
		/* data = */buf,
		/* size = */0,
		ftdi->io_timeout);
	if (ret < 0)
		goto err;

	kfree(buf);
	return ret;
err:
	pr_alert("%s(%u)-Err code: %d", __func__, __LINE__, ret);
	kfree(buf);
	return ret;
}

static int ftdi_spi_seteventchar(struct ftdi_usb *ftdi)
{
	int ret;
	char *buf;
	int curr_intf;
	int target_port;

	buf = kzalloc(BUFFER_SIZE, GFP_KERNEL);
	curr_intf = ftdi->interface->cur_altsetting->desc.bInterfaceNumber;
	target_port = curr_intf + 1;

	ret = usb_control_msg(
		ftdi->udev, usb_sndctrlpipe(ftdi->udev, 0),
		/* bRequest = */REQUEST_SET_EVENT_CHAR,
		/* bRequestType = */USB_CONTROL,
		/* wValue = */0x0000,
		/* wIndex = */target_port,
		/* data = */buf,
		/* size = */0,
		ftdi->io_timeout);
	if (ret < 0)
		goto err;

	kfree(buf);
	return ret;
err:
	pr_alert("%s(%u)-Err code: %d", __func__, __LINE__, ret);
	kfree(buf);
	return ret;
}

static int ftdi_spi_seterrorchar(struct ftdi_usb *ftdi)
{
	int ret;
	char *buf;
	int curr_intf;
	int target_port;

	buf = kzalloc(BUFFER_SIZE, GFP_KERNEL);
	curr_intf = ftdi->interface->cur_altsetting->desc.bInterfaceNumber;
	target_port = curr_intf + 1;

	ret = usb_control_msg(
		ftdi->udev, usb_sndctrlpipe(ftdi->udev, 0),
		/* bRequest = */REQUEST_SET_ERROR_CHAR,
		/* bRequestType = */USB_CONTROL,
		/* wValue = */0x0000,
		/* wIndex = */target_port,
		/* data = */buf,
		/* size = */0,
		ftdi->io_timeout);
	if (ret < 0)
		goto err;

	kfree(buf);
	return ret;
err:
	pr_alert("%s(%u)-Err code: %d", __func__, __LINE__, ret);
	kfree(buf);
	return ret;
}

static int ftdi_spi_reset(struct ftdi_usb *ftdi)
{
	int ret;
	char *buf;
	int curr_intf;
	int target_port;

	buf = kzalloc(BUFFER_SIZE, GFP_KERNEL);
	curr_intf = ftdi->interface->cur_altsetting->desc.bInterfaceNumber;
	target_port = curr_intf + 1;

	ret = usb_control_msg(
		ftdi->udev, usb_sndctrlpipe(ftdi->udev, 0),
		/* bRequest = */REQUEST_RESET,
		/* bRequestType = */USB_CONTROL,
		/* wValue = */PURGE_RX,
		/* wIndex = */target_port,
		/* data = */buf,
		/* size = */0,
		ftdi->io_timeout);
	if (ret < 0)
		goto err;

	ret = usb_control_msg(
		ftdi->udev, usb_sndctrlpipe(ftdi->udev, 0),
		/* bRequest = */REQUEST_RESET,
		/* bRequestType = */USB_CONTROL,
		/* wValue = */PURGE_TX,
		/* wIndex = */target_port,
		/* data = */buf,
		/* size = */0,
		ftdi->io_timeout);
	if (ret < 0)
		goto err;

	ret = usb_control_msg(
		ftdi->udev, usb_sndctrlpipe(ftdi->udev, 0),
		/* bRequest = */REQUEST_RESET,
		/* bRequestType = */USB_CONTROL,
		/* wValue = */PURGE_BOTH,
		/* wIndex = */target_port,
		/* data = */buf,
		/* size = */0,
		ftdi->io_timeout);
	if (ret < 0)
		goto err;

	kfree(buf);
	return ret;
err:
	pr_alert("%s(%u)-Err code: %d", __func__, __LINE__, ret);
	kfree(buf);
	return ret;
}

static int ftdi_spi_setup(struct ftdi_usb *ftdi)
{
	int ret;
	char *buf;
	int curr_intf;
	int act_len;

	buf = kzalloc(BUFFER_SIZE, GFP_KERNEL);
	curr_intf = ftdi->interface->cur_altsetting->desc.bInterfaceNumber;

	dev_info(&ftdi->interface->dev, "Setup FTDI device spi mode.\n");

	/* FTDI FT Request: Reset */
	ret = ftdi_spi_reset(ftdi);
	if (ret < 0)
		goto err;

	/* FTDI FT Request: SetBitMode */
	ret = ftdi_spi_setbitmode(ftdi, 0, BITMODE_RESET);
	if (ret < 0)
		goto err;

	/* FTDI FT Request: SetLatTimer */
	ret = ftdi_spi_setlattimer(ftdi, 0x0001);
	if (ret < 0)
		goto err;

	/* FTDI FT Request: SetLatTimer */
	ret = ftdi_spi_setlattimer(ftdi, 0x0010);
	if (ret < 0)
		goto err;

	/* FTDI FT Request: SetBitMode */
	ret = ftdi_spi_setbitmode(ftdi, 0, BITMODE_RESET);
	if (ret < 0)
		goto err;

	/* FTDI FT Request: Reset */
	ret = ftdi_spi_reset(ftdi);
	if (ret < 0)
		goto err;

	/* FTDI FT Request: SetEventChar */
	ret = ftdi_spi_seteventchar(ftdi);
	if (ret < 0)
		goto err;

	/* FTDI FT Request: SetErrorChar */
	ret = ftdi_spi_seterrorchar(ftdi);
	if (ret < 0)
		goto err;

	/* FTDI FT Request: SetBitMode */
	ret = ftdi_spi_setbitmode(ftdi, 0x0b, BITMODE_MPSSE);
	if (ret < 0)
		goto err;

	/* Set clk rate */
	buf[0] = CMD_CLOCK_SET_DIVISOR_EN_CLK_BY_5;
	buf[1] = CMD_CLOCK_SET_DIVISOR;
	buf[2] = TCK_MAX_30M_6M >> 8;
	buf[3] = TCK_MAX_30M_6M & 0xFF;
	act_len = 4;
	ret = spi_exchange_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

	/* FTDI FT Request: Reset */
	ret = ftdi_spi_reset(ftdi);
	if (ret < 0)
		goto err;

	buf[0] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[1] = 0x08;	/* Value */
	buf[2] = 0x0b;	/* Direction */
	buf[3] = CMD_SET_DATA_BITS_HIGH_BYTE;
	buf[4] = 0x00;	/* Value */
	buf[5] = 0x00;	/* Direction */
	act_len = 6;
	ret = spi_snd_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

	buf[0] = 0x85;	/* Disconnect TDI to TDO for Loopback */
	act_len = 1;
	ret = spi_snd_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;


	ret = spi_rcv_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

	buf[0] = CMD_CLOCK_SET_DIVISOR_TURN_OFF_ADAP_CLK;
	act_len = 1;
	ret = spi_snd_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

/* slave = spi.get_port(cs = cs, freq = freq, mode = mode) */
	buf[0] = 0x87;	/* Send Immediate (flush buffer back to the PC) */
	act_len = 1;
	ret = spi_snd_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

	/* FTDI FT Request: Reset */
	ret = ftdi_spi_reset(ftdi);
	if (ret < 0)
		goto err;

/* slave.exchange(write_cfg, 4) */
	buf[0] = CMD_CLOCK_SET_DIVISOR_DIS_CLK_BY_5;
	buf[1] = CMD_CLOCK_SET_DIVISOR;
	buf[2] = TCK_MAX_15M_3M >> 8;
	buf[3] = TCK_MAX_15M_3M & 0xFF;
	act_len = 4;
	ret = spi_exchange_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

	/* FTDI FT Request: Reset */
	ret = ftdi_spi_reset(ftdi);
	if (ret < 0)
		goto err;

	buf[0] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[1] = 0x0d;	/* Value */
	buf[2] = 0x0b;	/* Direction */
	buf[3] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[4] = 0x05;	/* Value */
	buf[5] = 0x0b;	/* Direction */
	buf[6] = 0x10;	/* Clock Data Bytes Out on + ve clock edge MSB first(no read) [Use if CLK starts at '1'] */
	buf[7] = 0x01;	/* Length 2 Bytes */
	buf[8] = 0x00;	/* Length 2 Bytes */
	buf[9] = 0x86;	/* Data High */
	buf[10] = 0x40;	/* Data Low */
	buf[11] = 0x20;	/* Clock Data Bytes In on +ve clock edge MSB first (no write) */
	buf[12] = 0x03;	/* Length 4 Bytes */
	buf[13] = 0x00;	/* Length 4 Bytes */
	buf[14] = 0x87;	/* Send Immediate (flush buffer back to the PC) */
	buf[15] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[16] = 0x05;	/* Value */
	buf[17] = 0x0b;	/* Direction */
	buf[18] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[19] = 0x0d;	/* Value */
	buf[20] = 0x0b;	/* Direction */
	buf[21] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[22] = 0x08;	/* Value */
	buf[23] = 0x0b;	/* Direction */
	act_len = 24;
	ret = spi_exchange_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

/* slave.exchange(b'\x30\x00', 4) */
	buf[0] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[1] = 0x0d;	/* Value */
	buf[2] = 0x0b;	/* Direction */
	buf[3] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[4] = 0x05;	/* Value */
	buf[5] = 0x0b;	/* Direction */
	buf[6] = 0x10;	/* Clock Data Bytes Out on + ve clock edge MSB first(no read) [Use if CLK starts at '1'] */
	buf[7] = 0x01;	/* Length 2 Bytes */
	buf[8] = 0x00;	/* Length 2 Bytes */
	buf[9] = 0x30;	/* Data High */
	buf[10] = 0x00;	/* Data Low */
	buf[11] = 0x20;	/* Clock Data Bytes In on +ve clock edge MSB first (no write) */
	buf[12] = 0x03;	/* Length 4 Bytes */
	buf[13] = 0x00;	/* Length 4 Bytes */
	buf[14] = 0x87;	/* Send Immediate (flush buffer back to the PC) */
	buf[15] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[16] = 0x05;	/* Value */
	buf[17] = 0x0b;	/* Direction */
	buf[18] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[19] = 0x0d;	/* Value */
	buf[20] = 0x0b;	/* Direction */
	buf[21] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[22] = 0x08;	/* Value */
	buf[23] = 0x0b;	/* Direction */
	act_len = 24;
	ret = spi_exchange_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

/* CFR = slave.exchange(readlen=6) */
	buf[0] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[1] = 0x0d;	/* Value */
	buf[2] = 0x0b;	/* Direction */
	buf[3] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[4] = 0x05;	/* Value */
	buf[5] = 0x0b;	/* Direction */
	buf[6] = 0x20;	/* Clock Data Bytes In on +ve clock edge MSB first (no write) */
	buf[7] = 0x05;	/* Length 6 Bytes */
	buf[8] = 0x00;	/* Length 6 Bytes */
	buf[9] = 0x87;	/* Send Immediate (flush buffer back to the PC) */
	buf[10] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[11] = 0x05;	/* Value */
	buf[12] = 0x0b;	/* Direction */
	buf[13] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[14] = 0x0d;	/* Value */
	buf[15] = 0x0b;	/* Direction */
	buf[16] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[17] = 0x08;	/* Value */
	buf[18] = 0x0b;	/* Direction */
	act_len = 19;
	ret = spi_exchange_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

/* slave.exchange(write_REFDAC, 4) */
	buf[0] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[1] = 0x0d;	/* Value */
	buf[2] = 0x0b;	/* Direction */
	buf[3] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[4] = 0x05;	/* Value */
	buf[5] = 0x0b;	/* Direction */
	buf[6] = 0x10;	/* Clock Data Bytes Out on + ve clock edge MSB first(no read) [Use if CLK starts at '1'] */
	buf[7] = 0x01;	/* Length 2 Bytes */
	buf[8] = 0x00;	/* Length 2 Bytes */
	buf[9] = 0x9f;	/* Data High */
	buf[10] = 0xf8;	/* Data Low */
	buf[11] = 0x20;	/* Clock Data Bytes In on +ve clock edge MSB first (no write) */
	buf[12] = 0x03;	/* Length 4 Bytes */
	buf[13] = 0x00;	/* Length 4 Bytes */
	buf[14] = 0x87;	/* Send Immediate (flush buffer back to the PC) */
	buf[15] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[16] = 0x05;	/* Value */
	buf[17] = 0x0b;	/* Direction */
	buf[18] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[19] = 0x0d;	/* Value */
	buf[20] = 0x0b;	/* Direction */
	buf[21] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[22] = 0x08;	/* Value */
	buf[23] = 0x0b;	/* Direction */
	act_len = 24;
	ret = spi_exchange_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

/* slave.exchange(b'\x10\x00', 4) */
	buf[0] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[1] = 0x0d;	/* Value */
	buf[2] = 0x0b;	/* Direction */
	buf[3] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[4] = 0x05;	/* Value */
	buf[5] = 0x0b;	/* Direction */
	buf[6] = 0x10;	/* Clock Data Bytes Out on + ve clock edge MSB first(no read) [Use if CLK starts at '1'] */
	buf[7] = 0x01;	/* Length 2 Bytes */
	buf[8] = 0x00;	/* Length 2 Bytes */
	buf[9] = 0x10;	/* Data High */
	buf[10] = 0x00;	/* Data Low */
	buf[11] = 0x20;	/* Clock Data Bytes In on +ve clock edge MSB first (no write) */
	buf[12] = 0x03;	/* Length 4 Bytes */
	buf[13] = 0x00;	/* Length 4 Bytes */
	buf[14] = 0x87;	/* Send Immediate (flush buffer back to the PC) */
	buf[15] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[16] = 0x05;	/* Value */
	buf[17] = 0x0b;	/* Direction */
	buf[18] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[19] = 0x0d;	/* Value */
	buf[20] = 0x0b;	/* Direction */
	buf[21] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[22] = 0x08;	/* Value */
	buf[23] = 0x0b;	/* Direction */
	act_len = 24;
	ret = spi_exchange_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

/* read_REFDAC = slave.exchange(readlen=6) */
	buf[0] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[1] = 0x0d;	/* Value */
	buf[2] = 0x0b;	/* Direction */
	buf[3] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[4] = 0x05;	/* Value */
	buf[5] = 0x0b;	/* Direction */
	buf[6] = 0x20;	/* Clock Data Bytes In on +ve clock edge MSB first (no write) */
	buf[7] = 0x05;	/* Length 6 Bytes */
	buf[8] = 0x00;	/* Length 6 Bytes */
	buf[9] = 0x87;	/* Send Immediate (flush buffer back to the PC) */
	buf[10] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[11] = 0x05;	/* Value */
	buf[12] = 0x0b;	/* Direction */
	buf[13] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[14] = 0x0d;	/* Value */
	buf[15] = 0x0b;	/* Direction */
	buf[16] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[17] = 0x08;	/* Value */
	buf[18] = 0x0b;	/* Direction */
	act_len = 19;
	ret = spi_exchange_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

/* slave.exchange(write_REFDAC, 4) */
	buf[0] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[1] = 0x0d;	/* Value */
	buf[2] = 0x0b;	/* Direction */
	buf[3] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[4] = 0x05;	/* Value */
	buf[5] = 0x0b;	/* Direction */
	buf[6] = 0x10;	/* Clock Data Bytes Out on + ve clock edge MSB first(no read) [Use if CLK starts at '1'] */
	buf[7] = 0x01;	/* Length 2 Bytes */
	buf[8] = 0x00;	/* Length 2 Bytes */
	buf[9] = 0xaf;	/* Data High */
	buf[10] = 0xf8;	/* Data Low */
	buf[11] = 0x20;	/* Clock Data Bytes In on +ve clock edge MSB first (no write) */
	buf[12] = 0x03;	/* Length 4 Bytes */
	buf[13] = 0x00;	/* Length 4 Bytes */
	buf[14] = 0x87;	/* Send Immediate (flush buffer back to the PC) */
	buf[15] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[16] = 0x05;	/* Value */
	buf[17] = 0x0b;	/* Direction */
	buf[18] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[19] = 0x0d;	/* Value */
	buf[20] = 0x0b;	/* Direction */
	buf[21] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[22] = 0x08;	/* Value */
	buf[23] = 0x0b;	/* Direction */
	act_len = 24;
	ret = spi_exchange_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

/* slave.exchange(b'\x20\x00', 4) */
	buf[0] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[1] = 0x0d;	/* Value */
	buf[2] = 0x0b;	/* Direction */
	buf[3] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[4] = 0x05;	/* Value */
	buf[5] = 0x0b;	/* Direction */
	buf[6] = 0x10;	/* Clock Data Bytes Out on + ve clock edge MSB first(no read) [Use if CLK starts at '1'] */
	buf[7] = 0x01;	/* Length 2 Bytes */
	buf[8] = 0x00;	/* Length 2 Bytes */
	buf[9] = 0x20;	/* Data High */
	buf[10] = 0x00;	/* Data Low */
	buf[11] = 0x20;	/* Clock Data Bytes In on +ve clock edge MSB first (no write) */
	buf[12] = 0x03;	/* Length 4 Bytes */
	buf[13] = 0x00;	/* Length 4 Bytes */
	buf[14] = 0x87;	/* Send Immediate (flush buffer back to the PC) */
	buf[15] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[16] = 0x05;	/* Value */
	buf[17] = 0x0b;	/* Direction */
	buf[18] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[19] = 0x0d;	/* Value */
	buf[20] = 0x0b;	/* Direction */
	buf[21] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[22] = 0x08;	/* Value */
	buf[23] = 0x0b;	/* Direction */
	act_len = 24;
	ret = spi_exchange_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

/* slave.exchange(readlen=6) */
	buf[0] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[1] = 0x0d;	/* Value */
	buf[2] = 0x0b;	/* Direction */
	buf[3] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[4] = 0x05;	/* Value */
	buf[5] = 0x0b;	/* Direction */
	buf[6] = 0x20;	/* Clock Data Bytes In on +ve clock edge MSB first (no write) */
	buf[7] = 0x05;	/* Length 6 Bytes */
	buf[8] = 0x00;	/* Length 6 Bytes */
	buf[9] = 0x87;	/* Send Immediate (flush buffer back to the PC) */
	buf[10] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[11] = 0x05;	/* Value */
	buf[12] = 0x0b;	/* Direction */
	buf[13] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[14] = 0x0d;	/* Value */
	buf[15] = 0x0b;	/* Direction */
	buf[16] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[17] = 0x08;	/* Value */
	buf[18] = 0x0b;	/* Direction */
	act_len = 19;
	ret = spi_exchange_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

	dev_info(&ftdi->interface->dev,
			"Finish setup FTDI device spi mode.\n");

	kfree(buf);
	return ret;
err:
	pr_alert("%s(%u)-Err code: %d", __func__, __LINE__, ret);
	kfree(buf);
	return ret;
}

static int ftdi_spi_get_data(struct ftdi_usb *ftdi)
{
	int ret;
	char *buf;
	int act_len;
	int cha, chb;

	buf = kzalloc(BUFFER_SIZE, GFP_KERNEL);

	dev_info(&ftdi->interface->dev, "Get spi Data\n");

	/* slave.exchange(readlen=6) */
	buf[0] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[1] = 0x0d;	/* Value */
	buf[2] = 0x0b;	/* Direction */
	buf[3] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[4] = 0x05;	/* Value */
	buf[5] = 0x0b;	/* Direction */
	buf[6] = 0x20;	/* Clock Data Bytes In on +ve clock edge MSB first (no write) */
	buf[7] = 0x05;	/* Length 6 Bytes */
	buf[8] = 0x00;	/* Length 6 Bytes */
	buf[9] = 0x87;	/* Send Immediate (flush buffer back to the PC) */
	buf[10] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[11] = 0x05;	/* Value */
	buf[12] = 0x0b;	/* Direction */
	buf[13] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[14] = 0x0d;	/* Value */
	buf[15] = 0x0b;	/* Direction */
	buf[16] = CMD_SET_DATA_BITS_LOW_BYTE;
	buf[17] = 0x08;	/* Value */
	buf[18] = 0x0b;	/* Direction */
	act_len = 19;
	ret = spi_exchange_data(ftdi, buf, &act_len);
	if (ret < 0 && act_len <= 0)
		goto err;

	print_hex_dump(KERN_ALERT, "Ret: ", DUMP_PREFIX_ADDRESS,
			16, 1, buf, act_len, 1);

	if (act_len < 8) {
		pr_alert("Not get spi data.");
		goto err;
	}

	memcpy(ftdi->buffer, buf + 2, FTDI_SPI_FRAME_SIZE);
	/* cha = 0x00 00 00 01 FF FE 00 00 */
	cha = (((0x01 & buf[3]) << 16) |
		 ((0xff & buf[4]) << 8) |
		 (0xfe & buf[5])) >> 1;
	/* chb = 0x00 00 00 00 00 01 FF FE */
	chb = (((0x01 & buf[5]) << 16) |
		 ((0xff & buf[6]) << 8) |
		 (0xfe & buf[7])) >> 1;

	/* Calc value */
	/* value / 65535 * 5 * 1000 = output */
	cha = cha + (cha << 2);	/* cha *= 5; */
	cha = (cha << 10) - (cha << 4) - (cha << 3);	/* cha *= 1000 */
	cha = cha >> 16;	/* cha /= 65535 */
	chb = chb + (chb << 2);	/* chb *= 5; */
	chb = (chb << 10) - (chb << 4) - (chb << 3);	/* chb *= 1000 */
	chb = chb >> 16;	/* chb /= 65535 */

	pr_alert("CH_A: %d", cha);
	pr_alert("CH_B: %d", chb);

	kfree(buf);
	return ret;
err:
	pr_alert("%s(%u)-Err code: %d", __func__, __LINE__, ret);
	pr_alert("ACT Len: %d", act_len);
	kfree(buf);
	return ret;
}

/* Driver Part */
static int ftdi_usb_probe(struct usb_interface *interface,
			  const struct usb_device_id *id)
{
	struct usb_device *dev = interface_to_usbdev(interface);
	struct ftdi_usb *ftdi;
	int ret;
	int intf, intf_nums;

	ftdi = kzalloc(sizeof(struct ftdi_usb), GFP_KERNEL);
	if (!ftdi)
		return -ENOMEM;

	ftdi->udev = usb_get_dev(dev);
	ftdi->interface = usb_get_intf(interface);
	ftdi->buffer = kzalloc(BUFFER_SIZE, GFP_KERNEL);
	ftdi->io_timeout = FTDI_IO_TIMEOUT;
	intf = interface->cur_altsetting->desc.bInterfaceNumber;
	intf_nums = interface->cur_altsetting->desc.iInterface;
	usb_set_intfdata(interface, ftdi);

	pr_alert("Our FTDI-based device has been connected\n");
	ret = ftdi_info_get(ftdi);

	dev_info(&interface->dev, "Initialized FTDI-based device\n");
	ret = ftdi_spi_setup(ftdi);
	ret = ftdi_spi_get_data(ftdi);

	/* Init cdev */
	ftdi->dev = MKDEV(
			 MAJOR(dev_node),
			 MINOR(dev_node) + dev->portnum * intf_nums + intf);
	pr_info("Major = %d Minor = %d\n", MAJOR(ftdi->dev), MINOR(ftdi->dev));

	/* Creating cdev structure */
	cdev_init(&ftdi->cdev, &ftdi_spi_fops);
	/* Adding character device to the system */
	if ((cdev_add(&ftdi->cdev, ftdi->dev, 1)) < 0) {
		pr_err("Cannot add the device to the system\n");
		goto err_cdev_add;
	}

	/* Creating device */
	if (IS_ERR(device_create(ftdi_spi_class,
		 NULL, ftdi->dev,
		 NULL, "ftdi_spi_device_%s", dev_name(&interface->dev)))
	) {
		pr_err("Cannot create the Device ftdi_spi_device_%s\n",
			 dev_name(&interface->dev));
		goto err_dev_create;
	}

	pr_info("Device Driver Insert...Done!!!\n");

	return 0;

err_dev_create:
	cdev_del(&ftdi->cdev);
err_cdev_add:
	kfree(ftdi->buffer);
	kfree(ftdi);
	return -EFAULT;
}

static void ftdi_usb_disconnect(struct usb_interface *interface)
{
	struct ftdi_usb *ftdi;

	ftdi = (struct ftdi_usb *) usb_get_intfdata(interface);
	pr_alert("Our FTDI-based device has been disconnected\n");
	dev_info(&interface->dev, "Exit FTDI-based device\n");

	cdev_del(&ftdi->cdev);
	device_destroy(ftdi_spi_class, ftdi->dev);
	kfree(ftdi->buffer);
	kfree(ftdi);
	pr_info("Device Driver Remove...Done!!!\n");
}

static struct usb_driver ftdi_usb_driver = {
	.name = "ftdi-spi",
	.probe = ftdi_usb_probe,
	.disconnect = ftdi_usb_disconnect,
	.id_table = ftdi_id_table,
};

static int __init ftdi_usb_driver_init(void)
{
	int ret;

	/* Allocating Major number */
	if (alloc_chrdev_region(&dev_node, 0,
		 FTDI_SPI_MAX_DEVICES, "ftdi_spi_Dev") < 0) {
		pr_err("The major number exist\n");
		goto err_cdev_alloc;
	}
	pr_info("Driver Major = %d, Minor = %d\n",
		 MAJOR(dev_node), MINOR(dev_node));

	/* Creating struct class */
	ftdi_spi_class = class_create(THIS_MODULE, "ftdi_spi_class");
	if (IS_ERR(ftdi_spi_class)) {
		pr_err("Cannot create the struct class\n");
		goto err_class_create;
	}

	ret = usb_register(&ftdi_usb_driver);
	return ret;
err_class_create:
	class_destroy(ftdi_spi_class);
	unregister_chrdev_region(dev_node, FTDI_SPI_MAX_DEVICES);
err_cdev_alloc:
	return -EFAULT;

}
static void __exit ftdi_usb_driver_exit(void)
{
	usb_deregister(&ftdi_usb_driver);
	unregister_chrdev_region(dev_node, FTDI_SPI_MAX_DEVICES);
	class_destroy(ftdi_spi_class);
}

module_init(ftdi_usb_driver_init);
module_exit(ftdi_usb_driver_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FTDI USB-to-spi based driver for RTK Energy Probe Kit");
MODULE_AUTHOR("Haru Zheng <haruzheng@realtek.com>");
