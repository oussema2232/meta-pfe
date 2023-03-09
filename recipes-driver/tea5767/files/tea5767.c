/*
 * driver/media/radio/radio-tea5767.c
 *
 * Driver for tea5767 radio chip for linux 2.6.
 * This driver is for tea5767 chip from NXP, used in EZX phones from Motorola.
 * The I2C protocol is used for communicate with chip.
 *
 * Based in radio-tea5761.c Copyright (C) 2005 Nokia Corporation
 *
 *  Copyright (c) 2008 Fabio Belavenuto <belavenuto@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * History:
 * 2008-12-06   Fabio Belavenuto <belavenuto@gmail.com>
 *              initial code
 *
 * TODO:
 *  add platform_data support for IRQs platform dependencies
 *  add RDS support
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>			/* Initdata			*/
#include <linux/videodev2.h>		/* kernel radio structs		*/
#include <linux/i2c.h>			/* I2C				*/
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>

#define DRIVER_VERSION	"0.0.2"

#define DRIVER_AUTHOR	"Fabio Belavenuto <belavenuto@gmail.com>"
#define DRIVER_DESC	"A driver for the tea5767 radio chip for EZX Phones."

#define PINFO(format, ...)\
	printk(KERN_INFO KBUILD_MODNAME ": "\
		DRIVER_VERSION ": " format "\n", ## __VA_ARGS__)
#define PWARN(format, ...)\
	printk(KERN_WARNING KBUILD_MODNAME ": "\
		DRIVER_VERSION ": " format "\n", ## __VA_ARGS__)
# define PDEBUG(format, ...)\
	printk(KERN_DEBUG KBUILD_MODNAME ": "\
		DRIVER_VERSION ": " format "\n", ## __VA_ARGS__)

/* Frequency limits in MHz -- these are European values.  For Japanese
devices, that would be 76000 and 91000.  */
#define FREQ_MIN  87500U
#define FREQ_MAX 108000U
#define FREQ_MUL 16

/* tea5767 registers */
#define tea5767_MANID		0x002b
#define tea5767_CHIPID		0x5764

#define tea5767_INTREG_BLMSK	0x0001
#define tea5767_INTREG_FRRMSK	0x0002
#define tea5767_INTREG_LEVMSK	0x0008
#define tea5767_INTREG_IFMSK	0x0010
#define tea5767_INTREG_BLMFLAG	0x0100
#define tea5767_INTREG_FRRFLAG	0x0200
#define tea5767_INTREG_LEVFLAG	0x0800
#define tea5767_INTREG_IFFLAG	0x1000

#define tea5767_FRQSET_SUD	0x8000
#define tea5767_FRQSET_SM	0x4000

#define tea5767_TNCTRL_PUPD1	0x8000
#define tea5767_TNCTRL_PUPD0	0x4000
#define tea5767_TNCTRL_BLIM	0x2000
#define tea5767_TNCTRL_SWPM	0x1000
#define tea5767_TNCTRL_IFCTC	0x0800
#define tea5767_TNCTRL_AFM	0x0400
#define tea5767_TNCTRL_SMUTE	0x0200
#define tea5767_TNCTRL_SNC	0x0100
#define tea5767_TNCTRL_MU	0x0080
#define tea5767_TNCTRL_SSL1	0x0040
#define tea5767_TNCTRL_SSL0	0x0020
#define tea5767_TNCTRL_HLSI	0x0010
#define tea5767_TNCTRL_MST	0x0008
#define tea5767_TNCTRL_SWP	0x0004
#define tea5767_TNCTRL_DTC	0x0002
#define tea5767_TNCTRL_AHLSI	0x0001

#define tea5767_TUNCHK_LEVEL(x)	(((x) & 0x00F0) >> 4)
#define tea5767_TUNCHK_IFCNT(x) (((x) & 0xFE00) >> 9)
#define tea5767_TUNCHK_TUNTO	0x0100
#define tea5767_TUNCHK_LD	0x0008
#define tea5767_TUNCHK_STEREO	0x0004

#define tea5767_TESTREG_TRIGFR	0x0800

struct tea5767_regs {
	u16 intreg;				/* INTFLAG & INTMSK */
	u16 frqset;				/* FRQSETMSB & FRQSETLSB */
	u16 tnctrl;				/* TNCTRL1 & TNCTRL2 */
	u16 frqchk;				/* FRQCHKMSB & FRQCHKLSB */
	u16 tunchk;				/* IFCHK & LEVCHK */
	u16 testreg;				/* TESTBITS & TESTMODE */
	u16 rdsstat;				/* RDSSTAT1 & RDSSTAT2 */
	u16 rdslb;				/* RDSLBMSB & RDSLBLSB */
	u16 rdspb;				/* RDSPBMSB & RDSPBLSB */
	u16 rdsbc;				/* RDSBBC & RDSGBC */
	u16 rdsctrl;				/* RDSCTRL1 & RDSCTRL2 */
	u16 rdsbbl;				/* PAUSEDET & RDSBBL */
	u16 manid;				/* MANID1 & MANID2 */
	u16 chipid;				/* CHIPID1 & CHIPID2 */
} __attribute__ ((packed));

struct tea5767_write_regs {
	u8 intreg;				/* INTMSK */
	__be16 frqset;				/* FRQSETMSB & FRQSETLSB */
	__be16 tnctrl;				/* TNCTRL1 & TNCTRL2 */
	__be16 testreg;				/* TESTBITS & TESTMODE */
	__be16 rdsctrl;				/* RDSCTRL1 & RDSCTRL2 */
	__be16 rdsbbl;				/* PAUSEDET & RDSBBL */
} __attribute__ ((packed));

#ifdef CONFIG_RADIO_tea5767_XTAL
#define RADIO_tea5767_XTAL 1
#else
#define RADIO_tea5767_XTAL 0
#endif

static int radio_nr = -1;
static int use_xtal = RADIO_tea5767_XTAL;

struct tea5767_device {
	struct v4l2_device		v4l2_dev;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct i2c_client		*i2c_client;
	struct video_device		vdev;
	struct tea5767_regs		regs;
	struct mutex			mutex;
};

/* I2C code related */
static int tea5767_i2c_read(struct tea5767_device *radio)
{
	int i;
	u16 *p = (u16 *) &radio->regs;

	struct i2c_msg msgs[1] = {
		{	.addr = radio->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(radio->regs),
			.buf = (void *)&radio->regs
		},
	};
	if (i2c_transfer(radio->i2c_client->adapter, msgs, 1) != 1)
		return -EIO;
	for (i = 0; i < sizeof(struct tea5767_regs) / sizeof(u16); i++)
		p[i] = __be16_to_cpu((__force __be16)p[i]);

	return 0;
}

static int tea5767_i2c_write(struct tea5767_device *radio)
{
	struct tea5767_write_regs wr;
	struct tea5767_regs *r = &radio->regs;
	struct i2c_msg msgs[1] = {
		{
			.addr = radio->i2c_client->addr,
			.len = sizeof(wr),
			.buf = (void *)&wr
		},
	};
	wr.intreg  = r->intreg & 0xff;
	wr.frqset  = __cpu_to_be16(r->frqset);
	wr.tnctrl  = __cpu_to_be16(r->tnctrl);
	wr.testreg = __cpu_to_be16(r->testreg);
	wr.rdsctrl = __cpu_to_be16(r->rdsctrl);
	wr.rdsbbl  = __cpu_to_be16(r->rdsbbl);
	if (i2c_transfer(radio->i2c_client->adapter, msgs, 1) != 1)
		return -EIO;
	return 0;
}

static void tea5767_power_up(struct tea5767_device *radio)
{
	struct tea5767_regs *r = &radio->regs;

	if (!(r->tnctrl & tea5767_TNCTRL_PUPD0)) {
		r->tnctrl &= ~(tea5767_TNCTRL_AFM | tea5767_TNCTRL_MU |
			       tea5767_TNCTRL_HLSI);
		if (!use_xtal)
			r->testreg |= tea5767_TESTREG_TRIGFR;
		else
			r->testreg &= ~tea5767_TESTREG_TRIGFR;

		r->tnctrl |= tea5767_TNCTRL_PUPD0;
		tea5767_i2c_write(radio);
	}
}

static void tea5767_power_down(struct tea5767_device *radio)
{
	struct tea5767_regs *r = &radio->regs;

	if (r->tnctrl & tea5767_TNCTRL_PUPD0) {
		r->tnctrl &= ~tea5767_TNCTRL_PUPD0;
		tea5767_i2c_write(radio);
	}
}

static void tea5767_set_freq(struct tea5767_device *radio, int freq)
{
	struct tea5767_regs *r = &radio->regs;

	/* formula: (freq [+ or -] 225000) / 8192 */
	if (r->tnctrl & tea5767_TNCTRL_HLSI)
		r->frqset = (freq + 225000) / 8192;
	else
		r->frqset = (freq - 225000) / 8192;
}

static int tea5767_get_freq(struct tea5767_device *radio)
{
	struct tea5767_regs *r = &radio->regs;

	if (r->tnctrl & tea5767_TNCTRL_HLSI)
		return (r->frqchk * 8192) - 225000;
	else
		return (r->frqchk * 8192) + 225000;
}

/* tune an frequency, freq is defined by v4l's TUNER_LOW, i.e. 1/16th kHz */
static void tea5767_tune(struct tea5767_device *radio, int freq)
{
	tea5767_set_freq(radio, freq);
	if (tea5767_i2c_write(radio))
		PWARN("Could not set frequency!");
}

static void tea5767_set_audout_mode(struct tea5767_device *radio, int audmode)
{
	struct tea5767_regs *r = &radio->regs;
	int tnctrl = r->tnctrl;

	if (audmode == V4L2_TUNER_MODE_MONO)
		r->tnctrl |= tea5767_TNCTRL_MST;
	else
		r->tnctrl &= ~tea5767_TNCTRL_MST;
	if (tnctrl != r->tnctrl)
		tea5767_i2c_write(radio);
}

static int tea5767_get_audout_mode(struct tea5767_device *radio)
{
	struct tea5767_regs *r = &radio->regs;

	if (r->tnctrl & tea5767_TNCTRL_MST)
		return V4L2_TUNER_MODE_MONO;
	else
		return V4L2_TUNER_MODE_STEREO;
}

static void tea5767_mute(struct tea5767_device *radio, int on)
{
	struct tea5767_regs *r = &radio->regs;
	int tnctrl = r->tnctrl;

	if (on)
		r->tnctrl |= tea5767_TNCTRL_MU;
	else
		r->tnctrl &= ~tea5767_TNCTRL_MU;
	if (tnctrl != r->tnctrl)
		tea5767_i2c_write(radio);
}

/* V4L2 vidioc */
static int vidioc_querycap(struct file *file, void  *priv,
					struct v4l2_capability *v)
{
	struct tea5767_device *radio = video_drvdata(file);
	struct video_device *dev = &radio->vdev;

	strlcpy(v->driver, dev->dev.driver->name, sizeof(v->driver));
	strlcpy(v->card, dev->name, sizeof(v->card));
	snprintf(v->bus_info, sizeof(v->bus_info),
		 "I2C:%s", dev_name(&dev->dev));
	v->device_caps = V4L2_CAP_TUNER | V4L2_CAP_RADIO;
	v->capabilities = v->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int vidioc_g_tuner(struct file *file, void *priv,
				struct v4l2_tuner *v)
{
	struct tea5767_device *radio = video_drvdata(file);
	struct tea5767_regs *r = &radio->regs;

	if (v->index > 0)
		return -EINVAL;

	strlcpy(v->name, "FM", sizeof(v->name));
	v->type = V4L2_TUNER_RADIO;
	tea5767_i2c_read(radio);
	v->rangelow   = FREQ_MIN * FREQ_MUL;
	v->rangehigh  = FREQ_MAX * FREQ_MUL;
	v->capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO;
	if (r->tunchk & tea5767_TUNCHK_STEREO)
		v->rxsubchans = V4L2_TUNER_SUB_STEREO;
	else
		v->rxsubchans = V4L2_TUNER_SUB_MONO;
	v->audmode = tea5767_get_audout_mode(radio);
	v->signal = tea5767_TUNCHK_LEVEL(r->tunchk) * 0xffff / 0xf;
	v->afc = tea5767_TUNCHK_IFCNT(r->tunchk);

	return 0;
}

static int vidioc_s_tuner(struct file *file, void *priv,
				const struct v4l2_tuner *v)
{
	struct tea5767_device *radio = video_drvdata(file);

	if (v->index > 0)
		return -EINVAL;

	tea5767_set_audout_mode(radio, v->audmode);
	return 0;
}

static int vidioc_s_frequency(struct file *file, void *priv,
				const struct v4l2_frequency *f)
{
	struct tea5767_device *radio = video_drvdata(file);
	unsigned freq = f->frequency;

	if (f->tuner != 0 || f->type != V4L2_TUNER_RADIO)
		return -EINVAL;
	if (freq == 0) {
		/* We special case this as a power down control. */
		tea5767_power_down(radio);
		/* Yes, that's what is returned in this case. This
		   whole special case is non-compliant and should really
		   be replaced with something better, but changing this
		   might well break code that depends on this behavior.
		   So we keep it as-is. */
		return -EINVAL;
	}
	freq = clamp(freq, FREQ_MIN * FREQ_MUL, FREQ_MAX * FREQ_MUL);
	tea5767_power_up(radio);
	tea5767_tune(radio, (freq * 125) / 2);
	return 0;
}

static int vidioc_g_frequency(struct file *file, void *priv,
				struct v4l2_frequency *f)
{
	struct tea5767_device *radio = video_drvdata(file);
	struct tea5767_regs *r = &radio->regs;

	if (f->tuner != 0)
		return -EINVAL;
	tea5767_i2c_read(radio);
	f->type = V4L2_TUNER_RADIO;
	if (r->tnctrl & tea5767_TNCTRL_PUPD0)
		f->frequency = (tea5767_get_freq(radio) * 2) / 125;
	else
		f->frequency = 0;

	return 0;
}

static int tea5767_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tea5767_device *radio =
		container_of(ctrl->handler, struct tea5767_device, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_MUTE:
		tea5767_mute(radio, ctrl->val);
		return 0;
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops tea5767_ctrl_ops = {
	.s_ctrl = tea5767_s_ctrl,
};

/* File system interface */
static const struct v4l2_file_operations tea5767_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= v4l2_fh_release,
	.poll		= v4l2_ctrl_poll,
	.unlocked_ioctl	= video_ioctl2,
};

static const struct v4l2_ioctl_ops tea5767_ioctl_ops = {
	.vidioc_querycap    = vidioc_querycap,
	.vidioc_g_tuner     = vidioc_g_tuner,
	.vidioc_s_tuner     = vidioc_s_tuner,
	.vidioc_g_frequency = vidioc_g_frequency,
	.vidioc_s_frequency = vidioc_s_frequency,
	.vidioc_log_status  = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/* V4L2 interface */
static const struct video_device tea5767_radio_template = {
	.name		= "tea5767",
	.fops           = &tea5767_fops,
	.ioctl_ops 	= &tea5767_ioctl_ops,
	.release	= video_device_release_empty,
};

/* I2C probe: check if the device exists and register with v4l if it is */
static int tea5767_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct tea5767_device *radio;
	struct v4l2_device *v4l2_dev;
	struct v4l2_ctrl_handler *hdl;
	struct tea5767_regs *r;
	int ret;

	PDEBUG("probe");
	printk("PROBE1\n");
	radio = kzalloc(sizeof(struct tea5767_device), GFP_KERNEL);
	printk("PROBE2\n");
	if (!radio)
		return -ENOMEM;

	v4l2_dev = &radio->v4l2_dev;
	printk("PROBE3\n");
	ret = v4l2_device_register(&client->dev, v4l2_dev);
	printk("PROBE4\n");
	if (ret < 0) {
		v4l2_err(v4l2_dev, "could not register v4l2_device\n");
		printk("PROBE5\n");
		goto errfr;
	}
	printk("PROBE6\n");
	hdl = &radio->ctrl_handler;
	printk("PROBE7\n");
	v4l2_ctrl_handler_init(hdl, 1);
	printk("PROBE8\n");
	v4l2_ctrl_new_std(hdl, &tea5767_ctrl_ops,
			V4L2_CID_AUDIO_MUTE, 0, 1, 1, 1);
	printk("PROBE9\n");
	v4l2_dev->ctrl_handler = hdl;
	printk("PROBE10\n");
	if (hdl->error) {
		ret = hdl->error;
		printk("PROBE11\n");
		v4l2_err(v4l2_dev, "Could not register controls\n");
		printk("PROBE12\n");
		goto errunreg;
	}

	mutex_init(&radio->mutex);
	printk("PROBE13\n");
	radio->i2c_client = client;
	printk("PROBE14\n");
	ret = tea5767_i2c_read(radio);
	printk("PROBE15\n");
	if (ret){
		printk("PROBE16\n");
		goto errunreg;
		}
	r = &radio->regs;

	radio->vdev = tea5767_radio_template;

	i2c_set_clientdata(client, radio);
	video_set_drvdata(&radio->vdev, radio);
	radio->vdev.lock = &radio->mutex;
	radio->vdev.v4l2_dev = v4l2_dev;

	/* initialize and power off the chip */
	tea5767_i2c_read(radio);
	tea5767_set_audout_mode(radio, V4L2_TUNER_MODE_STEREO);
	tea5767_mute(radio, 1);
	tea5767_power_down(radio);


	PINFO("registered.");
	return 0;
errunreg:
	v4l2_ctrl_handler_free(hdl);
	v4l2_device_unregister(v4l2_dev);
errfr:
	kfree(radio);
	return ret;
}

static int tea5767_i2c_remove(struct i2c_client *client)
{
	struct tea5767_device *radio = i2c_get_clientdata(client);

	PDEBUG("remove");
	if (radio) {
		tea5767_power_down(radio);
		video_unregister_device(&radio->vdev);
		v4l2_ctrl_handler_free(&radio->ctrl_handler);
		v4l2_device_unregister(&radio->v4l2_dev);
		kfree(radio);
	}
	return 0;
}
static const struct of_device_id tea5767_dt_ids[] = {
	{ .compatible = "actia,tea5767",},
	{ }
};
MODULE_DEVICE_TABLE(of, tea5767_dt_ids);

/* I2C subsystem interface */
static const struct i2c_device_id tea5767_id[] = {
	{ "tea5767", 0 },
	{ }					/* Terminating entry */
};
MODULE_DEVICE_TABLE(i2c, tea5767_id);

static struct i2c_driver tea5767_i2c_driver = {
	.driver = {
		.name = "tea5767",
		.of_match_table = tea5767_dt_ids,
	},
	.probe = tea5767_i2c_probe,
	.remove = tea5767_i2c_remove,
	.id_table = tea5767_id,
};

module_i2c_driver(tea5767_i2c_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_param(use_xtal, int, 0);
MODULE_PARM_DESC(use_xtal, "Chip have a xtal connected in board");
module_param(radio_nr, int, 0);
MODULE_PARM_DESC(radio_nr, "video4linux device number to use");
