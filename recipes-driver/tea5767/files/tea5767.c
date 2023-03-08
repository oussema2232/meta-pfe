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
#define tea5767_MANID		0xff
#define tea5767_CHIPID		0xff

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
struct tea5767_device {
	struct v4l2_device		v4l2_dev;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct i2c_client		*i2c_client;
	struct video_device		vdev;
	struct tea5767_regs		regs;
	struct mutex			mutex;
};


static const struct v4l2_ctrl_ops tea5767_ctrl_ops = {
    .s_ctrl = NULL,
};
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
static int tea5767_i2c_probe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{
    struct tea5767_device *radio;
    struct v4l2_device *v4l2_dev;
    struct v4l2_ctrl_handler *hdl;
    struct tea5767_regs *r;
    int ret;
    printk("ERROR\n");
    radio = kzalloc(sizeof(struct tea5767_device),GFP_KERNEL);
    if (!radio)
        return -ENOMEM;
     printk("ERROR2\n");
    v4l2_dev = &radio->v4l2_dev;
     printk("ERROR3\n");
    if (!client) {
    dev_err(&client->dev, "client is NULL\n");
    printk("ERROR4\n");
    return -EINVAL;
}


    ret = v4l2_device_register(&client->dev, v4l2_dev);
    printk("ERROR5\n");
    printk("%d\n",ret);
    if (ret < 0) {
        v4l2_err(v4l2_dev, "could not register v4l2_device\n");
        printk("ERROR6\n");
        printk("%d",ret);
        goto errfr;
    }
    else {
    printk(KERN_INFO "Successfully registered v4l2 device\n");
}

    hdl = &radio->ctrl_handler;
    printk("ERROR7\n");
    v4l2_ctrl_handler_init(hdl, 1);
    printk("ERROR8\n");
    v4l2_ctrl_new_std(hdl, &tea5767_ctrl_ops,
            V4L2_CID_AUDIO_MUTE, 0, 1, 1, 1);
    v4l2_dev->ctrl_handler = hdl;

    if (hdl->error) {
        ret = hdl->error;
        v4l2_err(v4l2_dev, "Could not register controls\n");
        goto errunreg;
    }
    mutex_init(&radio->mutex);
    radio->i2c_client = client;
    ret = tea5767_i2c_read(radio);
    if (ret)
		goto errunreg;
    i2c_set_clientdata(client, radio);
    printk("ERROR8\n");
    video_set_drvdata(&radio->vdev, radio);
    radio->vdev.lock = &radio->mutex;
    radio->vdev.v4l2_dev = v4l2_dev;
    tea5767_i2c_read(radio);
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
    printk("remove here\n");
    return 0;
}
static const struct of_device_id tea5767_dt_ids[] = {
	{ .compatible = "actia,tea5767",},
	{ }
};
MODULE_DEVICE_TABLE(of, tea5767_dt_ids);

static const struct i2c_device_id tea5767_id_table[] = {
    { .name = "tea5767"},
    { }
};

MODULE_DEVICE_TABLE(i2c, tea5767_id_table);


static struct i2c_driver tea5767 = {
    .probe = tea5767_i2c_probe,
    .remove = tea5767_i2c_remove,
    .id_table = tea5767_id_table,
    .driver = {
        .name = "tea5767",
        .owner = THIS_MODULE,
        .of_match_table = tea5767_dt_ids,
    },
};
module_i2c_driver(tea5767);



MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Linux driver for the TEA5767 FM radio module");
MODULE_LICENSE("GPL");
