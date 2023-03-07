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

#define TEA5767_I2C_ADDRESS 0x60
#define DRIVER_NAME "tea5767"
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

static int tea5767_i2c_probe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{
    struct tea5767_device *radio;
    struct v4l2_device *v4l2_dev;
    struct v4l2_ctrl_handler *hdl;
    int ret;
    printk("ERROR\n");
    radio = kzalloc(sizeof(struct tea5767_device), GFP_KERNEL);
    if (!radio)
        return -ENOMEM;
     printk("ERROR2\n");
    v4l2_dev = &radio->v4l2_dev;
     printk("ERROR3\n");
    if (!client) {
    dev_err(&client->dev, "client is NULL\n");
    printk("ERROR\n");
    return -EINVAL;
}


    ret = v4l2_device_register(&client->dev, v4l2_dev);
    if (ret < 0) {
        v4l2_err(v4l2_dev, "could not register v4l2_device\n");
        printk("%d",ret);
        goto errfr;
    }

    hdl = &radio->ctrl_handler;
    v4l2_ctrl_handler_init(hdl, 1);
    v4l2_ctrl_new_std(hdl, &tea5767_ctrl_ops,
            V4L2_CID_AUDIO_MUTE, 0, 1, 1, 1);
    v4l2_dev->ctrl_handler = hdl;

    if (hdl->error) {
        ret = hdl->error;
        v4l2_err(v4l2_dev, "Could not register controls\n");
        goto errunreg;
    }

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
    { "tea5767", 0,},
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

static int __init ModuleInit(void) {
	struct i2c_client *client;
	printk("this is init1\n");
	tea5767_i2c_probe(client, tea5767_id_table);
	printk("this is init2\n");
	return i2c_add_driver(&tea5767);
	}


static void __exit ModuleExit(void) {
	printk("GoodBye Kernel\n");
}

module_init(ModuleInit);
module_exit(ModuleExit);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Linux driver for the TEA5767 FM radio module");
MODULE_LICENSE("GPL"); 
