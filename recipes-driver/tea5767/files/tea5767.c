#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define TEA5767_I2C_ADDRESS 0x60
#define DRIVER_NAME "tea5767"

struct tea5767_device {
    struct v4l2_device v4l2_dev;
    struct v4l2_ctrl_handler ctrl_handler;
    struct mutex mutex;
    struct i2c_client *i2c_client;
};

static const struct v4l2_ctrl_ops tea5767_ctrl_ops = {
    .s_ctrl = NULL,
};

static int tea5767_i2c_probe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{
    printk("probe here  \n");
    return 0;
}

static int tea5767_i2c_remove(struct i2c_client *client)
{
    printk("remove here\n");
    return 0;
}
static const struct of_device_id tea5767_dt_ids[] = {
	{ .compatible = "actia,tea5767","brcm,bcm2711",},
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
	tea5767_i2c_probe(client, &tea5767_id_table[0]);
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
