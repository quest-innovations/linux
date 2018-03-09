
#include "quest_dma_core.h"
#include <linux/dma/quest_dma_core_ioctl.h>

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/of_platform.h>
#include <linux/delay.h>

#define CH_CNT 1
#define BUF_CNT 10


static struct device *dev_reg;

static dev_t dev;
static struct cdev c_dev;
static struct class *cl;

static struct quest_dma_struct qdma;

typedef enum
{
    IpCoreId        = 0x00,
    ClkSpeed        = 0x01,
    Status          = 0x02,
    Control         = 0x03,
    ImageLength     = 0x04,
    ImageWidth      = 0x05,
    ImageHeight     = 0x06,
    PointerSelect   = 0x07,
    Reserved0       = 0x08,
    Reserved1       = 0x09,
    Reserved2       = 0x0A,
    Reserved3       = 0x0B,
    Pointer0        = 0x0C,
    Pointer1        = 0x0D,
    Pointer2        = 0x0E,
    Pointer3        = 0x0F
} SensorDmaRegs;

typedef enum
{
    InitDone    = 1 << 0x00,
    Start       = 1 << 0x01,
    IrqAck      = 1 << 0x02
} ControlField;

typedef enum
{
    IpReady     = 1 << 0x00,
    Busy        = 1 << 0x01,
    ImageDone   = 1 << 0x02,
    Error       = 16
} StatusField;

/**
 * quest_clink_in_dma_getreg - Gets a register value
 * @offset: Offset of addr to read
 *
 * Return: value of the requested register
 */
static u32 quest_clink_in_dma_getreg(u32 offset)
{
	return quest_dma_core_getreg(&qdma, offset);
}

/**
 * quest_clink_in_dma_getreg - Gets a register value
 * @offset: Offset of addr to write
 * @value: Value to write
 *
 * Return: value of the requested register
 */
static void quest_clink_in_dma_setreg(u32 offset, u32 value)
{
	quest_dma_core_setreg(&qdma, offset, value);
}

static DECLARE_WAIT_QUEUE_HEAD(data_wait);
/**
 * quest_clink_in_dma_read - Starts read of selected channel, if not started, and returns when data is available
 * or when stop read is called
 * @ch: DMA channel
 * @buf: User space buffer to copy data to
 * @sizeBytes: Size in bytes of the user space buffer
 *
 * Return: TODO
 */
static int quest_clink_in_dma_read(unsigned int ch, u8 *buf, bool* buf_full, unsigned int sizeBytes)
{
	struct quest_dma_channel_struct *chan;

	// pr_info("Reading data\n");

	if(ch >= qdma.ch_cnt)
		return -1;

	if(buf == 0 ||
	        buf_full == 0)
		return -1;

	if(!qdma.reading)
	{
		// pr_info("Not reading\n");
	}

	chan = &qdma.channels[ch];

	if(chan->last_read == chan->last_write) {
		int result;

		// printk("GW   - Waiting for next image");
		result = wait_event_interruptible(data_wait, chan->data_ready || qdma.stop_reading);

		if(qdma.stop_reading) {
			// pr_info("STOPPING THE READ");
			return -1;
		} else if(chan->data_ready) {
			//printk("DW   - Done with waiting for frame ready!");
			chan->data_ready = false; // Reset frame ready signal
		} else {
			//pr_info("    Woken up by? %d", result);
			return result;
		}
	} else {
		// printk("    Catching up on images lr: %d lw: %d", chan->last_read, chan->last_write);

		if(chan->data_ready) // If frame ready was set while not waiting for a frame, reset the flag
			chan->data_ready = false;
	}

	if(chan->last_read == chan->bufs_cnt - 1)
		chan->last_read = 0;
	else
		++chan->last_read;

	/*if(chan->data_skipped) {
		if (copy_to_user(buf->data_skipped, &chan->data_skipped, sizeof(bool))) {
			//pr_info("Oops, frame_skipped transfer went wrong :(");
			return -EACCES;
		}
		chan->data_skipped = false; // Reset skip bit
	}*/

	//if(!dmatest_check_srcs_quest(chan->bufs[chan->last_read], chan->check_range, chan->image_size_bytes + chan->check_range, chan->bufs_size))
	////pr_info("Buffer overrun!");

	//--//printk("GRAB - Last read: %d > %d Last write: %d", prevLastRead, chan->last_read, chan->last_write);

	if (copy_to_user(buf, chan->bufs_user[chan->last_read], chan->data_size_bytes)) {
		// pr_info("Oops, transfer went wrong :(");
		return -EACCES;
	}

	if(chan->buffers_full) {
		if (copy_to_user(buf_full, &chan->buffers_full, sizeof(chan->buffers_full))) {
			// pr_info("Oops, transfer went wrong :(");
			return -EACCES;
		}
	}

	// pr_info("Reading done\n");

	return 0;
}

/**
 * quest_clink_in_dma_stop_read - Stops the reading of data
 *
 * Return: TODO
 */
static int quest_clink_in_dma_stop_read(void)
{
	u32 control = quest_clink_in_dma_getreg(Control);
	quest_clink_in_dma_setreg(Control, control & ~InitDone);

	qdma.stop_reading = true;
	wake_up_interruptible(&data_wait); // Awake process to return

	quest_clink_in_dma_setreg(Control, control | InitDone);

	return 0;
}

/**
 * quest_clink_in_dma_set_ch_size - Set the size of the incoming data of a channel
 *
 * Return: TODO
 */
static int quest_clink_in_dma_set_ch_size(unsigned int ch, unsigned int sizeBytes)
{
	int err;

	// pr_info("Ch size start \n");

	if(ch >= qdma.ch_cnt)
		return -1;

	// pr_info("Ch size \n");

	if(qdma.channels[ch].data_size_bytes == sizeBytes)
		return 0; // Nothing to be done

	// pr_info("Ch size \n");

	quest_clink_in_dma_setreg(Control, 0); // Put DMA module in init state

	if(qdma.channels[ch].data_size_bytes != 0)
		quest_dma_core_destroy_buffers(dev_reg, &qdma.channels[ch]); // Needs destroying before creating

	// pr_info("Ch size \n");

	if((err = quest_dma_core_create_buffers(dev_reg, &qdma.channels[ch], BUF_CNT, sizeBytes)) != 0)
	{
		// pr_info("Ch size err \n");
		quest_dma_core_destroy_buffers(dev_reg, &qdma.channels[ch]);
		return err;
	}

	quest_clink_in_dma_setreg(Pointer0, qdma.channels[ch].bufs_raw[0]);

	quest_clink_in_dma_setreg(Control, InitDone); // Set init done

	return 0;
}

static int quest_clink_in_dma_start_dma(struct quest_dma_channel_struct *chan)
{
	u32 control = 0;

	// pr_info("Starting dma\n");

	// TODO add return on start error

	// Set read and write to last buffer so they begin at 0 at a write
	chan->last_read = chan->bufs_cnt - 1;
	chan->last_write = chan->bufs_cnt - 1;

	quest_clink_in_dma_setreg(Control, IrqAck); // Module does not reset if still in irq
	quest_clink_in_dma_setreg(Control, 0); // Reset
	quest_clink_in_dma_setreg(Control, InitDone);
	quest_clink_in_dma_setreg(Pointer0, chan->bufs_raw[0]);
	control = quest_clink_in_dma_getreg(Control);
	quest_clink_in_dma_setreg(Control, control | Start);

	return 0;
}

/**
 * quest_clink_in_dma_drive_read - Drives the reading of data of the selected channel, keeps reading
 * untill stop or write is called
 * @ch: DMA channel
 *
 * Return: TODO
 */
static int quest_clink_in_dma_drive_read(struct quest_dma_channel_struct *chan)
{
	u32 control;
	u32 prevLastWrite;
	u32 nextWrite;
	u32 last_write_chk_buf;
	int err;

	// pr_info("In drive 1\n");

	if(chan->bufs_cnt < 3) // Reading process requires 3 buffers
		return -1;

	// pr_info("In drive 2\n");

	if(qdma.reading)
		quest_clink_in_dma_stop_read();

	usleep_range(100, 200); // Wait for process to return if another process was grabbing

	if((err = quest_clink_in_dma_start_dma(chan)) != 0)
		return err;

	qdma.reading = true;

	// pr_info("Drive started\n");

	while(!qdma.stop_reading) {
		if(!chan->buffers_full &&
		        quest_clink_in_dma_getreg(Status) & ImageDone) {
			pr_debug("Image received!");

			prevLastWrite = chan->last_write;

			// Check if skipping a frame read is required
			last_write_chk_buf = chan->last_write + 2; // + 2 to prevent race condition in read

			if(last_write_chk_buf >= chan->bufs_cnt)
				last_write_chk_buf -= (chan->bufs_cnt - 1);
			if(last_write_chk_buf == chan->last_read)
			{
				// pr_info("%s Buffers full", __func__);
				chan->buffers_full = true;
				continue;
				/*if(chan->last_read == chan->bufs_cnt - 1)
					chan->last_read = 0;
				else
					++chan->last_read;*/
			}

			if(chan->last_write == chan->bufs_cnt - 1)
				chan->last_write = 0;
			else
				++chan->last_write;

			// Flag buffer recieved AFTER increasing counter to ensure if read is waiting for frame it is processed correctly
			if(prevLastWrite == chan->last_read) // If read if waiting for a frame, signal it
				chan->data_ready = true;

			if(chan->last_write == chan->bufs_cnt - 1)
				nextWrite = 0;
			else
				nextWrite = chan->last_write + 1;

			//----//printk("IMG  - LR: %d LW: %d > %d Next: %d", chan->last_read, prevLastWrite, chan->last_write, nextWrite);

			// Write next phys buffer ptr in sensor DMA

			//dmatest_init_srcs_quest_single(chan->bufs[nextWrite], chan->check_range, chan->image_size_bytes, chan->bufs_size);

			//--//pr_info("    %s Writing next phys buffer %d lazy begin(-16B): %d, lazy end (-16B): %d", __func__, chan->bufs_raw[nextWrite], chan->bufs_raw[nextWrite] - 16, chan->bufs_raw[nextWrite] + chan->image_size_bytes - 16);
			quest_clink_in_dma_setreg(Pointer0, chan->bufs_raw[nextWrite]);

			// Acknowledge IRQ
			////pr_info("    %s Acknowledge IRQ", __func__);
			control = quest_clink_in_dma_getreg(Control);
			quest_clink_in_dma_setreg(Control, control | IrqAck);

			// Wait for image done to be low
			if(quest_clink_in_dma_getreg(Status) & (ImageDone | Busy)) {
				// pr_info("!!!!!Image done or busy still high 1!!!!!!!!");
				msleep(1000); // Wait for a bit
			}
			// Wait for image done to be low
			if(quest_clink_in_dma_getreg(Status) & (ImageDone | Busy)) {
				// pr_info("!!!!!Image done or busy still high. Control: %d Status: %d", quest_clink_in_dma_getreg(Control), quest_clink_in_dma_getreg(Status));
				msleep(1); // Wait for a bit
				break;
			}

			////pr_info("    %s Start", __func__);
			quest_clink_in_dma_setreg(Control, control | Start);

			wake_up_interruptible(&data_wait);
		} else {
			usleep_range(100, 200);
		}
	}

	wake_up_interruptible(&data_wait); // Awake process to return
	msleep(10); // Wait for grab function to see the stop flag before resetting it

	qdma.stop_reading = false;    // Reset stop flag
	qdma.reading = false;

	chan->data_ready = false;	// Reset frame ready flag
	chan->buffers_full = false;	// Reset full flag

	// pr_info("Stopping drive grabbing");

	return 0;
}

/**
 * quest_clink_in_dma_ioctl - ioctl entry point
 *
 * Return: Various
 */
static long quest_clink_in_dma_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	qdma_buf_arg_t buf;
	qdma_reg_arg_t reg;

	switch (cmd) {
	case QDMA_READ:
		// pr_info("QDMA_READ \n");
		if (copy_from_user(&buf, (qdma_buf_arg_t *)arg, sizeof(buf)))
			return -EACCES;

		return quest_clink_in_dma_read(buf.ch, buf.buf, buf.buffers_full, buf.size_bytes);
	case QDMA_STOP_READ:
		// pr_info("QDMA_STOP_READ \n");
		quest_clink_in_dma_stop_read();
		break;
	case QDMA_SET_CH_READ_SIZE:
		// pr_info("QDMA_SET_CH_READ_SIZE \n");
		if (copy_from_user(&buf, (qdma_buf_arg_t *)arg, sizeof(buf)))
			return -EACCES;

		return quest_clink_in_dma_set_ch_size(buf.ch, buf.size_bytes);
	case QDMA_DRIVE_READ:
		// pr_info("QDMA_DRIVE_READ \n");
		if (copy_from_user(&buf, (qdma_buf_arg_t *)arg, sizeof(buf)))
			return -EACCES;

		// pr_info("DRIVE %d", buf.size_bytes);

		if(buf.ch >= qdma.ch_cnt)
			return -1;

		return quest_clink_in_dma_drive_read(&qdma.channels[buf.ch]);
	case QDMA_GETREG:
		// pr_info("QDMA_GETREG \n");
		if (copy_from_user(&reg, (qdma_reg_arg_t *)arg, sizeof(reg)))
			return -EACCES;

		reg.value = quest_clink_in_dma_getreg(reg.offset);

		if (copy_to_user((qdma_reg_arg_t *)arg, &reg, sizeof(reg)))
			return -EACCES;

		return 0;
	case QDMA_SETREG:
		// pr_info("QDMA_SETREG \n");
		if (copy_from_user(&reg, (qdma_reg_arg_t *)arg, sizeof(reg)))
			return -EACCES;

		quest_clink_in_dma_setreg(reg.offset, reg.value);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int quest_clink_in_dma_open(struct inode *i, struct file *f)
{
    return 0;
}
static int quest_clink_in_dma_close(struct inode *i, struct file *f)
{
    return 0;
}

struct file_operations quest_clink_in_dma_fops = {
	open:   quest_clink_in_dma_open,
	unlocked_ioctl: quest_clink_in_dma_ioctl,
	release: quest_clink_in_dma_close
};

static int quest_clink_in_dma_create_channels(void)
{
	int i = 0;

	qdma.channels = kcalloc(CH_CNT, sizeof(*qdma.channels), GFP_KERNEL);

	if(!qdma.channels)
		return -1;

	for(; i < CH_CNT; ++i) {
		qdma.channels[i].bufs_size = 0;
		qdma.channels[i].bufs_cnt = 0;
		qdma.channels[i].data_size_bytes = 0;
		qdma.channels[i].buffers_full = false;
		qdma.channels[i].data_ready = false;
	}

	qdma.ch_cnt = CH_CNT;

	return 0;
}

static int quest_clink_in_dma_destroy_channels(void)
{
	int i = 0;

	for(; i < CH_CNT; ++i)
		quest_dma_core_destroy_buffers(dev_reg, &qdma.channels[i]);

	kfree(qdma.channels);

	return 0;
}

static int quest_clink_in_dma_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev_creation_ret;
	char *char_dev_name;

	struct device_node * node = pdev->dev.of_node;

	struct property *prop;

	/*irq_handler_t handler;
	unsigned long flags;
	const char *name;*/

	struct resource *reg_res;

	dev_reg = &pdev->dev;

	// Initialze dma structure
	qdma.stop_reading = false;
	qdma.reading = false;
	qdma.irqn = 0;

	// pr_info("Creating channels");

	if(quest_clink_in_dma_create_channels() == -1)
		return -1;

	// pr_info("Creating char dev");

	prop = of_find_property(node, "device", NULL);

	// pr_info("tests %s \n", (char *)prop->value);

	char_dev_name = (char *)prop->value;

	// pr_info("Device create\n");

	// Character device creation
	if ((ret = alloc_chrdev_region(&dev, FIRST_MINOR, MINOR_CNT, "quest_clink_in_dma_ioctl")) < 0)
	   return ret;

	cdev_init(&c_dev, &quest_clink_in_dma_fops);

	// pr_info("Device create\n");

	if ((ret = cdev_add(&c_dev, dev, MINOR_CNT)) < 0)
	   return ret;

	// pr_info("Device create\n");

	if (IS_ERR(cl = class_create(THIS_MODULE, "qdma_clink"))) {
	   cdev_del(&c_dev);
	   unregister_chrdev_region(dev, MINOR_CNT);
	   return PTR_ERR(cl);
	}

	// pr_info("Device create\n");

	if (IS_ERR(dev_creation_ret = device_create(cl, NULL, dev, NULL, char_dev_name))) {
	   class_destroy(cl);
	   cdev_del(&c_dev);
	   unregister_chrdev_region(dev, MINOR_CNT);
	   return PTR_ERR(dev_creation_ret);
	}

	/*//pr_info("Getting irqn");
	qcdma.irqn = platform_get_irq(pdev, 0);
	handler = test_interrupt;
	flags = IRQF_SHARED;
	name = "quest_clink_in_dma_driver";

	//pr_info("Requesting irq, irqn: %d", qcdma.irqn);
	if (request_irq(qcdma.irqn, handler, flags, name, &pdev->dev)) {
	    //printk(KERN_ERR "quest-dma-ctrl: cannot register IRQ %d\n", qcdma.irqn);
	    return -EIO;
	}*/

	// pr_info("Getting dma base\n");
	reg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!reg_res)
		return -ENOMEM;

	qdma.base_addr = devm_ioremap_resource(dev_reg, reg_res);

	if (IS_ERR(qdma.base_addr)) {
		dev_err(dev_reg, "devm_ioremap_resource failed\n");
		//retval = PTR_ERR(qcdma.sensor_dma_base);
		return -ENOMEM;
	}

	pr_info("INIT DONE %d %d", (u32)qdma.base_addr, quest_clink_in_dma_getreg(0));

	return 0;
}

static int quest_clink_in_dma_remove(struct platform_device *pdev)
{
	//pr_info("%s BEGIN quest_clink_in_dma_driver_remove\n", __func__);

	//free_irq(qcdma.irqn, &pdev->dev);

	quest_clink_in_dma_destroy_channels();

	device_destroy(cl, dev);
	class_destroy(cl);
	cdev_del(&c_dev);
	unregister_chrdev_region(dev, MINOR_CNT);

	//cam_dma_cleanup();

	//pr_info("END quest_clink_in_dma_driver_remove\n");
	return 0;
}

static const struct of_device_id quest_clink_in_dma_driver_of_ids[] = {
        { .compatible = "quest,qdma-clink-1.0.0",},
        {}
};

static struct platform_driver quest_clink_in_dma_driver = {
	.driver = {
	        .name = "qdma_clink",
	        .owner = THIS_MODULE,
	        .of_match_table = quest_clink_in_dma_driver_of_ids,
	},
	.probe = quest_clink_in_dma_probe,
	.remove = quest_clink_in_dma_remove,
};

static int __init quest_clink_in_dma_init(void)
{
	// pr_info("Hello init\n");
	return platform_driver_register(&quest_clink_in_dma_driver);
}
late_initcall(quest_clink_in_dma_init);

static void __exit quest_clink_in_dma_exit(void)
{
	// pr_info("Hello exit2\n");
	platform_driver_unregister(&quest_clink_in_dma_driver);
}
module_exit(quest_clink_in_dma_exit)

MODULE_AUTHOR("Quest Innovations B.V.");
MODULE_DESCRIPTION("Quest CameraLink DMA controller");
MODULE_LICENSE("GPL v2");
