/*
 * hello−1.c − The simplest kernel module.
 */
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched/task.h>

#include <linux/uaccess.h>

#include <linux/cdev.h>

#include <linux/dma-mapping.h>

#include <linux/io.h>

#include <linux/dma/quest_dma_ctrl.h>

#include <linux/interrupt.h>

#include <linux/of_platform.h>

#include <linux/wait.h>

#include <linux/delay.h>

/*
 * Initialization patterns. All bytes in the source buffer has bit 7
 * set, all bytes in the destination buffer has bit 7 cleared.
 *
 * Bit 6 is set for all bytes which are to be copied by the DMA
 * engine. Bit 5 is set for all bytes which are to be overwritten by
 * the DMA engine.
 *
 * The remaining bits are the inverse of a counter which increments by
 * one for each byte address.
 */
#define PATTERN_SRC		0x80
#define PATTERN_DST		0x00
#define PATTERN_COPY		0x40
#define PATTERN_OVERWRITE	0x20
#define PATTERN_COUNT_MASK	0x1f

#define FIRST_MINOR 0
#define MINOR_CNT 1

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

struct quest_cam_dma {
        u8 **bufs;
        u32 *bufs_raw;
        u8 **bufs_user;

        dma_addr_t *dma_handles;

        unsigned int bufs_size;
        unsigned int bufs_cnt;
        unsigned int image_size_bytes;

        unsigned int check_range;

        unsigned int last_read;
        unsigned int last_write;
        bool frame_skipped;
        bool frame_ready;
        bool stop_grabbing;
        bool grabbing;
        int irqn;
        void __iomem *sensor_dma_base;
};

static struct device *dev_reg;

static dev_t dev;
static struct cdev c_dev;
static struct class *cl;

static struct quest_cam_dma qcdma;


/**
 * sensor_dma_getreg - Get register from sensor DMA FPGA module
 * @offset: Offset (in words) to base
 *
 * Return: Register of sensor DMA FPGA module
 */
static u32 sensor_dma_getreg(SensorDmaRegs offset)
{
    u32 test = readl(qcdma.sensor_dma_base + (offset * 4));
    ////pr_info("    %s dma base: %d\n      offset: %d value: %d", __func__, (u32)qcdma.sensor_dma_base, (u32)offset, test);

    return test;
}

/**
 * sensor_dma_setreg - Set register in sensor DMA FPGA module
 * @offset: Offset (in words) to base
 * @value: Value to set in the register
 */
static void sensor_dma_setreg(SensorDmaRegs offset, u32 value)
{
    writel(value, qcdma.sensor_dma_base + (offset * 4));
    ////pr_info("    %s dma base: %d\n      offset: %d value: %d", __func__, (u32)qcdma.sensor_dma_base, (u32)offset, value);
    udelay(500);
}

/**
 * cam_dma_cleanup - Cleanup DMA buffer
 */
static void cam_dma_cleanup(void)
{
    u32 i = 0;
    ////pr_info("%s", __func__);

    if(qcdma.bufs)
    {
        for (; qcdma.bufs[i]; i++)
        {
                dma_free_coherent(dev_reg, qcdma.bufs_size, qcdma.bufs[i], qcdma.dma_handles[i]);
                //kfree(qcdma.bufs[i]);
        }

        kfree(qcdma.bufs);
    }

    if(qcdma.bufs_raw)
        kfree(qcdma.bufs_raw);

    if(qcdma.bufs_user)
        kfree(qcdma.bufs_user);

    qcdma.bufs_size = 0;
    qcdma.bufs_cnt = 0;

    //pr_info("%s", __func__);
}

/**
 * dmatest_init_srcs - Write testpattern in buffers
 * @start: Where to start writing the PATTERN_COPY
 * @len: How long to write PATTERN_COPY
 */
static void dmatest_init_srcs_quest(u8 **bufs, unsigned int start, unsigned int stop, unsigned int size)
{
        unsigned int i;
        u8 *buf;

        for (; (buf = *bufs); bufs++) {
                for (i = 0; i < start; i++)
                        buf[i] = PATTERN_SRC;
                for ( ; i < stop; i++)
                        buf[i] = PATTERN_COPY;
                for ( ; i < size; i++)
                        buf[i] = PATTERN_SRC;
                buf++;
        }
}

/*
static void dmatest_init_srcs_quest_single(u8 *buf, unsigned int start, unsigned int stop, unsigned int size)
{
        unsigned int i;

        for (i = 0; i < start; i++)
                buf[i] = PATTERN_SRC;
        for ( ; i < stop; i++)
                buf[i] = PATTERN_COPY;
        for ( ; i < size; i++)
                buf[i] = PATTERN_SRC;
}*/

static bool dmatest_check_srcs_quest(u8 *buf, unsigned int start, unsigned int stop, unsigned int size)
{
    unsigned int i;
    unsigned int fault = 0;

    for (i = 0; i < start; i++)
    {
        if(buf[i] != PATTERN_SRC)
        {
            //pr_info("i:%d d:%d", i, buf[i]);
            fault = 1;
            break;
        }
    }

    i = stop;

    for ( ; i < size; i++)
    {
        if(buf[i] != PATTERN_SRC)
        {
            //if(fault == 1)
            //    pr_info("\n");
            //else

            fault = 1;
            break;

            //pr_info("i:%d d:%d", i, buf[i]);
        }
    }

    if(fault == 1)
    {
        //pr_info("\n");
        return false;
    }

    return true;
}

/**
 * cam_dma_init - Initialize the camera dma and create the buffers
 * @bufs_size: Buffer size in bytes
 * @bufs_cnt: Nr of buffers
 *
 * Return: Buffer creation successful
 */
static int cam_dma_init(unsigned int bufs_size, unsigned int bufs_cnt)
{
    u32 i = 0;
    u32 j = 0;

    //pr_info("%s Begin %d %d 1", __func__, bufs_size, bufs_cnt);

    if(qcdma.bufs != 0)
        cam_dma_cleanup(); // If init was already done first do a cleanup

    qcdma.bufs_cnt = bufs_cnt;
    qcdma.image_size_bytes = bufs_size;

    qcdma.check_range = 1024;

    qcdma.bufs_size = qcdma.image_size_bytes + (qcdma.check_range * 2);

    qcdma.bufs = kcalloc(qcdma.bufs_cnt+1 , sizeof(u8 *), GFP_KERNEL);
    qcdma.bufs_raw = kcalloc(qcdma.bufs_cnt+1, sizeof(u32), GFP_KERNEL);
    qcdma.bufs_user = kcalloc(qcdma.bufs_cnt+1, sizeof(u8 *), GFP_KERNEL);
    qcdma.dma_handles = kcalloc(qcdma.bufs_cnt, sizeof(dma_addr_t), GFP_KERNEL);

    if(!qcdma.bufs ||
        !qcdma.bufs_raw ||
        !qcdma.bufs_user)
    {
        cam_dma_cleanup();
        return -1;
    }

    for(; i < qcdma.bufs_cnt; ++i)
    {
        //pr_info("Going to allocate memory %d", qcdma.bufs_size);

        // Assume caller knows what he is doing trying to alloc mem
        qcdma.bufs[i] = dma_alloc_coherent(dev_reg, qcdma.bufs_size, &qcdma.dma_handles[i], GFP_KERNEL);

        if(!qcdma.bufs[i])
        {
            cam_dma_cleanup();
            return -1;
        }

        qcdma.bufs_user[i] = qcdma.bufs[i] + qcdma.check_range;
        qcdma.bufs_raw[i] = qcdma.dma_handles[i] + qcdma.check_range;

        //pr_info("Raw assignment OK %d", qcdma.bufs_raw[i]);
    }

    qcdma.bufs[i+1] = NULL;
    qcdma.bufs_user[i+1] = NULL;

    qcdma.bufs_cnt = bufs_cnt;

    // Set read and write to last buffer so they begin at 0 at a write
    qcdma.last_read = qcdma.bufs_cnt - 1;
    qcdma.last_write = qcdma.bufs_cnt - 1;

    dmatest_init_srcs_quest(qcdma.bufs, qcdma.check_range, qcdma.image_size_bytes + qcdma.check_range, qcdma.bufs_size);

    for( ; j < qcdma.bufs_cnt; ++j)
    {
        //if(dmatest_check_srcs_quest(qcdma.bufs[j], qcdma.check_range, qcdma.image_size_bytes + qcdma.check_range, qcdma.bufs_size))
            //pr_info("Buffer overrun check OK");
        //else
            //pr_info("ERROR!ERROR!ERROR!ERROR!ERROR!ERROR!ERROR!ERROR!ERROR!ERROR!ERROR!ERROR!");
    }

    //pr_info("dmatest_init_srcs done");

    sensor_dma_setreg(Pointer0, qcdma.bufs_raw[0]);

    sensor_dma_setreg(Control, 0);
    sensor_dma_setreg(Control, InitDone);

    //pr_info("####### %s Done #######", __func__);

    return 0;
}

static DECLARE_WAIT_QUEUE_HEAD(frame_wait);

static int cam_dma_grab_img(qdma_buf_arg_t *buf)
{
    if(qcdma.last_read == qcdma.last_write)
    {
        int result;
        //--//printk("GW   - Waiting for next image");
        result = wait_event_interruptible(frame_wait, qcdma.frame_ready || qcdma.stop_grabbing);

        if(qcdma.stop_grabbing)
        {
            //printk("Stopping grabbing %d", result);
            // Commented out because we reset the stop grabbing in the driveGrabbing
            //qcdma.stop_grabbing = false; // Reset stop signal
            return -1;
        }
        else if(qcdma.frame_ready)
        {
            //--//printk("DW   - Done with waiting for frame ready!");
            qcdma.frame_ready = false; // Reset frame ready signal
        }
        else
        {
            //pr_info("    Woken up by? %d", result);
            return result;
        }
    }
    else
    {
        //--//printk("    Catching up on images lr: %d lw: %d", qcdma.last_read, qcdma.last_write);

        if(qcdma.frame_ready) // If frame ready was set while not waiting for a frame, reset the flag
            qcdma.frame_ready = false;
    }

    if(qcdma.last_read == qcdma.bufs_cnt - 1)
        qcdma.last_read = 0;
    else
        ++qcdma.last_read;

    if(qcdma.frame_skipped)
    {
        if (copy_to_user(buf->frame_skipped, &qcdma.frame_skipped, sizeof(bool)))
        {
            //pr_info("Oops, frame_skipped transfer went wrong :(");
            return -EACCES;
        }
        qcdma.frame_skipped = false; // Reset skip bit
    }

    //if(!dmatest_check_srcs_quest(qcdma.bufs[qcdma.last_read], qcdma.check_range, qcdma.image_size_bytes + qcdma.check_range, qcdma.bufs_size))
        ////pr_info("Buffer overrun!");

    //--//printk("GRAB - Last read: %d > %d Last write: %d", prevLastRead, qcdma.last_read, qcdma.last_write);

    if (copy_to_user(buf->buf_dst, qcdma.bufs_user[qcdma.last_read], qcdma.image_size_bytes))
    {
        //pr_info("Oops, transfer went wrong :(");
        return -EACCES;
    }

    return 0;
}
static int my_open(struct inode *i, struct file *f)
{
    return 0;
}
static int my_close(struct inode *i, struct file *f)
{
    return 0;
}

/*
static irqreturn_t test_interrupt(int irq, void *dev_id)
{
    // Check if skipping a frame read is required
    u32 control;
    u32 last_write_chk_buf;

    u32 ipId = sensor_dma_getreg(IpCoreId);

    //pr_info("    ------IpCoreId %d", ipId);

    if(qcdma.bufs_cnt == 0)
        return IRQ_HANDLED;

    last_write_chk_buf = qcdma.last_write + 2; // + 2 to prevent race condition in read

    //printk("IMG - %s LR: %d LW: %d", __func__, qcdma.last_read, qcdma.last_write);

    if(last_write_chk_buf >= qcdma.bufs_cnt)
        last_write_chk_buf -= qcdma.bufs_cnt;
    if(last_write_chk_buf == qcdma.last_read)
    {
        //pr_info("%s Skipping frame", __func__);
        qcdma.frame_skipped = true;
        if(qcdma.last_read == qcdma.bufs_cnt - 1)
            qcdma.last_read = 0;
        else
            ++qcdma.last_read;
    }

    // Flag buffer recieved
    if(qcdma.last_write == qcdma.last_read) // If read if waiting for a frame, signal it
        qcdma.frame_ready = true;

    if(qcdma.last_write == qcdma.bufs_cnt - 1)
        qcdma.last_write = 0;
    else
        ++qcdma.last_write;

    // Write next phys buffer ptr in sensor DMA
    //pr_info("    %s Writing next phys buffer", __func__);
    sensor_dma_setreg(Pointer0, qcdma.bufs_raw[qcdma.last_write]);

    // Acknowledge IRQ
    //pr_info("    %s Acknowledge IRQ", __func__);
    control = sensor_dma_getreg(Control);
    control |= IrqAck;
    sensor_dma_setreg(Control, control);

    wake_up_interruptible(&frame_wait);

    return IRQ_HANDLED;
}*/

static int driveGrabbing(void)
{
    u32 control;
    u32 prevLastWrite;
    u32 nextWrite;
    u32 last_write_chk_buf ;

    if(qcdma.bufs_cnt == 0) // Cannot grab if no buffers are set
        return -1;

    //pr_info("All ok. Starting grabbing driver");
    while(!qcdma.stop_grabbing)
    {
        if(sensor_dma_getreg(Status) & ImageDone)
        {
            //pr_info("Image received! \n");

            prevLastWrite = qcdma.last_write;

            // Check if skipping a frame read is required
            last_write_chk_buf = qcdma.last_write + 2; // + 2 to prevent race condition in read

            if(last_write_chk_buf >= qcdma.bufs_cnt)
                last_write_chk_buf -= (qcdma.bufs_cnt - 1);
            if(last_write_chk_buf == qcdma.last_read)
            {
                //pr_info("%s Skipping frame", __func__);
                qcdma.frame_skipped = true;
                if(qcdma.last_read == qcdma.bufs_cnt - 1)
                    qcdma.last_read = 0;
                else
                    ++qcdma.last_read;
            }

            if(qcdma.last_write == qcdma.bufs_cnt - 1)
                qcdma.last_write = 0;
            else
                ++qcdma.last_write;

            // Flag buffer recieved AFTER increasing counter to ensure if read is waiting for frame we can interrupt it
            if(prevLastWrite == qcdma.last_read) // If read if waiting for a frame, signal it
                qcdma.frame_ready = true;

            if(qcdma.last_write == qcdma.bufs_cnt - 1)
                nextWrite = 0;
            else
                nextWrite = qcdma.last_write + 1;

            //----//printk("IMG  - LR: %d LW: %d > %d Next: %d", qcdma.last_read, prevLastWrite, qcdma.last_write, nextWrite);

            // Write next phys buffer ptr in sensor DMA

            //dmatest_init_srcs_quest_single(qcdma.bufs[nextWrite], qcdma.check_range, qcdma.image_size_bytes, qcdma.bufs_size);

            //--//pr_info("    %s Writing next phys buffer %d lazy begin(-16B): %d, lazy end (-16B): %d", __func__, qcdma.bufs_raw[nextWrite], qcdma.bufs_raw[nextWrite] - 16, qcdma.bufs_raw[nextWrite] + qcdma.image_size_bytes - 16);
            sensor_dma_setreg(Pointer0, qcdma.bufs_raw[nextWrite]);

            // Acknowledge IRQ
            ////pr_info("    %s Acknowledge IRQ", __func__);
            control = sensor_dma_getreg(Control);
            sensor_dma_setreg(Control, control | IrqAck);

            ////pr_info("    %s Start", __func__);
            sensor_dma_setreg(Control, control | Start);

            // Wait for image done to be low
            if(sensor_dma_getreg(Status) & ImageDone)
            {
                //pr_info("!!!!!Image done still high 1!!!!!!!!");
                msleep(100); // Wait for a bit
            }
            if(sensor_dma_getreg(Status) & ImageDone)
            {
                //pr_info("!!!!!Image done still high 2!!!!!!");
                msleep(1000); // Wait for a bit
            }
            wake_up_interruptible(&frame_wait);
        }
        else
        {
            usleep_range(100, 200);
        }
    }

    wake_up_interruptible(&frame_wait); // Awake process to return
    msleep(10); // Wait for grab function to see the stop flag before resetting it

    qcdma.stop_grabbing = false;    // Reset stop flag
    qcdma.grabbing = false;
    //pr_info("Stopping drive grabbing");

    return 0;
}

static long my_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    qdma_alloc_arg_t al;
    qdma_buf_arg_t buf;
    qdma_reg_arg_t reg;

    switch (cmd)
    {
        case QDMA_ALLOC_MEM:
            if (copy_from_user(&al, (qdma_alloc_arg_t *)arg, sizeof(qdma_alloc_arg_t)))
            {
                return -EACCES;
            }
            return cam_dma_init(al.buf_size, al.buf_cnt);

        case QDMA_FREE_MEM:
            cam_dma_cleanup();
            break;

        case QDMA_START_GRAB:
            {
            u32 control = 0;

            // If not already done, stop any grabbing process
            ////pr_info("Setting stop bit");
            if(qcdma.grabbing == true)
            {
                qcdma.stop_grabbing = true;
                wake_up_interruptible(&frame_wait);

                usleep_range(100, 100);
            }
            if(qcdma.grabbing == true)
            {
                //pr_info("Stop grabbing failed");
                return -1;
            }
            qcdma.grabbing = true;
            //wake_up_interruptible(&frame_wait);

            // Sleep some time to be sure stop has been grabbed
            //msleep(1);

            //qcdma.stop_grabbing = false;

            // Set read and write to last buffer so they begin at 0 at a write
            qcdma.last_read = qcdma.bufs_cnt - 1;
            qcdma.last_write = qcdma.bufs_cnt - 1;

            sensor_dma_setreg(Control, IrqAck); // Module does not reset if still in irq
            sensor_dma_setreg(Control, 0); // Reset
            sensor_dma_setreg(Control, InitDone);
            sensor_dma_setreg(Pointer0, qcdma.bufs_raw[0]);
            control = sensor_dma_getreg(Control);
            sensor_dma_setreg(Control, control | Start);
            }
            break;

        case QDMA_STOP_GRAB:
            //pr_info("Setting stop bit");
            qcdma.stop_grabbing = true;
            wake_up_interruptible(&frame_wait);
            break;

        case QDMA_GRAB_IMG:
            if (copy_from_user(&buf, (qdma_buf_arg_t *)arg, sizeof(qdma_buf_arg_t)))
            {
                //pr_info("Grab - user>kernel data transfer went wrong");
                return -EACCES;
            }
            return cam_dma_grab_img(&buf);

        case QDMA_SENSOR_DMA_SETREG:
            if (copy_from_user(&reg, (qdma_reg_arg_t *)arg, sizeof(qdma_reg_arg_t)))
            {
                return -EACCES;
            }
            sensor_dma_setreg((SensorDmaRegs)reg.offset, reg.value);
            break;

        case QDMA_SENSOR_DMA_GETREG:
            if (copy_from_user(&reg, (qdma_reg_arg_t *)arg, sizeof(qdma_reg_arg_t)))
            {
                return -EACCES;
            }

            reg.value = sensor_dma_getreg((SensorDmaRegs)reg.offset);

            if (copy_to_user((query_arg_t *)arg, &reg, sizeof(qdma_reg_arg_t)))
            {
                return -EACCES;
            }
            break;

        case QDMA_DRIVE_GRABBING:
            //pr_info("Starting drive grabbing");
            return driveGrabbing();

        default:
            return -EINVAL;
    }

    return 0;
}

struct file_operations query_fops = {
 open:   my_open,
 unlocked_ioctl: my_ioctl,
 release: my_close
};

static int quest_dma_ctrl_probe(struct platform_device *pdev)
{
        // DMA stuff
        int ret;
        struct device *dev_ret;

        // IRQ stuff
        /*irq_handler_t handler;
        unsigned long flags;
        const char *name;*/

        struct resource *reg_res;

        dev_reg = &pdev->dev;

        qcdma.bufs_size = 0;
        qcdma.bufs_cnt = 0;
        qcdma.image_size_bytes = 0;
        qcdma.frame_ready = false;
        qcdma.frame_skipped = false;
        qcdma.stop_grabbing = false;
        qcdma.grabbing = false;

        if ((ret = alloc_chrdev_region(&dev, FIRST_MINOR, MINOR_CNT, "query_ioctl")) < 0)
        {
           return ret;
        }

        cdev_init(&c_dev, &query_fops);

        if ((ret = cdev_add(&c_dev, dev, MINOR_CNT)) < 0)
        {
           return ret;
        }

        if (IS_ERR(cl = class_create(THIS_MODULE, "char")))
        {
           cdev_del(&c_dev);
           unregister_chrdev_region(dev, MINOR_CNT);
           return PTR_ERR(cl);
        }
        if (IS_ERR(dev_ret = device_create(cl, NULL, dev, NULL, "query")))
        {
           class_destroy(cl);
           cdev_del(&c_dev);
           unregister_chrdev_region(dev, MINOR_CNT);
           return PTR_ERR(dev_ret);
        }

        /*//pr_info("Getting irqn");
        qcdma.irqn = platform_get_irq(pdev, 0);
        handler = test_interrupt;
        flags = IRQF_SHARED;
        name = "quest_dma_ctrl";

        //pr_info("Requesting irq, irqn: %d", qcdma.irqn);
        if (request_irq(qcdma.irqn, handler, flags, name, &pdev->dev)) {
            //printk(KERN_ERR "quest-dma-ctrl: cannot register IRQ %d\n", qcdma.irqn);
            return -EIO;
        }*/

        ////pr_info("Getting sensor dma base\n");
        reg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!reg_res)
                return -ENOMEM;

        qcdma.sensor_dma_base = devm_ioremap_resource(dev_reg, reg_res);

        if (IS_ERR(qcdma.sensor_dma_base)) {
                dev_err(dev_reg, "devm_ioremap_resource failed\n");
                //retval = PTR_ERR(qcdma.sensor_dma_base);
                return -ENOMEM;
        }

        //pr_info("Done probing\n");

        return 0;
}

static int quest_dma_ctrl_remove(struct platform_device *pdev)
{
        //struct dmatest_chan *dtc, *_dtc;
        //struct dma_chan *chan;

        //pr_info("%s BEGIN quest_dma_ctrl_remove\n", __func__);

        //free_irq(qcdma.irqn, &pdev->dev);

        device_destroy(cl, dev);
        class_destroy(cl);
        cdev_del(&c_dev);
        unregister_chrdev_region(dev, MINOR_CNT);

        cam_dma_cleanup();

        //pr_info("END quest_dma_ctrl_remove\n");
	return 0;
}

static const struct of_device_id quest_dma_ctrl_of_ids[] = {
	{ .compatible = "quest,quest-dma-ctrl-1.00.a",},
	{}
};

static struct platform_driver quest_dma_ctrl_driver = {
	.driver = {
		.name = "quest_dma_ctrl",
		.owner = THIS_MODULE,
		.of_match_table = quest_dma_ctrl_of_ids,
	},
	.probe = quest_dma_ctrl_probe,
        .remove = quest_dma_ctrl_remove,
};

static int __init questdma_init(void)
{
        //pr_info("Hello init. 15-jan-2018 18:36 \n");
	return platform_driver_register(&quest_dma_ctrl_driver);

}
late_initcall(questdma_init);

static void __exit questdma_exit(void)
{
        //pr_info("Hello exit2.\n");
	platform_driver_unregister(&quest_dma_ctrl_driver);
}
module_exit(questdma_exit)

MODULE_AUTHOR("Quest Innovations B.V.");
MODULE_DESCRIPTION("Quest dma controller");
MODULE_LICENSE("GPL v2");
