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

static unsigned int test_buf_size = 64;
module_param(test_buf_size, uint, S_IRUGO);
MODULE_PARM_DESC(test_buf_size, "Size of the memcpy test buffer");

static unsigned int iterations = 5;
module_param(iterations, uint, S_IRUGO);
MODULE_PARM_DESC(iterations,
		"Iterations before stopping test (default: infinite)");

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

        dma_addr_t *dma_handles;

        unsigned int bufs_size;
        unsigned int bufs_cnt;
        unsigned int image_size_bytes;
        unsigned int last_read;
        unsigned int last_write;
        bool frame_skipped;
        bool frame_ready;
        bool stop_grabbing;
        int irqn;
        void __iomem *sensor_dma_base;
};

struct dmatest_slave_thread {
        struct list_head node;
	struct task_struct *task;
	struct dma_chan *tx_chan;
	u8 **srcs;
	enum dma_transaction_type type;
	bool done;
};
struct dmatest_chan {
	struct list_head node;
	struct dma_chan *chan;
	struct list_head threads;
};

/*
 * These are protected by dma_list_mutex since they're only used by
 * the DMA filter function callback
 */
static DECLARE_WAIT_QUEUE_HEAD(thread_wait);
static LIST_HEAD(dmatest_channels);
static unsigned int nr_channels;

static int status = 1, dignity = 2, ego = 3;

static struct device *dev_reg;

static dev_t dev;
static struct cdev c_dev;
static struct class *cl;

static volatile struct quest_cam_dma qcdma;


/**
 * sensor_dma_getreg - Get register from sensor DMA FPGA module
 * @offset: Offset (in words) to base
 *
 * Return: Register of sensor DMA FPGA module
 */
static u32 sensor_dma_getreg(SensorDmaRegs offset)
{
    u32 test = readl(qcdma.sensor_dma_base + (offset * 4));
    pr_info("    %s dma base: %d\n      offset: %d value: %d", __func__, (u32)qcdma.sensor_dma_base, (u32)offset, test);

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
    pr_info("    %s dma base: %d\n      offset: %d value: %d", __func__, (u32)qcdma.sensor_dma_base, (u32)offset, value);
    udelay(100);
}

/**
 * cam_dma_cleanup - Cleanup DMA buffer
 */
static void cam_dma_cleanup(void)
{
    u32 i = 0;
    pr_info("%s", __func__);

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

    qcdma.bufs_size = 0;
    qcdma.bufs_cnt = 0;

    pr_info("%s", __func__);
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
                        buf[i] = PATTERN_SRC | (~i & PATTERN_COUNT_MASK);
                for ( ; i < stop; i++)
                        buf[i] = PATTERN_SRC | PATTERN_COPY
                                | (~i & PATTERN_COUNT_MASK);
                for ( ; i < size; i++)
                        buf[i] = PATTERN_SRC | (~i & PATTERN_COUNT_MASK);
                buf++;
        }
}

static void dmatest_init_srcs(u8 **bufs, unsigned int start, unsigned int len)
{
        unsigned int i;
        u8 *buf;

        for (; (buf = *bufs); bufs++) {
                for (i = 0; i < start; i++)
                        buf[i] = PATTERN_SRC | (~i & PATTERN_COUNT_MASK);
                for ( ; i < start + len; i++)
                        buf[i] = PATTERN_SRC | PATTERN_COPY
                                | (~i & PATTERN_COUNT_MASK);
                for ( ; i < test_buf_size; i++)
                        buf[i] = PATTERN_SRC | (~i & PATTERN_COUNT_MASK);
                buf++;
        }
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

    pr_info("%s Begin %d %d 1", __func__, bufs_size, bufs_cnt);

    if(qcdma.bufs != 0)
        cam_dma_cleanup(); // If init was already done first do a cleanup

    pr_info("Allocating twice as needed for testing buffer overrun");
    qcdma.bufs_size = bufs_size;
    qcdma.bufs_cnt = bufs_cnt;
    qcdma.image_size_bytes = 1920 * (1080 / 2) * 4;

    qcdma.bufs = kcalloc(qcdma.bufs_cnt+1 , sizeof(u8 *), GFP_KERNEL);
    qcdma.bufs_raw = kcalloc(qcdma.bufs_cnt+1, sizeof(u32), GFP_KERNEL);
    qcdma.dma_handles = kcalloc(qcdma.bufs_cnt, sizeof(dma_addr_t), GFP_KERNEL);

    if(!qcdma.bufs ||
        !qcdma.bufs_raw)
    {
        cam_dma_cleanup();
        return -1;
    }

    for(; i < qcdma.bufs_cnt; ++i)
    {
        pr_info("2Going to allocate memory %d", qcdma.bufs_size + 64);

        // Assume caller knows what he is doing trying to alloc mem
        //qcdma.bufs[i] = kmalloc(qcdma.bufs_size, GFP_DMA);

        qcdma.bufs[i] = dma_alloc_coherent(dev_reg, qcdma.bufs_size + 64, &qcdma.dma_handles[i], GFP_KERNEL);

        if(!qcdma.bufs[i])
        {
            cam_dma_cleanup();
            return -1;
        }


        qcdma.bufs_raw[i] = qcdma.dma_handles[i];

        pr_info("Burst alignment 64, org %d, mod %d",
                qcdma.bufs_raw[i],
                64 - ((u64)qcdma.bufs_raw[i] % 64));

        qcdma.bufs_raw[i] += (64 - ((u64)qcdma.bufs_raw[i] % 64));

        pr_info("Raw assignment OK %d", qcdma.bufs_raw[i]);
    }

    /*
    pr_info("Creating very special CAFEBABE memory :)");

    ++i;

    qcdma.bufs[i] = dma_alloc_coherent(dev_reg, qcdma.bufs_size + 64, &qcdma.dma_handles[i], GFP_KERNEL);

    if(!qcdma.bufs[i])
    {
        cam_dma_cleanup();
        return -1;
    }

    qcdma.bufs_raw[i] = qcdma.dma_handles[i];

    for(; j < qcdma.image_size_bytes / 4; ++j)
    {
        *(u32 *)qcdma.bufs[j] = 0xCAFEBABE;
    }

    pr_info("Raw assignment OK %d", qcdma.dma_handles[i]);*/

    qcdma.bufs[i+1] = NULL;

    pr_info("Buff set to null");

    qcdma.bufs_cnt = bufs_cnt;

    // Set read and write to last buffer so they begin at 0 at a write
    qcdma.last_read = qcdma.bufs_cnt - 1;
    qcdma.last_write = qcdma.bufs_cnt - 1;

    //dmatest_init_srcs_quest(qcdma.bufs, 0, qcdma.bufs_size, qcdma.bufs_size);

    pr_info("dmatest_init_srcs done");

    sensor_dma_setreg(Pointer0, qcdma.bufs_raw[0]);

    sensor_dma_setreg(Control, 0);
    sensor_dma_setreg(Control, InitDone);

    pr_info("%s Done", __func__);

    return 0;
}

static DECLARE_WAIT_QUEUE_HEAD(frame_wait);

static int cam_dma_grab_img(qdma_buf_arg_t *buf)
{
    u32 currRead = 0;
    printk("GRAB - %s Last read: %d Last write: %d", __func__, qcdma.last_read, qcdma.last_write);

    if(qcdma.last_read == qcdma.last_write)
    {
        int result;
        printk("GW   - Waiting for next image");
        result = wait_event_interruptible(frame_wait, qcdma.frame_ready || qcdma.stop_grabbing);

        if(qcdma.stop_grabbing)
        {
            printk("Stopping grabbing %d", result);
            //qcdma.stop_grabbing = false; // Reset stop signal
            return -1;
        }
        else if(qcdma.frame_ready)
        {
            printk("DW   - Done with waiting for frame ready!");
            qcdma.frame_ready = false; // Reset frame ready signal
        }
        else
        {
            pr_info("    Woken up by? %d", result);
            return result;
        }
    }
    else
    {
        printk("    Catching up on images lr: %d lw: %d", qcdma.last_read, qcdma.last_write);

        if(qcdma.frame_ready) // If frame ready was set while not waiting for a frame, reset the flag
            qcdma.frame_ready = false;
    }

    pr_info("Rethink the buffer, does currRead solution really work?");
    currRead = qcdma.last_read;

    if(qcdma.last_read == qcdma.bufs_cnt - 1)
        qcdma.last_read = 0;
    else
        ++qcdma.last_read;


    pr_info("    -Disabled copy in buffer write\n");
    if(qcdma.frame_skipped)
    {
        if (copy_to_user(buf->frame_skipped, &qcdma.frame_skipped, sizeof(bool)))
        {
            pr_info("Oops, frame_skipped transfer went wrong :(");
            return -EACCES;
        }
        qcdma.frame_skipped = false; // Reset skip bit
    }

    if (copy_to_user(buf->buf_dst, qcdma.bufs[currRead], qcdma.image_size_bytes))
    {
        pr_info("Oops, transfer went wrong :(");
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

static irqreturn_t test_interrupt(int irq, void *dev_id)
{
    // Write testpattern in buffer
    //dmatest_init_srcs();
    // Check if skipping a frame read is required
    u32 control;
    u32 last_write_chk_buf;

    u32 ipId = sensor_dma_getreg(IpCoreId);

    pr_info("    ------IpCoreId %d", ipId);

    if(qcdma.bufs_cnt == 0)
        return IRQ_HANDLED;

    last_write_chk_buf = qcdma.last_write + 2; // + 2 to prevent race condition in read

    printk("IMG - %s Last read: %d Last write: %d", __func__, qcdma.last_read, qcdma.last_write);

    if(last_write_chk_buf >= qcdma.bufs_cnt)
        last_write_chk_buf -= qcdma.bufs_cnt;
    if(last_write_chk_buf == qcdma.last_read)
    {
        pr_info("%s Skipping frame", __func__);
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
    pr_info("    %s Writing next phys buffer", __func__);
    sensor_dma_setreg(Pointer0, qcdma.bufs_raw[qcdma.last_write]);

    // Acknowledge IRQ
    pr_info("    %s Acknowledge IRQ", __func__);
    control = sensor_dma_getreg(Control);
    control |= IrqAck;
    sensor_dma_setreg(Control, control);

    wake_up_interruptible(&frame_wait);

    return IRQ_HANDLED;
}

static int driveGrabbing(void)
{
    // Write testpattern in buffer
    //dmatest_init_srcs();
    // Check if skipping a frame read is required
    u32 control;

    u32 ipId = sensor_dma_getreg(IpCoreId);

    pr_info("    ------IpCoreId %d", ipId);

    if(qcdma.bufs_cnt == 0) // Cannot grab if no buffers are set
        return -1;

    pr_info("All ok. Starting grabbing driver");
    while(!qcdma.stop_grabbing)
    {
        if(sensor_dma_getreg(Status) & ImageDone)
        {
            u32 last_write_chk_buf = qcdma.last_write + 2; // + 2 to prevent race condition in read

            printk("IMG - %s Last read: %d Last write: %d", __func__, qcdma.last_read, qcdma.last_write);

            if(last_write_chk_buf >= qcdma.bufs_cnt)
                last_write_chk_buf -= qcdma.bufs_cnt;
            if(last_write_chk_buf == qcdma.last_read)
            {
                pr_info("%s Skipping frame", __func__);
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

            pr_info("Test if write pointer has unwanted behaviour");

            // Write next phys buffer ptr in sensor DMA
            pr_info("    %s Writing next phys buffer %d", __func__, qcdma.bufs_raw[qcdma.last_write]);
            sensor_dma_setreg(Pointer0, qcdma.bufs_raw[qcdma.last_write]);

            // Acknowledge IRQ
            pr_info("    %s Acknowledge IRQ", __func__);
            control = sensor_dma_getreg(Control);
            //control |= IrqAck | Start;
            //sensor_dma_setreg(Control, control);
            sensor_dma_setreg(Control, control | IrqAck);
            msleep(1);
            pr_info("    %s Start", __func__);
            sensor_dma_setreg(Control, control | Start);

            // Wait for image done to be low
            if(sensor_dma_getreg(Status) & ImageDone)
            {
                pr_info("    Image done still high");
                msleep(100); // Wait for a bit
            }
            if(sensor_dma_getreg(Status) & ImageDone)
            {
                pr_info("Image done still high!!!!!!");
                msleep(1000); // Wait for a bit
            }
            wake_up_interruptible(&frame_wait);
        }
        else
        {
            // Wait for a bit
            msleep(1000); // Wait for a bit
        }
    }

    qcdma.stop_grabbing = false;    // Reset stop flag
    pr_info("Stopping drive grabbing");

    return 0;
}

static long my_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    query_arg_t q;
    qdma_alloc_arg_t al;
    qdma_buf_arg_t buf;
    qdma_reg_arg_t reg;

    switch (cmd)
    {
        case QUERY_GET_VARIABLES:
            q.status = status;
            q.dignity = dignity;
            q.ego = ego;
            if (copy_to_user((query_arg_t *)arg, &q, sizeof(query_arg_t)))
            {
                return -EACCES;
            }
            break;

        case QUERY_CLR_VARIABLES:
            status = 0;
            dignity = 0;
            ego = 0;
            break;

        case QUERY_SET_VARIABLES:
            if (copy_from_user(&q, (query_arg_t *)arg, sizeof(query_arg_t)))
            {
                return -EACCES;
            }
            status = q.status;
            dignity = q.dignity;
            ego = q.ego;
            break;

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
            sensor_dma_setreg(Pointer0, qcdma.bufs_raw[qcdma.last_write]);
            control = sensor_dma_getreg(Control);
            sensor_dma_setreg(Control, control | Start);
            }
            break;

        case QDMA_STOP_GRAB:
            pr_info("Setting stop bit");
            qcdma.stop_grabbing = true;
            wake_up_interruptible(&frame_wait);
            break;

        case QDMA_GRAB_IMG:
            if (copy_from_user(&buf, (qdma_buf_arg_t *)arg, sizeof(qdma_buf_arg_t)))
            {
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
            pr_info("Starting drive grabbing");
            return driveGrabbing();

        default:
            return -EINVAL;
    }

    return 0;
}

/*static void dmatest_cleanup_channel(struct dmatest_chan *dtc)
{
	struct dmatest_slave_thread *thread;
	struct dmatest_slave_thread *_thread;
	int ret;
	pr_info("Start dmatest_cleanup_channel\n");

	if(!dtc)
		pr_info("!dtc\n");

	if(!&dtc->threads)
		pr_info("!dtc->threads\n");

	if(!(&dtc->threads)->next)
		pr_info("!(&dtc->threads)->next\n");

	thread = list_entry((&dtc->threads)->next, typeof(*thread), node);
	if(!thread)
		pr_info("!thread\n");

	if(!thread->task)
		pr_info("!(&dtc->threads)->next->task\n");

	list_for_each_entry_safe(thread, _thread, &dtc->threads, node) {
		ret = kthread_stop(thread->task);
		pr_debug("dmatest: thread %s exited with status %d\n",
				thread->task->comm, ret);
		list_del(&thread->node);
		put_task_struct(thread->task);
		kfree(thread);
	}
	kfree(dtc);
}*/

/*static void dmatest_slave_tx_callback(void *completion)
{
	pr_info("In the callback, yay!");
	complete(completion);
}*/

static int dmatest_slave_func(void *data)
{
	struct dmatest_slave_thread	*thread = data;
	struct dma_chan *tx_chan;
        //unsigned int src_off, dst_off, len;
        unsigned int src_off, len;
	int src_cnt;
	int bd_cnt = 1;
	enum dma_ctrl_flags flags;
        //struct completion tx_cmp;
        //enum dma_status status;
        //dma_cookie_t tx_cookie;
	int i;
        //const char *thread_name = current->comm;

	tx_chan = thread->tx_chan;
	src_cnt = bd_cnt;

	// Allocate mem for stuff to transfer
	pr_info("Allocate stuff");
	thread->srcs = kcalloc(src_cnt+1, sizeof(u8 *), GFP_KERNEL);
	if (!thread->srcs)
		goto err_dsts;	

	thread->srcs[0] = kmalloc(test_buf_size, GFP_KERNEL);
	if (!thread->srcs[0])
		goto err_srcbuf;

	thread->srcs[1] = NULL;

	// Let's play nice today :)
	set_user_nice(current, 10);

	flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;

	{
	
	struct dma_device *tx_dev = tx_chan->device;
	struct dma_async_tx_descriptor *txd = NULL;
	dma_addr_t dma_srcs[src_cnt];

        //unsigned long tx_tmo = msecs_to_jiffies(30000);
	u8 align = 0;
	struct scatterlist tx_sg[bd_cnt];

	pr_info("Alignment stuff");
	align = tx_dev->copy_align;

	if (1 << align > test_buf_size) {
		pr_err("%u-byte buffer too small for %d-byte alignment\n",
			test_buf_size, 1 << align);
		goto err_dsts;
	}

	// Put a test pattern in the buffs
	pr_info("Inserting testpattern");
	len = test_buf_size;
	src_off = 0; // We don't scramble the offset to it's 0?
	dmatest_init_srcs(thread->srcs, 0, len);
	
	{
		u8 *buf = thread->srcs[0] + src_off;
		dma_srcs[0] = dma_map_single(tx_dev->dev, buf, len,
						DMA_MEM_TO_DEV);
	}
	
	sg_init_table(tx_sg, bd_cnt);

	sg_dma_address(&tx_sg[0]) = dma_srcs[0];
	sg_dma_len(&tx_sg[0]) = len;

	pr_info("Preparing slave sg");
	
	txd = tx_dev->device_prep_slave_sg(tx_chan, tx_sg, bd_cnt,
				DMA_MEM_TO_DEV, flags, NULL);

	if(!txd)
	{
		pr_err("!txd");
		goto err_dsts;
	}

	// Thread stuff callback
	/*pr_info("Inserting callback");
	init_completion(&tx_cmp);
	txd->callback = dmatest_slave_tx_callback;
	txd->callback_param = &tx_cmp;
	tx_cookie = txd->tx_submit(txd);

	if(!tx_cookie)
	{
		pr_err("No cookies :(");
		return -1;
	}
	

	// Submit dma transaction?
	

	// Issue pending request
	pr_info("Issueing pending request");
	dma_async_issue_pending(tx_chan);

	status = dma_async_is_tx_complete(tx_chan, tx_cookie,
							NULL, NULL);*/

	pr_info("Things are set up, yay!");	

	// Maybe some verify stuff after dma done?

	}
	/*while (!kthread_should_stop())
	{
		
	}*/

err_dsts:
	for (i = 0; thread->srcs[i]; i++)
		kfree(thread->srcs[i]);
err_srcbuf:
	kfree(thread->srcs);
//err_srcs:
	//pr_notice("%s: terminating after %u tests, %u failures (status %d)\n",
	//		thread_name, total_tests, failed_tests, ret);

	pr_info("Thread is done");
	thread->done = true;

	return 0;
}

static int dmatest_add_slave_threads(struct dmatest_chan *tx_dtc)
{
	struct dmatest_slave_thread *thread;
	struct dma_chan *tx_chan = tx_dtc->chan;
	pr_info("Start dmatest_add_slave_threads\n");

	thread = kzalloc(sizeof(struct dmatest_slave_thread), GFP_KERNEL);
	if (!thread) {
		pr_warn("dmatest: No memory for slave thread %s\n",
				dma_chan_name(tx_chan));

	}
	pr_info("mallock thread done\n");

	thread->tx_chan = tx_chan;
	thread->type = (enum dma_transaction_type)DMA_SLAVE;
	smp_wmb();
	thread->task = kthread_run(dmatest_slave_func, thread, "%s",
		dma_chan_name(tx_chan));
	if (IS_ERR(thread->task)) {
		pr_warn("dmatest: Failed to run thread %s\n",
				dma_chan_name(tx_chan));
		kfree(thread);
		return PTR_ERR(thread->task);
	}

	/* srcbuf and dstbuf are allocated by the thread itself */
	get_task_struct(thread->task);
	list_add_tail(&thread->node, &tx_dtc->threads);
	pr_info("thread added to list\n");

	/* Added one thread with 2 channels */
	return 1;
}

static int dmatest_add_slave_channels(struct dma_chan *tx_chan)
{
	struct dmatest_chan *tx_dtc;
	unsigned int thread_count = 0;
	pr_info("Start dmatest_add_slave_channels\n");

	tx_dtc = kmalloc(sizeof(struct dmatest_chan), GFP_KERNEL);
	if (!tx_dtc) {
		pr_warn("dmatest: No memory for tx %s\n",
				dma_chan_name(tx_chan));
		return -ENOMEM;
	}
	pr_info("mallock dma_channel done\n");



	tx_dtc->chan = tx_chan;

	//pr_info("tx channel client count: %d",tx_dtc->chan->client_count);
	INIT_LIST_HEAD(&tx_dtc->threads);

	dmatest_add_slave_threads(tx_dtc);
	thread_count += 1;

//	pr_info("dmatest: Started %u threads using %s\n",
//		thread_count, dma_chan_name(tx_chan));

	list_add_tail(&tx_dtc->node, &dmatest_channels);
//	list_add_tail(&rx_dtc->node, &dmatest_channels);
	nr_channels += 1;

//	if (iterations)
//		wait_event(thread_wait, !is_threaded_test_run(tx_dtc, rx_dtc));

	pr_info("END dmatest_add_slave_channels\n");
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
	struct dma_chan *chan;
	int err;
        const char *test = 0;
        //struct device_node * node = 0;
        int ret;
        struct device *dev_ret;

        // IRQ stuff
        irq_handler_t handler;
        unsigned long flags;
        const char *name;

        struct resource *reg_res;
        dev_reg = &pdev->dev;

        pr_info("Start dma_ctrl_probe1\n");
        pr_info("Trying to alloc\n");
        //u32 size = 1920 * 1080 * 2;
        u32 size = 1024 * 1024 * 4;
        // Test large alloc size
        u8* myTest = kmalloc(size, GFP_DMA);
        if(myTest == 0)
            pr_info("Alloc failed\n");
        else
            kfree(myTest);

        pr_info("Done\n");


        qcdma.bufs_size = 0;
        qcdma.bufs_cnt = 0;
        qcdma.image_size_bytes = 0;
        qcdma.frame_ready = false;
        qcdma.frame_skipped = false;
        qcdma.stop_grabbing = false;

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


        //pr_info("pdev->dev->of_node %d\n",
         //       of_property_count_strings(pdev->dev.of_node, "interrupts"));

        //struct device_node *node_test;
        //node_test = of_find_node_by_name(pdev->dev.of_node, "interrupts");

        //struct of_device *op;


/*
        op = of_find_device_by_node(pdev->dev.of_node);

        if(op->resource[0].start)
            pr_info("FOUND MY NODE %s", __func__);
        if(op->irqs[0])
            pr_info("FOUND MY NODE %s", __func__);*/



        /*pr_info("Getting irqn");
        qcdma.irqn = platform_get_irq(pdev, 0);
        handler = test_interrupt;
        flags = IRQF_SHARED;
        name = "quest_dma_ctrl";

        pr_info("Requesting irq, irqn: %d", qcdma.irqn);
        if (request_irq(qcdma.irqn, handler, flags, name, &pdev->dev)) {
            printk(KERN_ERR "quest-dma-ctrl: cannot register IRQ %d\n", qcdma.irqn);
            return -EIO;
        }*/

        pr_info("Getting sensor dma base\n");
        reg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!reg_res)
                return -ENOMEM;

        qcdma.sensor_dma_base = devm_ioremap_resource(dev_reg, reg_res);

        if (IS_ERR(qcdma.sensor_dma_base)) {
                dev_err(dev_reg, "devm_ioremap_resource failed\n");
                //retval = PTR_ERR(qcdma.sensor_dma_base);
                return -ENOMEM;
        }

        pr_info("Done sensor dma base\n");

        /*while(!qcdma.done)
        {
            ;
        }*/

        /*

        for_each_node_by_name(ebus_dp, "ebus") {
                struct device_node *dp;
                for (dp = ebus_dp; dp; dp = dp->sibling) {
                        if (!strcmp(dp->name, "rtc")) {
                                op = of_find_device_by_node(dp);
                                if (op) {
                                        rtc_port = op->resource[0].start;
                                        rtc_irq = op->irqs[0];
                                        goto found;
                                }
                        }
                }
        }*/

        return 0;

        if(pdev->dev.of_node)
                pr_info("pdev->dev->of_node\n");

        pr_info("pdev->dev->of_node %d\n",
                of_property_count_strings(pdev->dev.of_node, "dma-names"));


        of_property_read_string_index(pdev->dev.of_node, "dma-names", 0, &test);
        pr_info("0 = %s", test);

        of_property_read_string_index(pdev->dev.of_node, "dma-names", 1, &test);
        pr_info("1 = %s", test);

	
        /*node = of_find_all_nodes(NULL); // Start at root4294967277

	while(node)
	{
		pr_info("pdev->dev->of_node %s\n",
			node->name);
		if(node->parent)
		{
			pr_info("Parent name %s\n",
				node->parent->name);
		}
			
		node = of_find_all_nodes(node);
        }

	if (!of_find_property(pdev->dev.of_node, "dmas", NULL))
                pr_err("No dmas\n");*/

        pr_info("Node name %s", pdev->dev.of_node->name);

        pr_info("Doing the msgdma_0\n");

        chan = dma_request_chan(&pdev->dev, "msgdma0");
	if (IS_ERR(chan)) {
		pr_err("questdmatest: No Tx channel %lu\n",
			PTR_ERR(chan));
                return PTR_ERR(chan);
	}
	pr_info("DMA channel pointer: %x\n",(int)chan);


/*	rx_chan = dma_request_slave_channel(&pdev->dev, "axidma1");
	if (IS_ERR(rx_chan)) {
		err = PTR_ERR(rx_chan);
		pr_err("xilinx_dmatest: No Rx channel\n");
		goto free_tx;
	}*/

	err = dmatest_add_slave_channels(chan);
	if (err) {
		pr_err("xilinx_dmatest: Unable to add channels\n");
		goto free_tx;
	}
	return 0;

free_tx:
	dma_release_channel(chan);

	return err;
}

static int quest_dma_ctrl_remove(struct platform_device *pdev)
{
        //struct dmatest_chan *dtc, *_dtc;
        //struct dma_chan *chan;

        pr_info("%s BEGIN quest_dma_ctrl_remove\n", __func__);

        /*
	list_for_each_entry_safe(dtc, _dtc, &dmatest_channels, node) {
		list_del(&dtc->node);
		chan = dtc->chan;
		dmatest_cleanup_channel(dtc);
		pr_info("quest_dmatest: dropped channel %s\n",
			dma_chan_name(chan));
		dma_release_channel(chan);
        }*/

        //free_irq(qcdma.irqn, &pdev->dev);

        device_destroy(cl, dev);
        class_destroy(cl);
        cdev_del(&c_dev);
        unregister_chrdev_region(dev, MINOR_CNT);

        cam_dma_cleanup();

	pr_info("END quest_dma_ctrl_remove\n");
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
	pr_info("Hello init2.\n");
	return platform_driver_register(&quest_dma_ctrl_driver);

}
late_initcall(questdma_init);

static void __exit questdma_exit(void)
{
	pr_info("Hello exit2.\n");
	platform_driver_unregister(&quest_dma_ctrl_driver);
}
module_exit(questdma_exit)

MODULE_AUTHOR("Quest Innovations B.V.");
MODULE_DESCRIPTION("Quest dma controller");
MODULE_LICENSE("GPL v2");
