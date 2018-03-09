
#include "quest_dma_core.h"
#include <linux/dma/quest_dma_core_ioctl.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/slab.h>


/**
 * sensor_dma_getreg - Get register from sensor DMA FPGA module
 * @offset: Offset (in words) to base
 *
 * Return: Register of sensor DMA FPGA module
 */
u32 quest_dma_core_getreg(struct quest_dma_struct *dma, u32 offset)
{
    u32 test = readl(dma->base_addr + (offset * 4));
    //pr_info("    %s dma base: %d\n      offset: %d value: %d", __func__, (u32)dma->base_addr, (u32)offset, test);

    return test;
}

/**
 * sensor_dma_setreg - Set register in sensor DMA FPGA module
 * @offset: Offset (in words) to base
 * @value: Value to set in the register
 */
void quest_dma_core_setreg(struct quest_dma_struct *dma, u32 offset, u32 value)
{
    writel(value, dma->base_addr + (offset * 4));
    //pr_info("    %s dma base: %d\n      offset: %d value: %d", __func__, (u32)dma->base_addr, (u32)offset, value);
    udelay(1);
}

int quest_dma_core_create_buffers(struct device *dev, struct quest_dma_channel_struct *channel, unsigned int buf_cnt, unsigned int buf_size)
{
    u32 i = 0;
    //u32 j = 0;

    //pr_info("%s Begin %d %d 1", __func__, bufs_size, bufs_cnt);

    // pr_info("Core, 1");

    if(channel->bufs != 0)
	return -1;

    // pr_info("Core, 2");

    channel->bufs_cnt = buf_cnt;
    channel->data_size_bytes = buf_size;

    channel->check_range = 1024;

    channel->bufs_size = channel->data_size_bytes + (channel->check_range * 2);

    channel->bufs = kcalloc(channel->bufs_cnt + 1 , sizeof(*(channel->bufs)), GFP_KERNEL);
    channel->bufs_raw = kcalloc(channel->bufs_cnt + 1, sizeof(*(channel->bufs_raw)), GFP_KERNEL);
    channel->bufs_user = kcalloc(channel->bufs_cnt + 1, sizeof(*(channel->bufs_user)), GFP_KERNEL);
    channel->dma_handles = kcalloc(channel->bufs_cnt, sizeof(*(channel->dma_handles)), GFP_KERNEL);

  /*  channel->bufs = kcalloc(channel->bufs_cnt + 1 , sizeof(uint8_t *), GFP_KERNEL);
    channel->bufs_raw = kcalloc(channel->bufs_cnt + 1, sizeof(uint32_t), GFP_KERNEL);
    channel->bufs_user = kcalloc(channel->bufs_cnt + 1, sizeof(uint8_t *), GFP_KERNEL);
    channel->dma_handles = kcalloc(channel->bufs_cnt, sizeof(dma_addr_t), GFP_KERNEL);

    qcdma.bufs = kcalloc(qcdma.bufs_cnt+1 , sizeof(u8 *), GFP_KERNEL);
    qcdma.bufs_raw = kcalloc(qcdma.bufs_cnt+1, sizeof(u32), GFP_KERNEL);
    qcdma.bufs_user = kcalloc(qcdma.bufs_cnt+1, sizeof(u8 *), GFP_KERNEL);
    qcdma.dma_handles = kcalloc(qcdma.bufs_cnt, sizeof(dma_addr_t), GFP_KERNEL);*/

    //pr_info("%d %d %d %d %d", channel->bufs_cnt, channel->bufs, channel->bufs_raw, channel->bufs_user, channel->dma_handles);

    if(!channel->bufs ||
	!channel->bufs_raw ||
        !channel->bufs_user ||
        !channel->dma_handles)
	return -1;

    // pr_info("Core, 3");

    for(; i < channel->bufs_cnt; ++i)
    {
	//pr_info("Going to allocate memory %d", channel->bufs_size);

	// Assume caller knows what he is doing trying to alloc mem
	channel->bufs[i] = dma_alloc_coherent(dev, channel->bufs_size, &channel->dma_handles[i], GFP_KERNEL);

	if(!channel->bufs[i])
		return -1;

	channel->bufs_user[i] = channel->bufs[i] + channel->check_range;
	channel->bufs_raw[i] = channel->dma_handles[i] + channel->check_range;

	//pr_info("Raw assignment OK %d", channel->bufs_raw[i]);
    }

    channel->bufs[i+1] = NULL;
    channel->bufs_user[i+1] = NULL;

    channel->bufs_cnt = buf_cnt;

    // Set read and write to last buffer so they begin at 0 at a write
    channel->last_read = channel->bufs_cnt - 1;
    channel->last_write = channel->bufs_cnt - 1;

    //dmatest_init_srcs_quest(channel->bufs, channel->check_range, channel->data_size_bytes + channel->check_range, channel->bufs_size);

    // pr_info("Core, buffers created");

    return 0;
}

int quest_dma_core_create_write_buffer(struct device *dev, struct quest_dma_channel_struct *channel, unsigned int buf_size)
{
    //u32 i = 0;
    //u32 j = 0;

    // pr_info("%s Begin %d", __func__, buf_size);

    // pr_info("Core, 1");

    if(channel->write_buf != 0)
	return -1;
/*
    uint8_t *write_buf;
    uint32_t write_buf_raw;
    uint8_t *write_buf_user;

    dma_addr_t *write_dma_handle;

    unsigned int write_buf_size;
*/

    channel->write_data_size = buf_size;

    channel->check_range = 1024;

    channel->write_buf_size = channel->write_data_size + (channel->check_range * 2);

    // pr_info("Core, 3");

    channel->write_buf = dma_alloc_coherent(dev, channel->write_buf_size, &channel->write_dma_handle, GFP_KERNEL);

    // pr_info("Still alive!");

    if(!channel->write_buf)
    {
	    pr_info("Core, error creating write buffers");
	    return -1;
    }

    channel->write_buf_user = channel->write_buf + channel->check_range;
    channel->write_buf_raw = channel->write_dma_handle + channel->check_range;

    // pr_info("Core, write buffers created");

    return 0;
}

/**
 * cam_dma_cleanup - Cleanup DMA buffer
 */
int quest_dma_core_destroy_buffers(struct device *dev, struct quest_dma_channel_struct *channel)
{
    u32 i = 0;
    ////pr_info("%s", __func__);

    if(channel->bufs)
    {
	for (; channel->bufs[i]; i++)
	{
		dma_free_coherent(dev, channel->bufs_size, channel->bufs[i], channel->dma_handles[i]);
		//kfree(channel->bufs[i]);
	}

	kfree(channel->bufs);
    }

    if(channel->bufs_raw)
	kfree(channel->bufs_raw);

    if(channel->bufs_user)
	kfree(channel->bufs_user);

    channel->bufs = 0;
    channel->bufs_raw = 0;
    channel->bufs_user = 0;
    channel->bufs_size = 0;
    channel->bufs_cnt = 0;

    // pr_info("%s", __func__);

    return 0;
}

/**
 * cam_dma_cleanup - Cleanup DMA write buffer
 */
int quest_dma_core_destroy_write_buffer(struct device *dev, struct quest_dma_channel_struct *channel)
{
	if(channel->write_buf)
	{
		dma_free_coherent(dev, channel->write_buf_size, channel->write_buf, channel->write_dma_handle);
	}

	channel->write_buf = 0;

	// pr_info("%s", __func__);

	return 0;
}

/*MODULE_AUTHOR("Quest Innovations B.V.");
MODULE_DESCRIPTION("Quest DMA core");
MODULE_LICENSE("GPL v2");*/
