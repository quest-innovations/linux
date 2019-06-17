#ifndef QUEST_DMA_CORE_H_
#define QUEST_DMA_CORE_H_

#include <linux/types.h>
#include <linux/dmaengine.h>

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

struct quest_dma_channel_struct {
	uint8_t **bufs;
	uint32_t *bufs_raw;
	uint8_t **bufs_user;

	dma_addr_t *dma_handles;

	unsigned int bufs_size;
	unsigned int bufs_cnt;
	unsigned int data_size_bytes;

	uint8_t *write_buf;
	uint32_t write_buf_raw;
	uint8_t *write_buf_user;

	dma_addr_t write_dma_handle;
	unsigned int write_buf_size;
	unsigned int write_data_size;

	unsigned int check_range;

	unsigned int last_read;
	unsigned int last_write;
	bool buffers_full;
	bool data_ready;
};

struct quest_dma_struct {
	struct quest_dma_channel_struct *channels;
	unsigned int ch_cnt;

	bool stop_reading;
	bool stop_reading_acq;
	bool in_read;
	bool reading;

	int irqn;

	void __iomem *base_addr;
};


u32 quest_dma_core_getreg(struct quest_dma_struct *dma, u32 offset);
void quest_dma_core_setreg(struct quest_dma_struct *dma, u32 offset, u32 value);

int quest_dma_core_create_buffers(struct device *dev, struct quest_dma_channel_struct *channel, unsigned int buf_cnt, unsigned int buf_size);
int quest_dma_core_create_write_buffer(struct device *dev, struct quest_dma_channel_struct *channel, unsigned int buf_size);
int quest_dma_core_destroy_buffers(struct device *dev, struct quest_dma_channel_struct *channel);
int quest_dma_core_destroy_write_buffer(struct device *dev, struct quest_dma_channel_struct *channel);

#endif // QUEST_DMA_CORE_H_
