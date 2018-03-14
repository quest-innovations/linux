#ifndef QUEST_DMA_CORE_IOCTL_H_
#define QUEST_DMA_CORE_IOCTL_H_

#include <linux/ioctl.h>
#include <linux/types.h>

/************************************************************************************
*********************************  IOCTL INTERFACE  *********************************
************************************************************************************/

typedef struct {
	uint32_t offset;
	uint32_t value;
} qdma_reg_arg_t;

typedef struct {
	uint32_t ch;
	uint8_t *buf;
	uint32_t size_bytes;
	bool *buffers_full;
} qdma_buf_arg_t;

// Starts read of selected channel, if not started, and returns when data is available
// or when stop read is called
#define QDMA_READ _IOR('q', 1, qdma_buf_arg_t *)

// Writes data to selected channel, reading will be stopped and restarted if not
// already stopped
#define QDMA_WRITE _IOW('q', 2, qdma_buf_arg_t *)

// Stops the reading of data
#define QDMA_STOP_READ _IO('q', 3)

// Sets the size of the selected channel
#define QDMA_SET_CH_READ_SIZE _IOW('q', 4, qdma_buf_arg_t *)
#define QDMA_SET_CH_WRITE_SIZE _IOW('q', 5, qdma_buf_arg_t *)

// Drives the reading of buffers (when not using IRQ)
#define QDMA_DRIVE_READ _IOR('q', 6, qdma_buf_arg_t *)

// Set and get register functions
#define QDMA_SETREG _IOW('q', 7, qdma_reg_arg_t *)
#define QDMA_GETREG _IOR('q', 8, qdma_reg_arg_t *)

#endif // QUEST_DMA_CORE_IOCTL_H_
