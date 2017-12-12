#ifndef QUEST_DMA_CTRL_H_
#define QUEST_DMA_CTRL_H_

#include <linux/ioctl.h>

typedef struct
{
    int status, dignity, ego;
} query_arg_t;

typedef struct
{
    unsigned int buf_size, buf_cnt;
} qdma_alloc_arg_t;

typedef struct
{
    uint8_t *buf_dst;
    bool *frame_skipped;
} qdma_buf_arg_t;

typedef struct
{
    uint32_t offset;
    uint32_t value;
} qdma_reg_arg_t;

#define QUERY_GET_VARIABLES _IOR('q', 1, query_arg_t *)
#define QUERY_CLR_VARIABLES _IO('q', 2)
#define QUERY_SET_VARIABLES _IOW('q', 3, query_arg_t *)

#define QDMA_ALLOC_MEM _IOW('q', 4, qdma_alloc_arg_t *)
#define QDMA_FREE_MEM _IO('q', 5)

#define QDMA_START_GRAB _IO('q', 6)
#define QDMA_STOP_GRAB _IO('q', 7)
#define QDMA_GRAB_IMG _IOR('q', 8, qdma_buf_arg_t *)

#define QDMA_SENSOR_DMA_SETREG _IOW('q', 9, qdma_reg_arg_t *)
#define QDMA_SENSOR_DMA_GETREG _IOR('q', 10, qdma_reg_arg_t *)

#define QDMA_DRIVE_GRABBING _IO('q', 11)

#endif
