/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Copyright (C) 2010 Samsung Electronics Co. Ltd.
 *	Jaswinder Singh <jassi.brar@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl330.h>
#include <linux/scatterlist.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/err.h>
#include <linux/printk.h> /* For pr_debug() and pr_info() */

#include "dmaengine.h"
#include "pl330.h"

#define PL330_MAX_CHAN		8
#define PL330_MAX_IRQS		32
#define PL330_MAX_PERI		32

enum pl330_srccachectrl {
	SCCTRL0,  /* Noncacheable and nonbufferable */
	SCCTRL1,  /* Bufferable only */
	SCCTRL2,  /* Cacheable, but do not allocate */
	SCCTRL3,  /* Cacheable and bufferable, but do not allocate */
	SINVALID1,
	SINVALID2,
	SCCTRL6,  /* Cacheable write-through, allocate on reads only */
	SCCTRL7,  /* Cacheable write-back, allocate on reads only */
};

enum pl330_dstcachectrl {
	DCCTRL0,   /* Noncacheable and nonbufferable */
	DCCTRL1,   /* Bufferable only */
	DCCTRL2,   /* Cacheable, but do not allocate */
	DCCTRL3,   /* Cacheable and bufferable, but do not allocate */
	DINVALID1, /* AWCACHE = 0x1000 */
	DINVALID2,
	DCCTRL6,   /* Cacheable write-through, allocate on writes only */
	DCCTRL7,   /* Cacheable write-back, allocate on writes only */
};

enum pl330_byteswap {
	SWAP_NO,
	SWAP_2,
	SWAP_4,
	SWAP_8,
	SWAP_16,
};

enum pl330_reqtype {
	MEMTOMEM,
	MEMTODEV,
	DEVTOMEM,
	DEVTODEV,
};

/* Register and Bit field Definitions */
#define DS			0x0
#define DS_ST_STOP		0x0
#define DS_ST_EXEC		0x1
#define DS_ST_CMISS		0x2
#define DS_ST_UPDTPC		0x3
#define DS_ST_WFE		0x4
#define DS_ST_ATBRR		0x5
#define DS_ST_QBUSY		0x6
#define DS_ST_WFP		0x7
#define DS_ST_KILL		0x8
#define DS_ST_CMPLT		0x9
#define DS_ST_FLTCMP		0xe
#define DS_ST_FAULT		0xf

#define DPC			0x4
#define INTEN			0x20
#define ES			0x24
#define INTSTATUS		0x28
#define INTCLR			0x2c
#define FSM			0x30
#define FSC			0x34
#define FTM			0x38

#define _FTC			0x40
#define FTC(n)			(_FTC + (n)*0x4)

#define _CS			0x100
#define CS(n)			(_CS + (n)*0x8)
#define CS_CNS			(1 << 21)

#define _CPC			0x104
#define CPC(n)			(_CPC + (n)*0x8)

#define _SA			0x400
#define SA(n)			(_SA + (n)*0x20)

#define _DA			0x404
#define DA(n)			(_DA + (n)*0x20)

#define _CC			0x408
#define CC(n)			(_CC + (n)*0x20)

#define CC_SRCINC		(1 << 0)
#define CC_DSTINC		(1 << 14)
#define CC_SRCPRI		(1 << 8)
#define CC_DSTPRI		(1 << 22)
#define CC_SRCNS		(1 << 9)
#define CC_DSTNS		(1 << 23)
#define CC_SRCIA		(1 << 10)
#define CC_DSTIA		(1 << 24)
#define CC_SRCBRSTLEN_SHFT	4
#define CC_DSTBRSTLEN_SHFT	18
#define CC_SRCBRSTSIZE_SHFT	1
#define CC_DSTBRSTSIZE_SHFT	15
#define CC_SRCCCTRL_SHFT	11
#define CC_SRCCCTRL_MASK	0x7
#define CC_DSTCCTRL_SHFT	25
#define CC_DRCCCTRL_MASK	0x7
#define CC_SWAP_SHFT		28

#define _LC0			0x40c
#define LC0(n)			(_LC0 + (n)*0x20)

#define _LC1			0x410
#define LC1(n)			(_LC1 + (n)*0x20)

#define DBGSTATUS		0xd00
#define DBG_BUSY		(1 << 0)

#define DBGCMD			0xd04
#define DBGINST0		0xd08
#define DBGINST1		0xd0c

#define CR0			0xe00
#define CR1			0xe04
#define CR2			0xe08
#define CR3			0xe0c
#define CR4			0xe10
#define CRD			0xe14

#define PERIPH_ID		0xfe0
#define PERIPH_REV_SHIFT	20
#define PERIPH_REV_MASK		0xf
#define PERIPH_REV_R0P0		0
#define PERIPH_REV_R1P0		1
#define PERIPH_REV_R1P1		2

#define CR0_PERIPH_REQ_SET	(1 << 0)
#define CR0_BOOT_EN_SET		(1 << 1)
#define CR0_BOOT_MAN_NS		(1 << 2)
#define CR0_NUM_CHANS_SHIFT	4
#define CR0_NUM_CHANS_MASK	0x7
#define CR0_NUM_PERIPH_SHIFT	12
#define CR0_NUM_PERIPH_MASK	0x1f
#define CR0_NUM_EVENTS_SHIFT	17
#define CR0_NUM_EVENTS_MASK	0x1f

#define CR1_ICACHE_LEN_SHIFT      0
#define CR1_ICACHE_LEN_MASK       0x7
#define CR1_NUM_ICACHELINES_SHIFT 4
#define CR1_NUM_ICACHELINES_MASK  0xf

#define CRD_DATA_WIDTH_SHIFT	0
#define CRD_DATA_WIDTH_MASK	0x7
#define CRD_WR_CAP_SHIFT	4
#define CRD_WR_CAP_MASK		0x7
#define CRD_WR_Q_DEP_SHIFT	8
#define CRD_WR_Q_DEP_MASK	0xf
#define CRD_RD_CAP_SHIFT	12
#define CRD_RD_CAP_MASK		0x7
#define CRD_RD_Q_DEP_SHIFT	16
#define CRD_RD_Q_DEP_MASK	0xf
#define CRD_DATA_BUFF_SHIFT	20
#define CRD_DATA_BUFF_MASK	0x3ff

#define PART			0x330
#define DESIGNER		0x41
#define REVISION		0x0
#define INTEG_CFG		0x0
#define PERIPH_ID_VAL		((PART << 0) | (DESIGNER << 12))

#define CMD_DMAADDH		0x54
#define CMD_DMAEND		0x00
#define CMD_DMAFLUSHP		0x35
#define CMD_DMAGO		0xa0
#define CMD_DMALD		0x04
#define CMD_DMALDP		0x25
#define CMD_DMALP		0x20
#define CMD_DMALPEND		0x28
#define CMD_DMAKILL		0x01
#define CMD_DMAMOV		0xbc
#define CMD_DMANOP		0x18
#define CMD_DMARMB		0x12
#define CMD_DMASEV		0x34
#define CMD_DMAST		0x08
#define CMD_DMASTP		0x29
#define CMD_DMASTZ		0x0c
#define CMD_DMAWFE		0x36
#define CMD_DMAWFP		0x30
#define CMD_DMAWMB		0x13

#define SZ_DMAADDH		3
#define SZ_DMAEND		1
#define SZ_DMAFLUSHP		2
#define SZ_DMALD		1
#define SZ_DMALDP		2
#define SZ_DMALP		2
#define SZ_DMALPEND		2
#define SZ_DMAKILL		1
#define SZ_DMAMOV		6
#define SZ_DMANOP		1
#define SZ_DMARMB		1
#define SZ_DMASEV		2
#define SZ_DMAST		1
#define SZ_DMASTP		2
#define SZ_DMASTZ		1
#define SZ_DMAWFE		2
#define SZ_DMAWFP		2
#define SZ_DMAWMB		1
#define SZ_DMAGO		6

#define BRST_LEN(ccr)		((((ccr) >> CC_SRCBRSTLEN_SHFT) & 0xf) + 1)
#define BRST_SIZE(ccr)		(1 << (((ccr) >> CC_SRCBRSTSIZE_SHFT) & 0x7))

#define BYTE_TO_BURST(b, ccr)	((b) / BRST_SIZE(ccr) / BRST_LEN(ccr))
#define BURST_TO_BYTE(c, ccr)	((c) * BRST_SIZE(ccr) * BRST_LEN(ccr))

/* Logical states - not CS or DS in HW. */
#define PL330_STATE_STOPPED		(1 << 0)
#define PL330_STATE_EXECUTING		(1 << 1)
#define PL330_STATE_WFE			(1 << 2)
#define PL330_STATE_FAULTING		(1 << 3)
#define PL330_STATE_COMPLETING		(1 << 4)
#define PL330_STATE_WFP			(1 << 5)
#define PL330_STATE_KILLING		(1 << 6)
#define PL330_STATE_FAULT_COMPLETING	(1 << 7)
#define PL330_STATE_CACHEMISS		(1 << 8)
#define PL330_STATE_UPDTPC		(1 << 9)
#define PL330_STATE_ATBARRIER		(1 << 10)
#define PL330_STATE_QUEUEBUSY		(1 << 11)
#define PL330_STATE_INVALID		(1 << 15)

#define PL330_STABLE_STATES (PL330_STATE_STOPPED | PL330_STATE_EXECUTING \
				| PL330_STATE_WFE | PL330_STATE_FAULTING)

/*
 * With 256 bytes, we can do more than 2.5MB and 5MB xfers per req
 * at 1byte/burst for P<->M and M<->M respectively.
 * For typical scenario, at 1word/burst, 10MB and 20MB xfers per req
 * should be enough for P<->M and M<->M respectively.
 * For unrolled loops and maximum size, I need several 256 bytes buffers.
 * First one is with two nested loops and unrolled move operations.
 * Second one is with one nested loop and unrolled move operations.
 * Third one is just unrolled move operations.
 * Note that unrolled loops are slower when the entire loop can fit into
 * cache.
 */
#define MCODE_BUFF_PER_REQ PAGE_SIZE

/* If the _pl330_req is available to the client */
#define IS_FREE(req)	(*((u8 *)((req)->mc_cpu)) == CMD_DMAEND)

/*TODO: Should this macro be a function? */
/* Use this _only_ to wait on transient states */
#define UNTIL(t, s)	\
	do {\
		while (!(_state(t) & (s)))\
			cpu_relax();\
	} while (0)


#ifdef PL330_DEBUG_MCGEN
#define DCC_PR_INFO(_args...) pr_info(_args)
#else
#define DCC_PR_INFO(_args...)
#endif

/* The number of default descriptors */

#define NR_DEFAULT_DESC	16

/* Populated by the PL330 core driver for DMA API driver's info */
struct pl330_config {
	u32          periph_id;
#define DMAC_MODE_NS	(1 << 0)
	unsigned int mode;
	unsigned int data_bus_width; /* In number of bits */
	unsigned int data_buf_dep;
	unsigned int num_chan;
	unsigned int num_peri;
	u32          peri_ns;
	unsigned int num_events;
	u32          irq_ns;
	unsigned int icache_lines;
	unsigned int icache_len;
};

/* Handle to the DMAC provided to the PL330 core */
struct pl330_info {
	/* Owning device */
	struct device *dev;
	/* Size of MicroCode buffers for each channel. */
	unsigned mcbufsz;
	/* ioremap'ed address of PL330 registers. */
	void __iomem *base;
	/* Client can freely use it. */
	void *client_data;
	/* PL330 core data, Client must not touch it. */
	void *pl330_data;
	/* Populated by the PL330 core driver during pl330_add */
	struct pl330_config cfg;
	/*
	 * If the DMAC has some reset mechanism, then the
	 * client may want to provide pointer to the method.
	 */
	void (*dmac_reset)(struct pl330_info *pi);
};

/**
 * Request Configuration.
 * The PL330 core does not modify this and uses the last
 * working configuration if the request doesn't provide any.
 *
 * The Client may want to provide this info only for the
 * first request and a request with new settings.
 */
struct pl330_reqcfg {
	/* Address Incrementing */
	bool dst_inc;
	bool src_inc;

	/*
	 * For now, the SRC & DST protection levels
	 * and burst size/length are assumed same.
	 */
	bool nonsecure;
	bool privileged;
	bool insnaccess;

	/* TODO: A pointer to the channel would be easier than copying */
	/* TODO: all these over. */
	u32      brst_len;
	u32      brst_size; /* in power of 2 */
	bool     flush_once;
	bool     wfp_once;
	bool     unroll_loop;
	bool     device_fc;

	enum pl330_dstcachectrl dcctl;
	enum pl330_srccachectrl scctl;
	enum pl330_byteswap     swap;
	struct pl330_config    *pcfg;
};

/*
 * One cycle of DMAC operation.
 * There may be more than one xfer in a request.
 */
struct pl330_xfer {
	u32 src_addr;
	u32 dst_addr;
	/* Size to xfer */
	u32 bytes;
	/*
	 * Pointer to next xfer in the list.
	 * The last xfer in the req must point to NULL.
	 */
	struct pl330_xfer *next;
};

/* The xfer callbacks are made with one of these arguments. */
enum pl330_op_err {
	PL330_ERR_NONE, /* The all xfers in the request were success. */
	PL330_ERR_ABORT,/* If req aborted due to global error. */
	PL330_ERR_FAIL, /* If req failed due to problem with Channel. */
};

/* A request defining Scatter-Gather List ending with NULL xfer. */
struct pl330_req {
	enum pl330_reqtype rqtype;

	/* Index of peripheral for the xfer. */
	/* This is passed down into the microcode functions that expect a */
	/* u8 datatype. */
	u8 peri;

	/* Unique token for this xfer, set by the client. */
	void *token;

	/* Callback to be called after xfer. */
	void (*xfer_cb)(void *token, enum pl330_op_err err);

	/* If NULL, req will be done at last set parameters. */
	struct pl330_reqcfg *preqcfg;

	/* Pointer to first xfer in the request. */
	struct pl330_xfer *x;

	/* Hook to attach to DMAC's list of reqs with due callback */
	struct list_head rqd;
};

#if 0 /* NOT CURRENTLY USED */
/*
 * To know the status of the channel and DMAC, the client
 * provides a pointer to this structure. The PL330 core
 * fills it with current information.
 */
struct pl330_chanstatus {
	/*
	 * If the DMAC engine halted due to some error,
	 * the client should remove-add DMAC.
	 */
	bool dmac_halted;

	/*
	 * If channel is halted due to some error,
	 * the client should ABORT/FLUSH and START the channel.
	 */
	bool faulting;

	/* Location of last load */
	u32 src_addr;

	/* Location of last store */
	u32 dst_addr;

	/*
	 * Pointer to the currently active req, NULL if channel is
	 * inactive, even though the requests may be present.
	 */
	struct pl330_req *top_req;

	/* Pointer to req waiting second in the queue if any. */
	struct pl330_req *wait_req;
};
#endif

enum pl330_chan_op {
	PL330_OP_START, /* Start the channel */
	PL330_OP_ABORT, /* Abort the active xfer */
	PL330_OP_FLUSH, /* Stop xfer and flush queue */
};

struct _xfer_spec {
	u32           ccr;
	struct pl330_req  *r;
	struct pl330_xfer *x;
};

/* These are actual register numbers. */
enum dmamov_dst {
	SAR = 0,
	CCR,
	DAR,
};

/* These are actual register numbers. */
enum pl330_dst {
	SRC = 0,
	DST,
};

struct _pl330_req {
	u32               mc_bus;
	void             *mc_cpu;
	u32               mc_len;/* Bytes taken to setup MC for the req */
	struct pl330_req *r;
};

/* ToBeDone for tasklet */
struct _pl330_tbd {
	bool reset_dmac;
	bool reset_mngr;
	u8   reset_chan;
};

/* A DMAC Thread */
struct pl330_thread {
	u8 id;
	int ev;

	/* If the channel is not yet acquired by any client */
	bool free;

	/* Parent DMAC */
	struct pl330_dmac *dmac;

	/* Only two at a time */
	struct _pl330_req req[2];

	/* Index of the last enqueued request */
	unsigned lstenq;

	/* Index of the last submitted request or -1 if the DMA is stopped */
	int req_running;
};

enum pl330_dmac_state {
	UNINIT,
	INIT,
	DYING,
};

/* A DMAC */
struct pl330_dmac {
	spinlock_t lock;

	/* Holds list of reqs with due callbacks */
	struct list_head req_done;


	/* Pointer to platform specific stuff */
	struct pl330_info *pinfo;

	/* Maximum possible events/irqs */
	int events[32];

	/* BUS address of MicroCode buffer */
	dma_addr_t mcode_bus;

	/* CPU address of MicroCode buffer */
	void            *mcode_cpu;

	/* List of all Channel threads */
	struct pl330_thread  *channels;

	/* Pointer to the MANAGER thread */
	struct pl330_thread  *manager;

	/* To handle bad news in interrupt */
	struct tasklet_struct tasks;

	struct _pl330_tbd     dmac_tbd;

	/* State of DMAC operation */
	enum pl330_dmac_state state;
};

enum desc_status {
	/* In the DMAC pool */
	FREE,
	/*
	 * Allocated to some channel during prep_xxx
	 * Also may be sitting on the work_list.
	 */
	PREP,
	/*
	 * Sitting on the work_list and already submitted
	 * to the PL330 core. Not more than two descriptors
	 * of a channel can be BUSY at any time.
	 */
	BUSY,
	/*
	 * Sitting on the channel work_list but xfer done
	 * by PL330 core
	 */
	DONE,
};

struct dma_pl330_chan {
	/* Schedule desc completion */
	struct tasklet_struct task;

	/* DMA-Engine Channel */
	struct dma_chan chan;

	/* List of to be xfered descriptors */
	struct list_head work_list;
	/* List of completed descriptors */
	struct list_head completed_list;

	/* Pointer to the DMAC that manages this channel,
	 * NULL if the channel is available to be acquired.
	 * As the parent, this DMAC also provides descriptors
	 * to the channel.
	 */
	struct dma_pl330_dmac *dmac;

	/* To protect channel manipulation */
	spinlock_t lock;

	/* Token of a hardware channel thread of PL330 DMAC
	 * NULL if the channel is available to be acquired.
	 */
	void *pl330_chid;

	/* For D-to-M and M-to-D channels */
	u32        burst_sz;  /* the peripheral fifo width */
	u32        burst_len; /* the number of burst */
	dma_addr_t fifo_addr;
	bool       flush_once;
	bool       wfp_once;
	bool       unroll_loop;
	bool       device_fc;

	/* for cyclic capability */
	bool cyclic;
};

struct dma_pl330_dmac {
	struct pl330_info pif;

	/* DMA-Engine Device */
	struct dma_device ddma;

	/* Pool of descriptors available for the DMAC's channels */
	struct list_head desc_pool;

	/* To protect desc_pool manipulation */
	spinlock_t pool_lock;

	/* Peripheral channels connected to this DMAC */
	struct dma_pl330_chan *peripherals; /* keep at end */
};

struct dma_pl330_desc {
	/* To attach to a queue as child */
	struct list_head node;

	/* Descriptor for the DMA Engine API */
	struct dma_async_tx_descriptor txd;

	/* Xfer for PL330 core */
	struct pl330_xfer     px;
	struct pl330_reqcfg   rqcfg;
	struct pl330_req      req;
	enum desc_status status;

	/* The channel which currently holds this desc */
	struct dma_pl330_chan *pchan;
};

struct dma_pl330_filter_args {
	struct dma_pl330_dmac *pdmac;
	unsigned int      chan_id;
};

static inline void _callback(
	struct pl330_req *r,
	enum pl330_op_err err
)
{
	if (r && r->xfer_cb)
		r->xfer_cb(r->token, err);
}

static inline u32 get_revision(u32 periph_id)
{
	return (periph_id >> PERIPH_REV_SHIFT) & PERIPH_REV_MASK;
}

/*------------------------ assembler code start -----------------------------*/
static inline u32 _emit_ADDH(
	unsigned       dry_run,
	u8             buf[],
	enum pl330_dst da,
	u16            val
)
{
	if (dry_run)
		return SZ_DMAADDH;

	buf[0] = CMD_DMAADDH;
	buf[0] |= (da << 1);
	*((u16 *)&buf[1]) = val;

	DCC_PR_INFO("DMAADDH %s %u\n", da == 1 ? "DA" : "SA", val);
	return SZ_DMAADDH;
}

static inline u32 _emit_END(unsigned dry_run, u8 buf[])
{
	if (dry_run)
		return SZ_DMAEND;
	buf[0] = CMD_DMAEND;
	DCC_PR_INFO("DMAEND\n");
	return SZ_DMAEND;
}

/* Note that peripheral number is not range checked. */
static inline u32 _emit_FLUSHP(unsigned dry_run, u8 buf[], u8 peri)
{
	if (dry_run)
		return SZ_DMAFLUSHP;
	buf[0] = CMD_DMAFLUSHP;
	buf[1] = (peri & 0x1F) << 3;
	DCC_PR_INFO("DMAFLUSHP %u\n", peri);
	return SZ_DMAFLUSHP;
}

static inline u32 _emit_LD(unsigned dry_run, u8 buf[]) /* Always */
{
	if (dry_run)
		return SZ_DMALD;
	buf[0] = CMD_DMALD | (0 << 1) | (0 << 0);
	DCC_PR_INFO("DMALD\n");
	return SZ_DMALD;
}

static inline u32 _emit_LDB(unsigned dry_run, u8 buf[])/* Burst */
{
	if (dry_run)
		return SZ_DMALD;
	buf[0] = CMD_DMALD | (1 << 1) | (1 << 0);
	DCC_PR_INFO("DMALDB\n");
	return SZ_DMALD;
}

static inline u32 _emit_LDS(unsigned dry_run, u8 buf[])/* Single */
{
	if (dry_run)
		return SZ_DMALD;
	buf[0] = CMD_DMALD | (0 << 1) | (1 << 0);
	DCC_PR_INFO("DMALDS\n");
	return SZ_DMALD;
}

static inline u32 _emit_LDPB(unsigned dry_run, u8 buf[], u8 peri)
{
	if (dry_run)
		return SZ_DMALDP;
	buf[0] = CMD_DMALDP | (1 << 1);
	buf[1] = (peri & 0x1F) << 3;
	DCC_PR_INFO("DMALDPB %u\n", peri);
	return SZ_DMALDP;
}

static inline u32 _emit_LDPS(unsigned dry_run, u8 buf[], u8 peri)
{
	if (dry_run)
		return SZ_DMALDP;
	buf[0] = CMD_DMALDP | (0 << 1);
	buf[1] = (peri & 0x1F) << 3;
	DCC_PR_INFO("DMALDPS %u\n", peri);
	return SZ_DMALDP;
}

static inline u32 _emit_LP(unsigned dry_run, u8 buf[], unsigned cnt, u8 loop)
{
	if (dry_run)
		return SZ_DMALP;
	buf[0] = CMD_DMALP | (loop << 1);
	buf[1] = (u8)(cnt - 1); /* DMAC increments by 1 internally */
	DCC_PR_INFO("DMALP lp=%d, it=%u\n", loop, cnt);
	return SZ_DMALP;
}

/*
 * The following LPEND APIs assume the nf bit is always 1.
 * In other words, DMALPFE is not supported and never used.
 */
 /* Always */
static inline u32 _emit_LPEND(unsigned dry_run, u8 buf[], u8 bjump, u8 loop)
{
	if (dry_run)
		return SZ_DMALPEND;
	buf[0] = CMD_DMALPEND | (1 << 4) | (loop << 2) | (0 << 1) | (0 << 0);
	buf[1] = bjump;
	DCC_PR_INFO("DMALPEND lp=%d, jp=%x\n", loop, bjump);
	return SZ_DMALPEND;
}

 /* Burst */
static inline u32 _emit_LPENDB(unsigned dry_run, u8 buf[], u8 bjump, u8 loop)
{
	if (dry_run)
		return SZ_DMALPEND;
	buf[0] = CMD_DMALPEND | (1 << 4) | (loop << 2) | (1 << 1) | (1 << 0);
	buf[1] = bjump;
	DCC_PR_INFO("DMALPENDB lp=%d, jp=%x\n", loop, bjump);
	return SZ_DMALPEND;
}

/* Single */
static inline u32 _emit_LPENDS(unsigned dry_run, u8 buf[], u8 bjump, u8 loop)
{
	if (dry_run)
		return SZ_DMALPEND;
	buf[0] = CMD_DMALPEND | (1 << 4) | (loop << 2) | (0 << 1) | (1 << 0);
	buf[1] = bjump;
	DCC_PR_INFO("DMALPENDS lp=%d, jp=%x\n", loop, bjump);
	return SZ_DMALPEND;
}

/* Note that validity of dst is not checked. */
static inline u32 _emit_MOV(
	unsigned        dry_run,
	u8              buf[],
	enum dmamov_dst dst,
	u32             val
)
{
#ifdef PL330_DEBUG_MCGEN
	static const char * const dstnames[] = {"SAR", "CCR", "DAR"};
	u32                       saveval    = val;
#endif
	if (dry_run)
		return SZ_DMAMOV;

	buf[0] = CMD_DMAMOV;
	buf[1] = dst;
	buf[2] = (u8)(val & 0xFF); val >>= 8; /*LSB */
	buf[3] = (u8)(val & 0xFF); val >>= 8;
	buf[4] = (u8)(val & 0xFF); val >>= 8;
	buf[5] = (u8)(val & 0xFF);           /*MSB */

#ifdef PL330_DEBUG_MCGEN
	if (dst < 3)
		DCC_PR_INFO("DMAMOV %s, 0x%08x\n", dstnames[dst], saveval);
	else
		DCC_PR_INFO("DMAMOV 0x%x, 0x%08x\n", dst, saveval);
#endif
	return SZ_DMAMOV;
}

static inline u32 _emit_NOP(unsigned dry_run, u8 buf[])
{
	if (dry_run)
		return SZ_DMANOP;
	buf[0] = CMD_DMANOP;
	DCC_PR_INFO("DMANOP\n");
	return SZ_DMANOP;
}

static inline u32 _emit_RMB(unsigned dry_run, u8 buf[])
{
	if (dry_run)
		return SZ_DMARMB;
	buf[0] = CMD_DMARMB;
	DCC_PR_INFO("DMARMB\n");
	return SZ_DMARMB;
}

/* Note no check of event range. */
static inline u32 _emit_SEV(unsigned dry_run, u8 buf[], u8 ev)
{
	if (dry_run)
		return SZ_DMASEV;
	buf[0] = CMD_DMASEV;
	buf[1] = (ev & 0x1F) << 3;
	DCC_PR_INFO("DMASEV %u\n", ev);
	return SZ_DMASEV;
}

static inline u32 _emit_ST(unsigned dry_run, u8 buf[])/* Always*/
{
	if (dry_run)
		return SZ_DMAST;
	buf[0] = CMD_DMAST | (0 << 1) | (0 << 0);
	DCC_PR_INFO("DMAST\n");
	return SZ_DMAST;
}

static inline u32 _emit_STB(unsigned dry_run, u8 buf[])/*Burst*/
{
	if (dry_run)
		return SZ_DMAST;
	buf[0] = CMD_DMAST | (1 << 1) | (1 << 0);
	DCC_PR_INFO("DMASTB\n");
	return SZ_DMAST;
}

static inline u32 _emit_STS(unsigned dry_run, u8 buf[])/*Single*/
{
	if (dry_run)
		return SZ_DMAST;
	buf[0] = CMD_DMAST | (0 << 1) | (1 << 0);
	DCC_PR_INFO("DMASTS\n");
	return SZ_DMAST;
}

/* Note that peripheral number is not range checked. */
static inline u32 _emit_STPB(unsigned dry_run, u8 buf[], u8 peri)/*Burst*/
{
	if (dry_run)
		return SZ_DMASTP;
	buf[0] = CMD_DMASTP | (1 << 1);
	buf[1] = (peri & 0x1F) << 3;
	DCC_PR_INFO("DMASTPB %u\n", peri);
	return SZ_DMASTP;
}

/* Note that peripheral number is not range checked. */
static inline u32 _emit_STPS(unsigned dry_run, u8 buf[], u8 peri)/*Single*/
{
	if (dry_run)
		return SZ_DMASTP;
	buf[0] = CMD_DMASTP | (0 << 1);
	buf[1] = (peri & 0x1F) << 3;
	DCC_PR_INFO("DMASTPS %u\n", peri);
	return SZ_DMASTP;
}

static inline u32 _emit_STZ(unsigned dry_run, u8 buf[])
{
	if (dry_run)
		return SZ_DMASTZ;
	buf[0] = CMD_DMASTZ;
	DCC_PR_INFO("DMASTZ\n");
	return SZ_DMASTZ;
}

/* Note that event is not range checked. */
static inline u32 _emit_WFE(
	unsigned dry_run,
	u8       buf[],
	u8       ev,
	unsigned invalidate /*TODO: Review. This should be a bool. */
)
{
	if (dry_run)
		return SZ_DMAWFE;
	buf[0] = CMD_DMAWFE;
	buf[1] = (ev & 0x1F) << 3;
	if (invalidate)
		buf[1] |= (1 << 1);
	DCC_PR_INFO("DMAWFE %u, %s\n", ev, invalidate ? ", I" : "");
	return SZ_DMAWFE;
}

/* Note that peripheral number is not range checked. */
static inline u32 _emit_WFPB(unsigned dry_run, u8 buf[], u8 peri)/*Burst*/
{
	if (dry_run)
		return SZ_DMAWFP;
	buf[0] = CMD_DMAWFP | (1 << 1) | (0 << 0);
	buf[1] = (peri & 0x1F) << 3;
	DCC_PR_INFO("DMAWFPB %u\n", peri);
	return SZ_DMAWFP;
}

/* Note that peripheral number is not range checked. */
static inline u32 _emit_WFPP(unsigned dry_run, u8 buf[], u8 peri)/*Either*/
{
	if (dry_run)
		return SZ_DMAWFP;
	buf[0] = CMD_DMAWFP | (0 << 1) | (1 << 0);
	buf[1] = (peri & 0x1F) << 3;
	DCC_PR_INFO("DMAWFPP %u\n", peri);
	return SZ_DMAWFP;
}

/* Note that peripheral number is not range checked. */
static inline u32 _emit_WFPS(
	unsigned        dry_run,
	u8              buf[],
	u8              peri)/*Single*/
{
	if (dry_run)
		return SZ_DMAWFP;
	buf[0] = CMD_DMAWFP | (0 << 1) | (0 << 0);
	buf[1] = (peri & 0x1F) << 3;
	DCC_PR_INFO("DMAWFPS %u\n", peri);
	return SZ_DMAWFP;
}

static inline u32 _emit_WMB(unsigned dry_run, u8 buf[])
{
	if (dry_run)
		return SZ_DMAWMB;
	buf[0] = CMD_DMAWMB;
	DCC_PR_INFO("DMAWMB\n");
	return SZ_DMAWMB;
}

/*
 * The following functions is not used as part of a normal transfer microcode.
 * These are used for setting up the DGBINST0 register.
 * No dry run to check to for space should be required.
 */
static inline u16 _opcode_GO(u8 chan, bool ns)
{
	u16 opcode;
	chan    &= 0x07;
	opcode   = chan;
	opcode <<= 8;
	opcode  |= CMD_DMAGO;
	if (ns)
		opcode |= (1 << 1);
	return opcode;
}

static inline u16 _opcode_KILL(void)
{
	u16 opcode;
	opcode = CMD_DMAKILL;
	return opcode;
}


static inline int _ldst_memtomem(
	unsigned                 dry_run,
	u8                       buf[],
	const struct _xfer_spec *pxs,
	int cyc
)
{
	int                  off  = 0;
	struct pl330_config *pcfg = pxs->r->preqcfg->pcfg;

	/* check lock-up free version */
	if (get_revision(pcfg->periph_id) >= PERIPH_REV_R1P0) {
		while (cyc--) {
			off += _emit_LD(dry_run, &buf[off]);
			off += _emit_ST(dry_run, &buf[off]);
		}
	} else {
		while (cyc--) {
			off += _emit_LD(dry_run, &buf[off]);
			off += _emit_RMB(dry_run, &buf[off]);
			off += _emit_ST(dry_run, &buf[off]);
			off += _emit_WMB(dry_run, &buf[off]);
		}
	}
	return off;
}

static inline int _ldst_devtomem(
	unsigned                 dry_run,
	u8                       buf[],
	const struct _xfer_spec *pxs,
	int                      cyc
)
{
	int                  off        = 0;
	u8                   peri       = pxs->r->peri;
	struct pl330_reqcfg *preqcfg    = pxs->r->preqcfg;
	bool                 wfp_once   = preqcfg->wfp_once;
	bool                 flush_once = preqcfg->flush_once;

	  while (cyc--) {
		if (!wfp_once)
			off += _emit_WFPB(dry_run, &buf[off], peri);

		off += _emit_LDPB(dry_run, &buf[off], peri);
		off += _emit_STB(dry_run, &buf[off]);

		if (!flush_once)
			off += _emit_FLUSHP(dry_run, &buf[off], peri);
	}
	return off;
}

static inline int _ldst_memtodev(
	unsigned                 dry_run,
	u8                       buf[],
	const struct _xfer_spec *pxs,
	int                      cyc
)
{
	int                  off        = 0;
	u8                   peri       = pxs->r->peri;
	struct pl330_reqcfg *preqcfg    = pxs->r->preqcfg;
	bool                 wfp_once   = preqcfg->wfp_once;
	bool                 flush_once = preqcfg->flush_once;

	while (cyc--) {
		if (!wfp_once)
			off += _emit_WFPB(dry_run, &buf[off], peri);

		off += _emit_LDB(dry_run, &buf[off]);
		off += _emit_STPB(dry_run, &buf[off], peri);

		if (!flush_once)
			off += _emit_FLUSHP(dry_run, &buf[off], peri);
	}
	return off;
}

static int _bursts(
	unsigned                 dry_run,
	u8                       buf[],
	const struct _xfer_spec *pxs,
	int                      cyc
)
{
	int off = 0;

	switch (pxs->r->rqtype)	{

	case MEMTODEV:
		off += _ldst_memtodev(dry_run, &buf[off], pxs, cyc);
		break;

	case DEVTOMEM:
		off += _ldst_devtomem(dry_run, &buf[off], pxs, cyc);
		break;

	case MEMTOMEM:
		off += _ldst_memtomem(dry_run, &buf[off], pxs, cyc);
		break;

	default:
		off += 0x40000000; /* Scare off the Client */
		break;
	}

	return off;
}

static int _calc_cycmax(const struct _xfer_spec *pxs, int loops)
{
	struct pl330_config *pcfg         = pxs->r->preqcfg->pcfg;
	unsigned int         icache_lines = pcfg->icache_lines;
	unsigned int         icache_len   = pcfg->icache_len;
	int                  cache_size;

	int cycmax;
	int szlptst;
	int szlpend;
	int szlp;
	int szbrst;
	int szlptotal;

	szlptst = _emit_LP(1, NULL, 0, 0);
	szbrst  = _bursts(1, NULL, pxs, 1);
	szlpend = _emit_LPEND(1, NULL, 0, 0);

	szlp  = szlptst + szlpend;
	szlp *= loops;

	/*
	 * TODO: This calc is sort of tricky. Needs review.
	 * On my particular HW, icache_lines = 7 and icache_len = 16. This
	 * will be dependent on the DMAC implementation. The rules for what
	 * fits into a cache line is somewhat vague. In the worse case,
	 * I can fit 7 lines of commands. In the best case, it will be
	 * 7x16 bytes of assembled commands. The size of each line will depend
	 * on the instruction and that depends on what constitutes a loop
	 * body. The doc implies that I can get more one command per line as
	 * as long as commands are not split between lines.
	 */
	cache_size   = (icache_lines*icache_len);

	/*
	 * Max bursts that we can unroll due to limit on the
	 * size of backward jump that can be encoded in DMALPEND
	 * which is 8-bits and hence 255.
	 * However, we should not exceed the DMAC cache size.
	 * And I should never return 0.
	 */
	szlptotal = min(cache_size, 255);
	cycmax    = max(1, (szlptotal - szlp) / szbrst);

	return cycmax;
}

struct loop_limits {
	unsigned lcnt0;
	unsigned lcnt1;
	int      cyc;
};

/* Assumes no instruction cache. */
static unsigned long calc_loop_limits_unroll(
	const struct _xfer_spec *pxs,
	unsigned long            bursts,
	struct loop_limits      *p)
{
	int      cycmax;
	unsigned lcnt;

	/* Try no loops at all. */
	cycmax = _calc_cycmax(pxs, 0);
	if (bursts <= cycmax) {
		p->lcnt0 = 0;
		p->lcnt1 = 0;
		p->cyc   = bursts;
		return bursts;
	}

	/* Try one loop with cycmax bursts per interation. */
	cycmax = _calc_cycmax(pxs, 1);
	lcnt = bursts / cycmax;
	if (lcnt <= 256) {
		p->lcnt0 = 0;
		p->lcnt1 = lcnt;
		p->cyc   = cycmax;
		return lcnt*cycmax;
	}

	/* Try two loops with with cycmax bursts per interation. */
	cycmax = _calc_cycmax(pxs, 2);
	lcnt   = bursts / (cycmax*256);
	if (lcnt <= 256) {
		p->lcnt0 = lcnt;
		p->lcnt1 = 256;
		p->cyc   = cycmax;
		return lcnt*256*cycmax;
	}

	/*
	 * Two maximum size loops with cycmax bursts per interation.
	 * Remaining bursts will have to be handled in another loop.
	 */
	p->lcnt0 = 256;
	p->lcnt1 = 256;
	p->cyc   = cycmax;
	return 256*256*cycmax;
}

/* Assumes an instruction cache large enough to hold a loop body. */
static unsigned long calc_loop_limits_no_unroll(
	const struct _xfer_spec *pxs,
	unsigned long            bursts,
	struct loop_limits      *p)
{
	/* Max iterations possible in DMALP is 256 */
	if (bursts > (256*256)) {
		/* Two nested loops of more than one burst per iteration */
		/* This construct will result in cache misses. */
		int cycmax;
		int cycwant;

		cycmax   = _calc_cycmax(pxs, 2);
		cycwant  = bursts/(256*256);
		p->lcnt0 = 256;
		p->lcnt1 = 256;
		p->cyc   = min(cycmax, cycwant);
		bursts   = p->cyc * (256*256);
	} else if (bursts > 256) {
		/* Two nested loops of one burst per iteration */
		p->lcnt0 = bursts / 256;
		p->lcnt1 = 256;
		p->cyc   = 1;
		bursts   = p->lcnt0 * 256;
	} else {
		/* One loop of one burst per iteration */
		p->lcnt0 = 0;
		p->lcnt1 = bursts;
		p->cyc   = 1;
	}

	return bursts;
}

/* Returns bytes consumed and updates bursts */
/* This function will attempt to use 256 bytes. */
static inline int _loop(
	unsigned                 dry_run,
	u8                       buf[],
	unsigned long           *pbursts,
	const struct _xfer_spec *pxs)
{
	int                  off;
	struct loop_limits   loop;
	unsigned             ljmp0   = 0;
	unsigned             ljmp1   = 0;
	unsigned long        bursts  = *pbursts;
	struct pl330_reqcfg *preqcfg = pxs->r->preqcfg;

	if (preqcfg->unroll_loop)
		bursts = calc_loop_limits_unroll(pxs, bursts, &loop);
	else
		bursts = calc_loop_limits_no_unroll(pxs, bursts, &loop);

	off = 0;
	if (loop.lcnt0) {
		off += _emit_LP(dry_run, &buf[off], loop.lcnt0, 0);
		ljmp0 = off;
	}

	if (loop.lcnt1)  {
		off += _emit_LP(dry_run, &buf[off], loop.lcnt1, 1);
		ljmp1 = off;
	}

	off += _bursts(dry_run, &buf[off], pxs, loop.cyc);

	if (loop.lcnt1)
		off += _emit_LPEND(dry_run, &buf[off], off - ljmp1, 1);

	if (loop.lcnt0)
		off += _emit_LPEND(dry_run, &buf[off], off - ljmp0, 0);

	*pbursts = bursts;

	return off;
}

static inline int _setup_loops(
	unsigned                 dry_run,
	u8                       buf[],
	const struct _xfer_spec *pxs
)
{
	struct pl330_xfer *x = pxs->x;
	u32                ccr = pxs->ccr;
	unsigned long      c;
	unsigned long      bursts = BYTE_TO_BURST(x->bytes, ccr);
	int                off = 0;

	while (bursts) {
		c = bursts;
		off += _loop(dry_run, &buf[off], &c, pxs);
		bursts -= c;
	}

	return off;
}

static inline int _setup_xfer(
	unsigned                 dry_run,
	u8                       buf[],
	const struct _xfer_spec *pxs)
{
	struct pl330_xfer   *x        = pxs->x;
	int                  off      = 0;
	struct pl330_reqcfg *preqcfg  = pxs->r->preqcfg;

	/* DMAMOV SAR, x->src_addr */
	off += _emit_MOV(dry_run, &buf[off], SAR, x->src_addr);

	/* DMAMOV DAR, x->dst_addr */
	off += _emit_MOV(dry_run, &buf[off], DAR, x->dst_addr);

	/*
	 * The original code only supported Single Transfers and issued a
	 * DMAFLUSHP after every DMALD and DMAST. Code added for Burst
	 * transfers assume that DMAFLUSHP is called just once at the
	 * beginning.
	 */
	if (preqcfg->flush_once)
		off += _emit_FLUSHP(dry_run, &buf[off], pxs->r->peri);

	/*
	 * The original code only supported Single Transfers and issued a
	 * DMAWFP before every DMALD and DMAST. Code added for Burst
	 * transfers assume that DMAWFP is called just once at the
	 * beginning. Caller must know that the device on channel always has
	 * enough data/room available after the first burst request. The
	 * DMAC will read/write without waiting for subsequent burst
	 * requests.
	 */
	if (preqcfg->wfp_once)
		off += _emit_WFPB(dry_run, &buf[off], pxs->r->peri);

	/* Setup Loop(s) */
		off += _setup_loops(dry_run, &buf[off], pxs);

	return off;
}

/*
 * A req is a sequence of one or more xfer units.
 * Returns the number of bytes taken to setup the MC for the req.
 */
static int _setup_req(
	unsigned             dry_run,
	struct pl330_thread *thrd,
	unsigned             index,
	struct _xfer_spec   *pxs
)
{
	struct _pl330_req *req = &thrd->req[index];
	struct pl330_xfer *x;
	u8                *buf = req->mc_cpu;
	int                off = 0;

	/* DMAMOV CCR, ccr */
	off += _emit_MOV(dry_run, &buf[off], CCR, pxs->ccr);

	x = pxs->r->x;
	do {
		/* Error if xfer length is not aligned at burst size */
		if (x->bytes % (BRST_SIZE(pxs->ccr) * BRST_LEN(pxs->ccr)))
			return -EINVAL;

		pxs->x = x;
		off += _setup_xfer(dry_run, &buf[off], pxs);

		x = x->next;
	} while (x);

	/* DMASEV peripheral/event */
	off += _emit_SEV(dry_run, &buf[off], thrd->ev);

	/* DMAEND */
	off += _emit_END(dry_run, &buf[off]);

	return off;
}
/*------------------------assembler code end---------------------------------*/

/*-------------------------thread code start --------------------------------*/
static inline bool _queue_empty(struct pl330_thread *thrd)
{
	return (IS_FREE(&thrd->req[0]) && IS_FREE(&thrd->req[1]))
		? true : false;
}

static inline bool _queue_full(struct pl330_thread *thrd)
{
	return (IS_FREE(&thrd->req[0]) || IS_FREE(&thrd->req[1]))
		? false : true;
}

static inline bool is_manager(struct pl330_thread *thrd)
{
	struct pl330_dmac *pl330 = thrd->dmac;

	/* MANAGER is indexed at the end */
	if (thrd->id == pl330->pinfo->cfg.num_chan)
		return true;
	else
		return false;
}

/* If manager of the thread is in Non-Secure mode */
static inline bool _manager_ns(struct pl330_thread *thrd)
{
	struct pl330_dmac *pl330 = thrd->dmac;

	return (pl330->pinfo->cfg.mode & DMAC_MODE_NS) ? true : false;
}

#define msecs_to_loops(t) (loops_per_jiffy / 1000 * HZ * t)

/* Returns Time-Out */
static bool _until_dmac_idle(struct pl330_thread *thrd)
{
	void __iomem *regs  = thrd->dmac->pinfo->base;
	unsigned long loops = msecs_to_loops(5);

	do {
		/* Until Manager is Idle */
		if (!(readl(regs + DBGSTATUS) & DBG_BUSY))
			break;

		cpu_relax();
	} while (--loops);

	if (!loops)
		return true;

	return false;
}

static inline void _execute_DBGINSN(
	struct pl330_thread *thrd,
	u16                  opcode,
	u32                  imm,
	bool                 as_manager
)
{
	void __iomem *regs = thrd->dmac->pinfo->base;
	u32           val;

	val   = opcode;
	val <<= 16;
	if (!as_manager) {
		val  |= thrd->id; /* Channel Number */
		val <<= 8;
		val  |= (1 << 0);
	}
	writel(val, regs + DBGINST0);
	writel(imm, regs + DBGINST1);

	/* If timed out due to halted state-machine */
	/* TODO: Review what the calling routine should do in this case. */
	if (_until_dmac_idle(thrd)) {
		dev_err(thrd->dmac->pinfo->dev, "DMAC halted!\n");
		return;
	}

	/* Get going */
	writel(0, regs + DBGCMD);
}

/*
 * Mark a _pl330_req as free.
 * We do it by writing DMAEND as the first instruction
 * because no valid request is going to have DMAEND as
 * its first instruction to execute.
 */
static void mark_free(struct pl330_thread *thrd, int idx)
{
	struct _pl330_req *req = &thrd->req[idx];

	_emit_END(0, req->mc_cpu);
	req->mc_len = 0;

	thrd->req_running = -1;
}

/* Manager */
static const u32 state_from_ds[0x10] = {
	PL330_STATE_STOPPED,   /* 0x0 DS_ST_STOP */
	PL330_STATE_EXECUTING, /* 0x1 DS_ST_EXEC */
	PL330_STATE_CACHEMISS, /* 0x2 DS_ST_CMISS */
	PL330_STATE_UPDTPC,    /* 0x3 DS_ST_UPDTPC */
	PL330_STATE_WFE,       /* 0x4 DS_ST_WFE */
	PL330_STATE_INVALID,   /* 0x5 DS_ST_ATBRR */
	PL330_STATE_INVALID,   /* 0x6 DS_ST_QBUSY */
	PL330_STATE_INVALID,   /* 0x7 DS_ST_WFP */
	PL330_STATE_INVALID,   /* 0x8 DS_ST_KILL */
	PL330_STATE_INVALID,   /* 0x9 DS_ST_CMPLT */
	PL330_STATE_INVALID,   /* 0xA */
	PL330_STATE_INVALID,   /* 0xB */
	PL330_STATE_INVALID,   /* 0xC */
	PL330_STATE_INVALID,   /* 0xD */
	PL330_STATE_INVALID,   /* 0xE DS_ST_FLTCMP */
	PL330_STATE_FAULTING   /* 0xF DS_ST_FAULT */
};

/* Channel or not Manager */
static const u32 state_from_cs[0x10] = {
	PL330_STATE_STOPPED,          /* 0x0 DS_ST_STOP*/
	PL330_STATE_EXECUTING,        /* 0x1 DS_ST_EXEC*/
	PL330_STATE_CACHEMISS,        /* 0x2 DS_ST_CMISS*/
	PL330_STATE_UPDTPC,           /* 0x3 DS_ST_UPDTPC*/
	PL330_STATE_WFE,              /* 0x4 DS_ST_WFE*/
	PL330_STATE_ATBARRIER,        /* 0x5 DS_ST_ATBRR*/
	PL330_STATE_QUEUEBUSY,        /* 0x6 DS_ST_QBUSY*/
	PL330_STATE_WFP,              /* 0x7 DS_ST_WFP*/
	PL330_STATE_KILLING,          /* 0x8 DS_ST_KILL*/
	PL330_STATE_COMPLETING,       /* 0x9 DS_ST_CMPLT*/
	PL330_STATE_INVALID,          /* 0xA */
	PL330_STATE_INVALID,          /* 0xB */
	PL330_STATE_INVALID,          /* 0xC */
	PL330_STATE_INVALID,          /* 0xD */
	PL330_STATE_FAULT_COMPLETING, /* 0xE DS_ST_FLTCMP*/
	PL330_STATE_FAULTING          /* 0xF DS_ST_FAULT*/
};

static inline u32 _state(struct pl330_thread *thrd)
{
	void __iomem *regs = thrd->dmac->pinfo->base;
	u32           val;
	u32           st;

	if (is_manager(thrd)) {
		val = readl(regs + DS) & 0xF;
		st  = state_from_ds[val];
	} else {
		val = readl(regs + CS(thrd->id)) & 0xF;
		st  = state_from_cs[val];
	}
	return st;
}

static void _stop(struct pl330_thread *thrd)
{
	void __iomem *regs    = thrd->dmac->pinfo->base;

	if (_state(thrd) == PL330_STATE_FAULT_COMPLETING)
		UNTIL(thrd, PL330_STATE_FAULTING | PL330_STATE_KILLING);

	/* Return if nothing needs to be done */
	if (_state(thrd) == PL330_STATE_COMPLETING ||
	    _state(thrd) == PL330_STATE_KILLING ||
	    _state(thrd) == PL330_STATE_STOPPED)
		return;

	/* Stop generating interrupts for SEV */
	writel(readl(regs + INTEN) & ~(1 << thrd->ev), regs + INTEN);

	_execute_DBGINSN(thrd, _opcode_KILL(), 0, is_manager(thrd));
}

/* Start doing req 'idx' of thread 'thrd' */
static bool _trigger(struct pl330_thread *thrd)
{
	void __iomem      *regs = thrd->dmac->pinfo->base;
	struct _pl330_req *req;
	struct pl330_req  *r;
	bool               ns;
	int                idx;

	/* Return if already ACTIVE */
	if (_state(thrd) != PL330_STATE_STOPPED)
		return true;

	idx = 1 - thrd->lstenq;
	if (!IS_FREE(&thrd->req[idx]))
		req = &thrd->req[idx];
	else {
		idx = thrd->lstenq;
		if (!IS_FREE(&thrd->req[idx]))
			req = &thrd->req[idx];
		else
			req = NULL;
	}

	/* Return if no request */
	if (!req || !req->r)
		return true;

	r = req->r;

	if (r->preqcfg)
		ns = r->preqcfg->nonsecure;
	else if (readl(regs + CS(thrd->id)) & CS_CNS)
		ns = true;
	else
		ns = false;

	/* See 'Abort Sources' point-4 at Page 2-25 */
	if (_manager_ns(thrd) && !ns)
		dev_info(thrd->dmac->pinfo->dev, "%s:%d Recipe for ABORT!\n",
			__func__, __LINE__);

	/* Set to generate interrupts for SEV */
	writel(readl(regs + INTEN) | (1 << thrd->ev), regs + INTEN);

	/* Only manager can execute GO */
	_execute_DBGINSN(thrd, _opcode_GO(thrd->id, ns), req->mc_bus, true);

	thrd->req_running = idx;

	return true;
}

static bool _start(struct pl330_thread *thrd)
{
	switch (_state(thrd)) {
	case PL330_STATE_FAULT_COMPLETING:
		UNTIL(thrd, PL330_STATE_FAULTING | PL330_STATE_KILLING);

		if (_state(thrd) == PL330_STATE_KILLING)
			UNTIL(thrd, PL330_STATE_STOPPED);

	case PL330_STATE_FAULTING:
		_stop(thrd);

	case PL330_STATE_KILLING:
	case PL330_STATE_COMPLETING:
		UNTIL(thrd, PL330_STATE_STOPPED);

	case PL330_STATE_STOPPED:
		return _trigger(thrd);

	case PL330_STATE_WFP:
	case PL330_STATE_QUEUEBUSY:
	case PL330_STATE_ATBARRIER:
	case PL330_STATE_UPDTPC:
	case PL330_STATE_CACHEMISS:
	case PL330_STATE_EXECUTING:
		return true;

	case PL330_STATE_WFE: /* For RESUME, nothing yet */
	default:
		return false;
	}
}

static inline u32 _prepare_ccr(const struct pl330_reqcfg *rqc)
{
	u32 ccr = 0;

	if (rqc->src_inc)
		ccr |= CC_SRCINC;

	if (rqc->dst_inc)
		ccr |= CC_DSTINC;

	/* We set same protection levels for Src and DST for now */
	if (rqc->privileged)
		ccr |= CC_SRCPRI | CC_DSTPRI;
	if (rqc->nonsecure)
		ccr |= CC_SRCNS | CC_DSTNS;
	if (rqc->insnaccess)
		ccr |= CC_SRCIA | CC_DSTIA;

	ccr |= (((rqc->brst_len - 1) & 0xf) << CC_SRCBRSTLEN_SHFT);
	ccr |= (((rqc->brst_len - 1) & 0xf) << CC_DSTBRSTLEN_SHFT);

	ccr |= (rqc->brst_size << CC_SRCBRSTSIZE_SHFT);
	ccr |= (rqc->brst_size << CC_DSTBRSTSIZE_SHFT);

	ccr |= (rqc->scctl << CC_SRCCCTRL_SHFT);
	ccr |= (rqc->dcctl << CC_DSTCCTRL_SHFT);

	ccr |= (rqc->swap << CC_SWAP_SHFT);

	//**//
	//printk("ccr = %#x\n",ccr);
	//**//

	return ccr;
}

static inline bool _is_valid(u32 ccr)
{
	enum pl330_dstcachectrl dcctl;
	enum pl330_srccachectrl scctl;

	dcctl = (ccr >> CC_DSTCCTRL_SHFT) & CC_DRCCCTRL_MASK;
	scctl = (ccr >> CC_SRCCCTRL_SHFT) & CC_SRCCCTRL_MASK;

	if (dcctl == DINVALID1 || dcctl == DINVALID2
			|| scctl == SINVALID1 || scctl == SINVALID2)
		return false;
	else
		return true;
}

/*
 * Submit a list of xfers after which the client wants notification.
 * Client is not notified after each xfer unit, just once after all
 * xfer units are done or some error occurs.
 */
static int pl330_submit_req(void *ch_id, struct pl330_req *r)
{
	struct pl330_thread *thrd = ch_id;
	struct pl330_dmac   *pl330;
	struct pl330_info   *pi;
	struct _xfer_spec    xs;
	unsigned long        flags;
	void __iomem        *regs;
	unsigned             idx;
	u32                  ccr;
	int                  ret = 0;

	/* No Req or Unacquired Channel or DMAC */
	if (!r || !thrd || thrd->free)
		return -EINVAL;

	pl330 = thrd->dmac;
	pi = pl330->pinfo;
	regs = pi->base;

	if (pl330->state == DYING
		|| pl330->dmac_tbd.reset_chan & (1 << thrd->id)) {
		dev_info(thrd->dmac->pinfo->dev, "%s:%d\n",
			__func__, __LINE__);
		return -EAGAIN;
	}

	/* If request for non-existing peripheral */
	if (r->rqtype != MEMTOMEM && r->peri >= pi->cfg.num_peri) {
		dev_info(thrd->dmac->pinfo->dev,
				"%s:%d Invalid peripheral(%u)!\n",
				__func__, __LINE__, r->peri);
		return -EINVAL;
	}

	spin_lock_irqsave(&pl330->lock, flags);

	if (_queue_full(thrd)) {
		ret = -EAGAIN;
		goto xfer_exit;
	}


	/* Use last settings, if not provided */
	if (r->preqcfg) {
		/* Prefer Secure Channel */
		if (!_manager_ns(thrd))
			r->preqcfg->nonsecure = 0;
		else
			r->preqcfg->nonsecure = 1;

		ccr = _prepare_ccr(r->preqcfg);
	} else {
		ccr = readl(regs + CC(thrd->id));
	}

	/* If this req doesn't have valid xfer settings */
	if (!_is_valid(ccr)) {
		ret = -EINVAL;
		dev_info(thrd->dmac->pinfo->dev, "%s:%d Invalid CCR(%x)!\n",
			__func__, __LINE__, ccr);
		goto xfer_exit;
	}

	idx = IS_FREE(&thrd->req[0]) ? 0 : 1;

	xs.ccr = ccr;
	xs.r = r;

	/* First dry run to check if req is acceptable */
	ret = _setup_req(1, thrd, idx, &xs);
	if (ret < 0)
		goto xfer_exit;

	if (ret > pi->mcbufsz / 2) {
		dev_info(thrd->dmac->pinfo->dev,
			"%s:%d Trying increasing mcbufsz. Need %d\n",
				__func__, __LINE__, ret);
		ret = -ENOMEM;
		goto xfer_exit;
	}

	/* Hook the request */
	thrd->lstenq = idx;
	thrd->req[idx].mc_len = _setup_req(0, thrd, idx, &xs);
	thrd->req[idx].r = r;
	ret = 0;

xfer_exit:
	spin_unlock_irqrestore(&pl330->lock, flags);

	return ret;
}

static void pl330_dotask(unsigned long data)
{
	struct pl330_dmac *pl330 = (struct pl330_dmac *) data;
	struct pl330_info *pi    = pl330->pinfo;
	unsigned long      flags;
	int                i;

	spin_lock_irqsave(&pl330->lock, flags);

	/* The DMAC itself gone nuts */
	if (pl330->dmac_tbd.reset_dmac) {
		pl330->state = DYING;
		/* Reset the manager too */
		pl330->dmac_tbd.reset_mngr = true;
		/* Clear the reset flag */
		pl330->dmac_tbd.reset_dmac = false;
	}

	if (pl330->dmac_tbd.reset_mngr) {
		_stop(pl330->manager);
		/* Reset all channels */
		pl330->dmac_tbd.reset_chan = (1 << pi->cfg.num_chan) - 1;
		/* Clear the reset flag */
		pl330->dmac_tbd.reset_mngr = false;
	}

	for (i = 0; i < pi->cfg.num_chan; i++) {

		if (pl330->dmac_tbd.reset_chan & (1 << i)) {
			struct pl330_thread *thrd = &pl330->channels[i];
			void __iomem *regs = pi->base;
			enum pl330_op_err err;

			_stop(thrd);

			if (readl(regs + FSC) & (1 << thrd->id))
				err = PL330_ERR_FAIL;
			else
				err = PL330_ERR_ABORT;

			spin_unlock_irqrestore(&pl330->lock, flags);

			_callback(thrd->req[1 - thrd->lstenq].r, err);
			_callback(thrd->req[thrd->lstenq].r, err);

			spin_lock_irqsave(&pl330->lock, flags);

			thrd->req[0].r = NULL;
			thrd->req[1].r = NULL;
			mark_free(thrd, 0);
			mark_free(thrd, 1);

			/* Clear the reset flag */
			pl330->dmac_tbd.reset_chan &= ~(1 << i);
		}
	}

	spin_unlock_irqrestore(&pl330->lock, flags);

	return;
}

/* Returns 1 if state was updated, 0 otherwise */
static int pl330_update(const struct pl330_info *pi)
{
	struct pl330_req  *rqdone, *tmp;
	struct pl330_dmac *pl330;
	unsigned long      flags;
	void __iomem      *regs;
	u32                val;
	int                id;
	int                ev;
	int                ret = 0;

	if (!pi || !pi->pl330_data)
		return 0;

	regs = pi->base;
	pl330 = pi->pl330_data;

	spin_lock_irqsave(&pl330->lock, flags);

	val = readl(regs + FSM) & 0x1;
	if (val)
		pl330->dmac_tbd.reset_mngr = true;
	else
		pl330->dmac_tbd.reset_mngr = false;

	val = readl(regs + FSC) & ((1 << pi->cfg.num_chan) - 1);
	pl330->dmac_tbd.reset_chan |= val;
	if (val) {
		int i = 0;
		while (i < pi->cfg.num_chan) {
			if (val & (1 << i)) {
				dev_info(pi->dev,
					"Reset Channel-%d\t CS-%x FTC-%x\n",
						i, readl(regs + CS(i)),
						readl(regs + FTC(i)));
				_stop(&pl330->channels[i]);
			}
			i++;
		}
	}

	/* Check which event happened i.e, thread notified */
	val = readl(regs + ES);
	if (pi->cfg.num_events < 32
			&& val & ~((1 << pi->cfg.num_events) - 1)) {
		pl330->dmac_tbd.reset_dmac = true;
		dev_err(pi->dev, "%s:%d Unexpected!\n", __func__, __LINE__);
		ret = 1;
		goto updt_exit;
	}

	for (ev = 0; ev < pi->cfg.num_events; ev++) {
		if (val & (1 << ev)) { /* Event occurred */
			struct pl330_thread *thrd;
			u32 inten = readl(regs + INTEN);
			int active;

			/* Clear the event */
			if (inten & (1 << ev))
				writel(1 << ev, regs + INTCLR);

			ret = 1;

			id = pl330->events[ev];

			thrd = &pl330->channels[id];

			active = thrd->req_running;
			if (active == -1) /* Aborted */
				continue;

			/* Detach the req */
			rqdone = thrd->req[active].r;
			thrd->req[active].r = NULL;

			mark_free(thrd, active);

			/* Get going again ASAP */
			_start(thrd);

			/* For now, just make a list of callbacks to be done */
			list_add_tail(&rqdone->rqd, &pl330->req_done);
		}
	}

	/* Now that we are in no hurry, do the callbacks */
	list_for_each_entry_safe(rqdone, tmp, &pl330->req_done, rqd) {
		list_del(&rqdone->rqd);

		spin_unlock_irqrestore(&pl330->lock, flags);
		_callback(rqdone, PL330_ERR_NONE);
		spin_lock_irqsave(&pl330->lock, flags);
	}

updt_exit:
	spin_unlock_irqrestore(&pl330->lock, flags);

	if (pl330->dmac_tbd.reset_dmac
			|| pl330->dmac_tbd.reset_mngr
			|| pl330->dmac_tbd.reset_chan) {
		ret = 1;
		tasklet_schedule(&pl330->tasks);
	}

	return ret;
}

static int pl330_chan_ctrl(void *ch_id, enum pl330_chan_op op)
{
	struct pl330_thread *thrd = ch_id;
	struct pl330_dmac   *pl330;
	unsigned long        flags;
	int                  ret = 0;
	int                  active;

	if (!thrd || thrd->free || thrd->dmac->state == DYING)
		return -EINVAL;

	pl330 = thrd->dmac;
	active = thrd->req_running;

	spin_lock_irqsave(&pl330->lock, flags);

	switch (op) {
	case PL330_OP_FLUSH:
		/* Make sure the channel is stopped */
		_stop(thrd);

		thrd->req[0].r = NULL;
		thrd->req[1].r = NULL;
		mark_free(thrd, 0);
		mark_free(thrd, 1);
		break;

	case PL330_OP_ABORT:
		/* Make sure the channel is stopped */
		_stop(thrd);

		/* ABORT is only for the active req */
		if (active == -1)
			break;

		thrd->req[active].r = NULL;
		mark_free(thrd, active);

		/* Start the next */
	case PL330_OP_START:
		if ((active == -1) && !_start(thrd))
			ret = -EIO;
		break;

	default:
		ret = -EINVAL;
	}

	spin_unlock_irqrestore(&pl330->lock, flags);
	return ret;
}

/* Reserve an event */
static inline int _alloc_event(struct pl330_thread *thrd)
{
	struct pl330_dmac *pl330 = thrd->dmac;
	struct pl330_info *pi    = pl330->pinfo;
	int ev;

	for (ev = 0; ev < pi->cfg.num_events; ev++)
		if (pl330->events[ev] == -1) {
			pl330->events[ev] = thrd->id;
			return ev;
		}

	return -1;
}

static bool _chan_ns(const struct pl330_info *pi, int i)
{
	return pi->cfg.irq_ns & (1 << i);
}

/* Upon success, returns IdentityToken for the
 * allocated channel, NULL otherwise.
 */
static void *pl330_request_channel(const struct pl330_info *pi)
{
	struct pl330_thread *thrd = NULL;
	struct pl330_dmac   *pl330;
	unsigned long        flags;
	int                  chans;
	int                  i;

	if (!pi || !pi->pl330_data)
		return NULL;

	pl330 = pi->pl330_data;

	if (pl330->state == DYING)
		return NULL;

	chans = pi->cfg.num_chan;

	spin_lock_irqsave(&pl330->lock, flags);

	for (i = 0; i < chans; i++) {
		thrd = &pl330->channels[i];
		if ((thrd->free) && (!_manager_ns(thrd) ||
					_chan_ns(pi, i))) {
			thrd->ev = _alloc_event(thrd);
			if (thrd->ev >= 0) {
				thrd->free = false;
				thrd->lstenq = 1;
				thrd->req[0].r = NULL;
				mark_free(thrd, 0);
				thrd->req[1].r = NULL;
				mark_free(thrd, 1);
				break;
			}
		}
		thrd = NULL;
	}

	spin_unlock_irqrestore(&pl330->lock, flags);

	return thrd;
}

/* Release an event */
static inline void _free_event(struct pl330_thread *thrd, int ev)
{
	struct pl330_dmac *pl330 = thrd->dmac;
	struct pl330_info *pi    = pl330->pinfo;

	/* If the event is valid and was held by the thread */
	if (ev >= 0 && ev < pi->cfg.num_events
			&& pl330->events[ev] == thrd->id)
		pl330->events[ev] = -1;
}

static void pl330_release_channel(void *ch_id)
{
	struct pl330_thread *thrd = ch_id;
	struct pl330_dmac   *pl330;
	unsigned long        flags;

	if (!thrd || thrd->free)
		return;

	_stop(thrd);

	_callback(thrd->req[1 - thrd->lstenq].r, PL330_ERR_ABORT);
	_callback(thrd->req[thrd->lstenq].r, PL330_ERR_ABORT);

	pl330 = thrd->dmac;

	spin_lock_irqsave(&pl330->lock, flags);
	_free_event(thrd, thrd->ev);
	thrd->free = true;
	spin_unlock_irqrestore(&pl330->lock, flags);
}

/* Initialize the structure for PL330 configuration, that can be used
 * by the client driver the make best use of the DMAC
 */
static void read_dmac_config(
	void __iomem        *regs,
	struct pl330_config *pcfg)
{
	u32 val;
	u32 fld;

	/* CRD info */
	/* A bit inefficient. Read register more than once. */
	val = readl(regs + CRD) >> CRD_DATA_WIDTH_SHIFT;
	val &= CRD_DATA_WIDTH_MASK;
	pcfg->data_bus_width = 8 * (1 << val);

	val = readl(regs + CRD) >> CRD_DATA_BUFF_SHIFT;
	val &= CRD_DATA_BUFF_MASK;
	pcfg->data_buf_dep = val + 1;

	/* CR0 info */
	/* A bit inefficient. Read register more than once. */
	val = readl(regs + CR0) >> CR0_NUM_CHANS_SHIFT;
	val &= CR0_NUM_CHANS_MASK;
	val += 1;
	pcfg->num_chan = val;

	val = readl(regs + CR0);
	if (val & CR0_PERIPH_REQ_SET) {
		val = (val >> CR0_NUM_PERIPH_SHIFT) & CR0_NUM_PERIPH_MASK;
		val += 1;
		pcfg->num_peri = val;
		pcfg->peri_ns = readl(regs + CR4);
	} else {
		pcfg->num_peri = 0;
	}

	val = readl(regs + CR0);
	if (val & CR0_BOOT_MAN_NS)
		pcfg->mode |= DMAC_MODE_NS;
	else
		pcfg->mode &= ~DMAC_MODE_NS;

	val = readl(regs + CR0) >> CR0_NUM_EVENTS_SHIFT;
	val &= CR0_NUM_EVENTS_MASK;
	val += 1;
	pcfg->num_events = val;

	/* CR3 info */
	pcfg->irq_ns = readl(regs + CR3);

	/* CR1 info */
	val = readl(regs + CR1);

	fld = (val >> CR1_ICACHE_LEN_SHIFT) & CR1_ICACHE_LEN_MASK;
	pcfg->icache_len = (1U << fld);

	fld = (val >> CR1_NUM_ICACHELINES_SHIFT) & CR1_NUM_ICACHELINES_MASK;
	pcfg->icache_lines = fld;
}

static inline void _reset_thread(struct pl330_thread *thrd)
{
	struct pl330_dmac *pl330 = thrd->dmac;
	struct pl330_info *pi    = pl330->pinfo;

	thrd->req[0].mc_cpu = pl330->mcode_cpu + (thrd->id * pi->mcbufsz);
	thrd->req[0].mc_bus = pl330->mcode_bus + (thrd->id * pi->mcbufsz);
	thrd->req[0].r = NULL;
	mark_free(thrd, 0);

	thrd->req[1].mc_cpu = thrd->req[0].mc_cpu + pi->mcbufsz / 2;
	thrd->req[1].mc_bus = thrd->req[0].mc_bus + pi->mcbufsz / 2;
	thrd->req[1].r = NULL;
	mark_free(thrd, 1);
}

static int dmac_alloc_threads(struct pl330_dmac *pl330)
{
	struct pl330_info *pi = pl330->pinfo;
	int chans = pi->cfg.num_chan;
	struct pl330_thread *thrd;
	int i;

	/* Allocate 1 Manager and 'chans' Channel threads */
	pl330->channels = kzalloc((1 + chans) * sizeof(*thrd),
					GFP_KERNEL);
	if (!pl330->channels)
		return -ENOMEM;

	/* Init Channel threads */
	for (i = 0; i < chans; i++) {
		thrd = &pl330->channels[i];
		thrd->id = i;
		thrd->dmac = pl330;
		_reset_thread(thrd);
		thrd->free = true;
	}

	/* MANAGER is indexed at the end */
	thrd = &pl330->channels[chans];
	thrd->id = chans;
	thrd->dmac = pl330;
	thrd->free = false;
	pl330->manager = thrd;

	return 0;
}

static int dmac_alloc_resources(struct pl330_dmac *pl330)
{
	struct pl330_info *pi    = pl330->pinfo;
	int                chans = pi->cfg.num_chan;
	int                ret;

	/*
	 * Alloc MicroCode buffer for 'chans' Channel threads.
	 * A channel's buffer offset is (Channel_Id * MCODE_BUFF_PERCHAN)
	 */
	pl330->mcode_cpu = dma_alloc_coherent(pi->dev,
				chans * pi->mcbufsz,
				&pl330->mcode_bus, GFP_KERNEL);
	if (!pl330->mcode_cpu) {
		dev_err(pi->dev, "%s:%d Can't allocate memory!\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	ret = dmac_alloc_threads(pl330);
	if (ret) {
		dev_err(pi->dev, "%s:%d Can't to create channels for DMAC!\n",
			__func__, __LINE__);
		dma_free_coherent(pi->dev,
				chans * pi->mcbufsz,
				pl330->mcode_cpu, pl330->mcode_bus);
		return ret;
	}

	return 0;
}

static int pl330_add(struct pl330_info *pi)
{
	struct pl330_dmac *pl330;
	void __iomem      *regs;
	int                i;
	int                ret;

	if (!pi || !pi->dev)
		return -EINVAL;

	/* If already added */
	if (pi->pl330_data)
		return -EINVAL;

	/*
	 * If the SoC can perform reset on the DMAC, then do it
	 * before reading its configuration.
	 */
	if (pi->dmac_reset)
		pi->dmac_reset(pi);

	regs = pi->base;

	/* Check if we can handle this DMAC */
	if ((pi->cfg.periph_id & 0xfffff) != PERIPH_ID_VAL) {
		dev_err(pi->dev, "PERIPH_ID 0x%x !\n", pi->cfg.periph_id);
		return -EINVAL;
	}

	/* Read the configuration of the DMAC */
	read_dmac_config(pi->base, &pi->cfg);

	if (pi->cfg.num_events == 0) {
		dev_err(pi->dev, "%s:%d Can't work without events!\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	pl330 = kzalloc(sizeof(*pl330), GFP_KERNEL);
	if (!pl330) {
		dev_err(pi->dev, "%s:%d Can't allocate memory!\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	/* Assign the info structure and private data */
	pl330->pinfo = pi;
	pi->pl330_data = pl330;

	spin_lock_init(&pl330->lock);

	INIT_LIST_HEAD(&pl330->req_done);

	/* Use default MC buffer size if not provided */
	if (!pi->mcbufsz)
		pi->mcbufsz = MCODE_BUFF_PER_REQ * 2;

	/* Mark all events as free */
	for (i = 0; i < pi->cfg.num_events; i++)
		pl330->events[i] = -1;

	/* Allocate resources needed by the DMAC */
	ret = dmac_alloc_resources(pl330);
	if (ret) {
		dev_err(pi->dev, "Unable to create channels for DMAC\n");
		kfree(pl330);
		return ret;
	}

	tasklet_init(&pl330->tasks, pl330_dotask, (unsigned long) pl330);

	pl330->state = INIT;

	return 0;
}

static int dmac_free_threads(struct pl330_dmac *pl330)
{
	struct pl330_info   *pi    = pl330->pinfo;
	int                  chans = pi->cfg.num_chan;
	struct pl330_thread *thrd;
	int                  i;

	/* Release Channel threads */
	for (i = 0; i < chans; i++) {
		thrd = &pl330->channels[i];
		pl330_release_channel((void *)thrd);
	}

	/* Free memory */
	kfree(pl330->channels);

	return 0;
}

static void dmac_free_resources(struct pl330_dmac *pl330)
{
	struct pl330_info *pi    = pl330->pinfo;
	int                chans = pi->cfg.num_chan;

	dmac_free_threads(pl330);

	dma_free_coherent(pi->dev, chans * pi->mcbufsz,
				pl330->mcode_cpu, pl330->mcode_bus);
}

static void pl330_del(struct pl330_info *pi)
{
	struct pl330_dmac *pl330;

	if (!pi || !pi->pl330_data)
		return;

	pl330 = pi->pl330_data;

	pl330->state = UNINIT;

	tasklet_kill(&pl330->tasks);

	/* Free DMAC resources */
	dmac_free_resources(pl330);

	kfree(pl330);
	pi->pl330_data = NULL;
}

/* forward declaration */
static struct amba_driver pl330_driver;

static inline struct dma_pl330_chan *to_pchan(struct dma_chan *ch)
{
	if (!ch)
		return NULL;

	return container_of(ch, struct dma_pl330_chan, chan);
}

static inline struct dma_pl330_desc *to_desc(
	struct dma_async_tx_descriptor *tx)
{
	return container_of(tx, struct dma_pl330_desc, txd);
}

static inline void fill_queue(struct dma_pl330_chan *pch)
{
	struct dma_pl330_desc *desc;
	int                    ret;

	list_for_each_entry(desc, &pch->work_list, node) {

		/* If already submitted */
		if (desc->status == BUSY)
			continue;

		ret = pl330_submit_req(pch->pl330_chid,
						&desc->req);
		if (!ret) {
			desc->status = BUSY;
		} else if (ret == -EAGAIN) {
			/* QFull or DMAC Dying */
			break;
		} else {
			/* Unacceptable request */
			desc->status = DONE;
			dev_err(pch->dmac->pif.dev, "%s:%d Bad Desc(%d)\n",
					__func__, __LINE__, desc->txd.cookie);
			tasklet_schedule(&pch->task);
		}
	}
}

static void pl330_tasklet(unsigned long data)
{
	struct dma_pl330_chan *pch = (struct dma_pl330_chan *)data;
	struct dma_pl330_desc *desc;
	struct dma_pl330_desc *_dt;
	unsigned long          flags;

	spin_lock_irqsave(&pch->lock, flags);

	/* Pick up ripe tomatoes */
	list_for_each_entry_safe(desc, _dt, &pch->work_list, node)
		if (desc->status == DONE) {
			if (!pch->cyclic)
				dma_cookie_complete(&desc->txd);
			list_move_tail(&desc->node, &pch->completed_list);
		}

	/* Try to submit a req imm. next to the last completed cookie */
	fill_queue(pch);

	/* Make sure the PL330 Channel thread is active */
	pl330_chan_ctrl(pch->pl330_chid, PL330_OP_START);

	while (!list_empty(&pch->completed_list)) {
		dma_async_tx_callback callback;
		void *callback_param;

		desc = list_first_entry(&pch->completed_list,
					struct dma_pl330_desc, node);

		callback = desc->txd.callback;
		callback_param = desc->txd.callback_param;

		if (pch->cyclic) {
			desc->status = PREP;
			list_move_tail(&desc->node, &pch->work_list);
		} else {
			desc->status = FREE;
			list_move_tail(&desc->node, &pch->dmac->desc_pool);
		}

		dma_descriptor_unmap(&desc->txd);

		if (callback) {
			spin_unlock_irqrestore(&pch->lock, flags);
			callback(callback_param);
			spin_lock_irqsave(&pch->lock, flags);
		}
	}
	spin_unlock_irqrestore(&pch->lock, flags);
}

static void dma_pl330_rqcb(void *token, enum pl330_op_err err)
{
	struct dma_pl330_desc *desc = token;
	struct dma_pl330_chan *pch = desc->pchan;
	unsigned long          flags;

	/* If desc aborted */
	if (!pch)
		return;

	spin_lock_irqsave(&pch->lock, flags);

	desc->status = DONE;

	spin_unlock_irqrestore(&pch->lock, flags);

	tasklet_schedule(&pch->task);
}

static bool pl330_dt_filter(struct dma_chan *chan, void *param)
{
	struct dma_pl330_filter_args *fargs = param;

	if (chan->device != &fargs->pdmac->ddma)
		return false;

	return chan->chan_id == fargs->chan_id;
}

bool pl330_filter(struct dma_chan *chan, void *param)
{
	u8 *peri_id;

	if (chan->device->dev->driver != &pl330_driver.drv)
		return false;

	peri_id = chan->private;
	return *peri_id == (unsigned long)param;
}
EXPORT_SYMBOL(pl330_filter);

static struct dma_chan *of_dma_pl330_xlate(
	struct of_phandle_args *dma_spec,
	struct of_dma          *ofdma
)
{
	int                          count = dma_spec->args_count;
	struct dma_pl330_dmac       *pdmac = ofdma->of_dma_data;
	struct dma_pl330_filter_args fargs;
	dma_cap_mask_t cap;

	if (!pdmac)
		return NULL;

	if (count != 1)
		return NULL;

	fargs.pdmac = pdmac;
	fargs.chan_id = dma_spec->args[0];

	dma_cap_zero(cap);
	dma_cap_set(DMA_SLAVE, cap);
	dma_cap_set(DMA_CYCLIC, cap);

	return dma_request_channel(cap, pl330_dt_filter, &fargs);
}

static int pl330_alloc_chan_resources(struct dma_chan *chan)
{
	struct dma_pl330_chan *pch = to_pchan(chan);
	struct dma_pl330_dmac *pdmac = pch->dmac;
	unsigned long          flags;

	spin_lock_irqsave(&pch->lock, flags);

	dma_cookie_init(chan);
	pch->cyclic = false;

	pch->pl330_chid = pl330_request_channel(&pdmac->pif);
	if (!pch->pl330_chid) {
		spin_unlock_irqrestore(&pch->lock, flags);
		return -ENOMEM;
	}

	tasklet_init(&pch->task, pl330_tasklet, (unsigned long) pch);

	spin_unlock_irqrestore(&pch->lock, flags);

	return 1;
}

static int pl330_control_term_all(struct dma_pl330_chan *pch)
{
	struct dma_pl330_dmac *pdmac = pch->dmac;
	struct dma_pl330_desc *desc;
	unsigned long          flags;

	LIST_HEAD(list);
	spin_lock_irqsave(&pch->lock, flags);

	/* FLUSH the PL330 Channel thread */
	pl330_chan_ctrl(pch->pl330_chid, PL330_OP_FLUSH);

	/* Mark all desc done */
	list_for_each_entry(desc, &pch->work_list , node)
	{
		desc->status = FREE;
		dma_cookie_complete(&desc->txd);
	}

	list_for_each_entry(desc, &pch->completed_list , node)
	{
		desc->status = FREE;
		dma_cookie_complete(&desc->txd);
	}

	list_splice_tail_init(&pch->work_list, &pdmac->desc_pool);
	list_splice_tail_init(&pch->completed_list, &pdmac->desc_pool);
	spin_unlock_irqrestore(&pch->lock, flags);
	return 0;
}

static int pl330_control_slave_cfg(
	struct dma_pl330_chan *pch,
	unsigned long          arg
)
{
	struct dma_slave_config *cfg = (struct dma_slave_config *)arg;

	if (cfg->direction == DMA_MEM_TO_DEV) {
		if (cfg->dst_addr)
			pch->fifo_addr = cfg->dst_addr;
		if (cfg->dst_addr_width)
			pch->burst_sz = __ffs(cfg->dst_addr_width);
		if (cfg->dst_maxburst)
			pch->burst_len = cfg->dst_maxburst;
	} else if (cfg->direction == DMA_DEV_TO_MEM) {
		if (cfg->src_addr)
			pch->fifo_addr = cfg->src_addr;
		if (cfg->src_addr_width)
			pch->burst_sz = __ffs(cfg->src_addr_width);
		if (cfg->src_maxburst)
			pch->burst_len = cfg->src_maxburst;
	}

	/*
	 * TODO:
	 * This is a problem. Original code assumed flow control and ignored
	 * the device_fc field. If I support the device_fc field, I should
	 * default to true and set false only if the caller has specified it.
	 * Problem is that I cannot tell the difference between a default
	 * zero init and an explicit assigment to zero. No win.
	 * Perhaps save this value but have an enable flag via PL330
	 * slave config below.
	 */
	pch->device_fc  = cfg->device_fc;

	return 0;
}

static int pl330_control_slave_cfg2(
	struct dma_pl330_chan *pch,
	unsigned long          arg
)
{
	struct pl330_slave_config *cfg = (struct pl330_slave_config *)arg;

	pch->flush_once  = cfg->flush_once;
	pch->wfp_once    = cfg->wfp_once;
	pch->unroll_loop = cfg->unroll_loop;

	dev_info(pch->dmac->pif.dev,
		"flush_once-%d wfp_once-%d unroll_loop-%d\n",
		cfg->flush_once, cfg->wfp_once, cfg->unroll_loop);

	return 0;
}

static int pl330_control(
	struct dma_chan  *chan,
	enum dma_ctrl_cmd cmd,
	unsigned long     arg)
{
	int                    status;
	struct dma_pl330_chan *pch = to_pchan(chan);

	/* Cast to int to allow for pl330_dma_ctrl_cmd */
	switch ((int)cmd) {
	case DMA_TERMINATE_ALL:
		status = pl330_control_term_all(pch);
		break;

	case DMA_SLAVE_CONFIG:
		status = pl330_control_slave_cfg(pch, arg);
		break;

	case PL330_SLAVE_CONFIG:
		status = pl330_control_slave_cfg2(pch, arg);
		break;

	default:
		dev_err(pch->dmac->pif.dev,
			"Not supported command,%d.\n", cmd);
		status = -ENXIO;
	}

	return status;
}

static void pl330_free_chan_resources(struct dma_chan *chan)
{
	struct dma_pl330_chan *pch = to_pchan(chan);
	unsigned long          flags;

	tasklet_kill(&pch->task);

	spin_lock_irqsave(&pch->lock, flags);

	pl330_release_channel(pch->pl330_chid);
	pch->pl330_chid = NULL;

	if (pch->cyclic)
		list_splice_tail_init(&pch->work_list, &pch->dmac->desc_pool);

	spin_unlock_irqrestore(&pch->lock, flags);
}

static enum dma_status pl330_tx_status(
	struct dma_chan     *chan,
	dma_cookie_t         cookie,
	struct dma_tx_state *txstate
)
{
	return dma_cookie_status(chan, cookie, txstate);
}

static void pl330_issue_pending(struct dma_chan *chan)
{
	pl330_tasklet((unsigned long) to_pchan(chan));
}

/*
 * We returned the last one of the circular list of descriptor(s)
 * from prep_xxx, so the argument to submit corresponds to the last
 * descriptor of the list.
 */
static dma_cookie_t pl330_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct dma_pl330_desc *desc;
	struct dma_pl330_desc *last = to_desc(tx);
	struct dma_pl330_chan *pch  = to_pchan(tx->chan);
	dma_cookie_t           cookie;
	unsigned long          flags;

	spin_lock_irqsave(&pch->lock, flags);

	/* Assign cookies to all nodes */
	while (!list_empty(&last->node)) {
		desc = list_entry(last->node.next, struct dma_pl330_desc, node);
		if (pch->cyclic) {
			desc->txd.callback = last->txd.callback;
			desc->txd.callback_param = last->txd.callback_param;
		}

		dma_cookie_assign(&desc->txd);

		list_move_tail(&desc->node, &pch->work_list);
	}

	cookie = dma_cookie_assign(&last->txd);
	list_add_tail(&last->node, &pch->work_list);
	spin_unlock_irqrestore(&pch->lock, flags);

	return cookie;
}

static inline void _init_desc(struct dma_pl330_desc *desc)
{
	desc->req.x         = &desc->px;
	desc->req.token     = desc;
	desc->rqcfg.swap    = SWAP_NO;
	desc->rqcfg.scctl   = SCCTRL0;
	desc->rqcfg.dcctl   = DCCTRL0;
	desc->req.preqcfg   = &desc->rqcfg;
	desc->req.xfer_cb   = dma_pl330_rqcb;
	desc->txd.tx_submit = pl330_tx_submit;

	INIT_LIST_HEAD(&desc->node);
}

/* Returns the number of descriptors added to the DMAC pool */
static int add_desc(
	struct dma_pl330_dmac *pdmac,
	gfp_t                  flg,
	int                    count
)
{
	struct dma_pl330_desc *desc;
	unsigned long flags;
	int i;

	if (!pdmac)
		return 0;

	desc = kcalloc(count, sizeof(*desc), flg);
	if (!desc)
		return 0;

	spin_lock_irqsave(&pdmac->pool_lock, flags);

	for (i = 0; i < count; i++) {
		_init_desc(&desc[i]);
		list_add_tail(&desc[i].node, &pdmac->desc_pool);
	}

	spin_unlock_irqrestore(&pdmac->pool_lock, flags);

	return count;
}

static struct dma_pl330_desc *
pluck_desc(struct dma_pl330_dmac *pdmac)
{
	struct dma_pl330_desc *desc = NULL;
	unsigned long          flags;

	if (!pdmac)
		return NULL;

	spin_lock_irqsave(&pdmac->pool_lock, flags);

	if (!list_empty(&pdmac->desc_pool)) {
		desc = list_entry(pdmac->desc_pool.next,
				struct dma_pl330_desc, node);

		list_del_init(&desc->node);

		desc->status = PREP;
		desc->txd.callback = NULL;
	}

	spin_unlock_irqrestore(&pdmac->pool_lock, flags);

	return desc;
}

static struct dma_pl330_desc *pl330_get_desc(struct dma_pl330_chan *pch)
{
	struct dma_pl330_dmac *pdmac   = pch->dmac;
	u8                    *peri_id = pch->chan.private;
	struct dma_pl330_desc *desc;

	/* Pluck one desc from the pool of DMAC */
	desc = pluck_desc(pdmac);

	/* If the DMAC pool is empty, alloc new */
	if (!desc) {
		if (!add_desc(pdmac, GFP_ATOMIC, 1))
			return NULL;

		/* Try again */
		desc = pluck_desc(pdmac);
		if (!desc) {
			dev_err(pch->dmac->pif.dev,
				"%s:%d ALERT!\n", __func__, __LINE__);
			return NULL;
		}
	}

	/* Initialize the descriptor */
	desc->pchan = pch;
	desc->txd.cookie = 0;
	async_tx_ack(&desc->txd);

	/* This is passed down into the microcode functions that expect a */
	/* u8 datatype. Why use pch->chan.chan_id vs *peri_id? */
	desc->req.peri = peri_id ? pch->chan.chan_id : 0;

	desc->rqcfg.pcfg = &pch->dmac->pif.cfg;

	dma_async_tx_descriptor_init(&desc->txd, &pch->chan);

	return desc;
}

static inline void fill_px(
	struct pl330_xfer *px,
	dma_addr_t         dst,
	dma_addr_t         src,
	size_t             len
)
{
	px->next = NULL;
	px->bytes = len;
	px->dst_addr = dst;
	px->src_addr = src;
}

static struct dma_pl330_desc *__pl330_prep_dma_memcpy(
	struct dma_pl330_chan *pch,
	dma_addr_t             dst,
	dma_addr_t             src,
	size_t                 len
)
{
	struct dma_pl330_desc *desc = pl330_get_desc(pch);

	if (!desc) {
		dev_err(pch->dmac->pif.dev, "%s:%d Unable to fetch desc\n",
			__func__, __LINE__);
		return NULL;
	}

	/*
	 * Ideally we should lookout for reqs bigger than
	 * those that can be programmed with 256 bytes of
	 * MC buffer, but considering a req size is seldom
	 * going to be word-unaligned and more than 200MB,
	 * we take it easy.
	 * Also, should the limit is reached we'd rather
	 * have the platform increase MC buffer size than
	 * complicating this API driver.
	 */
	fill_px(&desc->px, dst, src, len);

	return desc;
}

/* Call after fixing burst size */
static inline u32 get_burst_len(
	struct dma_pl330_desc *desc,
	size_t                 len
)
{
	struct dma_pl330_chan *pch = desc->pchan;
	struct pl330_info     *pi  = &pch->dmac->pif;
	u32                    burst_len;

	burst_len   = pi->cfg.data_bus_width / 8;
	burst_len  *= pi->cfg.data_buf_dep;
	burst_len >>= desc->rqcfg.brst_size;

	/* src/dst_burst_len can't be more than 16 */
	if (burst_len > 16)
		burst_len = 16;

	while (burst_len > 1) {
		if (!(len % (burst_len << desc->rqcfg.brst_size)))
			break;
		burst_len--;
	}

	return burst_len;
}

static struct dma_async_tx_descriptor *pl330_prep_dma_cyclic(
	struct dma_chan            *chan,
	dma_addr_t                  dma_addr,
	size_t                      len,
	size_t                      period_len,
	enum dma_transfer_direction direction,
	unsigned long               flags,
	void                       *context
)
{
	struct dma_pl330_desc *desc = NULL, *first = NULL;
	struct dma_pl330_chan *pch = to_pchan(chan);
	struct dma_pl330_dmac *pdmac = pch->dmac;
	unsigned int           i;
	dma_addr_t             dst;
	dma_addr_t             src;

	if (len % period_len != 0)
		return NULL;

	if (!is_slave_direction(direction)) {
		dev_err(pch->dmac->pif.dev, "%s:%d Invalid dma direction\n",
		__func__, __LINE__);
		return NULL;
	}

	for (i = 0; i < len / period_len; i++) {
		desc = pl330_get_desc(pch);
		if (!desc) {
			dev_err(pch->dmac->pif.dev, "%s:%d Unable to fetch desc\n",
				__func__, __LINE__);

			if (!first)
				return NULL;

			spin_lock_irqsave(&pdmac->pool_lock, flags);

			while (!list_empty(&first->node)) {
				desc = list_entry(first->node.next,
						struct dma_pl330_desc, node);
				list_move_tail(&desc->node, &pdmac->desc_pool);
			}

			list_move_tail(&first->node, &pdmac->desc_pool);

			spin_unlock_irqrestore(&pdmac->pool_lock, flags);

			return NULL;
		}

		switch (direction) {
		case DMA_MEM_TO_DEV:
			desc->rqcfg.src_inc = true;
			desc->rqcfg.dst_inc = false;
			desc->req.rqtype = MEMTODEV;
			src = dma_addr;
			dst = pch->fifo_addr;
			break;
		case DMA_DEV_TO_MEM:
			desc->rqcfg.src_inc = false;
			desc->rqcfg.dst_inc = true;
			desc->req.rqtype = DEVTOMEM;
			src = pch->fifo_addr;
			dst = dma_addr;
			break;
		default:
			break;
		}

		desc->rqcfg.brst_size   = pch->burst_sz;
		desc->rqcfg.brst_len    = pch->burst_len;
		desc->rqcfg.flush_once  = pch->flush_once;
		desc->rqcfg.wfp_once    = pch->wfp_once;
		desc->rqcfg.unroll_loop = pch->unroll_loop;
		desc->rqcfg.device_fc   = pch->device_fc;
		fill_px(&desc->px, dst, src, period_len);

		if (!first)
			first = desc;
		else
			list_add_tail(&desc->node, &first->node);

		dma_addr += period_len;
	}

	if (!desc)
		return NULL;

	pch->cyclic = true;
	desc->txd.flags = flags;

	return &desc->txd;
}

static struct dma_async_tx_descriptor *pl330_prep_dma_memcpy(
	struct dma_chan *chan,
	dma_addr_t       dst,
	dma_addr_t       src,
	size_t           len,
	unsigned long    flags
)
{
	struct dma_pl330_desc *desc;
	struct dma_pl330_chan *pch = to_pchan(chan);
	struct pl330_info     *pi;
	int                    burst;

	if (unlikely(!pch || !len))
		return NULL;

	pi = &pch->dmac->pif;

	desc = __pl330_prep_dma_memcpy(pch, dst, src, len);
	if (!desc)
		return NULL;

	desc->rqcfg.src_inc = true;
	desc->rqcfg.dst_inc = true;
	desc->req.rqtype = MEMTOMEM;

	/* Select max possible burst size */
	burst = pi->cfg.data_bus_width / 8;
	
	while (burst > 1) {
		if (!(len % burst))
			break;
		burst /= 2;
	}
	
	desc->rqcfg.brst_size = 0;
	while (burst != (1 << desc->rqcfg.brst_size))
		desc->rqcfg.brst_size++;

	desc->rqcfg.brst_len = get_burst_len(desc, len);

	desc->txd.flags = flags;

	//**// vogelpi
	// Allow for caching in the AXI data width converter
	if (chan->chan_id == 0) { // Host -> PULP
	  desc->rqcfg.dcctl = DCCTRL3;
	}
	else if (chan->chan_id == 1) { // PULP -> Host
	  desc->rqcfg.scctl = SCCTRL3;
	}
	//printk("burst size   = %d \n",desc->rqcfg.brst_size);
	//printk("burst length = %d \n",desc->rqcfg.brst_len);
	//printk("scctl        = %d \n",desc->rqcfg.scctl);
	//printk("dcctl        = %d \n",desc->rqcfg.dcctl);
	//**// vogelpi

	return &desc->txd;
}

static void __pl330_giveback_desc(
	struct dma_pl330_dmac *pdmac,
	struct dma_pl330_desc *first
)
{
	unsigned long          flags;
	struct dma_pl330_desc *desc;

	if (!first)
		return;

	spin_lock_irqsave(&pdmac->pool_lock, flags);

	while (!list_empty(&first->node)) {
		desc = list_entry(first->node.next,
				struct dma_pl330_desc, node);
		list_move_tail(&desc->node, &pdmac->desc_pool);
	}

	list_move_tail(&first->node, &pdmac->desc_pool);

	spin_unlock_irqrestore(&pdmac->pool_lock, flags);
}

static struct dma_async_tx_descriptor *pl330_prep_slave_sg(
	struct dma_chan            *chan,
	struct scatterlist         *sgl,
	unsigned int                sg_len,
	enum dma_transfer_direction direction,
	unsigned long               flg,
	void                       *context
)
{
	struct dma_pl330_desc *first;
	struct dma_pl330_desc *desc = NULL;
	struct dma_pl330_chan *pch = to_pchan(chan);
	struct scatterlist    *sg;
	int                    i;
	dma_addr_t             addr;

	if (unlikely(!pch || !sgl || !sg_len))
		return NULL;

	addr = pch->fifo_addr;

	first = NULL;

	for_each_sg(sgl, sg, sg_len, i) {

		desc = pl330_get_desc(pch);
		if (!desc) {
			struct dma_pl330_dmac *pdmac = pch->dmac;

			dev_err(pch->dmac->pif.dev,
				"%s:%d Unable to fetch desc\n",
				__func__, __LINE__);
			__pl330_giveback_desc(pdmac, first);

			return NULL;
		}

		if (!first)
			first = desc;
		else
			list_add_tail(&desc->node, &first->node);

		if (direction == DMA_MEM_TO_DEV) {
			desc->rqcfg.src_inc = true;
			desc->rqcfg.dst_inc = false;
			desc->req.rqtype = MEMTODEV;
			fill_px(&desc->px,
				addr, sg_dma_address(sg), sg_dma_len(sg));
		} else {
			desc->rqcfg.src_inc = false;
			desc->rqcfg.dst_inc = true;
			desc->req.rqtype = DEVTOMEM;
			fill_px(&desc->px,
				sg_dma_address(sg), addr, sg_dma_len(sg));
		}

		desc->rqcfg.brst_size   = pch->burst_sz;
		desc->rqcfg.brst_len    = pch->burst_len;
		desc->rqcfg.flush_once  = pch->flush_once;
		desc->rqcfg.wfp_once    = pch->wfp_once;
		desc->rqcfg.unroll_loop = pch->unroll_loop;
		desc->rqcfg.device_fc   = pch->device_fc;
	}

	/* Return the last desc in the chain */
	desc->txd.flags = flg;
	return &desc->txd;
}

static irqreturn_t pl330_irq_handler(int irq, void *data)
{
	if (pl330_update(data))
		return IRQ_HANDLED;
	else
		return IRQ_NONE;
}

#define PL330_DMA_BUSWIDTHS \
	(\
	BIT(DMA_SLAVE_BUSWIDTH_UNDEFINED) | \
	BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) | \
	BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | \
	BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) | \
	BIT(DMA_SLAVE_BUSWIDTH_8_BYTES)\
	)

static int pl330_dma_device_slave_caps(
	struct dma_chan       *dchan,
	struct dma_slave_caps *caps
)
{
	caps->src_addr_widths = PL330_DMA_BUSWIDTHS;
	caps->dstn_addr_widths = PL330_DMA_BUSWIDTHS;
	caps->directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	caps->cmd_pause = false;
	caps->cmd_terminate = true;

	return 0;
}

static int pl330_probe(
	struct amba_device   *adev,
	const struct amba_id *id
)
{
	struct dma_pl330_platdata *pdat;
	struct dma_pl330_dmac     *pdmac;
	struct dma_pl330_chan     *pch;
	struct dma_pl330_chan     *_p;
	struct pl330_info         *pi;
	struct dma_device         *pd;
	struct resource           *res;
	int                        i;
	int                        ret;
	int                        irq;
	int                        num_chan;

	pdat = dev_get_platdata(&adev->dev);

	ret = dma_set_mask_and_coherent(&adev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/* Allocate a new DMAC and its Channels */
	pdmac = devm_kzalloc(&adev->dev, sizeof(*pdmac), GFP_KERNEL);
	if (!pdmac) {
		dev_err(&adev->dev, "unable to allocate mem\n");
		return -ENOMEM;
	}

	pi = &pdmac->pif;
	pi->dev = &adev->dev;
	pi->pl330_data = NULL;
	pi->mcbufsz = pdat ? pdat->mcbuf_sz : 0;

	res = &adev->res;
	pi->base = devm_ioremap_resource(&adev->dev, res);
	if (IS_ERR(pi->base))
		return PTR_ERR(pi->base);

	amba_set_drvdata(adev, pdmac);

	for (i = 0; i < AMBA_NR_IRQS; i++) {
		irq = adev->irq[i];
		if (irq) {
			ret = devm_request_irq(&adev->dev, irq,
					       pl330_irq_handler, 0,
					       dev_name(&adev->dev), pi);
			if (ret)
				return ret;
		} else {
			break;
		}
	}

	pi->cfg.periph_id = adev->periphid;
	ret = pl330_add(pi);
	if (ret)
		return ret;

	INIT_LIST_HEAD(&pdmac->desc_pool);
	spin_lock_init(&pdmac->pool_lock);

	/* Create a descriptor pool of default size */
	if (!add_desc(pdmac, GFP_KERNEL, NR_DEFAULT_DESC))
		dev_warn(&adev->dev, "unable to allocate desc\n");

	pd = &pdmac->ddma;
	INIT_LIST_HEAD(&pd->channels);

	/* Initialize channel parameters */
	if (pdat)
		num_chan = max_t(int, pdat->nr_valid_peri, pi->cfg.num_chan);
	else
		num_chan = max_t(int, pi->cfg.num_peri, pi->cfg.num_chan);

	pdmac->peripherals = kzalloc(num_chan * sizeof(*pch), GFP_KERNEL);
	if (!pdmac->peripherals) {
		ret = -ENOMEM;
		dev_err(&adev->dev, "unable to allocate pdmac->peripherals\n");
		goto probe_err2;
	}

	for (i = 0; i < num_chan; i++) {
		pch = &pdmac->peripherals[i];
		if (!adev->dev.of_node)
			pch->chan.private = pdat ? &pdat->peri_id[i] : NULL;
		else
			pch->chan.private = adev->dev.of_node;

		INIT_LIST_HEAD(&pch->work_list);
		INIT_LIST_HEAD(&pch->completed_list);
		spin_lock_init(&pch->lock);
		pch->pl330_chid = NULL;
		pch->chan.device = pd;
		pch->dmac = pdmac;

		/* Add the channel to the DMAC list */
		list_add_tail(&pch->chan.device_node, &pd->channels);
	}

	pd->dev = &adev->dev;
	if (pdat) {
		pd->cap_mask = pdat->cap_mask;
	} else {
		dma_cap_set(DMA_MEMCPY, pd->cap_mask);
		if (pi->cfg.num_peri) {
			dma_cap_set(DMA_SLAVE, pd->cap_mask);
			dma_cap_set(DMA_CYCLIC, pd->cap_mask);
			dma_cap_set(DMA_PRIVATE, pd->cap_mask);
		}
	}

	pd->device_alloc_chan_resources = pl330_alloc_chan_resources;
	pd->device_free_chan_resources = pl330_free_chan_resources;
	pd->device_prep_dma_memcpy = pl330_prep_dma_memcpy;
	pd->device_prep_dma_cyclic = pl330_prep_dma_cyclic;
	pd->device_tx_status = pl330_tx_status;
	pd->device_prep_slave_sg = pl330_prep_slave_sg;
	pd->device_control = pl330_control;
	pd->device_issue_pending = pl330_issue_pending;
	pd->device_slave_caps = pl330_dma_device_slave_caps;

	ret = dma_async_device_register(pd);
	if (ret) {
		dev_err(&adev->dev, "unable to register DMAC\n");
		goto probe_err3;
	}

	if (adev->dev.of_node) {
		ret = of_dma_controller_register(adev->dev.of_node,
					 of_dma_pl330_xlate, pdmac);
		if (ret) {
			dev_err(&adev->dev,
			"unable to register DMA to the generic DT DMA helpers\n");
		}
	}
	/*
	 * This is the limit for transfers with a buswidth of 1, larger
	 * buswidths will have larger limits.
	 */
	ret = dma_set_max_seg_size(&adev->dev, 1900800);
	if (ret)
		dev_err(&adev->dev, "unable to set the seg size\n");


	dev_info(&adev->dev,
		"Loaded driver for PL330 DMAC-%d\n", adev->periphid);
	dev_info(&adev->dev,
		"\tDBUFF-%ux%ubytes Num_Chans-%u Num_Peri-%u Num_Events-%u\n",
		pi->cfg.data_buf_dep,
		pi->cfg.data_bus_width / 8, pi->cfg.num_chan,
		pi->cfg.num_peri, pi->cfg.num_events);
	dev_info(&adev->dev,
		"\ticache_lines-%u icache_len-%u\n",
		pi->cfg.icache_lines, pi->cfg.icache_len);

	return 0;
probe_err3:
	/* Idle the DMAC */
	list_for_each_entry_safe(pch, _p, &pdmac->ddma.channels,
			chan.device_node) {

		/* Remove the channel */
		list_del(&pch->chan.device_node);

		/* Flush the channel */
		pl330_control(&pch->chan, DMA_TERMINATE_ALL, 0);
		pl330_free_chan_resources(&pch->chan);
	}
probe_err2:
	pl330_del(pi);

	return ret;
}

static int pl330_remove(struct amba_device *adev)
{
	struct dma_pl330_dmac *pdmac = amba_get_drvdata(adev);
	struct dma_pl330_chan *pch;
	struct dma_pl330_chan *_p;
	struct pl330_info     *pi;

	if (!pdmac)
		return 0;

	if (adev->dev.of_node)
		of_dma_controller_free(adev->dev.of_node);

	dma_async_device_unregister(&pdmac->ddma);

	/* Idle the DMAC */
	list_for_each_entry_safe(pch, _p, &pdmac->ddma.channels,
			chan.device_node) {

		/* Remove the channel */
		list_del(&pch->chan.device_node);

		/* Flush the channel */
		pl330_control(&pch->chan, DMA_TERMINATE_ALL, 0);
		pl330_free_chan_resources(&pch->chan);
	}

	pi = &pdmac->pif;

	pl330_del(pi);

	return 0;
}

static struct amba_id pl330_ids[] = {
	{
		.id	= 0x00041330,
		.mask	= 0x000fffff,
	},
	{ 0, 0 },
};

MODULE_DEVICE_TABLE(amba, pl330_ids);

static struct amba_driver pl330_driver = {
	.drv = {
		.owner = THIS_MODULE,
		.name = "dma-pl330",
	},
	.id_table = pl330_ids,
	.probe = pl330_probe,
	.remove = pl330_remove,
};

module_amba_driver(pl330_driver);

MODULE_AUTHOR("Jaswinder Singh <jassi.brar@samsung.com>");
MODULE_DESCRIPTION("API Driver for PL330 DMAC");
MODULE_LICENSE("GPL");
