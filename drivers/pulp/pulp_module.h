/*
 * This file is part of the PULP device driver.
 *
 * Copyright (C) 2018 ETH Zurich, University of Bologna
 *
 * Author: Pirmin Vogel <vogelpi@iis.ee.ethz.ch>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef _PULP_MODULE_H_
#define _PULP_MODULE_H_

#include <linux/cdev.h>   /* cdev struct */
#include <linux/device.h> /* device struct */

// PLATFORM is exported in sourceme.sh and passed by the Makefile
#define ZEDBOARD 1
#define ZC706    2
#define MINI_ITX 3
#define JUNO     4
#define TE0808   5

#ifndef PLATFORM
  #error "Define PLATFORM!"
#endif

#if   PLATFORM == JUNO
  #include "juno.h"
#elif PLATFORM == TE0808
  #include "zynqmp.h"
#else // PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX
  #include "zynq.h"
#endif

/*
 * Debug flags
 */
#define DEBUG_LEVEL_PULP    0
#define DEBUG_LEVEL_OF      0
#define DEBUG_LEVEL_MEM     0
#define DEBUG_LEVEL_RAB     0
#define DEBUG_LEVEL_RAB_STR 0
#define DEBUG_LEVEL_RAB_MH  0
#define DEBUG_LEVEL_SMMU    0
#define DEBUG_LEVEL_SMMU_FH 0
#define DEBUG_LEVEL_DMA     0
#define DEBUG_LEVEL_MBOX    0

//#define PROFILE_DMA
//#define PROFILE_RAB_MH_SIMPLE
#if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX
  //#define PROFILE_RAB_STR
  //#define PROFILE_RAB_MH
#endif

/*
 * Host memory map - Part 1 -- see Vivado block diagram
 */
#if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX

  #define H_GPIO_BASE_ADDR     0x51000000
  #define CLKING_BASE_ADDR     0x51010000
  #define RAB_CONFIG_BASE_ADDR 0x51030000
  #define INTR_REG_BASE_ADDR   0x51050000

  #if PLATFORM == ZEDBOARD

    #define RAB_AX_LOG_EN         0
    #define L3_MEM_SIZE_MB        16

  #elif PLATFORM == ZC706 || PLATFORM == MINI_ITX

    #define RAB_AR_LOG_BASE_ADDR  0x51100000
    #define RAB_AW_LOG_BASE_ADDR  0x51200000
    // #define RAB_CFG_LOG_BASE_ADDR TODO (also in HW)

    #define RAB_AX_LOG_EN         1
    #define RAB_AX_LOG_SIZE_B     0x6000   // size of BRAM, 192 KiB = 2 Ki entries
    #define RAB_AX_LOG_BUF_SIZE_B 0x600000 // size of buffer in driver, 6 MiB = 512 Ki entries
    #define RAB_CFG_LOG_SIZE_B    0x30000  // size of BRAM, 196 KiB = 16 Ki entries

    #define L3_MEM_SIZE_MB        128

  #endif // PLATFORM

#elif PLATFORM == JUNO

  #define H_GPIO_BASE_ADDR      0x6E000000
  #define CLKING_BASE_ADDR      0x6E010000
  #define RAB_CONFIG_BASE_ADDR  0x6E030000
  #define INTR_REG_BASE_ADDR    0x6E050000
  #define RAB_AR_LOG_BASE_ADDR  0x6E100000
  #define RAB_AW_LOG_BASE_ADDR  0x6E200000
  #define RAB_CFG_LOG_BASE_ADDR 0x6E300000

  #define RAB_AX_LOG_EN         1
  #define RAB_AX_LOG_SIZE_B     0xC0000   // size of BRAM, 786 KiB = 64 Ki entries
  #define RAB_CFG_LOG_SIZE_B    0x30000   // size of BRAM, 196 KiB = 16 Ki entries
  #define RAB_AX_LOG_BUF_SIZE_B 0x6000000 // size of buffer in driver, 96 MiB = 8 Mi entries

  #define L3_MEM_SIZE_MB        128

#else // TE0808

  #define H_GPIO_BASE_ADDR      0xAE000000
  #define CLKING_BASE_ADDR      0xAE010000
  #define RAB_CONFIG_BASE_ADDR  0xAE030000
  #define INTR_REG_BASE_ADDR    0xAE050000
  #define RAB_AR_LOG_BASE_ADDR  0xAE100000
  #define RAB_AW_LOG_BASE_ADDR  0xAE200000
  #define RAB_CFG_LOG_BASE_ADDR 0xAE300000

  #define RAB_AX_LOG_EN         1
  #define RAB_AX_LOG_SIZE_B     0xC0000   // size of BRAM, 786 KiB = 64 Ki entries
  #define RAB_CFG_LOG_SIZE_B    0x30000   // size of BRAM, 196 KiB = 16 Ki entries
  #define RAB_AX_LOG_BUF_SIZE_B 0x6000000 // size of buffer in driver, 96 MiB = 8 Mi entries

  #define L3_MEM_SIZE_MB        128

#endif // PLATFORM

#define H_GPIO_SIZE_B           0x1000
#define CLKING_SIZE_B           0x1000
#define INTR_REG_SIZE_B         0x1000
#define RAB_CONFIG_SIZE_B       0x10000

/*
 * Host memory map - Part 2 - PULP -- see Vivado block diagram
 */
#if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX
  #define PULP_H_BASE_ADDR            0x40000000 // Address at which the host sees PULP
  #define L1_MEM_H_BASE_ADDR          PULP_H_BASE_ADDR
  #define L2_MEM_H_BASE_ADDR          0x4C000000
  #define L3_MEM_H_BASE_ADDR          (DRAM_SIZE_MB - L3_MEM_SIZE_MB)*1024*1024 // = 0x38000000 for 1024 MB DRAM and 128 MB L3
  #define MBOX_H_BASE_ADDR            0x4A120000 // Interface 0
  #define SOC_PERIPHERALS_H_BASE_ADDR 0x4A100000
#elif PLATFORM == JUNO
  #define PULP_H_BASE_ADDR            0x60000000 // Address at which the host sees PULP
  #define L1_MEM_H_BASE_ADDR          PULP_H_BASE_ADDR
  #define L2_MEM_H_BASE_ADDR          0x67000000
  #define L3_MEM_H_BASE_ADDR         (0xA00000000LL - L3_MEM_SIZE_B)
  #define MBOX_H_BASE_ADDR            0x65120000 // Interface 0
  #define SOC_PERIPHERALS_H_BASE_ADDR 0x65100000
#else // PLATFORM == TE0808
  #define PULP_H_BASE_ADDR            0xA0000000 // Address at which the host sees PULP
  #define L1_MEM_H_BASE_ADDR          PULP_H_BASE_ADDR
  #define L2_MEM_H_BASE_ADDR          0xA7000000
  #define L3_MEM_H_BASE_ADDR         (0x80000000 - L3_MEM_SIZE_B)
  #define MBOX_H_BASE_ADDR            0xA5120000 // Interface 0
  #define SOC_PERIPHERALS_H_BASE_ADDR 0xA5100000
#endif // PLATFORM

/*
 * PULP memory map -- see PULP SDK and PULP HW
 */
#define PULP_BASE_ADDR   0x10000000
#define MBOX_BASE_ADDR   0x1A121000
#define L2_MEM_BASE_ADDR 0x1C000000
#define L3_MEM_BASE_ADDR 0x80000000 // address of the contiguous L3
#define PGD_BASE_ADDR    0x20000000 // address of the top-level page table of user-space process

#define CLUSTER_PERIPHERALS_OFFSET_B 0x200000
#define TIMER_OFFSET_B               0x400

#define TIMER_CLUSTER_OFFSET_B       (CLUSTER_PERIPHERALS_OFFSET_B + TIMER_OFFSET_B)
#define TIMER_START_OFFSET_B         (TIMER_CLUSTER_OFFSET_B + 0x00)
#define TIMER_STOP_OFFSET_B          (TIMER_CLUSTER_OFFSET_B + 0x04)
#define TIMER_RESET_OFFSET_B         (TIMER_CLUSTER_OFFSET_B + 0x08)
#define TIMER_GET_TIME_LO_OFFSET_B   (TIMER_CLUSTER_OFFSET_B + 0x0c)
#define TIMER_GET_TIME_HI_OFFSET_B   (TIMER_CLUSTER_OFFSET_B + 0x10)

#define BBMUX_CLKGATE_OFFSET_B       0x800
#define EU_SW_EVENTS_OFFSET_B        0x600

/*
 * PULP config -- see PULP SDK and PULP HW
 */
#if   PLATFORM == ZEDBOARD
  #define N_CLUSTERS       1
  #define N_CORES          2
  #define L2_MEM_SIZE_KB  64
  #define L1_MEM_SIZE_KB  32
#elif PLATFORM == MINI_ITX || PLATFORM == ZC706
  #define N_CLUSTERS       1
  #define N_CORES          8
  #define L2_MEM_SIZE_KB 256
  #define L1_MEM_SIZE_KB 256
#elif PLATFORM == TE0808
  #define N_CLUSTERS       1
  #define N_CORES          8
  #define L2_MEM_SIZE_KB 256
  #define L1_MEM_SIZE_KB 256
#else // JUNO
  #define N_CLUSTERS       4
  #define N_CORES          8
  #define L2_MEM_SIZE_KB 256
  #define L1_MEM_SIZE_KB 256
#endif

#define CLUSTER_SIZE_MB 4

#define L1_MEM_SIZE_B   (L1_MEM_SIZE_KB*1024)
#define L2_MEM_SIZE_B   (L2_MEM_SIZE_KB*1024)
#define L3_MEM_SIZE_B   (L3_MEM_SIZE_MB*1024*1024)

#define CLUSTER_SIZE_B  (CLUSTER_SIZE_MB*1024*1024)
#define CLUSTERS_SIZE_B (N_CLUSTERS*CLUSTER_SIZE_B)

#define SOC_PERIPHERALS_SIZE_B 0x50000

#define MBOX_SIZE_B          0x1000 // Interface 0 only

/*
 * Interrupts -- see bigpulp*_top.sv
 */
#define INTR_EOC_0              0
#define INTR_EOC_N  (N_CLUSTERS-1) // max 15

#define INTR_MBOX              16
#define INTR_RAB_MISS          17
#define INTR_RAB_MULTI         18
#define INTR_RAB_PROT          19
#define INTR_RAB_MHR_FULL      20
#define INTR_RAB_AR_LOG_FULL   21
#define INTR_RAB_AW_LOG_FULL   22
#define INTR_RAB_CFG_LOG_FULL  23

/*
 * PULP GPIOs -- see bigpulp*_top.sv
 */
#define GPIO_EOC_0              0
#define GPIO_EOC_N  (N_CLUSTERS-1) // max 15

#define GPIO_INTR_RAB_MISS_DIS 17

#define GPIO_RST_N             31
#define GPIO_CLK_EN            30
#define GPIO_RAB_CFG_LOG_RDY   27
#define GPIO_RAB_AR_LOG_RDY    28
#define GPIO_RAB_AW_LOG_RDY    29
#define GPIO_RAB_AR_LOG_CLR    28
#define GPIO_RAB_AW_LOG_CLR    29
#define GPIO_RAB_AX_LOG_EN     27
#define GPIO_RAB_CFG_LOG_CLR   26

/*
 * Mailbox
 */
#define MBOX_FIFO_DEPTH      16
#define MBOX_WRDATA_OFFSET_B 0x0
#define MBOX_RDDATA_OFFSET_B 0x8
#define MBOX_STATUS_OFFSET_B 0x10
#define MBOX_ERROR_OFFSET_B  0x14
#define MBOX_IS_OFFSET_B     0x20
#define MBOX_IE_OFFSET_B     0x24

/*
 * Mailbox signaling -- see PULP SDK and pulp_hsa.h
 */
#define PULP_READY             (0x01U  )
#define PULP_START             (0x02U  )
#define PULP_BUSY              (0x03U  )
#define PULP_DONE              (0x04U  )
#define PULP_STOP              (0x0FU  )

#define HOST_READY             (0x1000U)
#define HOST_DONE              (0x3000U)

#define MBOX_N_BITS_REQ_TYPE   (4U)          // #MSBs to specify the type
#define RAB_UPDATE_N_BITS_ELEM (8U)          // #bits to specify the mask of elements
#define RAB_UPDATE_N_BITS_TYPE (2U)          // #bits to specify the update type

#define TO_RUNTIME             (0x10000000U) // bypass PULP driver
#define RAB_UPDATE             (0x20000000U) // handled by PULP driver
#define RAB_SWITCH             (0x30000000U) // handled by PULP driver

/*
 * General settings
 */
#define PULP_N_DEV_NUMBERS 1

/*
 * IOCTL setup -- must match user-space side -- see pulp_func.h
 *
 * When defining a new IOCTL command, append a macro definition to the list below, using the
 * consecutively following command number, and increase the `PULP_IOC_NR_MAX` macro.
 */
#define PULP_IOCTL_MAGIC 'p'
#define PULP_IOC_NR_MIN 0xB0
#define PULP_IOC_NR_MAX 0xBE

#define PULP_IOCTL_RAB_REQ          _IOW(PULP_IOCTL_MAGIC,0xB0,unsigned) // ptr
#define PULP_IOCTL_RAB_FREE         _IOW(PULP_IOCTL_MAGIC,0xB1,unsigned) // value

#define PULP_IOCTL_RAB_REQ_STRIPED  _IOW(PULP_IOCTL_MAGIC,0xB2,unsigned) // ptr
#define PULP_IOCTL_RAB_FREE_STRIPED _IOW(PULP_IOCTL_MAGIC,0xB3,unsigned) // value

#define PULP_IOCTL_RAB_MH_ENA       _IOW(PULP_IOCTL_MAGIC,0xB4,unsigned) // ptr
#define PULP_IOCTL_RAB_MH_DIS       _IO(PULP_IOCTL_MAGIC,0xB5)

#define PULP_IOCTL_RAB_SOC_MH_ENA   _IOW(PULP_IOCTL_MAGIC,0xB6,unsigned) // value
#define PULP_IOCTL_RAB_SOC_MH_DIS   _IO(PULP_IOCTL_MAGIC,0xB7)

#define PULP_IOCTL_SMMU_ENA         _IOW(PULP_IOCTL_MAGIC,0xB8,unsigned) // value
#define PULP_IOCTL_SMMU_DIS         _IO(PULP_IOCTL_MAGIC,0xB9)

#define PULP_IOCTL_DMA_XFER_SYNC    _IOW(PULP_IOCTL_MAGIC,0xBA,unsigned) // ptr
#define PULP_IOCTL_DMA_XFER_ASYNC   _IOW(PULP_IOCTL_MAGIC,0xBB,unsigned) // ptr
#define PULP_IOCTL_DMA_XFER_WAIT    _IOW(PULP_IOCTL_MAGIC,0xBC,unsigned) // ptr

#define PULP_IOCTL_INFO_PASS        _IOW(PULP_IOCTL_MAGIC,0xBD,unsigned) // ptr

#define PULP_IOCTL_RAB_AX_LOG_READ  _IOW(PULP_IOCTL_MAGIC,0xBE,unsigned) // ptr

/*
 * Macros
 */
#if PLATFORM == JUNO || PLATFORM == TE0808
  #define IOWRITE_L(value, addr) ( iowrite64(value, addr) )
  #define IOREAD_L(addr)         ( ioread64(addr)         )
#else
  #define IOWRITE_L(value, addr) ( iowrite32(value, addr) )
  #define IOREAD_L(addr)         ( ioread32(addr)         )
#endif

#define BIT_N_SET(n)        ( 1UL << (n) )
#define BIT_GET(y, mask)    ( y & mask )
#define BIT_SET(y, mask)    ( y |=  (mask) )
#define BIT_CLEAR(y, mask)  ( y &= ~(mask) )
#define BIT_FLIP(y, mask)   ( y ^=  (mask) )
// Create a bitmask of length len.
#define BIT_MASK_GEN(len)       ( BIT_N_SET(len)-1 )
// Create a bitfield mask of length len starting at bit start.
#define BF_MASK_GEN(start, len)      ( BIT_MASK_GEN(len) << (start) )
// Prepare a bitmask for insertion or combining.
#define BF_PREP(x, start, len)   ( ((x)&BIT_MASK_GEN(len)) << (start) )
// Extract a bitfield of length len starting at bit start from y.
#define BF_GET(y, start, len)    ( ((y)>>(start)) & BIT_MASK_GEN(len) )
// Insert a new bitfield value x into y.
#define BF_SET(y, x, start, len) \
  ( y= ((y) &~ BF_MASK_GEN(start, len)) | BF_PREP(x, start, len) )

/*
 * Type Definitions
 */
typedef struct {
  dev_t dev; // device number
  struct file_operations *fops;
  struct cdev cdev;
  int minor;
  int major;
  // device tree
  struct device * dt_dev_ptr;
  int intr_reg_irq;
  // virtual address pointers for ioremap_nocache()
  void *mbox;
  void *rab_config;
  void *gpio;
  void *soc_periph;
  void *clusters;
  void *l3_mem;
  void *l2_mem;
  void *intr_reg;
  #if PLATFORM == TE0808
    void *cci;
    void *smmu;
  #endif // PLATFORM
  #if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX
    void *slcr;
  #endif // PLATFORM
  #if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX || PLATFORM == TE0808
    void *uart;
  #endif // PLATFORM
  #if RAB_AX_LOG_EN == 1
    void *rab_ar_log;
    void *rab_aw_log;
    void *rab_cfg_log;
  #endif // RAB_AX_LOG_EN == 1
} PulpDev;

#endif // _PULP_MODULE_H_
