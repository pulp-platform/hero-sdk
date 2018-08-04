/*
 * This file is part of the PULP user-space runtime library.
 *
 * Copyright 2018 ETH Zurich, University of Bologna
 *
 * Author: Pirmin Vogel <vogelpi@iis.ee.ethz.ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef PULP_FUNC_H__
#define PULP_FUNC_H__

#include <stdio.h>
#include <string.h>
#include <sys/mman.h>   // for mmap
#include <fcntl.h>
#include <errno.h>      // for error codes
#include <stdbool.h>    // for bool
#include <sys/ioctl.h>  // for ioctl
#include <stdlib.h>     // for system
#include <unistd.h>     // for usleep, access

#include <errno.h>

#include "pulp.h"

// PLATFORM is exported in sourceme.sh and passed by the Makefile
#define ZEDBOARD 1
#define ZC706    2
#define MINI_ITX 3
#define JUNO     4
#define TE0808   5

// from include/archi/pulp.h
#define CHIP_BIGPULP        7
#define CHIP_BIGPULP_Z_7020 8
#define CHIP_BIGPULP_Z_7045 9

#ifndef PLATFORM
  #error "Define PLATFORM!"
#endif

#define PULP_CHIP_FAMILY CHIP_BIGPULP
#if PLATFORM == ZEDBOARD
  #define PULP_CHIP CHIP_BIGPULP_Z_7020
#elif PLATFORM == ZC706 || PLATFORM == MINI_ITX || PLATFORM == TE0808
  #define PULP_CHIP CHIP_BIGPULP_Z_7045
#else // PLATFORM == JUNO
  #define PULP_CHIP CHIP_BIGPULP
#endif

#if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX
  #include "zynq_pmm_user.h"
#endif

/*
 * Debug flags
 */
#ifndef DEBUG_LEVEL
  #define DEBUG_LEVEL 0
#endif

/*
 * Host memory map - Part 1 -- see Vivado block diagram
 */
#if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX

  #if PLATFORM == ZEDBOARD
    #define RAB_AX_LOG_EN  0
    #define L3_MEM_SIZE_MB 16

  #elif PLATFORM == ZC706 || PLATFORM == MINI_ITX
    #define RAB_AX_LOG_EN         1
    #define RAB_AX_LOG_BUF_SIZE_B 0x600000

    #define L3_MEM_SIZE_MB        128

  #endif // PLATFORM

#elif PLATFORM == JUNO
  #define RAB_AX_LOG_EN         1
  #define RAB_AX_LOG_BUF_SIZE_B 0x6000000

  #define L3_MEM_SIZE_MB        128

#else // TE0808
  #define RAB_AX_LOG_EN         1
  #define RAB_AX_LOG_BUF_SIZE_B 0x6000000

  #define L3_MEM_SIZE_MB        128

#endif // PLATFORM

#define H_GPIO_SIZE_B      0x1000
#define CLKING_SIZE_B      0x1000
#define RAB_CONFIG_SIZE_B  0x10000

/*
 * Host memory map - Part 2 - PULP -- see Vivado block diagram
 */
#if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX
  #define PULP_H_BASE_ADDR   0x40000000 // Address at which the host sees PULP
  #define L2_MEM_H_BASE_ADDR 0x4C000000
  #define MBOX_H_BASE_ADDR   0x4A120000 // Interface 0
#elif PLATFORM == JUNO
  #define PULP_H_BASE_ADDR   0x60000000 // Address at which the host sees PULP
  #define L2_MEM_H_BASE_ADDR 0x67000000
  #define MBOX_H_BASE_ADDR   0x65120000 // Interface 0
#else // PLATFORM == TE0808
  #define PULP_H_BASE_ADDR   0xA0000000 // Address at which the host sees PULP
  #define L2_MEM_H_BASE_ADDR 0xA7000000
  #define MBOX_H_BASE_ADDR   0xA5120000 // Interface 0
#endif // PLATFORM

/*
 * PULP memory map -- see PULP SDK and PULP HW
 */
#define L3_MEM_BASE_ADDR 0x80000000 // address of the contiguous L3

#define CLUSTER_PERIPHERALS_OFFSET_B 0x200000
#define TIMER_OFFSET_B               0x400

#define TIMER_CLUSTER_OFFSET_B       (CLUSTER_PERIPHERALS_OFFSET_B + TIMER_OFFSET_B)
#define TIMER_START_OFFSET_B         (TIMER_CLUSTER_OFFSET_B + 0x00)
#define TIMER_STOP_OFFSET_B          (TIMER_CLUSTER_OFFSET_B + 0x04)
#define TIMER_RESET_OFFSET_B         (TIMER_CLUSTER_OFFSET_B + 0x08)
#define TIMER_GET_TIME_LO_OFFSET_B   (TIMER_CLUSTER_OFFSET_B + 0x0c)
#define TIMER_GET_TIME_HI_OFFSET_B   (TIMER_CLUSTER_OFFSET_B + 0x10)

/*
 * PULP config -- see PULP SDK and PULP HW
 */
#if   PLATFORM == ZEDBOARD
  #define N_CLUSTERS               1
  #define N_CORES                  2
  #define L2_MEM_SIZE_KB          64
  #define L1_MEM_SIZE_KB          32
  #define PULP_DEFAULT_FREQ_MHZ   25
  #define CLKING_INPUT_FREQ_MHZ   50
  #define RAB_L1_N_SLICES_PORT_1   8
#elif PLATFORM == MINI_ITX || PLATFORM == ZC706
  #define N_CLUSTERS               1
  #define N_CORES                  8
  #define L2_MEM_SIZE_KB         256
  #define L1_MEM_SIZE_KB         256
  #define PULP_DEFAULT_FREQ_MHZ   50
  #define CLKING_INPUT_FREQ_MHZ  100
  #define RAB_L1_N_SLICES_PORT_1  32
#elif PLATFORM == TE0808
  #define N_CLUSTERS               1
  #define N_CORES                  8
  #define L2_MEM_SIZE_KB         256
  #define L1_MEM_SIZE_KB         256
  #define PULP_DEFAULT_FREQ_MHZ   50
  #define CLKING_INPUT_FREQ_MHZ  100
  #define RAB_L1_N_SLICES_PORT_1  32
#else // JUNO
  #define N_CLUSTERS               4
  #define N_CORES                  8
  #define L2_MEM_SIZE_KB         256
  #define L1_MEM_SIZE_KB         256
  #define PULP_DEFAULT_FREQ_MHZ   25
  #define CLKING_INPUT_FREQ_MHZ  100
  #define RAB_L1_N_SLICES_PORT_1  32
#endif

#define CLUSTER_MASK  (unsigned)((1 << N_CLUSTERS) - 1)

#define CLUSTER_SIZE_MB 4

#define L1_MEM_SIZE_B   (L1_MEM_SIZE_KB*1024)
#define L2_MEM_SIZE_B   (L2_MEM_SIZE_KB*1024)
#define L3_MEM_SIZE_B   (L3_MEM_SIZE_MB*1024*1024)

#define CLUSTER_SIZE_B  (CLUSTER_SIZE_MB*1024*1024)
#define CLUSTERS_SIZE_B (N_CLUSTERS*CLUSTER_SIZE_B)

#define PULP_SIZE_B            0x10000000
#define SOC_PERIPHERALS_SIZE_B 0x50000
#define MBOX_SIZE_B            0x1000 // Interface 0 only

/*
 * System-Level Control Register
 */
#if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX
  #define SLCR_SIZE_B                 0x1000
  #define SLCR_FPGA_RST_CTRL_OFFSET_B 0x240
  #define SLCR_FPGA_OUT_RST           1
  #define SLCR_FPGA0_THR_CNT_OFFSET_B 0x178
  #define SLCR_FPGA0_THR_STA_OFFSET_B 0x17C
#endif

/*
 * Performance Monitor Unit
 */
#if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX
  #define ARM_PMU_CLK_DIV 64 // clock divider for clock cycle counter
#endif

/*
 * Clocking
 */
#define CLKING_CONFIG_REG_0_OFFSET_B  0x200
#define CLKING_CONFIG_REG_2_OFFSET_B  0x208
#define CLKING_CONFIG_REG_5_OFFSET_B  0x214
#define CLKING_CONFIG_REG_8_OFFSET_B  0x220
#define CLKING_CONFIG_REG_23_OFFSET_B 0x25C
#define CLKING_STATUS_REG_OFFSET_B    0x4

#if   PLATFORM == ZEDBOARD || PLATFORM == MINI_ITX || PLATFORM == ZC706
  #define ARM_CLK_FREQ_MHZ 666
#elif PLATFORM == TE0808
  #define ARM_CLK_FREQ_MHZ 1200
#else // JUNO
  #define ARM_CLK_FREQ_MHZ 1100 // A57 overdrive
#endif

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
 * RAB
 */
#define RAB_CONFIG_MAX_GAP_SIZE_B 0x1000 // one page

/*
 * IOCTL setup -- must match kernel-space side -- see pulp_module.h
 *
 * When defining a new IOCTL command, append a macro definition to the list below, using the
 * consecutively following command number, and increase the `PULP_IOC_NR_MAX` macro.
 */
#define PULP_IOCTL_MAGIC 'p'

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
 * Constants for profiling -- must match user-space side -- see pulp_func.h
 *
 * used for RAB profiling with profile_rab_striping & profile_rab_miss_handling
 */
#define MAX_STRIPE_SIZE_B 0x2000

// needed for profile_rab_striping & profile_rab_miss_handling
#define N_CYC_TOT_RESPONSE_OFFSET_B 0x00
#define N_CYC_TOT_UPDATE_OFFSET_B   0x04
#define N_CYC_TOT_SETUP_OFFSET_B    0x08
#define N_CYC_TOT_CLEANUP_OFFSET_B  0x0c
#define N_UPDATES_OFFSET_B          0x10
#define N_SLICES_UPDATED_OFFSET_B   0x14
#define N_PAGES_SETUP_OFFSET_B      0x18
#define N_CLEANUPS_OFFSET_B         0x1c

#define N_CYC_TOT_CACHE_FLUSH_OFFSET_B    0x20
#define N_CYC_TOT_GET_USER_PAGES_OFFSET_B 0x24
#define N_CYC_TOT_MAP_SG_OFFSET_B         0x28

#define N_MISSES_OFFSET_B           0x2C
#define N_FIRST_MISSES_OFFSET_B     0x30
#define N_CYC_TOT_SCHEDULE_OFFSET_B 0x34

#define PROFILE_RAB_N_UPDATES     100000
#define PROFILE_RAB_N_REQUESTS    100
#define PROFILE_RAB_N_ELEMENTS    (PROFILE_RAB_N_REQUESTS * 10)

/*
 * Macros
 */
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
 * RAB Macros -- must match kernel-space side -- see pulp_rab.h
 */
#define RAB_CONFIG_N_BITS_PORT 1
#define RAB_CONFIG_N_BITS_ACP  1
#define RAB_CONFIG_N_BITS_LVL  2
#define RAB_CONFIG_N_BITS_PROT 3
#define RAB_CONFIG_N_BITS_DATE 8

#define RAB_MAX_DATE    BIT_MASK_GEN(RAB_CONFIG_N_BITS_DATE)
#define RAB_MAX_DATE_MH (RAB_MAX_DATE-2)

#define RAB_GET_PROT(prot, request) ( prot = request & 0x7 )
#define RAB_SET_PROT(request, prot) \
  ( BF_SET(request, prot, 0, RAB_CONFIG_N_BITS_PROT) )

#define RAB_GET_ACP(use_acp, request) \
  ( use_acp = BF_GET(request, RAB_CONFIG_N_BITS_PROT, RAB_CONFIG_N_BITS_ACP) )
#define RAB_SET_ACP(request, use_acp) \
  ( BF_SET(request, use_acp, RAB_CONFIG_N_BITS_PROT, RAB_CONFIG_N_BITS_ACP) )

#define RAB_GET_PORT(port, request) \
  ( port = BF_GET(request, RAB_CONFIG_N_BITS_PROT + RAB_CONFIG_N_BITS_ACP, RAB_CONFIG_N_BITS_PORT) )
#define RAB_SET_PORT(request, port) \
  ( BF_SET(request, port, RAB_CONFIG_N_BITS_PROT + RAB_CONFIG_N_BITS_ACP, RAB_CONFIG_N_BITS_PORT) )

#define RAB_GET_LVL(rab_lvl, request) \
  ( rab_lvl = BF_GET(request, RAB_CONFIG_N_BITS_PROT + RAB_CONFIG_N_BITS_PORT \
           + RAB_CONFIG_N_BITS_ACP, RAB_CONFIG_N_BITS_LVL) )
#define RAB_SET_LVL(request, rab_lvl) \
  ( BF_SET(request, rab_lvl, RAB_CONFIG_N_BITS_PROT + RAB_CONFIG_N_BITS_PORT \
           + RAB_CONFIG_N_BITS_ACP, RAB_CONFIG_N_BITS_LVL) )

#define RAB_GET_DATE_EXP(date_exp, request) \
  ( date_exp = BF_GET(request, RAB_CONFIG_N_BITS_PROT + RAB_CONFIG_N_BITS_PORT \
           + RAB_CONFIG_N_BITS_ACP + RAB_CONFIG_N_BITS_LVL, RAB_CONFIG_N_BITS_DATE) )
#define RAB_SET_DATE_EXP(request, date_exp) \
  ( BF_SET(request, date_exp, RAB_CONFIG_N_BITS_PROT + RAB_CONFIG_N_BITS_PORT \
           + RAB_CONFIG_N_BITS_ACP + RAB_CONFIG_N_BITS_LVL, RAB_CONFIG_N_BITS_DATE) )

#define RAB_GET_DATE_CUR(date_cur, request) \
  ( date_cur = BF_GET(request, RAB_CONFIG_N_BITS_PROT + RAB_CONFIG_N_BITS_PORT \
           + RAB_CONFIG_N_BITS_DATE + RAB_CONFIG_N_BITS_ACP + RAB_CONFIG_N_BITS_LVL, RAB_CONFIG_N_BITS_DATE) )
#define RAB_SET_DATE_CUR(request, date_cur) \
  ( BF_SET(request, date_cur, RAB_CONFIG_N_BITS_PROT + RAB_CONFIG_N_BITS_PORT \
           + RAB_CONFIG_N_BITS_DATE + RAB_CONFIG_N_BITS_ACP + RAB_CONFIG_N_BITS_LVL, RAB_CONFIG_N_BITS_DATE) )

/*
 * App execution - used for some old apps like ROD, CT, JPEG
 */
//#define OLD_APPS
#ifdef OLD_APPS
  #define SYNC_OFFSET_B 0xB000
  #define PROFILE
  #define MEM_SHARING   2 // 1, 2 ,3
  #define ZYNQ_PMM
#endif // OLD_APPS

/*
 * Type Definitions - Part 1 -- must match kernel-space side -- see pulp_module.h
 */
typedef struct {
  unsigned short id;
  unsigned short n_elements;
  unsigned       rab_stripe_elem_user_addr; // 32b user-space addr of stripe element array
} RabStripeReqUser;

typedef enum {
  inout = 0,
  in    = 1,
  out   = 2,
} ElemType;

typedef struct {
  unsigned char id;
  ElemType type;
  unsigned char flags;
  unsigned      max_stripe_size_b;
  unsigned      n_stripes;
  unsigned      stripe_addr_start; // 32b user-space addr of addr_start array
  unsigned      stripe_addr_end;   // 32b user-space addr of addr_end array
} RabStripeElemUser;

/*
 * Type Definitions - Part 2
 */
typedef struct {
  unsigned *v_addr;
  size_t size;
} PulpSubDev;

typedef struct {
  int fd; // file descriptor
  PulpSubDev clusters;
  PulpSubDev soc_periph;
  PulpSubDev mbox;
  PulpSubDev l2_mem;
  PulpSubDev l3_mem;
  PulpSubDev gpio;
  PulpSubDev clking;
  PulpSubDev rab_config;
  PulpSubDev pulp_res_v_addr;
  PulpSubDev l3_mem_res_v_addr;
#if PLATFORM != JUNO
  PulpSubDev slcr;
#endif
  unsigned int l3_offset;         // used for pulp_l3_malloc
  unsigned int cluster_sel;       // cluster select
  unsigned int rab_ax_log_en;     // enable RAB AR/AW logger
  unsigned int intr_rab_miss_dis; // disable RAB miss interrupt to host
  unsigned int host_clk_freq_mhz;
  unsigned int pulp_clk_freq_mhz;
} PulpDev;

// striping informationg structure
typedef struct {
  unsigned n_stripes;
  unsigned first_stripe_size_b;
  unsigned last_stripe_size_b;
  unsigned stripe_size_b;
} StripingDesc;

typedef enum {
  copy          = 0x0, // no SVM, copy-based sharing using contiguous L3 memory
  svm_static    = 0x1, // SVM, set up static mapping at offload time, might fail - use with caution
  svm_stripe    = 0x2, // SVM, use striping (L1 only), might fail - use with caution
  svm_mh        = 0x3, // SVM, use miss handling
  copy_tryx     = 0x4, // no SVM, copy-based sharing using contiguous L3 memory, but let PULP do the tryx()
  svm_smmu      = 0x5, // SVM, use SMMU instead of RAB
  svm_smmu_shpt = 0x6, // SVM, use SMMU, emulate sharing of user-space page table, no page faults
  custom        = 0xF, // do not touch (custom marshalling)
} ShMemType;

typedef enum {
  val            = 0x0,  // pass by value

  ref_copy       = 0x10, // pass by reference, no SVM, use contiguous L3 memory
  ref_svm_static = 0x11, // pass by reference, SVM, set up mapping at offload time
  ref_svm_stripe = 0x12, // pass by reference, SVM, set up striped mapping
  ref_svm_mh     = 0x13, // pass by reference, SVM, do not set up mapping, use miss handling
  ref_copy_tryx  = 0x14, // pass by reference, no SVM, use contiguous L3 memory, but to the tryx() - mapped to 0x10
  ref_custom     = 0x1F, // pass by reference, do not touch (custom marshalling)
} ElemPassType;

// shared variable data structure
typedef struct {
  void         * ptr;         // address in host virtual memory
  void         * ptr_l3_v;    // host virtual address in contiguous L3 memory   - filled by runtime library based on sh_mem_ctrl
  void         * ptr_l3_p;    // PULP physical address in contiguous L3 memory  - filled by runtime library based on sh_mem_ctrl
  size_t         size;        // size in Bytes
  ElemType       type;        // inout, in, out
  ShMemType      sh_mem_ctrl;
  unsigned char  cache_ctrl;  // 0: flush caches, access through DRAM
                              // 1: do not flush caches, access through caches
  unsigned char  rab_lvl;     // 0: first L1, L2 when full
                              // 1: L1 only
                              // 2: L2 only
  StripingDesc * stripe_desc; // only used if sh_mem_ctrl = 2
} DataDesc;

// task descriptor created by the compiler
typedef struct {
  int        task_id; // used for RAB managment -> expiration date
  char     * name;
  int        n_clusters;
  int        n_data;
  DataDesc * data_desc;
} TaskDesc;

/*
 * Method declarations
 */
int  pulp_reserve_v_addr(PulpDev *pulp);
int  pulp_free_v_addr(const PulpDev *pulp);
void pulp_print_v_addr(PulpDev *pulp);

int  pulp_read32(const unsigned *base_addr, unsigned off, char off_type);
void pulp_write32(unsigned *base_addr, unsigned off, char off_type, unsigned value);

int pulp_mmap(PulpDev *pulp);
int pulp_munmap(PulpDev *pulp);
int pulp_init(PulpDev *pulp);

int pulp_mbox_read(const PulpDev *pulp, unsigned *buffer, unsigned n_words);
int pulp_mbox_write(PulpDev *pulp, unsigned word);
void pulp_mbox_clear_is(PulpDev *pulp);

int pulp_clking_set_freq(PulpDev *pulp, unsigned des_freq_mhz);
int pulp_clking_measure_freq(PulpDev *pulp);

int pulp_rab_req(const PulpDev *pulp, unsigned addr_start, unsigned size_b,
                 unsigned char prot, unsigned char port,
                 unsigned char date_exp, unsigned char date_cur,
                 unsigned char use_acp, unsigned char rab_lvl);
void pulp_rab_free(const PulpDev *pulp, unsigned char date_cur);

int pulp_rab_req_striped(const PulpDev *pulp, const TaskDesc *task,
                         ElemPassType **pass_type, int n_elements);
void pulp_rab_free_striped(const PulpDev *pulp);

int  pulp_rab_mh_enable(const PulpDev *pulp, unsigned char use_acp, unsigned char rab_mh_lvl);
void pulp_rab_mh_disable(const PulpDev *pulp);

int pulp_rab_soc_mh_enable(const PulpDev* pulp, const unsigned static_2nd_lvl_slices);
int pulp_rab_soc_mh_disable(const PulpDev* pulp);

int pulp_rab_ax_log_read(const PulpDev* pulp);

int pulp_smmu_enable(const PulpDev* pulp, const unsigned char flags);
int pulp_smmu_disable(const PulpDev *pulp);

int pulp_dma_xfer(const PulpDev *pulp,
                  unsigned addr_l3, unsigned addr_pulp, unsigned size_b,
                  unsigned host_read);

int pulp_omp_offload_task(PulpDev *pulp, TaskDesc *task);

void pulp_reset(PulpDev *pulp, unsigned full);
int  pulp_boot(PulpDev *pulp, const TaskDesc *task);

int  pulp_load_bin(PulpDev *pulp, const char *name);
int  pulp_load_bin_from_mem(PulpDev *pulp, void *ptr, unsigned size);
void pulp_exe_start(PulpDev *pulp);
void pulp_exe_stop(PulpDev *pulp);
int  pulp_exe_wait(const PulpDev *pulp, int timeout_s);

unsigned int pulp_l3_malloc(PulpDev *pulp, size_t size_b, unsigned *p_addr);
void         pulp_l3_free(PulpDev *pulp, unsigned v_addr, unsigned p_addr);

int pulp_offload_get_pass_type(const TaskDesc *task, ElemPassType **pass_type);
int pulp_offload_rab_setup(const PulpDev *pulp, const TaskDesc *task, ElemPassType **pass_type, int n_ref);
int pulp_offload_l3_copy_raw_out(PulpDev *pulp, TaskDesc *task, const ElemPassType **pass_type);
int pulp_offload_l3_copy_raw_in(PulpDev *pulp, const TaskDesc *task, const ElemPassType **pass_type);
int pulp_offload_pass_desc(PulpDev *pulp, const TaskDesc *task, const ElemPassType **pass_type);
int pulp_offload_get_desc(const PulpDev *pulp, TaskDesc *task, const ElemPassType **pass_type);

int pulp_offload_out(PulpDev *pulp, TaskDesc *task);
int pulp_offload_in(PulpDev *pulp, TaskDesc *task);

#ifdef OLD_APPS
  int pulp_offload_out_contiguous(PulpDev *pulp, TaskDesc *task, TaskDesc **ftask);
  int pulp_offload_in_contiguous(PulpDev *pulp, TaskDesc *task, TaskDesc **ftask);

  int pulp_offload_start(PulpDev *pulp, const TaskDesc *task);
  int pulp_offload_wait(const PulpDev *pulp, const TaskDesc *task);
#endif // OLD_APPS

// for random_forest
int pulp_rab_req_striped_mchan_img(const PulpDev *pulp, unsigned char prot, unsigned char port,
                                   unsigned p_height, unsigned p_width, unsigned i_step,
                                   unsigned n_channels, unsigned char **channels,
                                   unsigned *s_height);

#endif // PULP_FUNC_H__
