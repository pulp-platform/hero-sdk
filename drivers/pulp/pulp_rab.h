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
#ifndef _PULP_RAB_H_
#define _PULP_RAB_H_

#include <linux/module.h>  /* Needed by all modules */
#include <linux/kernel.h>  /* KERN_ALERT, container_of */
#include <linux/mm.h>      /* vm_area_struct struct, page struct, PAGE_SHIFT, pageo_phys */
#include <linux/pagemap.h> /* page_put() */
#include <linux/slab.h>    /* kmalloc() */
#include <asm/io.h>        /* ioremap, iounmap, iowrite32 */
#include <linux/delay.h>   /* udelay */
#include <linux/vmalloc.h>
#include <linux/types.h>   /* uint32_t */

#include "pulp_module.h"

/*
 * Constants
 */
#define RAB_N_PORTS              2
#define RAB_L1_N_MAPPINGS_PORT_1 2

#if   PLATFORM == ZEDBOARD
  #define RAB_L1_N_SLICES_PORT_0  4
  #define RAB_L1_N_SLICES_PORT_1  8
  #define RAB_MH_FIFO_DEPTH       8
#else
  #define RAB_L1_N_SLICES_PORT_0  4
  #define RAB_L1_N_SLICES_PORT_1 32
  #define RAB_MH_FIFO_DEPTH      64
#endif

// Specify for each of the RAB_N_PORTS if L2 is active on that port: {Port 0, Port 1}.
#if   PLATFORM == ZEDBOARD
  static const unsigned RAB_L2_EN_ON_PORT[RAB_N_PORTS] = {0, 0};
#else
  static const unsigned RAB_L2_EN_ON_PORT[RAB_N_PORTS] = {0, 1};
#endif

#define RAB_L2_N_ENTRIES_PER_SET 32
#define RAB_L2_N_SETS            32

#if PLATFORM == JUNO || PLATFORM == TE0808
  #define RAB_N_STATIC_2ND_LEVEL_SLICES (1 << (32 - PGDIR_SHIFT))
#endif

#define RAB_FLAGS_DRV_CONST   0b00000001 // const mapping
#define RAB_FLAGS_DRV_STRIPED 0b00000010 // striped mapping
#define RAB_FLAGS_DRV_EVERY   0b00000100 // set up in every RAB mapping

#define RAB_FLAGS_HW_EN       0b00000001 // enable mapping
#define RAB_FLAGS_HW_READ     0b00000010 // enable read
#define RAB_FLAGS_HW_WRITE    0b00000100 // enable write
#define RAB_FLAGS_HW_CC       0b00001000 // cache-coherent mapping

#define RAB_SLICE_SIZE_B               0x20
#define RAB_SLICE_BASE_OFFSET_B        0x20
#define RAB_SLICE_ADDR_START_OFFSET_B  0x0
#define RAB_SLICE_ADDR_END_OFFSET_B    0x8
#define RAB_SLICE_ADDR_OFFSET_OFFSET_B 0x10
#define RAB_SLICE_FLAGS_OFFSET_B       0x18

#define RAB_MH_ADDR_FIFO_OFFSET_B 0x0
#define RAB_MH_META_FIFO_OFFSET_B 0x8

#define RAB_WAKEUP_OFFSET_B       0x0

#define AXI_ID_WIDTH_CORE    4
#define AXI_ID_WIDTH_CLUSTER 2
#if PLATFORM == JUNO
  #define AXI_ID_WIDTH_SOC   3
#else // !JUNO
  #define AXI_ID_WIDTH_SOC   1
#endif
#define AXI_ID_WIDTH         (AXI_ID_WIDTH_CORE + AXI_ID_WIDTH_CLUSTER + AXI_ID_WIDTH_SOC)

#define AXI_USER_WIDTH       6

#define RAB_AX_LOG_PRINT_FORMAT 0 // 0 = DEBUG, 1 = MATLAB

/*
 * Constants for profiling -- must match user-space side -- see pulp_func.h
 */
#if defined(PROFILE_RAB_STR) || defined(PROFILE_RAB_MH)
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
#endif

/*
 * Macros -- must match user-space side -- see pulp_func.h
 */
#define RAB_CONFIG_N_BITS_PORT 1
#define RAB_CONFIG_N_BITS_ACP  1
#define RAB_CONFIG_N_BITS_LVL  2
#define RAB_CONFIG_N_BITS_PROT 3
#define RAB_CONFIG_N_BITS_DATE 8

#define RAB_MAX_DATE    BIT_MASK_GEN(RAB_CONFIG_N_BITS_DATE)
#define RAB_MAX_DATE_MH (RAB_MAX_DATE-2)

#define RAB_GET_FLAGS_HW(flags_hw, request) \
  ( flags_hw = BF_GET(request, 0, RAB_CONFIG_N_BITS_PROT + RAB_CONFIG_N_BITS_ACP) )
#define RAB_SET_FLAGS_HW(request, flags_hw) \
  ( BF_SET(request, flags_hw, 0, RAB_CONFIG_N_BITS_PROT + RAB_CONFIG_N_BITS_ACP) )

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

#define RAB_UPDATE_GET_ELEM(elem_mask, request) \
  ( elem_mask = BF_GET(request, 0, RAB_UPDATE_N_BITS_ELEM) )
#define RAB_UPDATE_GET_TYPE(type, request) \
  ( type = BF_GET(request, RAB_UPDATE_N_BITS_ELEM, RAB_UPDATE_N_BITS_TYPE) )

/*
 * Type Definitions - Part 1 -- must match user-space side -- see pulp_func.h
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
  // management
  unsigned char date_cur;
  unsigned char date_exp;
  unsigned char page_ptr_idx;
  unsigned char page_idx_start;
  unsigned char page_idx_end;
  unsigned char rab_port;  
  unsigned      rab_mapping;
  unsigned      rab_slice;
  unsigned char flags_drv; // FLAGS_DRV
  // actual config
  unsigned      addr_start;
  unsigned      addr_end;
  unsigned long addr_offset;
  unsigned char flags_hw;  // FLAGS_HW
} RabSliceReq;

// Stripe request structs - kernel space 
typedef struct {
  unsigned      addr_start;
  unsigned      addr_end;
  unsigned long addr_offset;
} RabStripeSlice;

typedef struct {
  unsigned         n_slices;      // number of slices used for that stripe 
  RabStripeSlice * slice_configs; // ptr to array of slice configs
} RabStripe;

typedef struct {
  // management
  unsigned char id;
  ElemType      type;
  unsigned char page_ptr_idx;
  unsigned      n_slices;            // number of slices allocated
  unsigned      n_slices_per_stripe; // number of slices used per stripe
  unsigned *    slice_idxs;          // ptr to array containing idxs of allocated slices 
  unsigned      set_offset;          // offset in mapping of stripes to sets of allocated slices, may change on wrap around
  // actual config
  unsigned      stripe_idx;          // idx next stripe to configure
  unsigned      n_stripes; 
  RabStripe *   stripes;             // ptr to array of stripe structs
  unsigned char flags_hw;
} RabStripeElem;

typedef struct {
  unsigned short  id;
  unsigned short  n_elements;
  RabStripeElem * elements;
} RabStripeReq;

// L1 TLB structs
typedef struct {
  // management
  unsigned char date_cur;
  unsigned char date_exp;
  // actual config
  unsigned      addr_start;
  unsigned      addr_end;
  unsigned long addr_offset;
  unsigned char flags_hw;
} L1EntryPort0;

typedef struct {
  L1EntryPort0 slices[RAB_L1_N_SLICES_PORT_0];
} L1TlbPort0;

typedef struct {
  // management
  unsigned char date_cur;
  unsigned char date_exp;
  unsigned char page_ptr_idx;
  unsigned char page_idx_start;
  unsigned char page_idx_end;
  unsigned char flags_drv;
  // actual config
  unsigned      addr_start;
  unsigned      addr_end;
  unsigned long addr_offset;
  unsigned char flags_hw;
} L1EntryPort1;

typedef struct {
  L1EntryPort1   slices            [RAB_L1_N_SLICES_PORT_1];
  struct page ** page_ptrs         [RAB_L1_N_SLICES_PORT_1];
  unsigned       page_ptr_ref_cntrs[RAB_L1_N_SLICES_PORT_1];
} L1TlbMappingPort1;

typedef struct {
  L1TlbMappingPort1 mappings[RAB_L1_N_MAPPINGS_PORT_1];
  unsigned          mapping_active;
} L1TlbPort1;

typedef struct {
  L1TlbPort0 port_0;
  L1TlbPort1 port_1;
} L1Tlb;

// L2 TLB structs
typedef struct {
  unsigned char flags; 
  unsigned      pfn_v; 
  unsigned      pfn_p;    // the HW supports PFN widths up to 32 bit, (max 44-bit physical address witdh)
  struct page * page_ptr;   
} L2Entry;

typedef struct {
  L2Entry       entry[RAB_L2_N_ENTRIES_PER_SET];
  unsigned char next_entry_idx;
  unsigned char is_full;
} L2Set;

typedef struct {
  L2Set set[RAB_L2_N_SETS];
} L2Tlb;

/*
 * Method declarations
 */
int pulp_rab_init(PulpDev * pulp_ptr);
int pulp_rab_release(void);

void pulp_rab_l1_init(void);
int  pulp_rab_page_ptrs_get_field(RabSliceReq *rab_slice_req);
int  pulp_rab_slice_check(RabSliceReq *rab_slice_req);
int  pulp_rab_slice_get(RabSliceReq *rab_slice_req);
void pulp_rab_slice_free(void *rab_config, RabSliceReq *rab_slice_req);
int  pulp_rab_slice_setup(void *rab_config, RabSliceReq *rab_slice_req, struct page **pages);
int  pulp_rab_num_free_slices(RabSliceReq *rab_slice_req);
int  pulp_rab_mapping_get_active(void);
void pulp_rab_mapping_switch(void *rab_config, unsigned rab_mapping);
void pulp_rab_mapping_print(void *rab_config, unsigned rab_mapping);

void pulp_rab_l2_init(void *rab_config);
void pulp_rab_l2_clear_hw(void *rab_config, unsigned char port);
int  pulp_rab_l2_setup_entry(void *rab_config, L2Entry *tlb_entry, char port, char enable_replace);
int  pulp_rab_l2_check_availability(L2Entry *tlb_entry, char port);
int  pulp_rab_l2_invalidate_all_entries(void *rab_config, char port);
int  pulp_rab_l2_invalidate_entry(void *rab_config, char port, int set_num, int entry_num);
int  pulp_rab_l2_print_all_entries(char port);
int  pulp_rab_l2_print_valid_entries(char port);

long pulp_rab_req(void *rab_config, unsigned long arg);
long pulp_rab_req_striped(void *rab_config, unsigned long arg);
void pulp_rab_free(void *rab_config, unsigned long arg);
void pulp_rab_free_striped(void *rab_config, unsigned long arg);

void pulp_rab_update(unsigned update_req);
void pulp_rab_switch(void);

int pulp_rab_soc_mh_ena(void* const rab_config, unsigned static_2nd_lvl_slices);
int pulp_rab_soc_mh_dis(void* const rab_config);

long     pulp_rab_mh_ena(void *rab_config, unsigned long arg);
void     pulp_rab_mh_dis(void);
unsigned pulp_rab_mh_sched(void);
void     pulp_rab_handle_miss(unsigned unused);

#if RAB_AX_LOG_EN == 1
  int  pulp_rab_ax_log_init(void);
  void pulp_rab_ax_log_free(void);
  void pulp_rab_ax_log_read(unsigned gpio_value, unsigned clear);
  void pulp_rab_ax_log_print(void);
  void pulp_rab_ax_log_to_user(unsigned long arg);
#endif

#if defined(PROFILE_RAB_STR) || defined(PROFILE_RAB_MH)
  int  pulp_rab_prof_init(void);
  void pulp_rab_prof_free(void);
  void pulp_rab_prof_print(void);
#endif

#endif/*_PULP_RAB_H_*/
