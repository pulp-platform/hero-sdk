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
#ifndef _PULP_SMMU_H_
#define _PULP_SMMU_H_

#include <linux/iommu.h>    /* iommu stuff */

#include "pulp_module.h"

#define PULP_SMMU_GLOBAL_BYPASS // enable global SMMU bypassing for maximum bypass performance

// constants
#define SMMU_GR0_OFFSET_B       0x0
#define SMMU_SMR_OFFSET_B       0x800
#define SMMU_S2CR_OFFSET_B      0xC00
#define SMMU_CBAR_OFFSET_B      0x1000

#define SMMU_CB_SCTLR_OFFSET_B  0x0
#define SMMU_CB_RESUME_OFFSET_B 0x8
#define SMMU_CB_MAIR0_OFFSET_B  0x38
#define SMMU_CB_MAIR1_OFFSET_B  0x3C
#define SMMU_CB_FSR_OFFSET_B    0x58

#define SMMU_GR0_CLIENTPD       0
#define SMMU_GR0_TRANSIENTCFG   6
#define SMMU_GR0_MEMATTR        16
#define SMMU_GR0_MTCFG          20
#define SMMU_GR0_SHCFG          22
#define SMMU_GR0_RACFG          24
#define SMMU_GR0_WACFG          26
#define SMMU_GR0_NSCFG          28

#define SMMU_S2CR_SHCFG         8
#define SMMU_S2CR_MTCFG         11
#define SMMU_S2CR_MEMATTR       12
#define SMMU_S2CR_TYPE          16
#define SMMU_S2CR_NSCFG         18
#define SMMU_S2CR_RACFG         20
#define SMMU_S2CR_WACFG         22
#define SMMU_S2CR_TRANSIENTCFG  28

#define SMMU_CBAR_BPSHCFG       8

#define SMMU_CB_FSR_TF          1

#define SMMU_SCTLR_CFIE        (1 << 6)

#define SMMU_N_BITS_STREAM_ID   14

#define SMMU_FLAGS_CC           0b00000001
#define SMMU_FLAGS_SHPT_EMU     0b00000010

// type definitions
typedef enum {
  READY = 0,
  WAIT  = 1,
} WorkerStatus;

typedef struct SmmuPage SmmuPage;
struct SmmuPage {
  unsigned long     iova;
  struct page *     page_ptr;
  struct SmmuPage * previous;
};

// methods declarations
int pulp_smmu_init(PulpDev * pulp_ptr);

int pulp_smmu_ena(PulpDev * pulp_ptr, unsigned flags);
int pulp_smmu_dis(PulpDev * pulp_ptr);

int pulp_smmu_fh_sched(struct iommu_domain *smmu_domain_ptr, struct device *dev_ptr,
                       unsigned long iova, int flags, void * smmu_token_ptr);
void pulp_smmu_handle_fault(void);

#endif/*_PULP_SMMU_H_*/
