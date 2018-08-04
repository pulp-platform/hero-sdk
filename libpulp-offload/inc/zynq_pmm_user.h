/* Copyright (C) 2017 ETH Zurich, University of Bologna
 * All rights reserved.
 *
 * This code is under development and not yet released to the public.
 * Until it is released, the code is under the copyright of ETH Zurich and
 * the University of Bologna, and may contain confidential and/or unpublished 
 * work. Any reuse/redistribution is strictly forbidden without written
 * permission from ETH Zurich.
 *
 * Bug fixes and contributions will eventually be released under the
 * SolderPad open hardware license in the context of the PULP platform
 * (http://www.pulp-platform.org), under the copyright of ETH Zurich and the
 * University of Bologna.
 */
#ifndef ZYNQ_PMM_USER__
#define ZYNQ_PMM_USER__

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h> // for error codes
#include <unistd.h>
#include <math.h>

/*
 * Constants
 */
#define N_ARM_CORES 2

#define ZYNQ_PMM_PROC_N_CHARS_MAX 1000

#define ARM_PMU_CLK_DIV 64 // clock divider for clock cycle counter

/*
 * Method declarations
 */
int zynq_pmm_open(int *fd);
void zynq_pmm_close(int *fd);

int zynq_pmm_read(int *fd, char *buffer);
int zynq_pmm_parse(char *buffer, long long *counters, int accumulate);
int zynq_pmm_compute_rates(double *miss_rates, long long *counters);
int zynq_pmm_print_rates(double *miss_rates);

unsigned arm_clk_cntr_get_overhead();

/*
 * Reset the clock cycle counter in the PMU
 */
inline void arm_clk_cntr_reset()
{
  // enable clock counter divider (by 64), reset & enable clock counter, PMCR register
  asm volatile("mcr p15, 0, %0, c9, c12, 0" :: "r"(0xD));
}

/*
 * Read the clock cycle counter in the PMU
 */
inline unsigned arm_clk_cntr_read()
{
  unsigned value;
  // Read the counter value, PMCCNTR register
  asm volatile("mrc p15, 0, %0, c9, c13, 0" : "=r"(value) : );

  return value;
}

#endif // ZYNQ_PMM_USER__
