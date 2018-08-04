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
#include "zynq_pmm_user.h"

/*
 * Open the device
 */
int zynq_pmm_open(int *fd){
  *fd = open("/dev/ZYNQ_PMM", O_RDONLY);
  if (*fd < 0) {
    printf("Opening failed \n");
    return ENOENT;
  }
  return 0;
}

/*
 * Close the device
 */
void zynq_pmm_close(int *fd){
  close(*fd);
}

/*
 * Read the device
 */
int zynq_pmm_read(int *fd, char *buffer){

  int n_char, n_char_left, ret;
  ret = 1;
  n_char = 0;
  n_char_left = ZYNQ_PMM_PROC_N_CHARS_MAX;

  // read until eof
  while (ret > 0 ) {
    ret = read(*fd, &buffer[n_char],n_char_left*sizeof(char));
    if (ret < 0) {
      printf("Error reading file\n");
      return ret;
    }
    n_char += ret;
    n_char_left -= ret;
  }

  return 0;
}

/*
 * Parse the string
 */
int zynq_pmm_parse(char *buffer, long long *counters, int accumulate){

  int i,j;
  int long number;
  char temp_buffer[100];
  char identifier[10];
  char *position;

  // reset accumulation counters
  if (accumulate == -1) {
    for (i=0; i<N_ARM_CORES+1; i++) {
      for (j=0;j<2;j++) {
        counters[i*2+j] = 0;
      }
    }
    return 0;
  }

  // Copy
  for (i=0; i<N_ARM_CORES+1; i++) {
    // search for identifiers
    if (i < N_ARM_CORES)
      sprintf(identifier,"L1 Core %i",i);
    else
      sprintf(identifier,"| L2");
    position = strstr(buffer,identifier);
    if (position == NULL) {
      printf("Error: Identifier '%s' not found.\n",identifier);
      return -1;
    }
    strncpy(temp_buffer,position,100);

    // extract the individual numbers
    position = strtok(temp_buffer,"|");
    if (position == NULL) {
      printf("Error: '|' not found in remaining string.\n");
      return -1;
    }
    for (j=0;j<2;j++) {
      // next
      position = strtok(NULL,"|");
      if (position == NULL) {
        printf("Error: '|' not found in remaining string.\n");
        return -1;
      }
      // remove leading white spaces
      while (isspace(*position))
        position++;
        //*position++;
      // convert to number
      number = strtol(position,NULL,10);
      // save
      if (!accumulate)
        counters[i*2+j] = (long long)number;
      else //(accumulate == 1)
        counters[i*2+j] += (long long)number;
    }

    // check for WARNING = overflow
    position = strstr(buffer,"WARNING");
    if (position != NULL) {
      printf("%s",position);
    }
  }

  return 0;
}

/*
 * Compute the miss rates
 */
int zynq_pmm_compute_rates(double *miss_rates, long long *counters) {
  int i;
  double n_misses, n_accesses;

  // local miss rates
  for (i=0;i<N_ARM_CORES+1;i++) {
    n_misses = (double)counters[i*2];
    n_accesses = (double)counters[i*2+1];
    if (!n_accesses)
      miss_rates[i] = 0;
    else
      miss_rates[i] = n_misses/n_accesses;
  }

  // global miss rates
  // mr1_x * mr2
  for (i=0;i<N_ARM_CORES;i++) {
    miss_rates[N_ARM_CORES+1+i] = miss_rates[N_ARM_CORES] * miss_rates[i];
  }

  // total global miss rate
  // mr2 * (m1_0 + m1_1)/(a1_0 + a1_1)
  n_misses = 0;
  n_accesses = 0;
  for (i=0;i<N_ARM_CORES;i++) {
    n_misses += (double)counters[i*2];
    n_accesses += (double)counters[i*2+1];
  }
  if (!n_accesses)
    miss_rates[(N_ARM_CORES+1)*2-1] = miss_rates[N_ARM_CORES];
  else
    miss_rates[(N_ARM_CORES+1)*2-1] = miss_rates[N_ARM_CORES] * n_misses/n_accesses;

  return 0;
}

/*
 * Print the  miss rates
 */
int zynq_pmm_print_rates(double *miss_rates) {
  int i;

  printf("----- -----\n");
  printf("Local cache miss rates:\n");
  for (i=0;i<N_ARM_CORES;i++) {
    printf("L1 Core %i: %.3f \n",i,miss_rates[i]);
  }
  printf("L2: %.3f \n",miss_rates[N_ARM_CORES]);

  printf("-----\n");
  printf("Global cache miss rates:\n");
  for (i=0;i<N_ARM_CORES;i++) {
    printf("Core %i: %.3f \n",i,miss_rates[N_ARM_CORES+1+i]);
  }
  printf("-----\n");
  printf("Total global cache miss rate: %.3f\n",miss_rates[(N_ARM_CORES+1)*2-1]);
  printf("-----\n");

  return 0;
}

/*
 * Get the overhead in clk cycles for reading the cntr value
 */
unsigned arm_clk_cntr_get_overhead(){
  unsigned n_trials,start,acc,overhead;

  n_trials = 32;
  acc = 0;

  // Reset the counter
  arm_clk_cntr_reset();

  start = arm_clk_cntr_read();

  // Read the counter n_trials times
  arm_clk_cntr_read();
  arm_clk_cntr_read();
  arm_clk_cntr_read();

  arm_clk_cntr_read();
  arm_clk_cntr_read();
  arm_clk_cntr_read();

  arm_clk_cntr_read();
  arm_clk_cntr_read();
  arm_clk_cntr_read();
  arm_clk_cntr_read();

  arm_clk_cntr_read();
  arm_clk_cntr_read();
  arm_clk_cntr_read();
  arm_clk_cntr_read();

  arm_clk_cntr_read();
  arm_clk_cntr_read();
  arm_clk_cntr_read();
  arm_clk_cntr_read();

  arm_clk_cntr_read();
  arm_clk_cntr_read();
  arm_clk_cntr_read();
  arm_clk_cntr_read();

  arm_clk_cntr_read();
  arm_clk_cntr_read();
  arm_clk_cntr_read();
  arm_clk_cntr_read();

  arm_clk_cntr_read();
  arm_clk_cntr_read();
  arm_clk_cntr_read();

  acc = arm_clk_cntr_read();

  // Compute the overhead
  overhead = (unsigned)((float)((acc - start)*ARM_PMU_CLK_DIV) / (float)n_trials);

  return overhead;
}
