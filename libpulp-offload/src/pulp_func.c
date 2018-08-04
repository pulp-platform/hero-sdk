/*
 * PULP user-space runtime library
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
#include <stdio.h>      // fclose(), fopen(), printf(), sprintf()
#include <stdlib.h>     // free(), malloc()
#include <sys/ioctl.h>  // ioctl()
#include <time.h>       // struct tm, localtime(), time(), time_t
#include <sys/stat.h>   // fstat()
#include <errno.h>

#include "pulp_func.h"

//printf("%s %d\n",__FILE__,__LINE__);

/**
 * Reserve the virtual address space overlapping with the physical
 * address map of pulp using the mmap() syscall with MAP_FIXED and
 * MAP_ANONYMOUS
 *
 * @pulp: pointer to the PulpDev structure
 */
int pulp_reserve_v_addr(PulpDev *pulp)
{
  pulp->pulp_res_v_addr.size = PULP_SIZE_B;
  pulp->pulp_res_v_addr.v_addr = mmap((int *)ARCHI_CLUSTER_GLOBAL_ADDR(0),pulp->pulp_res_v_addr.size,
                                      PROT_NONE,MAP_PRIVATE | MAP_FIXED | MAP_ANONYMOUS,-1,0);
  if (pulp->pulp_res_v_addr.v_addr == MAP_FAILED) {
    printf("MMAP failed to reserve virtual addresses overlapping with physical address map of PULP.\n");
    return -EIO;
  }
  else if (DEBUG_LEVEL > 0) {
    printf("Reserved virtual addresses starting at %p and overlapping with physical address map of PULP. \n",
           pulp->pulp_res_v_addr.v_addr);
  }

  pulp->l3_mem_res_v_addr.size = L3_MEM_SIZE_B;
  pulp->l3_mem_res_v_addr.v_addr = mmap((int *)L3_MEM_BASE_ADDR,pulp->l3_mem_res_v_addr.size,
                                        PROT_NONE,MAP_PRIVATE | MAP_FIXED | MAP_ANONYMOUS,-1,0);
  if (pulp->l3_mem_res_v_addr.v_addr == MAP_FAILED) {
    printf("MMAP failed to reserve virtual addresses overlapping with physically contiguous L3 memory.\n");
    return -EIO;
  }
  else if (DEBUG_LEVEL > 0) {
    printf("Reserved virtual addresses starting at %p and overlapping with physically contiguous L3 memory.\n",
           pulp->l3_mem_res_v_addr.v_addr);
  }

  return 0;
}

/**
 * Free the virtual address space overlapping with the physical
 * address map of pulp using the munmap() syscall
 *
 * @pulp: pointer to the PulpDev structure
 */
int pulp_free_v_addr(const PulpDev *pulp)
{
  int status;

  if (DEBUG_LEVEL > 0)
    printf("Freeing reserved virtual addresses overlapping with physical address map of PULP.\n");
  status = munmap(pulp->pulp_res_v_addr.v_addr,pulp->pulp_res_v_addr.size);
  if (status) {
    printf("MUNMAP failed to free reserved virtual addresses overlapping with physical address map of PULP.\n");
  }

#if PLATFORM != JUNO
  if (DEBUG_LEVEL > 0)
    printf("Freeing reserved virtual addresses overlapping with with physically contiguous L3 memory.\n");
  status = munmap(pulp->l3_mem_res_v_addr.v_addr,pulp->l3_mem_res_v_addr.size);
  if (status) {
    printf("MUNMAP failed to free reserved virtual addresses overlapping with physically contiguous L3 memory.\n");
  }
#endif

  return 0;
}

/**
 * Print information about the reserved virtual memory on the host
 */
void pulp_print_v_addr(PulpDev *pulp)
{
  // check the reservation
  printf("\nMemory map of the process:\n");
  printf("# cat /proc/getpid()/maps\n");

  char cmd[20];
  sprintf(cmd,"cat /proc/%i/maps",getpid());
  system(cmd);

  // check wether the reservation contributes to the kernels overcommit accounting -> Committed_AS
  printf("\nInformation about the system's memory:\n");
  printf("# cat /proc/meminfo\n");
  system("cat /proc/meminfo");

  return;
}

/**
 * Read 32 bits
 *
 * @base_addr : virtual address pointer to base address
 * @off       : offset
 * @off_type  : type of the offset, 'b' = byte offset, else word offset
 */
int pulp_read32(const unsigned *base_addr, unsigned off, char off_type)
{
  if (DEBUG_LEVEL > 4) {
    const unsigned *addr;
    if (off_type == 'b')
      addr = base_addr + (off>>2);
    else
      addr = base_addr + off;
    printf("Reading from %p\n",addr);
  }
  if (off_type == 'b')
    return *(base_addr + (off>>2));
  else
    return *(base_addr + off);
}

/**
 * Write 32 bits
 *
 * @base_addr : virtual address pointer to base address
 * @off       : offset
 * @off_type  : type of the offset, 'b' = byte offset, else word offset
 */
void pulp_write32(unsigned *base_addr, unsigned off, char off_type, unsigned value)
{
  if (DEBUG_LEVEL > 4) {
    unsigned *addr;
    if (off_type == 'b')
      addr = base_addr + (off>>2);
    else
      addr = base_addr + off;
    printf("Writing to %p\n",addr);
  }
  if (off_type == 'b')
    *(base_addr + (off>>2)) = value;
  else
    *(base_addr + off) = value;
}

/**
 * Memory map the device to virtual user space using the mmap()
 * syscall
 *
 * @pulp: pointer to the PulpDev structure
 */
int pulp_mmap(PulpDev *pulp)
{
  int offset;

  /*
   *  open the device
   */
  pulp->fd = open("/dev/PULP", O_RDWR | O_SYNC);
  if (pulp->fd < 0) {
    printf("ERROR: Opening failed \n");
    return -ENOENT;
  }

  /*
   *  do the different remappings
   */
  // PULP internals
  // Clusters
  offset = 0; // start of clusters
  pulp->clusters.size = CLUSTERS_SIZE_B;

  pulp->clusters.v_addr = mmap(NULL,pulp->clusters.size,
                               PROT_READ | PROT_WRITE,MAP_SHARED,pulp->fd,offset);
  if (pulp->clusters.v_addr == MAP_FAILED) {
    printf("MMAP failed for clusters.\n");
    return -EIO;
  }
  else if (DEBUG_LEVEL > 0) {
    printf("Clusters mapped to virtual user space at %p.\n",pulp->clusters.v_addr);
  }

  // SOC_PERIPHERALS
  offset = CLUSTERS_SIZE_B; // start of peripherals
  pulp->soc_periph.size = SOC_PERIPHERALS_SIZE_B;

  pulp->soc_periph.v_addr = mmap(NULL,pulp->soc_periph.size,
                                 PROT_READ | PROT_WRITE,MAP_SHARED,pulp->fd,offset);
  if (pulp->soc_periph.v_addr == MAP_FAILED) {
    printf("MMAP failed for SoC peripherals.\n");
    return -EIO;
  }
  else if (DEBUG_LEVEL > 0) {
    printf("SoC peripherals mapped to virtual user space at %p.\n",pulp->soc_periph.v_addr);
  }

  // Mailbox
  offset = CLUSTERS_SIZE_B + SOC_PERIPHERALS_SIZE_B; // start of mailbox
  pulp->mbox.size = MBOX_SIZE_B;

  pulp->mbox.v_addr = mmap(NULL,pulp->mbox.size,
                              PROT_READ | PROT_WRITE,MAP_SHARED,pulp->fd,offset);
  if (pulp->mbox.v_addr == MAP_FAILED) {
    printf("MMAP failed for Mailbox.\n");
    return -EIO;
  }
  else if (DEBUG_LEVEL > 0) {
    printf("Mailbox mapped to virtual user space at %p.\n",pulp->mbox.v_addr);
  }

  // L2
  offset = CLUSTERS_SIZE_B + SOC_PERIPHERALS_SIZE_B + MBOX_SIZE_B; // start of L2
  pulp->l2_mem.size = L2_MEM_SIZE_B;

  pulp->l2_mem.v_addr = mmap(NULL,pulp->l2_mem.size,
                             PROT_READ | PROT_WRITE,MAP_SHARED,pulp->fd,offset);

  if (pulp->l2_mem.v_addr == MAP_FAILED) {
    printf("MMAP failed for L2 memory.\n");
    return -EIO;
  }
  else if (DEBUG_LEVEL > 0) {
    printf("L2 memory mapped to virtual user space at %p.\n",pulp->l2_mem.v_addr);
  }

  // Platform
  // L3
  offset = CLUSTERS_SIZE_B + SOC_PERIPHERALS_SIZE_B + MBOX_SIZE_B + L2_MEM_SIZE_B; // start of L3
  pulp->l3_mem.size = L3_MEM_SIZE_B;

  pulp->l3_mem.v_addr = mmap(NULL,pulp->l3_mem.size,
                             PROT_READ | PROT_WRITE,MAP_SHARED,pulp->fd,offset);
  if (pulp->l3_mem.v_addr == MAP_FAILED) {
    printf("MMAP failed for shared L3 memory.\n");
    return -EIO;
  }
  else if (DEBUG_LEVEL > 0) {
    printf("Shared L3 memory mapped to virtual user space at %p.\n",pulp->l3_mem.v_addr);
  }

  // PULP external
  // GPIO
  offset = CLUSTERS_SIZE_B + SOC_PERIPHERALS_SIZE_B + MBOX_SIZE_B + L2_MEM_SIZE_B
    + L3_MEM_SIZE_B; // start of GPIO
  pulp->gpio.size = H_GPIO_SIZE_B;

  pulp->gpio.v_addr = mmap(NULL,pulp->gpio.size,
                           PROT_READ | PROT_WRITE,MAP_SHARED,pulp->fd,offset);
  if (pulp->gpio.v_addr == MAP_FAILED) {
    printf("MMAP failed for shared L3 memory.\n");
    return -EIO;
  }
  else if (DEBUG_LEVEL > 0) {
    printf("GPIO memory mapped to virtual user space at %p.\n",pulp->gpio.v_addr);
  }

  // CLKING
  offset = CLUSTERS_SIZE_B + SOC_PERIPHERALS_SIZE_B + MBOX_SIZE_B + L2_MEM_SIZE_B
    + L3_MEM_SIZE_B + H_GPIO_SIZE_B; // start of Clking
  pulp->clking.size = CLKING_SIZE_B;

  pulp->clking.v_addr = mmap(NULL,pulp->clking.size,
                             PROT_READ | PROT_WRITE,MAP_SHARED,pulp->fd,offset);
  if (pulp->clking.v_addr == MAP_FAILED) {
    printf("MMAP failed for shared L3 memory.\n");
    return -EIO;
  }
  else if (DEBUG_LEVEL > 0) {
    printf("Clock Manager memory mapped to virtual user space at %p.\n",pulp->clking.v_addr);
  }

  // RAB config
  offset = CLUSTERS_SIZE_B + SOC_PERIPHERALS_SIZE_B + MBOX_SIZE_B + L2_MEM_SIZE_B
    + L3_MEM_SIZE_B + H_GPIO_SIZE_B + CLKING_SIZE_B; // start of RAB config
  pulp->rab_config.size = RAB_CONFIG_SIZE_B;

  pulp->rab_config.v_addr = mmap(NULL,pulp->rab_config.size,
                                 PROT_READ | PROT_WRITE,MAP_SHARED,pulp->fd,offset);
  if (pulp->rab_config.v_addr == MAP_FAILED) {
    printf("MMAP failed for shared L3 memory.\n");
    return -EIO;
  }
  else if (DEBUG_LEVEL > 0) {
    printf("RAB config memory mapped to virtual user space at %p.\n",pulp->rab_config.v_addr);
  }

#if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI-ITX
  // SLCR
  offset = CLUSTERS_SIZE_B + SOC_PERIPHERALS_SIZE_B + MBOX_SIZE_B + L2_MEM_SIZE_B
    + L3_MEM_SIZE_B + H_GPIO_SIZE_B + CLKING_SIZE_B + RAB_CONFIG_SIZE_B; // start of SLCR
  pulp->slcr.size = SLCR_SIZE_B;

  pulp->slcr.v_addr = mmap(NULL,pulp->slcr.size,
                           PROT_READ | PROT_WRITE,MAP_SHARED,pulp->fd,offset);
  if (pulp->slcr.v_addr == MAP_FAILED) {
    printf("MMAP failed for Zynq SLCR.\n");
    return -EIO;
  }
  else if (DEBUG_LEVEL > 0) {
    printf("Zynq SLCR memory mapped to virtual user space at %p.\n",pulp->slcr.v_addr);
  }
#endif

  return 0;
}

/**
 * Undo the memory mapping of the device to virtual user space using
 * the munmap() syscall
 *
 * @pulp: pointer to the PulpDev structure
 */
int pulp_munmap(PulpDev *pulp)
{
  unsigned status;

  // undo the memory mappings
  printf("Undo the memory mappings.\n");
#if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI-ITX
  status = munmap(pulp->slcr.v_addr,pulp->slcr.size);
  if (status) {
    printf("MUNMAP failed for SLCR.\n");
  }
#endif
  status = munmap(pulp->gpio.v_addr,pulp->gpio.size);
  if (status) {
    printf("MUNMAP failed for GPIO.\n");
  }
  status = munmap(pulp->rab_config.v_addr,pulp->rab_config.size);
  if (status) {
    printf("MUNMAP failed for RAB config.\n");
  }
  status = munmap(pulp->l2_mem.v_addr,pulp->l2_mem.size);
  if (status) {
    printf("MUNMAP failed for L2 memory.\n");
  }
  status = munmap(pulp->soc_periph.v_addr,pulp->soc_periph.size);
  if (status) {
    printf("MUNMAP failed for SoC peripherals\n.");
  }
  status = munmap(pulp->clusters.v_addr,pulp->clusters.size);
  if (status) {
    printf("MUNMAP failed for clusters\n.");
  }
  status = munmap(pulp->l3_mem.v_addr,pulp->l3_mem.size);
  if (status) {
    printf("MUNMAP failed for shared L3 memory\n.");
  }

  // close the file descriptor
  printf("Close the file descriptor. \n");
  close(pulp->fd);
  pulp->fd = -1;

  return 0;
}

/**
 * Set the clock frequency of PULP, only do this at startup of PULP!!!
 *
 * clk_in   = CLKING_INPUT_FREQ_MHZ (100 or 50 MHz) multiplied to 1000 MHz
 * clk_out1 = CLKOUT0: divide 1000 MHz by CLKING_CONFIG_REG_2 -> ClkSoc_C
 * clk_out2 = CLKOUT1: divide 1000 MHz by CLKING_CONFIG_REG_5 -> ClkCluster_C
 *
 * @pulp:         pointer to the PulpDev structure
 * @des_freq_mhz: desired frequency in MHz
 */
int pulp_clking_set_freq(PulpDev *pulp, unsigned des_freq_mhz)
{
  unsigned status;
  int timeout;

  // set default clk values in pulp struct
  pulp->pulp_clk_freq_mhz = PULP_DEFAULT_FREQ_MHZ;
  pulp->host_clk_freq_mhz = ARM_CLK_FREQ_MHZ;

  int freq_mhz = des_freq_mhz - (des_freq_mhz % 5);
  if(freq_mhz <= 0)
    freq_mhz = 5;
  else if(freq_mhz >= 200)
    freq_mhz = 200;

  // Bring the input clock to 1000 MHz
#if CLKING_INPUT_FREQ_MHZ == 50
  int divclk_divide = 1;
  int clkfbout_mult = 20;
#elif CLKING_INPUT_FREQ_MHZ == 100
  int divclk_divide = 1;
  int clkfbout_mult = 10;
#else
  #error CLKING_INPUT_FREQ_MHZ value is not supported
#endif

  // default output clock = 50 MHz
  int clkout0_divide = 1000/freq_mhz;
  int clkout0_divide_frac = ((1000 % freq_mhz) << 10)/freq_mhz;

  // config DIVCLK_DIVIDE, CLKFBOUT_MULT, CLKFBOUT_FRAC, CLKFBOUT_PHASE
  unsigned value;
  value = 0x04000000 + 0x100*clkfbout_mult + 0x1*divclk_divide;
  pulp_write32(pulp->clking.v_addr,CLKING_CONFIG_REG_0_OFFSET_B,'b',value);
  if (DEBUG_LEVEL > 3)
    printf("CLKING_CONFIG_REG_0: %#x\n",value);

  // config CLKOUT0/1: DIVIDE, FRAC, FRAC_EN
  value = 0x00040000 + 0x100*clkout0_divide_frac + 0x1*clkout0_divide;
  pulp_write32(pulp->clking.v_addr,CLKING_CONFIG_REG_2_OFFSET_B,'b',value);
  pulp_write32(pulp->clking.v_addr,CLKING_CONFIG_REG_5_OFFSET_B,'b',value);
  if (DEBUG_LEVEL > 3)
    printf("CLKING_CONFIG_REG_2/5: %#x\n",value);

  // check status
  if ( !(pulp_read32(pulp->clking.v_addr,CLKING_STATUS_REG_OFFSET_B,'b') & 0x1) ) {
    timeout = 10;
    status = 1;
    while ( status && (timeout > 0) ) {
      usleep(10000);
      timeout--;
      status = !(pulp_read32(pulp->clking.v_addr,CLKING_STATUS_REG_OFFSET_B,'b') & 0x1);
    }
    if ( status ) {
      printf("ERROR: Clock manager not locked, cannot start reconfiguration.\n");
      pulp->pulp_clk_freq_mhz = PULP_DEFAULT_FREQ_MHZ;
      return -EBUSY;
    }
  }

  // start reconfiguration
  pulp_write32(pulp->clking.v_addr,CLKING_CONFIG_REG_23_OFFSET_B,'b',0x7);
  usleep(1000);
  pulp_write32(pulp->clking.v_addr,CLKING_CONFIG_REG_23_OFFSET_B,'b',0x2);

  // check status
  if ( !(pulp_read32(pulp->clking.v_addr,CLKING_STATUS_REG_OFFSET_B,'b') & 0x1) ) {
    timeout = 10;
    status = 1;
    while ( status && (timeout > 0) ) {
      usleep(10000);
      timeout--;
      status = !(pulp_read32(pulp->clking.v_addr,CLKING_STATUS_REG_OFFSET_B,'b') & 0x1);
    }
    if ( status ) {
      printf("ERROR: Clock manager not locked, clock reconfiguration failed.\n");
      pulp->pulp_clk_freq_mhz = PULP_DEFAULT_FREQ_MHZ;
      return -EBUSY;
    }
  }

#if PLATFORM != JUNO
  // reconfigure PULP -> host UART
  int baudrate = 115200;
  char cmd[40];
  char baudrate_s[10];
  float ratio,baudrate_f;

  // compute custom baudrate
  ratio = (float)freq_mhz/(float)PULP_DEFAULT_FREQ_MHZ;
  baudrate_f = (float)baudrate * ratio;
  baudrate = (int)baudrate_f;
  sprintf(baudrate_s, "%i", baudrate);

  printf("Please configure /dev/ttyPS1 for %i baud.\n",baudrate);

  // prepare command
  //strcpy(cmd,"stty -F /dev/ttyPS1 ");  // only supports standard baudrates
  strcpy(cmd,"/media/nfs/apps/uart "); // supports non-standard baudrates
  strcat(cmd,baudrate_s);
  strcat(cmd," 0");

  //printf("%s\n",cmd);

  // set the baudrate -- can cause crashes
  //system(cmd);
#endif

  pulp->pulp_clk_freq_mhz = freq_mhz;

  return freq_mhz;
}

/**
 * Measure the clock frequency of PULP. Can only be executed with the
 * RAB configured to allow accessing the cluster peripherals. To
 * validate the measurement, the ZYNQ_PMM needs to be loaded for
 * access to the ARM clock counter.
 *
 * @pulp:         pointer to the PulpDev structure
 */
int pulp_clking_measure_freq(PulpDev *pulp)
{
  unsigned seconds = 1;
  unsigned limit = 0;

  unsigned arm_clk_freq_mhz = ARM_CLK_FREQ_MHZ;
  FILE *fp;
  char arm_clk_freq_khz_string[20];
  float deviation;

  // if possible get host clock frequency from sysfs
  if( access("/sys/devices/system/cpu/cpufreq/policy0/cpuinfo_cur_freq", F_OK ) != -1 ) {

    if((fp = fopen("/sys/devices/system/cpu/cpufreq/policy0/cpuinfo_cur_freq", "r")) == NULL)
      printf("ERROR: Could not open sysfs.\n");
    else if ( fgets(arm_clk_freq_khz_string, 20, fp) != NULL)
      arm_clk_freq_mhz = (strtoul(arm_clk_freq_khz_string, NULL, 10)+1)/1000;

    fclose(fp);
  }
  pulp->host_clk_freq_mhz = arm_clk_freq_mhz;

#if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI-ITX
   limit = (unsigned)((float)(arm_clk_freq_mhz*100000*1.61)*seconds);
#elif PLATFORM == TE0808
   limit = (unsigned)((float)(arm_clk_freq_mhz*100000*2.33)*seconds);
#else // PLATFORM == JUNO
   limit = (unsigned)((float)(arm_clk_freq_mhz*100000*1.61)*seconds);
#endif

  unsigned pulp_clk_counter, arm_clk_counter;
  unsigned zynq_pmm;

  volatile unsigned k;
  int mes_freq_mhz;

#if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI-ITX
  if( access("/dev/ZYNQ_PMM", F_OK ) != -1 )
    zynq_pmm = 1;
  else
    zynq_pmm = 0;
#else
  zynq_pmm = 0;
#endif

  // start clock counters
  if (zynq_pmm) {
    // enable clock counter divider (by 64), reset & enable clock counter, PMCR register
    asm volatile("mcr p15, 0, %0, c9, c12, 0" :: "r"(0xD));
  }

  pulp_write32(pulp->clusters.v_addr,TIMER_STOP_OFFSET_B,'b',0x1);
  pulp_write32(pulp->clusters.v_addr,TIMER_RESET_OFFSET_B,'b',0x1);
  pulp_write32(pulp->clusters.v_addr,TIMER_START_OFFSET_B,'b',0x1);

  // wait but don't sleep
  k = 0;
  while (k<limit) {
    k++;
    k++;
    k++;
    k++;
    k++;
    k++;
    k++;
    k++;
    k++;
    k++;
  }

  // stop and read clock counters
  if (zynq_pmm) {
    // Read the counter value, PMCCNTR register
    asm volatile("mrc p15, 0, %0, c9, c13, 0" : "=r"(arm_clk_counter) : );
  }
  pulp_clk_counter = pulp_read32(pulp->clusters.v_addr,TIMER_GET_TIME_LO_OFFSET_B,'b');

#if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI-ITX
  if (zynq_pmm) {
    mes_freq_mhz = (int)((float)pulp_clk_counter/((float)(arm_clk_counter*ARM_PMU_CLK_DIV)/arm_clk_freq_mhz));
  }
  else {
    mes_freq_mhz = (int)((float)pulp_clk_counter/seconds/1000000);
  }
#else
  mes_freq_mhz = (int)((float)pulp_clk_counter/seconds/1000000);
#endif

  // How far away is the measured frequency from the configured one?
  deviation = (float)pulp->pulp_clk_freq_mhz - (float)mes_freq_mhz;
  if (deviation < 0)
    deviation = -deviation;
  if (deviation/(float)pulp->pulp_clk_freq_mhz > 0.2)
    printf("WARNING: Clock configuration probably failed. Configured for %d MHz, measured %d MHz.\n",
      pulp->pulp_clk_freq_mhz, mes_freq_mhz);

  return mes_freq_mhz;
}

/**
 * Initialize the memory mapped device
 *
 * @pulp: pointer to the PulpDev structure
 */
int pulp_init(PulpDev *pulp)
{
  // set fetch enable to 0, set global clk enable, disable reset
  pulp_write32(pulp->gpio.v_addr,0x8,'b',0xC0000000);

  // RAB setup
  // port 0: Host -> PULP
  pulp_rab_req(pulp,L2_MEM_H_BASE_ADDR,L2_MEM_SIZE_B,0x7,0,RAB_MAX_DATE,RAB_MAX_DATE, 0, 1);     // L2
  //pulp_rab_req(pulp,MBOX_H_BASE_ADDR,MBOX_SIZE_B,0x7,0,RAB_MAX_DATE,RAB_MAX_DATE, 0, 1); // Mailbox, Interface 0
  pulp_rab_req(pulp,MBOX_H_BASE_ADDR,MBOX_SIZE_B*2,0x7,0,RAB_MAX_DATE,RAB_MAX_DATE, 0, 1); // Mailbox, Interface 0 and Interface 1
  pulp_rab_req(pulp,PULP_H_BASE_ADDR,CLUSTERS_SIZE_B,0x7,0,RAB_MAX_DATE,RAB_MAX_DATE, 0, 1);     // TCDM + Cluster Peripherals
  // port 1: PULP -> Host
  pulp_rab_req(pulp,L3_MEM_BASE_ADDR,L3_MEM_SIZE_B,0x7,1,RAB_MAX_DATE,RAB_MAX_DATE, 0, 1);       // contiguous L3 memory

  // enable mbox interrupts
  pulp_write32(pulp->mbox.v_addr,MBOX_IE_OFFSET_B,'b',0x6);

  // check
  if (DEBUG_LEVEL > 1)
    printf("Mailbox interrupt enable register = %#x\n", pulp_read32(pulp->mbox.v_addr,MBOX_IE_OFFSET_B,'b'));

  // reset the l3_offset pointer
  pulp->l3_offset = 0;

  // disable host interrupt for RAB misses if PULP handles them
  #ifdef RAB_MH_ON_SOC
    pulp->intr_rab_miss_dis = 1;
  #else
    pulp->intr_rab_miss_dis = 0;
  #endif

  // pass information to driver
  unsigned gpio_val = 0;
  gpio_val |= pulp->cluster_sel & CLUSTER_MASK;

  // Enable RAB AX Loggers.
  if (pulp->rab_ax_log_en == 1)
    gpio_val |= 1 << GPIO_RAB_AX_LOG_EN;
  // Disable RAB miss interrupt to host.
  if (pulp->intr_rab_miss_dis == 1)
    gpio_val |= 1 << GPIO_INTR_RAB_MISS_DIS;

  ioctl(pulp->fd,PULP_IOCTL_INFO_PASS,(unsigned *)&gpio_val);

  return 0;
}

/**
 * Read n_words words from mbox, can block if the mbox does not
 * contain enough data.
 *
 * @pulp      : pointer to the PulpDev structure
 * @buffer    : pointer to read buffer
 * @n_words   : number of words to read
 */
int pulp_mbox_read(const PulpDev *pulp, unsigned *buffer, unsigned n_words)
{
  int n_char, n_char_left, ret;
  ret = 1;
  n_char = 0;
  n_char_left = n_words*sizeof(buffer[0]);

  // read n_words words or until error
  while (n_char_left) {
    ret = read(pulp->fd, (char *)&buffer[n_char],n_char_left*sizeof(char));
    if (ret < 0) {
      printf("ERROR: Could not read mbox.\n");
      return ret;
    }
    n_char += ret;
    n_char_left -= ret;
  }

  return 0;
}

/**
 * Write one word to mbox.
 *
 * @pulp      : pointer to the PulpDev structure
 * @word      : word to write
 */
int pulp_mbox_write(PulpDev *pulp, unsigned word)
{
  unsigned timeout, status;
  unsigned us_delay = 100;

  // check if mbox is full
  if ( pulp_read32(pulp->mbox.v_addr, MBOX_STATUS_OFFSET_B, 'b') & 0x2 ) {
    timeout = 1000;
    status = 1;
    // wait for not full or timeout
    while ( status && (timeout > 0) ) {
      usleep(us_delay);
      timeout--;
      status = (pulp_read32(pulp->mbox.v_addr, MBOX_STATUS_OFFSET_B, 'b') & 0x2);
    }
    if ( status ) {
      printf("ERROR: mbox timeout.\n");
      return -ETIME;
    }
  }

  // mbox is ready to receive
  pulp_write32(pulp->mbox.v_addr,MBOX_WRDATA_OFFSET_B,'b', word);
  if (DEBUG_LEVEL > 3)
    printf("Wrote %#x to mbox.\n",word);

  return 0;
}

/**
 * Clear interrupt status flag in mbox. The next write of PULP will
 * again be handled by the PULP driver.
 *
 * @pulp: pointer to the PulpDev structure
 */
void pulp_mbox_clear_is(PulpDev *pulp)
{
  pulp_write32(pulp->mbox.v_addr,MBOX_IS_OFFSET_B,'b',0x7);
}

/**
 * Request a remapping (one or more RAB slices)
 *
 * @pulp      : pointer to the PulpDev structure
 * @addr_start: (virtual) start address
 * @size_b    : size of the remapping in bytes
 * @prot      : protection flags, one bit each for write, read, and enable
 * @port      : RAB port, 0 = Host->PULP, 1 = PULP->Host
 * @date_exp  : expiration date of the mapping
 * @date_cur  : current date, used to check for suitable slices
 */
int pulp_rab_req(const PulpDev *pulp, unsigned addr_start, unsigned size_b,
                 unsigned char prot, unsigned char port,
                 unsigned char date_exp, unsigned char date_cur,
                 unsigned char use_acp, unsigned char rab_lvl)
{
  int err;
  unsigned request[3];

  // setup the request
  request[0] = 0;
  RAB_SET_PROT(request[0], prot);
  RAB_SET_ACP(request[0], use_acp);
  RAB_SET_PORT(request[0], port);
  RAB_SET_LVL(request[0], rab_lvl);
  RAB_SET_DATE_EXP(request[0], date_exp);
  RAB_SET_DATE_CUR(request[0], date_cur);
  request[1] = addr_start;
  request[2] = size_b;

  // make the request
  err = ioctl(pulp->fd,PULP_IOCTL_RAB_REQ,request);
  if (err) {
    printf("ERROR: ioctl for RAB request failed. err = %d, errno = %d\n", err, errno);
  }

  return err;
}

/**
 * Free RAB slices
 *
 * @pulp      : pointer to the PulpDev structure
 * @date_cur  : current date, 0 = RAB_MAX_DATE = free all slices
 */
void pulp_rab_free(const PulpDev *pulp, unsigned char date_cur) {

  // make the request
  ioctl(pulp->fd,PULP_IOCTL_RAB_FREE,(unsigned)date_cur);
}

/**
 * Request striped remappings
 *
 * @pulp:       pointer to the PulpDev structure
 * @task:       pointer to the TaskDesc structure
 * @pass_type:  pointer to array marking the elements to pass by reference
 * @n_elements: number of striped data elements
 */
int pulp_rab_req_striped(const PulpDev *pulp, const TaskDesc *task,
                         ElemPassType **pass_type, int n_elements)
{
  int i,j,k, err;
  unsigned n_data;

  n_data = task->n_data;

  RabStripeReqUser stripe_request;

  stripe_request.id         = 0;
  stripe_request.n_elements = (short)n_elements;

  RabStripeElemUser * elements = (RabStripeElemUser *)
    malloc((size_t)(stripe_request.n_elements*sizeof(RabStripeElemUser)));
  if ( elements == NULL ) {
    printf("ERROR: Malloc failed for RabStripeElemUser structs.\n");
    return -ENOMEM;
  }

  stripe_request.rab_stripe_elem_user_addr = (unsigned)&elements[0];

  unsigned * addr_start;
  unsigned * addr_end;

  unsigned flags, prot;
  unsigned offset_start, size_b;
  unsigned max_stripe_size_b, overlap;

  offset_start = 0;
  size_b = 0;

  /*
   * fill stripe elements - except actual addresses - these are filled later
   */
  j = 0;
  for (i=0; i<n_data; i++) {

    if ( (task->data_desc[i].sh_mem_ctrl) == 2 ) {

      // determine max_stripe_size_b
      max_stripe_size_b = task->data_desc[i].stripe_desc->first_stripe_size_b;
      if ( task->data_desc[i].stripe_desc->last_stripe_size_b > max_stripe_size_b )
        max_stripe_size_b = task->data_desc[i].stripe_desc->last_stripe_size_b;
      if ( task->data_desc[i].stripe_desc->stripe_size_b > max_stripe_size_b )
        max_stripe_size_b = task->data_desc[i].stripe_desc->stripe_size_b;

      // fill stripe elements
      elements[j].max_stripe_size_b = max_stripe_size_b;
      elements[j].n_stripes         = task->data_desc[i].stripe_desc->n_stripes;

      if (DEBUG_LEVEL > 2) {
        printf("size of striped element %d     = %#x\n", j, task->data_desc[i].size);
        printf("elements[%d].n_stripes         = %d\n", j, elements[j].n_stripes);
        printf("elements[%d].max_stripe_size_b = %#x\n", j, elements[j].max_stripe_size_b);
      }

      j++;
    } // if
  } // for

  /*
   * generate address tables
   */
  i = -1;
  for (k=0; k<n_data; k++) {

    if ( (task->data_desc[k].sh_mem_ctrl) == 2 ) {
      i++;

      // set protection and cache ctrl flags
      flags = 0;
      if      (task->data_desc[k].type == in)  // in = read
        prot = 0x2 | 0x1;
      else if (task->data_desc[k].type == out) // out = write
        prot = 0x4 | 0x1;
      else // 0: inout = read & write
        prot = 0x4 | 0x2 | 0x1;
      RAB_SET_PROT(flags, prot);
      RAB_SET_ACP(flags, task->data_desc[k].cache_ctrl);

      elements[i].id    = (unsigned char)i; // not used right now
      elements[i].type  = (unsigned char)(task->data_desc[k].type);
      elements[i].flags = (unsigned char)flags;

      // allocate memory to hold striping information
      addr_start = (unsigned *)malloc((size_t)elements[i].n_stripes*sizeof(unsigned));
      addr_end   = (unsigned *)malloc((size_t)elements[i].n_stripes*sizeof(unsigned));
      if ( (addr_start == NULL) || (addr_end == NULL) ) {
        printf("ERROR: Malloc failed for addr_start/addr_end.\n");
        return -ENOMEM;
      }

      // fill pointer values
      elements[i].stripe_addr_start = (unsigned)&addr_start[0];
      elements[i].stripe_addr_end   = (unsigned)&addr_end[0];

      // some apps need an overlap in some elements
      overlap = 0;
      if ( !strcmp(task->name, "rod") ) {
        if ( task->data_desc[k].type == in ) // input elements are overlapped only

          overlap = 2 * sizeof(unsigned char)
            * (3 * (*(unsigned *)(task->data_desc[3].ptr)) ); // (R * w);
      } // if

      // fill in stripe data
      for (j=0; j<elements[i].n_stripes; j++) {
        if      ( j == 0 ) {
          // first stripe
          offset_start = 0;
          size_b       = task->data_desc[i].stripe_desc->first_stripe_size_b;
        }
        else if (j == (elements[i].n_stripes-1) ) {
          // last stripe
          offset_start += (size_b - overlap);
          size_b       = task->data_desc[i].stripe_desc->last_stripe_size_b;
        }
        else {
          // intermediate stripes
          offset_start += (size_b - overlap);
          size_b       = task->data_desc[i].stripe_desc->stripe_size_b;
        }

        // write the address arrays
        addr_start[j] = (unsigned)(task->data_desc[k].ptr) + offset_start;
        addr_end[j]   = addr_start[j] + size_b;
      } // for n_stripes

      if (DEBUG_LEVEL > 2) {
        printf("Shared Element %d: \n",k);
        printf("stripe_addr_start @ %#x\n",elements[i].stripe_addr_start);
        printf("stripe_addr_end   @ %#x\n",elements[i].stripe_addr_end);
        for (j=0; j<elements[i].n_stripes; j++) {
          if (j>2 && j<(elements[i].n_stripes-3))
            continue;
          printf("%d\t",j);
          printf("%#x - ", ((unsigned *)(elements[i].stripe_addr_start))[j]);
          printf("%#x\n",  ((unsigned *)(elements[i].stripe_addr_end))[j]);
        }
        printf("\n");
      } // DEBUG_LEVEL > 2
    } // sh_mem_ctrl == 2
  } // for n_data

  // make the request
  err = ioctl(pulp->fd,PULP_IOCTL_RAB_REQ_STRIPED,(unsigned *)&stripe_request);
  if (err) {
    printf("ERROR: ioctl() for stripe request failed. err = %d, errno = %d\n", err, errno);
  }

  // free memory
  for (i=0; i<stripe_request.n_elements; i++) {
    free((unsigned *)elements[i].stripe_addr_start);
    free((unsigned *)elements[i].stripe_addr_end);
  }
  free(elements);

  return err;
}

/**
 * Free striped remappings
 *
 * @pulp      : pointer to the PulpDev structure
 */
void pulp_rab_free_striped(const PulpDev *pulp)
{

  unsigned offload_id = 0;

  // make the request
  ioctl(pulp->fd,PULP_IOCTL_RAB_FREE_STRIPED,offload_id);
}

int pulp_rab_mh_enable(const PulpDev *pulp, unsigned char use_acp, unsigned char rab_mh_lvl)
{
  unsigned rab_mh_cfg[2];
  rab_mh_cfg[0] = (unsigned)use_acp;
  rab_mh_cfg[1] = (unsigned)rab_mh_lvl;

  return ioctl(pulp->fd,PULP_IOCTL_RAB_MH_ENA,rab_mh_cfg);
}

void pulp_rab_mh_disable(const PulpDev *pulp)
{
  ioctl(pulp->fd,PULP_IOCTL_RAB_MH_DIS);
}

/**
 * Enable handling of RAB misses by the SoC.
 *
 * The PULP driver function called by this function basically does two things:  First, it sets up
 * RAB so that the page table hierarchy can be accessed from PULP.  For this, slices either for the
 * first-level page table or for all second-level page tables are configured in RAB (definable, see
 * parameter below).  Second, the driver function disables handling of RAB misses in the Kernel
 * driver itself and prohibits the driver to write to those slices that are now managed by the SoC.
 *
 * @param   pulp                    Pointer to the PulpDev struct.
 * @param   static_2nd_lvl_slices   If 0, the driver sets up a single RAB slice for the first level
 *                                  of the page table; if 1, the driver sets up RAB slices for all
 *                                  valid second-level page tables.  The latter is not supported by
 *                                  all architectures.  If unsupported, the driver will fall back to
 *                                  the former behavior and emit a warning.
 *
 * @return  0 on success; negative value with an errno on errors.  See the documentation of
 *          `pulp_rab_soc_mh_ena()` for particular values.
 */
int pulp_rab_soc_mh_enable(const PulpDev* pulp, const unsigned static_2nd_lvl_slices)
{
    return ioctl(pulp->fd, PULP_IOCTL_RAB_SOC_MH_ENA, static_2nd_lvl_slices & 1);
}

/**
 * Disable handling of RAB misses by the SoC.
 *
 * The PULP driver function called by this function frees and deconfigures all slices used to map
 * the initial level of the page table and hands the slices that were reserved to be managed by the
 * SoC back to the host.
 *
 * @param   pulp    Pointer to the PulpDev struct.
 *
 * @return  0 on success; negative value with an errno on errors.  See the documentation of
 *          `pulp_rab_soc_mh_dis()` for particular values.
 */
int pulp_rab_soc_mh_disable(const PulpDev* const pulp)
{
    return ioctl(pulp->fd, PULP_IOCTL_RAB_SOC_MH_DIS);
}

/**
 * Store the content of the RAB AX Logger to a file.
 *
 * This function reads the content of the RAB AX Logger from kernel space, sorts the entries
 * according the timestamp, and writes the data into the file `rab_ax_log_%Y-%m-%d_%H-%M-%S.txt`
 * (see `man date` for the exact meaning of the format specifiers).
 *
 * This function must be called before freeing the RAB, otherwise the RAB free will already empty
 * the kernel space buffers.
 *
 * @param   pulp    Pointer to the PulpDev struct.
 *
 * @return  0 on success; negative value with an errno on errors.
 */
int pulp_rab_ax_log_read(const PulpDev* const pulp)
{
  int err = 0;

  #if RAB_AX_LOG_EN == 1
    // allocate memory for ptrs
    unsigned ** ptrs = (unsigned **)malloc(4*sizeof(unsigned *));
    if ( !ptrs ) {
      printf("ERROR: Malloc failed for ptrs.\n");
      return -ENOMEM;
    }

    // allocate memory for status
    unsigned * status = (unsigned *)malloc(3*sizeof(unsigned));
    if ( !status ) {
      printf("ERROR: Malloc failed for status.\n");
      return -ENOMEM;
    }

    // allocate the buffers in user space
    unsigned * rab_ar_log_buf = (unsigned *)malloc(RAB_AX_LOG_BUF_SIZE_B);
    unsigned * rab_aw_log_buf = (unsigned *)malloc(RAB_AX_LOG_BUF_SIZE_B);
    unsigned * rab_cfg_log_buf = (unsigned *)malloc(RAB_AX_LOG_BUF_SIZE_B);
    if ( (rab_ar_log_buf == NULL) || (rab_aw_log_buf == NULL) || (rab_cfg_log_buf == NULL) ) {
      printf("ERROR: Malloc failed for rab_ar_log_buf/rab_aw_log_buf/rab_cfg_log_buf.\n");
      return -ENOMEM;
    }
    memset((void *)rab_ar_log_buf, 0, (size_t)RAB_AX_LOG_BUF_SIZE_B);
    memset((void *)rab_aw_log_buf, 0, (size_t)RAB_AX_LOG_BUF_SIZE_B);
    memset((void *)rab_cfg_log_buf, 0, (size_t)RAB_AX_LOG_BUF_SIZE_B);

    ptrs[0] = &status[0];
    ptrs[1] = &rab_ar_log_buf[0];
    ptrs[2] = &rab_aw_log_buf[0];
    ptrs[3] = &rab_cfg_log_buf[0];

    // get the data from kernel space
    err = ioctl(pulp->fd,PULP_IOCTL_RAB_AX_LOG_READ,&ptrs[0]);
    if (err) {
      printf("ERROR: ioctl for RAB AX log read failed. err = %d, errno = %d\n", err, errno);
    }

    // Obtain the current date and time for the file name.
    const time_t t = time(NULL);
    if (t < 0) {
      printf("ERROR: Could not get time!\n");
      return -ENODATA;
    }
    const struct tm* const lt = localtime(&t);
    if (lt == NULL) {
      printf("ERROR: Could not convert time to local time!\n");
      return -ENODATA;
    }
    char lt_str[20];
    sprintf(lt_str, "%04d-%02d-%02d_%02d-%02d-%02d",
        lt->tm_year+1900, lt->tm_mon+1, lt->tm_mday, lt->tm_hour, lt->tm_min, lt->tm_sec);
    char filename[64];
    sprintf(filename, "rab_ax_log_%s.txt", lt_str);

    // write the data to a file
    FILE *fp;
    if((fp = fopen(filename, "w")) == NULL) {
      printf("ERROR: Could not open RAB AX log file.\n");
      return -ENOENT;
    }

    unsigned ar_idx = 0;
    unsigned aw_idx = 0;
    unsigned cfg_idx = 0;
    unsigned ar_idx_max = status[0];
    unsigned aw_idx_max = status[1];
    unsigned cfg_idx_max = status[2];

    unsigned ts, meta, addr, len, id, type;
    unsigned ar_ts = 0, ar_meta = 0, ar_addr = 0;
    unsigned aw_ts = 0, aw_meta = 0, aw_addr = 0;

    type = 0;

    // TODO: sorting of cfg
    // (really necessary? what if we also want to add the read/write response buffers?)
    while ( (ar_idx+aw_idx) < (ar_idx_max+aw_idx_max) ) {

      // read next entry from buffers
      if (ar_idx < ar_idx_max) {
        ar_ts   = rab_ar_log_buf[ar_idx+0];
        ar_meta = rab_ar_log_buf[ar_idx+1];
        ar_addr = rab_ar_log_buf[ar_idx+2];
      }
      if (aw_idx < aw_idx_max) {
        aw_ts   = rab_aw_log_buf[aw_idx+0];
        aw_meta = rab_aw_log_buf[aw_idx+1];
        aw_addr = rab_aw_log_buf[aw_idx+2];
      }

      // determine which entry to write to log file
      if ( (ar_idx < ar_idx_max) && (aw_idx < aw_idx_max) ) { // both buffers have valid entries
        if (ar_ts < aw_ts) {
          ts   = ar_ts;
          meta = ar_meta;
          addr = ar_addr;
          ar_idx+=3;
          type = 0;
        }
        else {
          ts   = aw_ts;
          meta = aw_meta;
          addr = aw_addr;
          aw_idx+=3;
          type = 1;
        }
      }
      else {
        if (ar_idx < ar_idx_max) {
          ts   = ar_ts;
          meta = ar_meta;
          addr = ar_addr;
          ar_idx+=3;
          type = 0;
        }
        else { // aw_idx < aw_idx_max
          ts   = aw_ts;
          meta = aw_meta;
          addr = aw_addr;
          aw_idx+=3;
          type = 1;
        }
      }

      // write the entry into the log file
      len = BF_GET(meta, 0, 8 );
      id  = BF_GET(meta, 8, 10);
      #if RAB_AX_LOG_PRINT_FORMAT == 0 // DEBUG
        fprintf(fp, "%10u 0x%08x %3u 0x%03x %u\n", ts, addr, len, id, type);
      #else // 1 = MATLAB
        fprintf(fp, "%u %u %u %u %u\n", ts, addr, len, id, type);
      #endif
    }

    for (cfg_idx = 0; cfg_idx < cfg_idx_max; cfg_idx += 3) {
      type = 2;
      ts   = rab_cfg_log_buf[cfg_idx+0];
      meta = rab_cfg_log_buf[cfg_idx+1];
      addr = rab_cfg_log_buf[cfg_idx+2];
      len  = BF_GET(meta, 0, 8);
      id   = BF_GET(meta, 8, 10);
      #if RAB_AX_LOG_PRINT_FORMAT == 0 // DEBUG
        fprintf(fp, "%10u 0x%08x %3u 0x%03x %u\n", ts, addr, len, id, type);
      #else // 1 = MATLAB
        fprintf(fp, "%u %u %u %u %u\n", ts, addr, len, id, type);
      #endif
    }

    fclose(fp);

    // free the buffers
    free(rab_aw_log_buf);
    free(rab_ar_log_buf);
    free(rab_cfg_log_buf);
    free(status);
    free(ptrs);

  #endif

  return err;
}

int pulp_smmu_enable(const PulpDev* pulp, const unsigned char flags)
{
  // make the request
  return ioctl(pulp->fd, PULP_IOCTL_SMMU_ENA, (unsigned) flags);
}

int pulp_smmu_disable(const PulpDev *pulp)
{
  // make the request
  return ioctl(pulp->fd,PULP_IOCTL_SMMU_DIS);
}

/**
 * Setup a DMA transfer using the Host DMA engine
 *
 * @pulp      : pointer to the PulpDev structure
 * @addr_l3   : virtual address in host's L3
 * @addr_pulp : physical address in PULP, so far, only L2 tested
 * @size_b    : size in bytes
 * @host_read : 0: Host -> PULP, 1: PULP -> Host (not tested)
 */
int pulp_dma_xfer(const PulpDev *pulp,
                  unsigned addr_l3, unsigned addr_pulp, unsigned size_b,
                  unsigned host_read)
{
  unsigned request[3];

  // check & process arguments
  if (size_b >> 31) {
    printf("ERROR: Requested transfer size too large - cannot encode DMA transfer direction.\n ");
    return -EINVAL;
  }
  else if (host_read) {
    BF_SET(size_b,1,31,1);
  }

  // setup the request
  request[0] = addr_l3;
  request[1] = addr_pulp;
  request[2] = size_b;

  // make the request
  ioctl(pulp->fd,PULP_IOCTL_DMA_XFER_ASYNC,request);

  return 0;
}

/**
 * Offload an OpenMP task to PULP and setup the RAB
 *
 * Currently only used by profile_rab_striping, may be removed soon.
 *
 * @pulp : pointer to the PulpDev structure
 * @task : pointer to the TaskDesc structure
 */
int pulp_omp_offload_task(PulpDev *pulp, TaskDesc *task) {

  int err;

  // offload data
  err = pulp_offload_out(pulp, task);
  if (err) {
    printf("ERROR: Offloading data to PULP failed.\n");
    return err;
  }

  // load binary, boot PULP
  err = pulp_load_bin(pulp, task->name);
  if (err) {
    printf("ERROR: Load of PULP binary failed.\n");
    return err;
  }

  // start execution
  pulp_exe_start(pulp);

  return 0;
}

/**
 * Reset PULP
 * @pulp : pointer to the PulpDev structure
 * @full : type of reset: 0 for PULP reset, 1 for entire FPGA
 */
void pulp_reset(PulpDev *pulp, unsigned full)
{

#if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI-ITX
  unsigned slcr_value;

  // FPGA reset control register
  slcr_value = pulp_read32(pulp->slcr.v_addr, SLCR_FPGA_RST_CTRL_OFFSET_B, 'b');

  // extract the FPGA_OUT_RST bits
  slcr_value = slcr_value & 0xF;

  if (full) {
    // enable reset
    pulp_write32(pulp->slcr.v_addr, SLCR_FPGA_RST_CTRL_OFFSET_B, 'b', 0xF);

    // wait
    usleep(100000);

    // disable reset
    pulp_write32(pulp->slcr.v_addr, SLCR_FPGA_RST_CTRL_OFFSET_B, 'b', slcr_value);
  }
  else {
    // enable reset
    pulp_write32(pulp->slcr.v_addr, SLCR_FPGA_RST_CTRL_OFFSET_B, 'b',
                 slcr_value | (0x1 << SLCR_FPGA_OUT_RST));

    // wait
    usleep(100000);

    // disable reset
    pulp_write32(pulp->slcr.v_addr, SLCR_FPGA_RST_CTRL_OFFSET_B, 'b',
                 slcr_value );
#endif

    // reset using GPIO register
    pulp_write32(pulp->gpio.v_addr,0x8,'b',0x00000000);
    usleep(100000);
    pulp_write32(pulp->gpio.v_addr,0x8,'b',0xC0000000);
    usleep(100000);

#if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI-ITX
  }
  // temporary fix: global clk enable
  pulp_write32(pulp->gpio.v_addr,0x8,'b',0xC0000000);
#endif

}

/**
 * Boot PULP.
 *
 * @pulp : pointer to the PulpDev structure
 * @task : pointer to the TaskDesc structure
 */
int pulp_boot(PulpDev *pulp, const TaskDesc *task)
{
  int err;

  // load the binary
  err = pulp_load_bin(pulp, task->name);
  if (err) {
    printf("ERROR: Loading PULP binary failed.\n");
    return err;
  }

  // start execution
  pulp_exe_start(pulp);

  return 0;
}

/**
 * Load binaries to PULP.
 *
 * This function loads the specified binaries to the start of the TCDM and the start of the L2
 * memories inside PULP
 *
 * Not yet uses the host DMA engine.
 *
 * @pulp : pointer to the PulpDev structure
 * @name : pointer to the string containing the name of the
 *         application to load
 */
int pulp_load_bin(PulpDev *pulp, const char *name)
{
  int i;
  char * bin_name_tcdm;
  char * bin_name_l2;

  #define MAX_PATH_LENGTH 32

  /*
   * prepare binary name
   */
  // remove anything after the first dot
  char * first_dot;
  unsigned length;
  first_dot = strchr(name, '.');
  if (NULL == first_dot)
    length = strlen(name);
  else
    length = (unsigned)first_dot - (unsigned)name;

  // allocate memory for file names
  bin_name_tcdm = (char *)malloc((length + 1 + 9 + MAX_PATH_LENGTH)*sizeof(char));
  if (!bin_name_tcdm) {
    printf("ERROR: Malloc failed for bin_name_tcdm.\n");
    return -ENOMEM;
  }
  bin_name_l2 = (char *)malloc((length + 1 + 7 + MAX_PATH_LENGTH)*sizeof(char));
  if (!bin_name_l2) {
    printf("ERROR: Malloc failed for bin_name_l2.\n");
    return -ENOMEM;
  }

  // generate file names
  strncpy(bin_name_tcdm, name, length);
  bin_name_tcdm[length] = '\0';
  strcat(bin_name_tcdm,".tcdm.bin");

  strncpy(bin_name_l2, name, length);
  bin_name_l2[length] = '\0';
  strcat(bin_name_l2,".l2.bin");

  /*
   * load the binaries
   */
  int fd_tcdm, fd_l2;
  size_t size_b_tcdm, size_b_l2;
  struct stat file_stats;
  unsigned * bin_tcdm, * bin_l2;
  unsigned status;
  unsigned has_tcdm_bin = 0;

  // TCDM
  // open TCDM bin file and get size if it exists
  if( access( bin_name_tcdm, F_OK ) != -1 ) {

    has_tcdm_bin = 1;
    fd_tcdm = open(bin_name_tcdm, O_RDONLY);
    if (fd_tcdm < 0) {
      printf("ERROR: Could not open PULP binary %s.\n", bin_name_tcdm);
      return -ENOENT;
    }
    fstat(fd_tcdm, &file_stats);
    size_b_tcdm = file_stats.st_size;

    // memory map the binary, MAP_POPULATE makes sure there will be no page faults later on (DMA)
    bin_tcdm = (unsigned *)mmap(NULL, size_b_tcdm, PROT_READ, MAP_SHARED | MAP_POPULATE, fd_tcdm, 0);
    if (bin_tcdm  == MAP_FAILED) {
      printf("MMAP failed for PULP binary %s.\n", bin_name_tcdm);
      return -EIO;
    }

    // write binary to TCDM
    if (DEBUG_LEVEL > 0)
      printf("Loading binary file: %s, size = %d B\n", bin_name_tcdm, size_b_tcdm);

    for (i=0; i<size_b_tcdm/4; i++)
      pulp->clusters.v_addr[i] = bin_tcdm[i];
  }

  // L2
  // open L2 bin file - alternatively, there is just one single binary .bin (legacy)
  if( access( bin_name_l2, F_OK ) == -1 ) {
    bin_name_l2[0] = '\0';
    strncpy(bin_name_l2, name, length);
    bin_name_l2[length] = '\0';
    strcat(bin_name_l2,".bin");
  }

  // open TCDM bin file and get size
  fd_l2 = open(bin_name_l2, O_RDONLY);
  if (fd_l2 < 0) {
    printf("ERROR: Could not open PULP binary %s.\n", bin_name_l2);
    return -ENOENT;
  }
  fstat(fd_l2, &file_stats);
  size_b_l2 = file_stats.st_size;

  // memory map the binary, MAP_POPULATE makes sure there will be no page faults later on (DMA)
  bin_l2 = (unsigned *)mmap(NULL, size_b_l2, PROT_READ, MAP_SHARED | MAP_POPULATE, fd_l2, 0);
  if (bin_l2  == MAP_FAILED) {
    printf("MMAP failed for PULP binary %s.\n", bin_name_l2);
    return -EIO;
  }

  // write binary to L2
  if (DEBUG_LEVEL > 0)
    printf("Loading binary file: %s, size = %d B\n",bin_name_l2, size_b_l2);

  for (i=0; i<size_b_l2/4; i++)
    pulp->l2_mem.v_addr[i] = bin_l2[i];

  /*
   * check the binaries
   */
  if (DEBUG_LEVEL > 0) {
    int j;
    int n_failed;

    // TCDM
    if (has_tcdm_bin) {
      n_failed = 0;

      for (j=0; j<size_b_tcdm/4; j++) {
        if (pulp->clusters.v_addr[j] != bin_tcdm[j]) {
          n_failed++;
        }
      }
      if (n_failed)
        printf("WARNING: PULP binary %s not successfully copied to TCDM. Failed for %i words.\n",
          bin_name_tcdm, n_failed);
      else
        printf("PULP binary %s successfully copied to TCDM.\n", bin_name_tcdm);
    }

    // L2
    n_failed = 0;
    for (j=0; j<size_b_l2/4; j++) {
      if (pulp->l2_mem.v_addr[j] != bin_l2[j]) {
        n_failed++;
      }
    }
    if (n_failed)
      printf("WARNING: PULP binary %s not successfully copied to L2. Failed for %i words.\n",
        bin_name_l2, n_failed);
    else
      printf("PULP binary %s successfully copied to L2.\n", bin_name_l2);
  }

  /*
   * cleanup
   */
  if (has_tcdm_bin) {
    status = munmap(bin_tcdm, size_b_tcdm);
    if (status) {
      printf("MUNMAP failed for PULP binary.\n");
    }
    close(fd_tcdm);
  }

  status = munmap(bin_l2, size_b_l2);
  if (status) {
    printf("MUNMAP failed for PULP binary.\n");
  }
  close(fd_l2);

  free(bin_name_tcdm);
  free(bin_name_l2);

  return 0;
}

/**
 * Load binary to PULP. Not yet uses the Zynq PS DMA engine.
 *
 * @pulp : pointer to the PulpDev structure
 * @ptr  : pointer to mem where the binary is loaded
 * @size : binary size in bytes
 */
int pulp_load_bin_from_mem(PulpDev *pulp, void *ptr, size_t size)
{
  unsigned int *intptr = (unsigned int *)ptr;
  size_t i;
  
  // write binary to L2
  for (i=0; i<size/0x4U; i++)
    pulp->l2_mem.v_addr[i] = intptr[i];

  return 0;
}

/**
 * Starts programm execution on PULP.
 *
 * @pulp : pointer to the PulpDev structure
 */
void pulp_exe_start(PulpDev *pulp)
{
  // Enable clock and deactivate reset.
  unsigned gpio_val = (1 << GPIO_RST_N) | (1 << GPIO_CLK_EN);

  // Activate selected clusters.
  gpio_val |= pulp->cluster_sel & CLUSTER_MASK;

  // Enable RAB AX Loggers.
  if (pulp->rab_ax_log_en == 1)
    gpio_val |= 1 << GPIO_RAB_AX_LOG_EN;
  // Disable RAB miss interrupt to host.
  if (pulp->intr_rab_miss_dis == 1)
    gpio_val |= 1 << GPIO_INTR_RAB_MISS_DIS;

  printf("Starting program execution.\n");
  pulp_write32(pulp->gpio.v_addr,0x8,'b',gpio_val);
}

/**
 * Stops programm execution on PULP.
 *
 * @pulp : pointer to the PulpDev structure
 */
void pulp_exe_stop(PulpDev *pulp)
{
  // Enable clock and deactivate reset.
  unsigned gpio_val = (1 << GPIO_RST_N) | (1 << GPIO_CLK_EN);

  // Enable RAB AX Loggers.
  if (pulp->rab_ax_log_en == 1)
    gpio_val |= 1 << GPIO_RAB_AX_LOG_EN;
  // Disable RAB miss interrupt to host.
  if (pulp->intr_rab_miss_dis == 1)
    gpio_val |= 1 << GPIO_INTR_RAB_MISS_DIS;

  printf("Stopping program execution.\n");
  pulp_write32(pulp->gpio.v_addr,0x8,'b',gpio_val);

  sleep(1);
}

/**
 * Polls the GPIO register for the end of computation signal for at
 * most timeout_s seconds.
 *
 * @pulp      : pointer to the PulpDev structure
 * @timeout_s : maximum number of seconds to wait for end of
 *              computation
 */
int pulp_exe_wait(const PulpDev *pulp, int timeout_s)
{
  unsigned status, gpio_eoc;
  float interval_us = 100000;
  float timeout = (float)timeout_s*1000000/interval_us;

  gpio_eoc = BF_GET(pulp_read32(pulp->gpio.v_addr,0,'b'), GPIO_EOC_0, GPIO_EOC_N-GPIO_EOC_0+1);
  status   = (gpio_eoc == pulp->cluster_sel) ? 0 : 1;

  while ( status && (timeout > 0) ) {
    usleep(interval_us);
    timeout--;
    gpio_eoc = BF_GET(pulp_read32(pulp->gpio.v_addr,0,'b'), GPIO_EOC_0, GPIO_EOC_N-GPIO_EOC_0+1);
    status   = (gpio_eoc == pulp->cluster_sel) ? 0 : 1;
  }
  if ( status ) {
    printf("ERROR: PULP execution timeout.\n");
    return -ETIME;
  }

  return 0;
}

/**
 * Allocate memory in contiguous L3
 *
 * @pulp:   pointer to the PulpDev structure
 * @size_b: size in Bytes of the requested chunk
 * @p_addr: pointer to store the physical address to
 *
 * ATTENTION: This function can only allocate each address once!
 *
 */
unsigned int pulp_l3_malloc(PulpDev *pulp, size_t size_b, unsigned *p_addr)
{
  unsigned int v_addr;

  // round l3_offset to next higher 64-bit word -> required for PULP DMA
  if (pulp->l3_offset & 0x7) {
    pulp->l3_offset = (pulp->l3_offset & 0xFFFFFFF8) + 0x8;
  }

  if ( (pulp->l3_offset + size_b) >= L3_MEM_SIZE_B) {
    printf("WARNING: overflow in contiguous L3 memory.\n");
    pulp->l3_offset = 0;
  }

  v_addr = (unsigned int)pulp->l3_mem.v_addr + pulp->l3_offset;
  *p_addr = L3_MEM_BASE_ADDR + pulp->l3_offset;

  pulp->l3_offset += size_b;

  if (DEBUG_LEVEL > 2) {
    printf("Host virtual address = %#x \n",v_addr);
    printf("PMCA physical address = %#x \n",*p_addr);
  }

  return v_addr;
}

/**
 * Free memory previously allocated in contiguous L3
 *
 * @pulp:   pointer to the PulpDev structure
 * @v_addr: pointer to unsigned containing the virtual address
 * @p_addr: pointer to unsigned containing the physical address
 *
 * ATTENTION: This function does not do anything!
 *
 */
void pulp_l3_free(PulpDev *pulp, unsigned v_addr, unsigned p_addr)
{
  return;
}

/**
 * Find out which shared data elements to pass by reference and if yes,
 * with which type of memory sharing.
 *
 * @task :     pointer to the TaskDesc structure
 * @pass_type: pointer to array marking the elements to pass by reference
 */
int pulp_offload_get_pass_type(const TaskDesc *task, ElemPassType **pass_type) {

  int i, n_data, n_ref, size_b;

  n_data = task->n_data;
  n_ref = 0;
  size_b = sizeof(unsigned);

  for (i=0; i<n_data; i++) {
    if ( task->data_desc[i].size > size_b ) {
      switch (task->data_desc[i].sh_mem_ctrl) {

        case copy:
          (*pass_type)[i] = ref_copy;
          break;

        case svm_static:
          (*pass_type)[i] = ref_svm_static;
          break;

        case svm_stripe:
          (*pass_type)[i] = ref_svm_stripe;
          break;

        case svm_mh:
          (*pass_type)[i] = ref_svm_mh;
          break;

        case copy_tryx:
          (*pass_type)[i] = ref_copy; // the runtime maps ref_copy_tryx to ref_copy
          break;

        case svm_smmu:
          (*pass_type)[i] = ref_svm_mh; // the miss handling is performed by the SMMU
          break;

        case svm_smmu_shpt:
          (*pass_type)[i] = ref_copy; // the runtime maps ref_svm_smmu_shpt to ref_copy
          break;

        case custom:
          (*pass_type)[i] = ref_custom;
          break;

        default:
          break;

        n_ref++;
      }
    }
    else
      (*pass_type)[i] = val;
  }

  return n_ref;
}

/**
 * Set up the RAB for the offload based on the task descriptor struct.
 *
 * Try to reorder the shared data elements to minimize the number of
 * slices used.
 *
 * @task:      pointer to the TaskDesc structure
 * @pass_type: pointer to array marking the elements to pass by reference
 * @n_ref:     number of shared data elements passed by reference
 */
int pulp_offload_rab_setup(const PulpDev *pulp, const TaskDesc *task, ElemPassType **pass_type, int n_ref)
{
  int i, j, err;
  unsigned      n_ref_svm_static, n_ref_svm_stripe;
  unsigned      n_data, n_data_int, gap_size, temp;
  unsigned char prot, port, date_cur, date_exp;

  unsigned      * v_addr_int;
  unsigned      * size_int;
  unsigned      * order;
  unsigned char * cache_ctrl_int;
  unsigned char * rab_lvl_int;
  unsigned char * prot_int;

  n_data = task->n_data;

  date_cur = (unsigned char)(task->task_id + 1);
  date_exp = (unsigned char)(task->task_id + 30);

  n_ref_svm_static = 0;
  n_ref_svm_stripe = 0;

  // determine number of elements to pass in which form
  for (i=0; i<n_data; i++) {
    if      ( (*pass_type)[i] == ref_svm_static )
      n_ref_svm_static++;
    else if ( (*pass_type)[i] == ref_svm_stripe )
      n_ref_svm_stripe++;
  }

  if (DEBUG_LEVEL > 1) {
    printf("n_ref_svm_static = %d, n_ref_svm_stripe = %d\n", n_ref_svm_static, n_ref_svm_stripe);
  }

  /*
   *  set up regular RAB slices
   */
  if (n_ref_svm_static) {
    port       = 1;
    n_data_int = 1;

    // memory allocation for intervals
    v_addr_int     = (unsigned *)malloc((size_t)n_ref_svm_static*sizeof(unsigned));
    size_int       = (unsigned *)malloc((size_t)n_ref_svm_static*sizeof(unsigned));
    order          = (unsigned *)malloc((size_t)n_ref_svm_static*sizeof(unsigned));
    cache_ctrl_int = (unsigned char *)malloc((size_t)n_ref_svm_static*sizeof(unsigned char));
    rab_lvl_int    = (unsigned char *)malloc((size_t)n_ref_svm_static*sizeof(unsigned char));
    prot_int       = (unsigned char *)malloc((size_t)n_ref_svm_static*sizeof(unsigned char));
    if (!v_addr_int | !size_int | !order | !cache_ctrl_int | !rab_lvl_int) {
      printf("Malloc failed for RAB setup.\n");
      return -ENOMEM;
    }

    // select the elements to order
    j=0;
    for (i=0;i<n_data;i++) {
      if ( (*pass_type)[i] == ref_svm_static ) {
        order[j] = i;
        j++;
      }
    }

    // order the elements - bubble sort
    for (i=n_ref_svm_static; i>1; i--) {
      for (j=0; j<i-1; j++) {
        if (task->data_desc[j].ptr > task->data_desc[j+1].ptr) {
          temp       = order[j];
          order[j]   = order[j+1];
          order[j+1] = temp;
        }
      }
    }
    if (DEBUG_LEVEL > 2) {
      printf("Reordered %d data elements: \n",n_ref_svm_static);
      for (i=0; i<n_ref_svm_static; i++) {
        printf("%d \t %#x \t %#x \n", order[i],
          (unsigned)task->data_desc[order[i]].ptr,
          (unsigned)task->data_desc[order[i]].size);
      }
    }

    // determine the number of remappings/intervals to request
    v_addr_int[0]     = (unsigned)task->data_desc[order[0]].ptr;
    size_int[0]       = (unsigned)task->data_desc[order[0]].size;
    cache_ctrl_int[0] = task->data_desc[order[0]].cache_ctrl;
    rab_lvl_int[0]    = task->data_desc[order[0]].rab_lvl;
    if      ( task->data_desc[order[0]].type == inout) // input & output
      prot_int[0] = 0x7; // read & write
    else if ( task->data_desc[order[0]].type == in) // input only
      prot_int[0] = 0x3; // read only
    else
      prot_int[0] = 0x5; // write only

    for (i=1;i<n_ref_svm_static;i++) {
      j = order[i];
      gap_size = (unsigned)task->data_desc[j].ptr - (v_addr_int[n_data_int-1]
                                                     + size_int[n_data_int-1]);

      if      ( task->data_desc[j].type == inout) // input & output
        prot = 0x7; // read & write
      else if ( task->data_desc[j].type == in) // input only
        prot = 0x3; // read only
      else
        prot = 0x5; // write only

      if ( gap_size > RAB_CONFIG_MAX_GAP_SIZE_B
           || prot                          != prot_int[n_data_int-1]
           || task->data_desc[j].cache_ctrl != cache_ctrl_int[n_data_int-1]
           || task->data_desc[j].rab_lvl    != rab_lvl_int[n_data_int-1]) {
        // gap too large, different prot, different cache ctrl setting, different RAB level
        // -> create a new interval/mapping
        n_data_int++;
        v_addr_int    [n_data_int-1] = (unsigned)task->data_desc[j].ptr;
        size_int      [n_data_int-1] = (unsigned)task->data_desc[j].size;
        cache_ctrl_int[n_data_int-1] = task->data_desc[j].cache_ctrl;
        rab_lvl_int   [n_data_int-1] = task->data_desc[j].rab_lvl;
        prot_int      [n_data_int-1] = prot;
      }
      else {
        // extend the previous mapping
        size_int[n_data_int-1] += (gap_size + task->data_desc[j].size);
      }
    }

    // set up the RAB
    if (DEBUG_LEVEL > 2) {
      printf("Requesting %d remapping(s):\n",n_data_int);
    }
    for (i=0;i<n_data_int;i++) {
      if (DEBUG_LEVEL > 2) {
        printf("%d \t %#x \t %#x \n",i,v_addr_int[i],size_int[i]);
        usleep(1000000);
      }
      pulp_rab_req(pulp, v_addr_int[i], size_int[i], prot_int[i],
        port, date_exp, date_cur, cache_ctrl_int[i], rab_lvl_int[i]);
    }

    // free memory
    free(v_addr_int);
    free(size_int);
    free(cache_ctrl_int);
    free(rab_lvl_int);
    free(prot_int);
    free(order);
  }

  /*
   * set up striped RAB slices
   */
  if (n_ref_svm_stripe) {
    err = pulp_rab_req_striped(pulp, task, pass_type, n_ref_svm_stripe);
    if (err) {
      printf("Requesting striped RAB mappings failed: %d\n", err);
      return err;
    }
  }

  return 0;
}

/**
 * Free RAB mappings for the offload based on the task descriptor struct.
 *
 * @task:      pointer to the TaskDesc structure
 * @pass_type: pointer to array marking the elements to pass by reference
 * @n_ref:     number of shared data elements passed by reference
 */
int pulp_offload_rab_free(const PulpDev *pulp, const TaskDesc *task, const ElemPassType **pass_type, int n_ref)
{
  int i;
  unsigned      n_ref_svm_static, n_ref_svm_stripe;
  unsigned      n_data;
  unsigned char date_cur;

  n_data = task->n_data;

  date_cur = (unsigned char)(task->task_id + 41);

  n_ref_svm_static = 0;
  n_ref_svm_stripe = 0;

  // determine number of elements to passed in which form
  for (i=0; i<n_data; i++) {
    if      ( (*pass_type)[i] == ref_svm_static )
      n_ref_svm_static++;
    else if ( (*pass_type)[i] == ref_svm_stripe )
      n_ref_svm_stripe++;
  }

  // free regular RAB slices
  if (n_ref_svm_static) {
    pulp_rab_free(pulp, date_cur);
  }

  // free striped RAB slices
  if (n_ref_svm_stripe) {
    pulp_rab_free_striped(pulp);
  }

  return 0;
}

/**
 * Copy raw data elements to contiguous L3 memory and fill pointer values in data descriptor.
 *
 * WARNING: Pointers inside the data element are not changed.
 *
 * @task:      pointer to the TaskDesc structure
 * @pass_type: pointer to array marking the elements to pass by reference
 */
int pulp_offload_l3_copy_raw_out(PulpDev *pulp, TaskDesc *task, const ElemPassType **pass_type)
{
  int i;
  unsigned n_ref_copy;
  int n_data;
  ElemType type;
  size_t size;

  n_data = task->n_data;

  n_ref_copy = 0;

  // determine number of elements to pass in which form
  for (i=0; i<n_data; i++) {
    if ( (*pass_type)[i] == ref_copy )
      n_ref_copy++;
  }

  // do the data copies
  if (n_ref_copy) {
    for (i=0; i<n_data; i++) {

      if ( (*pass_type)[i] == ref_copy ) {
        type = task->data_desc[i].type;
        size = task->data_desc[i].size;

        switch(type) {
          case inout: // input and output
            task->data_desc[i].ptr_l3_v =
              (void *)pulp_l3_malloc(pulp, size, (unsigned *)&(task->data_desc[i].ptr_l3_p));
            memcpy((void *)(task->data_desc[i].ptr_l3_v), (void *)(task->data_desc[i].ptr), size);
            break;

          case in: // input only
            task->data_desc[i].ptr_l3_v =
              (void *)pulp_l3_malloc(pulp, size, (unsigned *)&(task->data_desc[i].ptr_l3_p));
            memcpy((void *)(task->data_desc[i].ptr_l3_v), (void *)(task->data_desc[i].ptr), size);
            break;

          case out: // output only
            task->data_desc[i].ptr_l3_v =
              (void *)pulp_l3_malloc(pulp, size, (unsigned *)&(task->data_desc[i].ptr_l3_p));
            break;

          default:
            printf("ERROR: Invalid sh_mem_ctrl\n.");
            return -EINVAL;
        } // switch
      } // if
    } // for
  } // if

  return 0;
}

/**
 * Copy raw data elements back from contiguous L3 memory.
 *
 * WARNING: Pointers inside the data element are not changed.
 *
 * @task:      pointer to the TaskDesc structure
 * @pass_type: pointer to array marking the elements to pass by reference
 */
int pulp_offload_l3_copy_raw_in(PulpDev *pulp, const TaskDesc *task, const ElemPassType **pass_type)
{
  int i;
  unsigned n_ref_copy;
  int n_data;
  ElemType type;
  size_t size;

  n_data = task->n_data;

  n_ref_copy = 0;

  // determine number of elements passed in which form
  for (i=0; i<n_data; i++) {
    if ( (*pass_type)[i] == ref_copy )
      n_ref_copy++;
  }

  // do the data copies
  if (n_ref_copy) {
    for (i=0; i<n_data; i++) {

      if ( (*pass_type)[i] == ref_copy ) {
        type = task->data_desc[i].type;
        size = task->data_desc[i].size;

        switch(type) {
          case inout: // input and output
            memcpy((void *)(task->data_desc[i].ptr), (void *)(task->data_desc[i].ptr_l3_v), size);
            pulp_l3_free(pulp, (unsigned)(task->data_desc[i].ptr_l3_v),
              (unsigned)(task->data_desc[i].ptr_l3_p));
            break;

          case in: // input only
            pulp_l3_free(pulp, (unsigned)(task->data_desc[i].ptr_l3_v),
              (unsigned)(task->data_desc[i].ptr_l3_p));
            break;

          case out: // output only
            memcpy((void *)(task->data_desc[i].ptr), (void *)(task->data_desc[i].ptr_l3_v), size);
            pulp_l3_free(pulp, (unsigned)(task->data_desc[i].ptr_l3_v),
              (unsigned)(task->data_desc[i].ptr_l3_p));
            break;

          default:
            printf("ERROR: Invalid sh_mem_ctrl\n.");
            return -EINVAL;
        } // switch
      } // if
    } // for
  } // if

  return 0;
}

/**
 * Pass the descriptors of the shared data elements to PULP.
 * @pulp:      pointer to the PulpDev structure
 * @task:      pointer to the TaskDesc structure
 * @pass_type: pointer to array marking the elements to pass by reference
 */
int pulp_offload_pass_desc(PulpDev *pulp, const TaskDesc *task, const ElemPassType **pass_type)
{
  int i;
  unsigned n_data, addr;

  n_data = task->n_data;

  if (DEBUG_LEVEL > 2) {
    printf("Mailbox status = %#x.\n",pulp_read32(pulp->mbox.v_addr, MBOX_STATUS_OFFSET_B, 'b'));
  }

  for (i=0; i<n_data; i++) {

    if ( (*pass_type)[i] ) {
      // pass data element by reference
      if ( ((*pass_type)[i] == ref_copy) || ((*pass_type)[i] == ref_custom) )
        addr = (unsigned)(task->data_desc[i].ptr_l3_p); // pass phys addr in contiguous L3
      else
        addr = (unsigned)(task->data_desc[i].ptr); // pass virt addr

      pulp_mbox_write(pulp, addr);
      if (DEBUG_LEVEL > 2)
        printf("Element %d: wrote addr %#x to mbox.\n", i, addr);
    }
    else {
      // pass data element by value and of type input/output or input
      if ( (task->data_desc[i].type == inout) || (task->data_desc[i].type == in) ) {
        pulp_mbox_write(pulp, *(unsigned *)(task->data_desc[i].ptr));
        if (DEBUG_LEVEL > 2)
          printf("Element %d: wrote val  %#x to mbox.\n",i,*(unsigned*)(task->data_desc[i].ptr));
      }
      else {// pass by value, but type is output only
        if (DEBUG_LEVEL > 2)
          printf("Element %d: not passed to mbox.\n",i);
      }
    }
  }

  return 0;
}

/**
 * Get back the shared data elements from PULP that were passed by value.
 *
 * @pulp:      pointer to the PulpDev structure
 * @task:      pointer to the TaskDesc structure
 * @pass_type: pointer to array marking the elements to pass by reference
 */
int pulp_offload_get_desc(const PulpDev *pulp, TaskDesc *task, const ElemPassType **pass_type)
{
  int i,j, n_data, n_values, ret;
  unsigned *buffer;

  j = 0;
  n_data = task->n_data;
  n_values = 0;
  ret = 0;

  for (i=0; i<n_data; i++) {
    // check if argument has been passed by value and is of type output or input and output
    if ( ((*pass_type)[i] == val)
          && ((task->data_desc[i].type == inout) || (task->data_desc[i].type == out)) ) {
      n_values++;
    }
  }

  buffer = (unsigned *)malloc(n_values*sizeof(unsigned));
  if ( buffer == NULL ) {
    printf("ERROR: Malloc failed for buffer.\n");
    return -ENOMEM;
  }

  // read from mbox
  pulp_mbox_read(pulp, &buffer[0], n_values);

  for (i=0; i<n_data; i++) {
    // check if argument has been passed by value and is of type input/output or output
    if ( ((*pass_type)[i] == val)
          && ((task->data_desc[i].type == inout) || (task->data_desc[i].type == out)) ) {
      // read from buffer
      *(unsigned *)(task->data_desc[i].ptr) = buffer[j];
      j++;
    }
  }

  if (j != n_values ) {
    printf("ERROR: Got back only %d of %d data elements from PULP.\n",j,n_values);
    ret = -EIO;
  }

  //#ifndef JPEG // generates error
  free(buffer);
  //#endif

  return ret;
}

/**
 * Offload a new task to PULP, set up RAB slices and pass descriptors
 * to PULP.
 *
 * @pulp: pointer to the PulpDev structure
 * @task: pointer to the TaskDesc structure
 */
int pulp_offload_out(PulpDev *pulp, TaskDesc *task)
{
  int n_ref, err;
  ElemPassType *pass_type;

  pass_type = (ElemPassType *)malloc(task->n_data*sizeof(ElemPassType));
  if ( pass_type == NULL ) {
    printf("ERROR: Malloc failed for pass_type.\n");
    return -ENOMEM;
  }

  // only remap addresses belonging to data elements larger than 32 bit
  n_ref = pulp_offload_get_pass_type(task, &pass_type);

  // RAB setup
  err = pulp_offload_rab_setup(pulp, task, &pass_type, n_ref);
  if (err) {
    printf("ERROR: pulp_offload_rab_setup failed.\n");
    return err;
  }

  // copy raw data out to contiguous L3 - pointers inside the data are not modified
  err = pulp_offload_l3_copy_raw_out(pulp, task, (const ElemPassType **)&pass_type);
  if (err) {
    printf("ERROR: pulp_offload_l3_copy_raw_out failed.\n");
    return err;
  }

  // pass data descriptor to PULP
  err = pulp_offload_pass_desc(pulp, task, (const ElemPassType **)&pass_type);
  if (err) {
    printf("ERROR: pulp_offload_pass_desc failed.\n");
    return err;
  }

  // free memory
  free(pass_type);

  return 0;
}

/**
 * Finish a task offload, clear RAB slices and get back descriptors
 * passed by value.
 *
 * @pulp: pointer to the PulpDev structure
 * @task: pointer to the TaskDesc structure
 */
int pulp_offload_in(PulpDev *pulp, TaskDesc *task)
{
  int n_ref, err;
  ElemPassType *pass_type;

  pass_type = (ElemPassType *)malloc(task->n_data*sizeof(ElemPassType));
  if ( pass_type == NULL ) {
    printf("ERROR: Malloc failed for pass_type.\n");
    return -ENOMEM;
  }

  // read back data elements with sizes up to 32 bit from mbox
  n_ref = pulp_offload_get_pass_type(task, &pass_type);

  // RAB free
  err = pulp_offload_rab_free(pulp, task, (const ElemPassType**)&pass_type, n_ref);
  if (err) {
    printf("ERROR: pulp_offload_rab_free failed.\n");
    return err;
  }

  // copy raw data in from contiguous L3 - pointers inside the data are not modified
  err = pulp_offload_l3_copy_raw_in(pulp, task, (const ElemPassType**)&pass_type);
  if (err) {
    printf("ERROR: pulp_offload_l3_copy_raw_in failed.\n");
    return err;
  }

  // fetch values of data elements passed by value
  err = pulp_offload_get_desc(pulp, task, (const ElemPassType**)&pass_type);
  if (err) {
    printf("ERROR: pulp_offload_get_desc failed.\n");
    return err;
  }

  // free memory
  free(pass_type);

  return 0;
}

#ifdef OLD_APPS
/**
 * Start offload execution on PULP.
 *
 * @pulp: pointer to the PulpDev structure
 * @task: pointer to the TaskDesc structure
 */
int pulp_offload_start(PulpDev *pulp, const TaskDesc *task)
{
  unsigned status;

  if (DEBUG_LEVEL > 2) {
    printf("Mailbox status = %#x.\n",pulp_read32(pulp->mbox.v_addr, MBOX_STATUS_OFFSET_B, 'b'));
  }

  // read status
  pulp_mbox_read(pulp, &status, 1);
  if ( status != PULP_READY ) {
    printf("ERROR: PULP status not ready. PULP status = %#x.\n",status);
    return -EBUSY;
  }

  // start execution
  pulp_mbox_write(pulp, PULP_START);

  return 0;
}

/**
 * Wait for an offloaded task to finish on PULP.
 *
 * @pulp: pointer to the PulpDev structure
 * @task: pointer to the TaskDesc structure
 */
int pulp_offload_wait(const PulpDev *pulp, const TaskDesc *task)
{
  unsigned status;

  // check if sync = 0x1
  status = 0;
  while ( status != PULP_DONE ) {
    status = pulp_read32(pulp->l2_mem.v_addr, SYNC_OFFSET_B,'b');
  }

  // read status
  pulp_mbox_read(pulp, &status, 1);
  if ( status != PULP_DONE ) {
    printf("ERROR: PULP status not done. PULP status = %#x.\n",status);
    return -EBUSY;
  }

  return 0;
}

int pulp_offload_out_contiguous(PulpDev *pulp, TaskDesc *task, TaskDesc **ftask)
{
  // similar to pulp_offload_out() but without RAB setup
  int err;
  unsigned *pass_type;
  pass_type = (unsigned *)malloc(task->n_data*sizeof(unsigned));
  if ( pass_type == NULL ) {
    printf("ERROR: Malloc failed for pass_type.\n");
    return -ENOMEM;
  }

  // only remap addresses belonging to data elements larger than 32 bit
  pulp_offload_get_pass_type(task, &pass_type);

  int i;
  int data_size, data_ptr;
  ElemType data_type;

  *ftask = (TaskDesc *)malloc(sizeof(TaskDesc));
  if ( *ftask == NULL ) {
    printf("ERROR: Malloc failed for ftask.\n");
    return -ENOMEM;
  }

  (*ftask)->task_id = task->task_id;
  (*ftask)->name = task->name;
  (*ftask)->n_clusters = task->n_clusters;
  (*ftask)->n_data = task->n_data;

  if ( !strcmp(task->name, "rod") ) {
#ifdef PROFILE
    (*ftask)->data_desc  = (DataDesc *)malloc(9*sizeof(DataDesc));
#else
    (*ftask)->data_desc  = (DataDesc *)malloc(6*sizeof(DataDesc));
#endif // PROFILE
  }
  else if ( !strcmp(task->name, "ct") ) {
#ifdef PROFILE
    (*ftask)->data_desc  = (DataDesc *)malloc(8*sizeof(DataDesc));
#else
    (*ftask)->data_desc  = (DataDesc *)malloc(6*sizeof(DataDesc));
#endif // PROFILE
  }
  else if ( !strcmp(task->name, "jpeg") ) {
#ifdef PROFILE
    (*ftask)->data_desc  = (DataDesc *)malloc(10*sizeof(DataDesc));
#else
    (*ftask)->data_desc  = (DataDesc *)malloc(7*sizeof(DataDesc));
#endif // PROFILE
  }
  else {
    printf("ERROR: Unknown task name %s\n",task->name);
  }

  if ( ((*ftask)->data_desc) == NULL ) {
    printf("ERROR: Malloc failed for data_desc.\n");
    return -ENOMEM;
  }

  // memory allocation in contiguous L3 memory for IN and OUT
  // copy to contiguous L3 memory for IN
  for (i = 0; i < (*ftask)->n_data; i++) {

    data_size = task->data_desc[i].size;
    data_ptr  = (int)task->data_desc[i].ptr;

    // data element to pass by reference, allocate contiguous L3 memory
    if (pass_type[i] != val) {

      data_type = task->data_desc[i].type;

      // we are going to abuse type to store the virtual host address and
      // ptr to store the physical address in contiguous L3 used by PULP

      switch(data_type) {
      case inout:
        (*ftask)->data_desc[i].type = (int)pulp_l3_malloc(pulp, data_size,
                                                          (unsigned *)&((*ftask)->data_desc[i].ptr));
        memcpy((void *)(*ftask)->data_desc[i].type, (void *)data_ptr ,data_size);
        break;

      case in:
        (*ftask)->data_desc[i].type = (int)pulp_l3_malloc(pulp, data_size,
                                                          (unsigned *)&((*ftask)->data_desc[i].ptr));
        memcpy((void *)(*ftask)->data_desc[i].type, (void *)data_ptr ,data_size);
        break;

      case out:
        (*ftask)->data_desc[i].type = (int)pulp_l3_malloc(pulp, data_size,
                                                          (unsigned *)&((*ftask)->data_desc[i].ptr));
        break;

      default:
        break;
      }
    }

    // data element to pass by value (just copy descriptor)
    else  {
      (*ftask)->data_desc[i].ptr = (void *)data_ptr;
    }
    (*ftask)->data_desc[i].size = data_size;
  }

  // Pass data descriptor to PULP
  err = pulp_offload_pass_desc(pulp, *ftask, (const ElemPassType**)&pass_type);
  if (err) {
    printf("ERROR: pulp_offload_pass_desc failed.\n");
    return err;
  }

  // free memory
  free(pass_type);

  return 0;
}

int pulp_offload_in_contiguous(PulpDev *pulp, TaskDesc *task, TaskDesc **ftask)
{
  int i;

  // similar to pulp_offload_in() but without RAB stuff
  int err;
  unsigned *pass_type;
  pass_type = (unsigned *)malloc(task->n_data*sizeof(unsigned));
  if ( pass_type == NULL ) {
    printf("ERROR: Malloc failed for pass_type.\n");
    return -ENOMEM;
  }

  // read back data elements with sizes up to 32 bit from mbox
  pulp_offload_get_pass_type(task, &pass_type);

  // fetch values of data elements passed by value
  err = pulp_offload_get_desc(pulp, task, (const ElemPassType**)&pass_type);
  if (err) {
    printf("ERROR: pulp_offload_get_desc failed.\n");
    return err;
  }

  // copy back result to virtual paged memory
  for (i = 0; i < (*ftask)->n_data; i++) {

    //if not passed by value
    if (pass_type[i] != val) {

      // we are abusing type of ftask->data_desc to store the virtual host address and
      // ptr to store the physical address in contiguous L3 used by PULP

      switch(task->data_desc[i].type) {
      case inout:
        memcpy((void *)task->data_desc[i].ptr, (void *)(*ftask)->data_desc[i].type, task->data_desc[i].size);
        pulp_l3_free(pulp, (unsigned)(*ftask)->data_desc[i].type, (unsigned)(*ftask)->data_desc[i].ptr);
        break;

      case in:
        pulp_l3_free(pulp, (unsigned)(*ftask)->data_desc[i].type, (unsigned)(*ftask)->data_desc[i].ptr);
        break;

      case out:
        memcpy((void *)task->data_desc[i].ptr, (void *)(*ftask)->data_desc[i].type, task->data_desc[i].size);
        pulp_l3_free(pulp, (unsigned)(*ftask)->data_desc[i].type, (unsigned)(*ftask)->data_desc[i].ptr);
        break;

      default:
        break;
      }
    }
  }

  // free memory
  free((*ftask)->data_desc);
  free(*ftask);
  free(pass_type);

  return 0;
}
#endif // OLD_APPS

/****************************************************************************************/

int pulp_rab_req_striped_mchan_img(const PulpDev *pulp, unsigned char prot, unsigned char port,
                                   unsigned p_height, unsigned p_width, unsigned i_step,
                                   unsigned n_channels, unsigned char **channels,
                                   unsigned *s_height)
{
  int i,j;

  RabStripeReqUser stripe_request;

  stripe_request.id         = 0;
  stripe_request.n_elements = 1;

  RabStripeElemUser * elements = (RabStripeElemUser *)
    malloc((size_t)(stripe_request.n_elements*sizeof(RabStripeElemUser)));
  if ( elements == NULL ) {
    printf("ERROR: Malloc failed for RabStripeElemUser structs.\n");
    return -ENOMEM;
  }

  stripe_request.rab_stripe_elem_user_addr = (unsigned)&elements[0];

  unsigned * addr_start;
  unsigned * addr_end;

  unsigned addr_start_tmp, addr_end_tmp;
  unsigned stripe_size_b, stripe_height, n_stripes_per_channel;

  unsigned page_size_b;

  page_size_b = getpagesize();

  // compute max stripe height
  stripe_size_b = (RAB_L1_N_SLICES_PORT_1/2-1)*page_size_b;
  stripe_height = stripe_size_b / i_step;

  n_stripes_per_channel = p_height / stripe_height;
  if (p_height % stripe_height)
    n_stripes_per_channel++;

  // compute effective stripe height
  stripe_height = p_height / n_stripes_per_channel;
  *s_height = stripe_height;
  stripe_size_b = stripe_height * i_step;

  if (DEBUG_LEVEL > 2) {
    printf("n_stripes_per_channel = %d\n", n_stripes_per_channel);
    printf("stripe_size_b = %#x\n", stripe_size_b);
    printf("s_height = %d\n", stripe_height);
    printf("page_size_b = %#x\n",page_size_b);
  }

  // generate the rab_stripes table
  elements[0].id    = 0;   // not used right now
  elements[0].type  = in;  // for now in
  elements[0].flags = 0x7;
  elements[0].max_stripe_size_b = stripe_size_b;
  elements[0].n_stripes = n_stripes_per_channel * n_channels;

  addr_start = (unsigned *)malloc((size_t)elements[0].n_stripes*sizeof(unsigned));
  addr_end   = (unsigned *)malloc((size_t)elements[0].n_stripes*sizeof(unsigned));
  if ( (addr_start == NULL) || (addr_end == NULL) ) {
    printf("ERROR: Malloc failed for addr_start/addr_end.\n");
    return -ENOMEM;
  }

  elements[0].stripe_addr_start = (unsigned)&addr_start[0];
  elements[0].stripe_addr_end   = (unsigned)&addr_end[0];

  // fill in stripe data
  for (i=0; i<n_channels; i++) {
    for (j=0; j<n_stripes_per_channel; j++) {
      // align to words
      addr_start_tmp = ((unsigned)channels[i] + stripe_size_b*j);
      BF_SET(addr_start_tmp,0,0,4);
      addr_end_tmp   = ((unsigned)channels[i] + stripe_size_b*(j+1));
      BF_SET(addr_end_tmp,0,0,4);
      addr_end_tmp += 0x4;

      addr_start[i*n_stripes_per_channel + j] = addr_start_tmp;
      addr_end  [i*n_stripes_per_channel + j] = addr_end_tmp;
    }
  }

  if (DEBUG_LEVEL > 2) {
    printf("Shared Element %d: \n",0);
    printf("stripe_addr_start @ %#x\n",elements[0].stripe_addr_start);
    printf("stripe_addr_end   @ %#x\n",elements[0].stripe_addr_end);
    for (j=0; j<elements[0].n_stripes; j++) {
      if (j>2 && j<(elements[0].n_stripes-3))
        continue;
      printf("%d\t",j);
      printf("%#x - ", ((unsigned *)(elements[0].stripe_addr_start))[j]);
      printf("%#x\n",  ((unsigned *)(elements[0].stripe_addr_end))[j]);
      printf("\n");
    }
  }

  // write the img to a file
  //#define PATCH2FILE
  //#define IMG2FILE
#if defined(IMG2FILE) || defined(PATCH2FILE)

  FILE *fp;

  // open the file
  if((fp = fopen("img.h", "a")) == NULL) {
    printf("ERROR: Could not open img.h.\n");
    return -ENOENT;
  }

  printf("p_width  = %d\n",p_width);
  printf("p_height = %d\n",p_height);
  printf("i_step   = %d\n",i_step);

#if defined(PATCH2FILE)

  //write start
  printf(fp, "unsigned char img[%d] = {\n",p_width * p_height * n_channels);

  for (i=0; i<n_channels; i++) {
    fprintf(fp,"// Channel %d: \n",i);
    for (j=0; j<p_height; j++) {
      for (k=0; k<p_width; k++) {
        fprintf(fp,"\t%#x,",(unsigned) *(channels[i]+j*i_step+k));
      }
      fprintf(fp,"\n");
    }
  }

#elif defined(IMG2FILE)

  unsigned i_height, i_width;
  i_height = 21;
  i_width = 37;

  // write start
  fprintf(fp, "unsigned char img[%d] = {\n",i_height * i_step * n_channels);

  for (i=0; i<n_channels; i++) {
    //if ((i == 0) || (i == 16)) {
    fprintf(fp,"// Channel %d: \n",i);
    for (j=0; j<i_height; j++) {
      for (k=0; k<i_step; k++) {
        fprintf(fp,"\t%#x,",(unsigned) *(channels[i]+j*i_step+k));
      }
      fprintf(fp,"\n");
    }
    //}
  }

  // write end
  fprintf(fp, "};\n\n");
#endif

  fclose(fp);
#endif

  // make the request
  ioctl(pulp->fd,PULP_IOCTL_RAB_REQ_STRIPED,(unsigned *)&stripe_request);

  // free memory
  free((unsigned *)elements[0].stripe_addr_start);
  free((unsigned *)elements[0].stripe_addr_end);
  free(elements);

  return 0;
}

// vim: ts=2 sw=2 sts=2 et tw=100
