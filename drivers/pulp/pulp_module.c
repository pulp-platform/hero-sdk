/*
 * PULP device driver
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
/* --=========================================================================--
 *
 * ██████╗ ██╗   ██╗██╗     ██████╗     ██████╗ ██████╗ ██╗██╗   ██╗███████╗██████╗ 
 * ██╔══██╗██║   ██║██║     ██╔══██╗    ██╔══██╗██╔══██╗██║██║   ██║██╔════╝██╔══██╗
 * ██████╔╝██║   ██║██║     ██████╔╝    ██║  ██║██████╔╝██║██║   ██║█████╗  ██████╔╝
 * ██╔═══╝ ██║   ██║██║     ██╔═══╝     ██║  ██║██╔══██╗██║╚██╗ ██╔╝██╔══╝  ██╔══██╗
 * ██║     ╚██████╔╝███████╗██║         ██████╔╝██║  ██║██║ ╚████╔╝ ███████╗██║  ██║
 * ╚═╝      ╚═════╝ ╚══════╝╚═╝         ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═══╝  ╚══════╝╚═╝  ╚═╝
 *                                                                                  
 * 
 * Author: Pirmin Vogel - vogelpi@iis.ee.ethz.ch
 * 
 * Purpose : Linux kernel-level driver for PULP
 *           RAB management for shared virtual memory between host and PULP
 * 
 * --=========================================================================--
 */

/***************************************************************************************/

#include <linux/module.h>     /* Needed by all modules */
#include <linux/kernel.h>     /* KERN_ALERT, container_of */
#include <linux/kdev_t.h>     /* major, minor */
#include <linux/interrupt.h>  /* interrupt handling */
#include <linux/workqueue.h>  /* schedule non-atomic work */
#include <linux/wait.h>       /* wait_queue */
#include <asm/io.h>           /* ioremap, iounmap, iowrite32 */
#include <linux/cdev.h>       /* cdev struct */
#include <linux/fs.h>         /* file_operations struct */
#include <linux/mm.h>         /* vm_area_struct struct, page struct, PAGE_SHIFT, page_to_phys */
#include <linux/pagemap.h>    /* page_put() */
#include <asm/cacheflush.h>   /* __cpuc_flush_dcache_area, outer_cache.flush_range */
#include <asm/uaccess.h>      /* copy_to_user, copy_from_user, access_ok */
#include <linux/time.h>       /* do_gettimeofday */
#include <linux/ioctl.h>      /* ioctl */
#include <linux/slab.h>       /* kmalloc */
#include <linux/errno.h>      /* errno */
#include <linux/sched.h>      /* wake_up_interruptible(), TASK_INTERRUPTIBLE */
#include <linux/delay.h>      /* udelay */
#include <linux/device.h>     // class_create, device_create

/***************************************************************************************/
//#include <linux/wait.h>
#include <linux/ktime.h>      /* For ktime_get(), ktime_us_delta() */
/***************************************************************************************/

#include "pulp_module.h"
#include "pulp_mem.h"
#include "pulp_rab.h"
#include "pulp_dma.h"
#include "pulp_mbox.h"
#if PLATFORM == TE0808
  #include "pulp_smmu.h"
#endif

#include <linux/platform_device.h> /* for device tree stuff*/
#include <linux/device.h>
#include <linux/of_device.h>
#if PLATFORM == JUNO || PLATFORM == TE0808
  #include <asm/compat.h>            /* for compat_ioctl */
#endif

/***************************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pirmin Vogel");
MODULE_DESCRIPTION("PULP driver");

/***************************************************************************************/


// static variables {{{
static PulpDev my_dev;
static struct class *my_class;

static unsigned pulp_rab_ax_log_en = 0;
static unsigned gpio_value         = 0xC0000000;

// for DMA
static struct dma_chan * pulp_dma_chan[2];
static DmaCleanup pulp_dma_cleanup[2];
// }}}

// Device Tree {{{
/***********************************************************************************
 *
 * ██████╗ ███████╗██╗   ██╗██╗ ██████╗███████╗    ████████╗██████╗ ███████╗███████╗
 * ██╔══██╗██╔════╝██║   ██║██║██╔════╝██╔════╝    ╚══██╔══╝██╔══██╗██╔════╝██╔════╝
 * ██║  ██║█████╗  ██║   ██║██║██║     █████╗         ██║   ██████╔╝█████╗  █████╗
 * ██║  ██║██╔══╝  ╚██╗ ██╔╝██║██║     ██╔══╝         ██║   ██╔══██╗██╔══╝  ██╔══╝
 * ██████╔╝███████╗ ╚████╔╝ ██║╚██████╗███████╗       ██║   ██║  ██║███████╗███████╗
 * ╚═════╝ ╚══════╝  ╚═══╝  ╚═╝ ╚═════╝╚══════╝       ╚═╝   ╚═╝  ╚═╝╚══════╝╚══════╝
 *
 * The IRQ is allocated through the device tree at boot time. In order to
 * register the interrupt service routine, request_irq() needs the interrupt index
 * reserved at boot time instead of the physical IRQ. This can be obtained through
 * the device tree. The addresses we still "pass" through the header files.
 *
 ***********************************************************************************/
// connect to the device tree
static struct of_device_id pulp_of_match[] = {
  {
    .compatible = "pulp,bigpulp",
  }, { /* sentinel */}
};

MODULE_DEVICE_TABLE(of, pulp_of_match);

#if PLATFORM == TE0808

  /**
   * Compare device name with string.
   *
   * This function compares the name of a device with a string provided in data.
   * To protect the system from faulty device trees, the comparison is aborted
   * if a NULL pointer would have to be derefenced.
   *
   * @param   dev  Pointer to device struct of interest
   * @param   data Pointer to name of interest.
   *
   * @return  non-zero on success, 0 if the name does not match the string.
   */
  static int compare_dev_name(struct device *dev, void *data)
  {
    const char *name = data;

    // make sure to not dereference NULL pointers - in case we start with device zero
    if      ( dev->of_node == NULL ) {
      if (DEBUG_LEVEL_OF > 0)
        printk(KERN_INFO "PULP: No of_node for:\n - dev %p\n", dev);
      return 0;
    }
    else if ( dev->of_node->name == NULL ) {
      if (DEBUG_LEVEL_OF > 0)
        printk(KERN_INFO "PULP: No of_node->name for:\n - dev %p\n - of_node %p", dev, dev->of_node);
      return 0;
    }
    else
      return sysfs_streq(name, dev->of_node->name);
  }

  /**
   * Find a device by name in the device tree.
   *
   * @param   name Name of the device to search for
   * @param   dev  Pointer to struct device pointer to fill
   *
   * @return  0 on success, -ENXIO if the device cannot be found.
   */
  static int find_dev(const char * name, struct device ** dev)
  {
    // search the device on the platform bus using the custom compare function
    *dev = bus_find_device(&platform_bus_type, NULL, (void *)name, compare_dev_name);
    if (dev == NULL)
      return -ENODEV;

    return 0;
  }
#endif

// method definition
static int pulp_probe(struct platform_device *pdev)
{
#if PLATFORM == TE0808
  int err;
#endif

  printk(KERN_ALERT "PULP: Probing device.\n");

#if PLATFORM == TE0808
  // Get struct device pointer
  err = find_dev("pulp", &my_dev.dt_dev_ptr);
  if (err) {
    printk(KERN_WARNING "PULP: Could not get device struct pointer.\n");
    return -ENODEV;
  }

  if (DEBUG_LEVEL_OF > 0) {
    printk(KERN_INFO "PULP: &(pdev->dev)      = %p\n", &(pdev->dev));
    printk(KERN_INFO "PULP: my_dev.dt_dev_ptr = %p\n", my_dev.dt_dev_ptr);
  }
#endif

  // store device struct pointer
  my_dev.dt_dev_ptr = &(pdev->dev);

  // IRQ
  my_dev.intr_reg_irq = platform_get_irq(pdev,0);
  if (my_dev.intr_reg_irq <= 0) {
    printk(KERN_WARNING "PULP: Could not allocate IRQ resource.\n");
    return -ENODEV;
  }

#if PLATFORM == TE0808
  // DMA Channels

  // TX: Host -> PULP
  pulp_dma_chan[0] = dma_request_slave_channel(my_dev.dt_dev_ptr, "tx_channel");
  if (pulp_dma_chan[0] == NULL) {
    printk(KERN_WARNING "PULP: Could not allocate TX DMA channel.\n");
    return -ENODEV;
  }

  // RX: Host -> PULP
  pulp_dma_chan[1] = dma_request_slave_channel(my_dev.dt_dev_ptr, "rx_channel");
  if (pulp_dma_chan[1] == NULL) {
    printk(KERN_WARNING "PULP: Could not allocate RX DMA channel.\n");
    dma_release_channel(pulp_dma_chan[0]);
    return -ENODEV;
  }
#endif

  return 0;
}

static int pulp_remove(struct platform_device *pdev)
{
  printk(KERN_ALERT "PULP: Removing device.\n");

#if PLATFORM == TE0808
  // DMA Channels
  dma_release_channel(pulp_dma_chan[0]);
  dma_release_channel(pulp_dma_chan[1]);
#endif

  return 0;
}

// (un-)register to/from the device tree
static struct platform_driver pulp_platform_driver = {
  .probe  = pulp_probe,
  .remove = pulp_remove,
  .driver = {
    .name = "pulp",
    .owner = THIS_MODULE,
    .of_match_table = pulp_of_match,
    },
};

/***************************************************************************************/
// }}}

// VM_RESERVERD for mmap
#ifndef VM_RESERVED
  #define VM_RESERVED (VM_DONTEXPAND | VM_DONTDUMP)
#endif

// method declarations {{{
static int  pulp_open   (struct inode *inode, struct file *filp);
static int  pulp_release(struct inode *p_inode, struct file *filp);
static int  pulp_mmap   (struct file *filp, struct vm_area_struct *vma);
static long pulp_ioctl  (struct file *filp, unsigned int cmd, unsigned long arg);
#ifdef CONFIG_COMPAT
  static long pulp_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif

static irqreturn_t pulp_isr(int irq, void *ptr);

// }}}

// important structs {{{
struct file_operations pulp_fops = {
  .owner          = THIS_MODULE,
  .open           = pulp_open,
  .release        = pulp_release,
  .read           = pulp_mbox_read,
  .mmap           = pulp_mmap,
  .unlocked_ioctl = pulp_ioctl,
#ifdef CONFIG_COMPAT
  .compat_ioctl = pulp_compat_ioctl,
#endif
};
// }}}

// methods definitions

// init {{{
/***********************************************************************************
 *
 * ██╗███╗   ██╗██╗████████╗
 * ██║████╗  ██║██║╚══██╔══╝
 * ██║██╔██╗ ██║██║   ██║   
 * ██║██║╚██╗██║██║   ██║   
 * ██║██║ ╚████║██║   ██║   
 * ╚═╝╚═╝  ╚═══╝╚═╝   ╚═╝    
 *
 ***********************************************************************************/
static int __init pulp_init(void)
{
  int err;
  unsigned gpio;

  printk(KERN_ALERT "PULP: Loading device driver.\n");

  my_dev.minor = 0;
  
  /*
   *  init module char device
   */
  // get dynamic device numbers
  err = alloc_chrdev_region(&my_dev.dev, my_dev.minor, PULP_N_DEV_NUMBERS, "PULP");
  if (err) {
    printk(KERN_WARNING "PULP: Can't get major device number.\n");
    goto fail_alloc_chrdev_region;
  }
  // create class struct
  if ((my_class = class_create(THIS_MODULE, "pmca")) == NULL) {
    printk(KERN_WARNING "PULP: Error creating class.\n");
    err = -1;
    goto fail_create_class;
  }
  // create device and register it with sysfs
  if (device_create(my_class, NULL, my_dev.dev, NULL, "PULP") == NULL) {
    printk(KERN_WARNING "PULP: Error creating device.\n");
    err = -1;
    goto fail_create_device;
  }
  printk(KERN_INFO "PULP: Device created.\n");

  my_dev.major = MAJOR(my_dev.dev);
  my_dev.fops = &pulp_fops;

  /*
   *  struct cdev
   */
  cdev_init(&my_dev.cdev, my_dev.fops);
  my_dev.cdev.owner = THIS_MODULE;

  /**********
   *
   * ioremap
   *
   **********/
  my_dev.mbox = ioremap_nocache(MBOX_H_BASE_ADDR, MBOX_SIZE_B*2);
  if (DEBUG_LEVEL_PULP > 0)
    printk(KERN_INFO "PULP: Mailbox mapped to virtual kernel space @ %#lx.\n",
      (long unsigned int) my_dev.mbox);
  pulp_mbox_init(my_dev.mbox);

  #if PLATFORM == TE0808
    my_dev.cci = ioremap_nocache(CCI_BASE_ADDR, CCI_SIZE_B);
    if (DEBUG_LEVEL_PULP > 0)
      printk(KERN_INFO "PULP: CCI mapped to virtual kernel space @ %#lx.\n",
        (long unsigned int) my_dev.cci);

    /*
     * Make sure to enable AxDOMAIN shareability overriding on Slave 0 (HPC0/1 PL ports).
     *
     * Note: The CCI does not accept any shareable transactions from the SMMU. Thus, we
     * must force the SMMU to inject non-shareable transactions into CCI only. To enable
     * coherency, we then must configure the CCI to force the transactions coming from
     * the HPC0/1 ports to be shareable.
     *
     * This requires non-secure access to CCI registers being enabled.
     */
    iowrite32(0x3,(void *)((unsigned long)my_dev.cci+CCI_SHOR_S0_OFFSET_B));

    my_dev.smmu = ioremap_nocache(SMMU_BASE_ADDR, SMMU_SIZE_B);
    if (DEBUG_LEVEL_PULP > 0)
      printk(KERN_INFO "PULP: SMMU mapped to virtual kernel space @ %#lx.\n",
        (long unsigned int) my_dev.smmu);
    err = pulp_smmu_init(&my_dev);
    if (err) {
      printk(KERN_WARNING "PULP: Could not initialize SMMU.\n");
      goto fail_smmu_init;
    }
  #endif // PLATFORM

  my_dev.intr_reg = ioremap_nocache(INTR_REG_BASE_ADDR,INTR_REG_SIZE_B);
  if (DEBUG_LEVEL_PULP > 0)
    printk(KERN_INFO "PULP: Interrupt register mapped to virtual kernel space @ %#lx.\n",
      (long unsigned int) my_dev.intr_reg);

  #if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX
    my_dev.slcr = ioremap_nocache(SLCR_BASE_ADDR,SLCR_SIZE_B);
    if (DEBUG_LEVEL_PULP > 0)
      printk(KERN_INFO "PULP: Zynq SLCR mapped to virtual kernel space @ %#lx.\n",
        (long unsigned int) my_dev.slcr);
  
    // make sure to enable the PL clock
    if ( !BF_GET(ioread32((void *)((unsigned long)my_dev.slcr+SLCR_FPGA0_THR_STA_OFFSET_B)),16,1) ) {
      iowrite32(0x0,(void *)((unsigned long)my_dev.slcr+SLCR_FPGA0_THR_CNT_OFFSET_B));
  
      if ( !BF_GET(ioread32((void *)((unsigned long)my_dev.slcr+SLCR_FPGA0_THR_STA_OFFSET_B)),16,1) ) {
        printk(KERN_WARNING "PULP: Could not enable reference clock for PULP.\n");
        goto fail_ioremap;
      }
    }
  #endif // PLATFORM

  #if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX || PLATFORM == TE0808
    my_dev.uart = ioremap_nocache(HOST_UART_BASE_ADDR,HOST_UART_SIZE_B);
    if (DEBUG_LEVEL_PULP > 0)
      printk(KERN_INFO "PULP: Zynq UART control register mapped to virtual kernel space @ %#lx.\n",
        (long unsigned int) my_dev.uart);

    // make sure to enable automatic flow control on PULP -> Host UART
    iowrite32(0x20,(void *)((unsigned long)my_dev.uart+MODEM_CTRL_REG0_OFFSET_B));
  #endif // PLATFORM

  #if RAB_AX_LOG_EN == 1
    my_dev.rab_ar_log = ioremap_nocache(RAB_AR_LOG_BASE_ADDR, RAB_AX_LOG_SIZE_B);
    if (DEBUG_LEVEL_PULP > 0)
      printk(KERN_INFO "PULP: RAB AR log mapped to virtual kernel space @ %#lx.\n",
        (long unsigned int) my_dev.rab_ar_log);

    my_dev.rab_aw_log = ioremap_nocache(RAB_AW_LOG_BASE_ADDR, RAB_AX_LOG_SIZE_B);
    if (DEBUG_LEVEL_PULP > 0)
      printk(KERN_INFO "PULP: RAB AW log mapped to virtual kernel space @ %#lx.\n",
        (long unsigned int) my_dev.rab_aw_log);

    #if PLATFORM == JUNO //|| PLATFORM == TE0808
      my_dev.rab_cfg_log = ioremap_nocache(RAB_CFG_LOG_BASE_ADDR, RAB_CFG_LOG_SIZE_B);
      if (DEBUG_LEVEL_PULP > 0)
        printk(KERN_INFO "PULP: RAB CFG log mapped to virtual kernel space @ %#lx.\n",
          (long unsigned int) my_dev.rab_cfg_log);
    #endif
  #endif // RAB_AX_LOG_EN == 1

  my_dev.gpio = ioremap_nocache(H_GPIO_BASE_ADDR, H_GPIO_SIZE_B);
  if (DEBUG_LEVEL_PULP > 0)
    printk(KERN_INFO "PULP: Host GPIO mapped to virtual kernel space @ %#lx.\n",
      (long unsigned int) my_dev.gpio);
  
  // remove GPIO reset
  gpio = 0;
  BIT_SET(gpio,BF_MASK_GEN(GPIO_RST_N,1));
  BIT_SET(gpio,BF_MASK_GEN(GPIO_CLK_EN,1));
  iowrite32(gpio,(void *)((unsigned long)my_dev.gpio+0x8));
  
  // RAB config  
  my_dev.rab_config = ioremap_nocache(RAB_CONFIG_BASE_ADDR, RAB_CONFIG_SIZE_B);
  if (DEBUG_LEVEL_PULP > 0)
    printk(KERN_INFO "PULP: RAB config mapped to virtual kernel space @ %#lx.\n",
      (long unsigned int) my_dev.rab_config);
  err = pulp_rab_init(&my_dev);
  if (err) {
    printk(KERN_WARNING "PULP: RAB initialization failed.\n");
    goto fail_ioremap;
  }

  // PULP timer used for RAB profiling
  my_dev.clusters = ioremap_nocache(PULP_H_BASE_ADDR, CLUSTERS_SIZE_B);
  if (DEBUG_LEVEL_PULP > 0)
    printk(KERN_INFO "PULP: Clusters mapped to virtual kernel space @ %#lx.\n",
      (long unsigned int) my_dev.clusters);
  
  // actually not needed - handled in user space
  my_dev.soc_periph = ioremap_nocache(SOC_PERIPHERALS_H_BASE_ADDR, SOC_PERIPHERALS_SIZE_B);
  if (DEBUG_LEVEL_PULP > 0)
    printk(KERN_INFO "PULP: SoC peripherals mapped to virtual kernel space @ %#lx.\n",
      (long unsigned int) my_dev.soc_periph);
  
  // actually not needed - handled in user space
  my_dev.l2_mem = ioremap_nocache(L2_MEM_H_BASE_ADDR, L2_MEM_SIZE_B);
  if (DEBUG_LEVEL_PULP > 0)
    printk(KERN_INFO "PULP: L2 memory mapped to virtual kernel space @ %#lx.\n",
      (long unsigned int) my_dev.l2_mem);

  // for profiling
  my_dev.l3_mem = ioremap_nocache(L3_MEM_H_BASE_ADDR, L3_MEM_SIZE_B);
  if (my_dev.l3_mem == NULL) {
    printk(KERN_WARNING "PULP: ioremap_nocache not allowed for non-reserved RAM.\n");
    err = EPERM;
    goto fail_ioremap;
  }
  if (DEBUG_LEVEL_PULP > 0)
    printk(KERN_INFO "PULP: Shared L3 memory (DRAM) mapped to virtual kernel space @ %#lx.\n",
      (long unsigned int) my_dev.l3_mem);
 
  /*********************
   *
   * interrupts
   *
   *********************/
  // register the device to get the interrupt index
  err = platform_driver_register(&pulp_platform_driver);
  if (err) {
    printk(KERN_WARNING "PULP: Error registering platform driver: %d\n",err);
    goto fail_request_irq;
  }

  // request interrupts and install top-half handler
  err = request_irq(my_dev.intr_reg_irq, pulp_isr, 0 , "PULP", NULL);
  if (err) {
    printk(KERN_WARNING "PULP: Error requesting IRQ.\n");
    goto fail_request_irq;
  }

  /************************************
   *
   *  request DMA channels
   *
   ************************************/
  #if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX
    err = pulp_dma_chan_req(&pulp_dma_chan[0],0);
    if (err) {
      printk(KERN_WARNING "PULP: Error requesting DMA channel.\n");
      goto fail_request_dma;
    }
    err = pulp_dma_chan_req(&pulp_dma_chan[1],1);
    if (err) {
      printk(KERN_WARNING "PULP: Error requesting DMA channel.\n");
      goto fail_request_dma;
    }
  #endif // PLATFORM

  /************************************
   *
   *  tell the kernel about the device
   *
   ************************************/
  err = cdev_add(&my_dev.cdev, my_dev.dev, PULP_N_DEV_NUMBERS);
  if (err) {
    printk(KERN_WARNING "PULP: Error registering the device.\n");
    goto fail_register_device;
  }
  printk(KERN_INFO "PULP: Device registered.\n");

  return 0;

  /*
   * error handling
   */
  fail_register_device:
    #if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX
      pulp_dma_chan_clean(pulp_dma_chan[1]);
      pulp_dma_chan_clean(pulp_dma_chan[0]);
  fail_request_dma:
    #endif // PLATFORM
    free_irq(my_dev.intr_reg_irq,NULL);
    platform_driver_unregister(&pulp_platform_driver);
  fail_request_irq:
    #if defined(PROFILE_RAB_STR) || defined(PROFILE_RAB_MH)
      pulp_rab_prof_free();
    #endif
    iounmap(my_dev.rab_config);
    #if RAB_AX_LOG_EN == 1
      pulp_rab_ax_log_free();
      iounmap(my_dev.rab_ar_log);
      iounmap(my_dev.rab_aw_log);
      #if PLATFORM == JUNO //|| PLATFORM == TE0808
        iounmap(my_dev.rab_cfg_log);
      #endif
    #endif // RAB_AX_LOG_EN == 1
    iounmap(my_dev.intr_reg);
    #if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX
      iounmap(my_dev.slcr);
    #endif // PLATFORM
    #if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX || PLATFORM == TE0808
      iounmap(my_dev.uart);
    #endif // PLATFORM
    iounmap(my_dev.gpio);
    iounmap(my_dev.clusters);
    iounmap(my_dev.soc_periph);
    iounmap(my_dev.l2_mem);
    iounmap(my_dev.l3_mem);
    #if PLATFORM == TE0808
  fail_smmu_init:
      iounmap(my_dev.smmu);
      iounmap(my_dev.cci);
    #endif // PLATFORM
    iounmap(my_dev.mbox);
  fail_ioremap:
    cdev_del(&my_dev.cdev);
  fail_create_device: 
    class_destroy(my_class);
  fail_create_class: 
    unregister_chrdev_region(my_dev.dev, 1);
  fail_alloc_chrdev_region:
    return err;
}
module_init(pulp_init);
// }}}

// exit {{{
/***********************************************************************************
 *
 * ███████╗██╗  ██╗██╗████████╗
 * ██╔════╝╚██╗██╔╝██║╚══██╔══╝
 * █████╗   ╚███╔╝ ██║   ██║   
 * ██╔══╝   ██╔██╗ ██║   ██║   
 * ███████╗██╔╝ ██╗██║   ██║   
 * ╚══════╝╚═╝  ╚═╝╚═╝   ╚═╝   
 * 
 ***********************************************************************************/
static void __exit pulp_exit(void)
{
  unsigned gpio;

  pulp_mbox_clear();

  // reset PULP
  gpio = 0;
  iowrite32(gpio,(void *)((unsigned long)my_dev.gpio+0x8));

  // disable the reset
  BIT_SET(gpio,BF_MASK_GEN(GPIO_RST_N,1));
  BIT_SET(gpio,BF_MASK_GEN(GPIO_CLK_EN,1));
  iowrite32(gpio,(void *)((unsigned long)my_dev.gpio+0x8));

  printk(KERN_ALERT "PULP: Unloading device driver.\n");
  // undo __init pulp_init
  free_irq(my_dev.intr_reg_irq,NULL);
  platform_driver_unregister(&pulp_platform_driver);
  #if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX
    pulp_dma_chan_clean(pulp_dma_chan[1]);
    pulp_dma_chan_clean(pulp_dma_chan[0]);
  #endif // PLATFORM
  #if defined(PROFILE_RAB_STR) || defined(PROFILE_RAB_MH)
    pulp_rab_prof_free();
  #endif
  iounmap(my_dev.rab_config);
  #if RAB_AX_LOG_EN == 1
    pulp_rab_ax_log_free();
    iounmap(my_dev.rab_ar_log);
    iounmap(my_dev.rab_aw_log);
    #if PLATFORM == JUNO //|| PLATFORM == TE0808
      iounmap(my_dev.rab_cfg_log);
    #endif
  #endif // RAB_AX_LOG_EN == 1
  iounmap(my_dev.intr_reg);
  #if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX
    iounmap(my_dev.slcr);
  #endif // PLATFORM
  #if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX || PLATFORM == TE0808
    iounmap(my_dev.uart);
  #endif // PLATFORM
  iounmap(my_dev.gpio);
  iounmap(my_dev.clusters);
  iounmap(my_dev.soc_periph);
  iounmap(my_dev.l2_mem);
  iounmap(my_dev.l3_mem);
  iounmap(my_dev.mbox);
  #if PLATFORM == TE0808
    iounmap(my_dev.smmu);
    iounmap(my_dev.cci);
  #endif // PLATFORM
  cdev_del(&my_dev.cdev);
  device_destroy(my_class, my_dev.dev);
  class_destroy(my_class);
  unregister_chrdev_region(my_dev.dev, 1);
}
module_exit(pulp_exit);
// }}}

// open {{{
/***********************************************************************************
 *
 *  ██████╗ ██████╗ ███████╗███╗   ██╗
 * ██╔═══██╗██╔══██╗██╔════╝████╗  ██║
 * ██║   ██║██████╔╝█████╗  ██╔██╗ ██║
 * ██║   ██║██╔═══╝ ██╔══╝  ██║╚██╗██║
 * ╚██████╔╝██║     ███████╗██║ ╚████║
 *  ╚═════╝ ╚═╝     ╚══════╝╚═╝  ╚═══╝
 * 
 ***********************************************************************************/

int pulp_open(struct inode *inode, struct file *filp)
{
  // get a pointer to the PulpDev structure 
  PulpDev *dev;
  dev = container_of(inode->i_cdev, PulpDev, cdev);
  filp->private_data = dev;

  // // trim to 0 the length of the device if open is write_only
  // if ((filp->f_flags & O_ACCMODE) == O_WRONLY) {
  // 
  // }

  printk(KERN_INFO "PULP: Device opened.\n");
  
  return 0;
}
// }}}

// release {{{
/***********************************************************************************
 *
 * ██████╗ ███████╗██╗     ███████╗ █████╗ ███████╗███████╗
 * ██╔══██╗██╔════╝██║     ██╔════╝██╔══██╗██╔════╝██╔════╝
 * ██████╔╝█████╗  ██║     █████╗  ███████║███████╗█████╗  
 * ██╔══██╗██╔══╝  ██║     ██╔══╝  ██╔══██║╚════██║██╔══╝  
 * ██║  ██║███████╗███████╗███████╗██║  ██║███████║███████╗
 * ╚═╝  ╚═╝╚══════╝╚══════╝╚══════╝╚═╝  ╚═╝╚══════╝╚══════╝
 * 
 ***********************************************************************************/

int pulp_release(struct inode *p_inode, struct file *filp)
{
  pulp_mbox_clear();
  pulp_rab_release();

  printk(KERN_INFO "PULP: Device released.\n");
 
  return 0;
}
// }}}

// mmap {{{
/***********************************************************************************
 *
 * ███╗   ███╗███╗   ███╗ █████╗ ██████╗ 
 * ████╗ ████║████╗ ████║██╔══██╗██╔══██╗
 * ██╔████╔██║██╔████╔██║███████║██████╔╝
 * ██║╚██╔╝██║██║╚██╔╝██║██╔══██║██╔═══╝ 
 * ██║ ╚═╝ ██║██║ ╚═╝ ██║██║  ██║██║     
 * ╚═╝     ╚═╝╚═╝     ╚═╝╚═╝  ╚═╝╚═╝     
 *
 ***********************************************************************************/
int pulp_mmap(struct file *filp, struct vm_area_struct *vma)
{
  unsigned long base_addr; // base address to use for the remapping
  unsigned long size_b;    // size for the remapping
  unsigned long off;       // address offset in VMA
  unsigned long physical;  // PFN of physical page
  unsigned long vsize;
  unsigned long psize;
  char type[12]; 
   
  off = (vma->vm_pgoff) << PAGE_SHIFT; 
  
  /*
   * based on the offset, set the proper base_addr and size_b, and adjust off  
   */
  // PULP internals
  if (off < CLUSTERS_SIZE_B) {
    // Clusters
    base_addr = PULP_H_BASE_ADDR;
    size_b = CLUSTERS_SIZE_B;
    strcpy(type,"Clusters");
  }
  else if (off < (CLUSTERS_SIZE_B + SOC_PERIPHERALS_SIZE_B)) {
    // SoC peripherals
    base_addr = SOC_PERIPHERALS_H_BASE_ADDR;
    off = off - CLUSTERS_SIZE_B;
    size_b = SOC_PERIPHERALS_SIZE_B;
    strcpy(type,"Peripherals");
  }
  else if (off < (CLUSTERS_SIZE_B + SOC_PERIPHERALS_SIZE_B + MBOX_SIZE_B)) {
    // Mailbox
    base_addr = MBOX_H_BASE_ADDR;
    off = off - CLUSTERS_SIZE_B - SOC_PERIPHERALS_SIZE_B;
    size_b = MBOX_SIZE_B;
    strcpy(type,"Mailbox");
  }
  else if (off < (CLUSTERS_SIZE_B + SOC_PERIPHERALS_SIZE_B + MBOX_SIZE_B + L2_MEM_SIZE_B)) {
    // L2
    base_addr = L2_MEM_H_BASE_ADDR;
    off = off - CLUSTERS_SIZE_B - SOC_PERIPHERALS_SIZE_B - MBOX_SIZE_B;
    size_b = L2_MEM_SIZE_B;
    strcpy(type,"L2");
  }
  // Platform
  else if (off < (CLUSTERS_SIZE_B + SOC_PERIPHERALS_SIZE_B + MBOX_SIZE_B + L2_MEM_SIZE_B + L3_MEM_SIZE_B)) {
    // Shared L3
    base_addr = L3_MEM_H_BASE_ADDR;
    off = off - CLUSTERS_SIZE_B - SOC_PERIPHERALS_SIZE_B - MBOX_SIZE_B - L2_MEM_SIZE_B;
    size_b = L3_MEM_SIZE_B;
    strcpy(type,"Shared L3");
  }
  // PULP external, PULPEmu 
  else if (off < (CLUSTERS_SIZE_B + SOC_PERIPHERALS_SIZE_B + MBOX_SIZE_B + L2_MEM_SIZE_B + L3_MEM_SIZE_B 
                  + H_GPIO_SIZE_B)) {
    // H_GPIO
    base_addr = H_GPIO_BASE_ADDR;
    off = off - CLUSTERS_SIZE_B - SOC_PERIPHERALS_SIZE_B - MBOX_SIZE_B - L2_MEM_SIZE_B - L3_MEM_SIZE_B;
    size_b = H_GPIO_SIZE_B;
    strcpy(type,"Host GPIO");
  }
  else if (off < (CLUSTERS_SIZE_B + SOC_PERIPHERALS_SIZE_B + MBOX_SIZE_B + L2_MEM_SIZE_B + L3_MEM_SIZE_B 
                  + H_GPIO_SIZE_B + CLKING_SIZE_B)) {
    // CLKING
    base_addr = CLKING_BASE_ADDR;
    off = off - CLUSTERS_SIZE_B - SOC_PERIPHERALS_SIZE_B - MBOX_SIZE_B - L2_MEM_SIZE_B - L3_MEM_SIZE_B 
      - H_GPIO_SIZE_B;
    size_b = CLKING_SIZE_B;
    strcpy(type,"Clking");
  }
  else if (off < (CLUSTERS_SIZE_B + SOC_PERIPHERALS_SIZE_B + MBOX_SIZE_B + L2_MEM_SIZE_B + L3_MEM_SIZE_B 
                  + H_GPIO_SIZE_B + CLKING_SIZE_B + RAB_CONFIG_SIZE_B)) {
    // RAB config
    base_addr = RAB_CONFIG_BASE_ADDR;
    off = off - CLUSTERS_SIZE_B - SOC_PERIPHERALS_SIZE_B - MBOX_SIZE_B - L2_MEM_SIZE_B - L3_MEM_SIZE_B 
      - H_GPIO_SIZE_B - CLKING_SIZE_B;
    size_b = RAB_CONFIG_SIZE_B;
    strcpy(type,"RAB config");
  }
  #if PLATFORM == ZEDBOARD || PLATFORM == ZC706 || PLATFORM == MINI_ITX
    // Zynq System
    else if (off < (CLUSTERS_SIZE_B + SOC_PERIPHERALS_SIZE_B + MBOX_SIZE_B + L2_MEM_SIZE_B + L3_MEM_SIZE_B 
                    + H_GPIO_SIZE_B + CLKING_SIZE_B + RAB_CONFIG_SIZE_B + SLCR_SIZE_B)) {
      // Zynq SLCR
      base_addr = SLCR_BASE_ADDR;
      off = off - CLUSTERS_SIZE_B - SOC_PERIPHERALS_SIZE_B - MBOX_SIZE_B - L2_MEM_SIZE_B - L3_MEM_SIZE_B 
        - H_GPIO_SIZE_B - CLKING_SIZE_B - RAB_CONFIG_SIZE_B;
      size_b = SLCR_SIZE_B;
      strcpy(type,"Zynq SLCR");
    }
  #endif // PLATFORM
  else {
    printk(KERN_INFO "PULP: Invalid VMA offset for mmap.\n");
    return -EINVAL;
  }

  // set physical PFN and virtual size
  physical = (base_addr + off) >> PAGE_SHIFT;
  vsize = vma->vm_end - vma->vm_start;
  psize = size_b - off;

  // set protection flags to avoid caching and paging
  vma->vm_flags |= VM_IO | VM_RESERVED;
  vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

  if (DEBUG_LEVEL_PULP > 0) {
    printk(KERN_INFO
      "PULP: %s memory mapped. \nPhysical address = %#lx, user-space virtual address = %#lx, vsize = %#lx.\n",
      type, physical << PAGE_SHIFT, vma->vm_start, vsize);
  }
  
  if (vsize > psize)
    return -EINVAL; /*  spans too high */
  
  // map physical kernel space addresses to virtual user space addresses
  remap_pfn_range(vma, vma->vm_start, physical, vsize, vma->vm_page_prot);

  return 0;
}

// // avoid the extension of the mapping using mremap
// //   to avoid remapping past the end of the physical device area
// struct page *simple_nopage(struct vm_area_struct *vma, unsigned long address, int *type);
// {
//   return NOPAGE_SIGBUS;
// }

// }}}

// isr {{{
/***********************************************************************************
 *
 * ██╗███████╗██████╗ 
 * ██║██╔════╝██╔══██╗
 * ██║███████╗██████╔╝
 * ██║╚════██║██╔══██╗
 * ██║███████║██║  ██║
 * ╚═╝╚══════╝╚═╝  ╚═╝
 *
 ***********************************************************************************/
irqreturn_t pulp_isr(int irq, void *ptr)
{
  unsigned long intr_reg_value;
  struct timeval time;
  int i;
  unsigned rab_mh;

  // read and clear the interrupt register
  intr_reg_value = IOREAD_L((void *)(unsigned long)my_dev.intr_reg);
  //printk(KERN_INFO "PULP: Interrupt status %#lx \n",intr_reg_value);

 /*********************
  *
  * top-half handler
  *
  *********************/
  do_gettimeofday(&time);

  if ( BF_GET(intr_reg_value,INTR_MBOX,1) ) { // mailbox
    pulp_mbox_intr(my_dev.mbox);
  }
  if ( BF_GET(intr_reg_value,INTR_RAB_MISS,1) ) { // RAB miss

    rab_mh = pulp_rab_mh_sched();

    if ( (DEBUG_LEVEL_RAB_MH > 1) && (rab_mh == 1) ) {
      if ( printk_ratelimit() ) {
        printk(KERN_INFO "PULP: RAB miss interrupt handled at %02li:%02li:%02li.\n",
          (time.tv_sec / 3600) % 24, (time.tv_sec / 60) % 60, time.tv_sec % 60);

        // for debugging
        pulp_rab_mapping_print(my_dev.rab_config,0xAAAA);
      }
    }
  }
  if ( BF_GET(intr_reg_value,INTR_RAB_MHR_FULL,1) ) { // RAB mhr full
    printk(KERN_ALERT "PULP: RAB mhr full interrupt received at %02li:%02li:%02li.\n",
      (time.tv_sec / 3600) % 24, (time.tv_sec / 60) % 60, time.tv_sec % 60);
  }
  if ( BF_GET(intr_reg_value,INTR_RAB_MULTI,1) ) { // RAB multi
    printk(KERN_ALERT "PULP: RAB multi interrupt received at %02li:%02li:%02li.\n",
      (time.tv_sec / 3600) % 24, (time.tv_sec / 60) % 60, time.tv_sec % 60);
  }
  if ( BF_GET(intr_reg_value,INTR_RAB_PROT,1) ) { // RAB prot
    printk(KERN_ALERT "PULP: RAB prot interrupt received at %02li:%02li:%02li.\n",
      (time.tv_sec / 3600) % 24, (time.tv_sec / 60) % 60, time.tv_sec % 60);
  }
  if ( BF_GET(intr_reg_value, INTR_EOC_0, INTR_EOC_N-INTR_EOC_0+1) ) { // EOC
    for (i=0; i<N_CLUSTERS; i++) {
      if ( BF_GET(intr_reg_value, INTR_EOC_0+i, 1) ) {
        printk(KERN_INFO "PULP: End of Computation Cluster %i at %02li:%02li:%02li.\n",
          i, (time.tv_sec / 3600) % 24, (time.tv_sec / 60) % 60, time.tv_sec % 60);
      }
    }
    #if RAB_AX_LOG_EN == 1
      if (pulp_rab_ax_log_en)
        pulp_rab_ax_log_read(gpio_value, 1);
    #endif
  }
  #if RAB_AX_LOG_EN == 1
    if ( BF_GET(intr_reg_value, INTR_RAB_AR_LOG_FULL, 1)
      || BF_GET(intr_reg_value, INTR_RAB_AW_LOG_FULL, 1)
      || BF_GET(intr_reg_value, INTR_RAB_CFG_LOG_FULL, 1) ) {
      printk(KERN_INFO "PULP: RAB AX log full interrupt received at %02li:%02li:%02li.\n",
        (time.tv_sec / 3600) % 24, (time.tv_sec / 60) % 60, time.tv_sec % 60);

      pulp_rab_ax_log_read(gpio_value, 1);
    }
  #endif

  return IRQ_HANDLED;
}
// }}}

// ioctl {{{
/***********************************************************************************
 *
 * ██╗ ██████╗  ██████╗████████╗██╗     
 * ██║██╔═══██╗██╔════╝╚══██╔══╝██║     
 * ██║██║   ██║██║        ██║   ██║     
 * ██║██║   ██║██║        ██║   ██║     
 * ██║╚██████╔╝╚██████╗   ██║   ███████╗
 * ╚═╝ ╚═════╝  ╚═════╝   ╚═╝   ╚══════╝
 *
 ***********************************************************************************/
#ifdef CONFIG_COMPAT
long pulp_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  return pulp_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

long pulp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  int err = 0, i, j;
  long retval = 0;
  
  // to read from user space
  unsigned request[3];
  unsigned long n_bytes_read, n_bytes_left;
  unsigned byte;

  // what we get from user space
  unsigned size_b;
   
  // what get_user_pages needs
  struct page ** pages;
  unsigned       len; 
  
  // what mem_map_sg needs needs
  unsigned *      addr_start_vec;
  unsigned *      addr_end_vec;
  unsigned long * addr_offset_vec;
  unsigned *      page_idxs_start;
  unsigned *      page_idxs_end;
  unsigned        n_segments;

  // needed for cache flushing
  unsigned offset_start, offset_end;

  // needed for DMA management
  struct dma_async_tx_descriptor ** descs;
  unsigned addr_l3, addr_pulp;
  unsigned char dma_cmd;
  unsigned addr_src, addr_dst;

#ifdef PROFILE_DMA
  unsigned long ret;
  ktime_t time_dma_start, time_dma_end;
  unsigned time_dma_acc;
  time_dma_acc = 0;
#endif

  /*
   * extract the type and number bitfields, and don't decode wrong
   * cmds: return ENOTTY before access_ok()
   */
  if (_IOC_TYPE(cmd) != PULP_IOCTL_MAGIC) return -ENOTTY;
  if ( (_IOC_NR(cmd) < PULP_IOC_NR_MIN) | (_IOC_NR(cmd) > PULP_IOC_NR_MAX) ) return -ENOTTY;

  /*
   * the direction is a bitmask, and VERIFY_WRITE catches R/W
   * transfers. 'Type' is user-oriented, while access_ok is
   * kernel-oriented, so the concept of "read" and "write" is reversed
   */
  if (_IOC_DIR(cmd) & _IOC_READ)
    err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
  else if (_IOC_DIR(cmd) & _IOC_WRITE)
    err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
  if (err) return -EFAULT;

  // the actual ioctls {{{
  switch(cmd) {

  case PULP_IOCTL_RAB_REQ: // request new RAB slices

    retval = pulp_rab_req(my_dev.rab_config, arg);

    break;

  case PULP_IOCTL_RAB_FREE: // free RAB slices based on time code

    pulp_rab_free(my_dev.rab_config, arg);

    #if RAB_AX_LOG_EN == 1
      if (pulp_rab_ax_log_en) {
        pulp_rab_ax_log_read(gpio_value, 1);
        pulp_rab_ax_log_print();
      }
    #endif

    #if defined(PROFILE_RAB_STR) || defined(PROFILE_RAB_MH)
      pulp_rab_prof_print();
    #endif

    break;
    
  case PULP_IOCTL_RAB_REQ_STRIPED: // request striped RAB slices

    retval = pulp_rab_req_striped(my_dev.rab_config, arg);

    break;

  case PULP_IOCTL_RAB_FREE_STRIPED: // free striped RAB slices

    pulp_rab_free_striped(my_dev.rab_config, arg);

    #if RAB_AX_LOG_EN == 1
      if (pulp_rab_ax_log_en) {
        pulp_rab_ax_log_read(gpio_value, 1);
        pulp_rab_ax_log_print();
      }
    #endif

    #if defined(PROFILE_RAB_STR) || defined(PROFILE_RAB_MH)
      pulp_rab_prof_print();
    #endif

    break;

  // DMA_XFER {{{
  case PULP_IOCTL_DMA_XFER_SYNC:
    printk(KERN_WARNING "PULP - DMA: IOCTL call not implemented.\n");
    break;

  case PULP_IOCTL_DMA_XFER_ASYNC: // setup a transfer using the host DMA engine
  
    // get transfer data from user space - arg already checked above
    byte = 0;
    n_bytes_left = 3*sizeof(unsigned);
    n_bytes_read = n_bytes_left;
    while (n_bytes_read > 0) {
      n_bytes_left = __copy_from_user((void *)((char *)request+byte),
                                      (void __user *)((char *)arg+byte), n_bytes_read);
      byte += (n_bytes_read - n_bytes_left);
      n_bytes_read = n_bytes_left;
    }
    
    addr_l3   = request[0];
    addr_pulp = request[1];
    
    size_b = request[2] & 0x7FFFFFFF;
    dma_cmd = (request[2] >> 31); 
    
    if (DEBUG_LEVEL_DMA > 0) {
      printk(KERN_INFO "PULP: New DMA request:\n");
      printk(KERN_INFO "PULP: addr_l3   = %#x.\n",addr_l3);
      printk(KERN_INFO "PULP: addr_pulp = %#x.\n",addr_pulp);
      printk(KERN_INFO "PULP: size_b = %#x.\n",size_b);
      printk(KERN_INFO "PULP: dma_cmd = %#x.\n",dma_cmd);
    }
    
    // number of pages
    len = pulp_mem_get_num_pages(addr_l3, size_b);
    
    // get and lock user-space pages
    err = pulp_mem_get_user_pages(&pages, addr_l3, len, dma_cmd);
    if (err) {
      printk(KERN_WARNING "PULP: Locking of user-space pages failed.\n");
      return err;
    }
   
    // virtual to physcial address translation + segmentation
    n_segments = pulp_mem_map_sg(&addr_start_vec, &addr_end_vec, &addr_offset_vec,
                                 &page_idxs_start, &page_idxs_end, &pages, len, 
                                 addr_l3, addr_l3+size_b);
    if ( n_segments < 1 ) {
      printk(KERN_WARNING "PULP: Virtual to physical address translation failed.\n");
      return n_segments;
    }
    
    // allocate memory to hold the transaction descriptors
    descs = (struct dma_async_tx_descriptor **)
      kmalloc((size_t)(n_segments*sizeof(struct dma_async_tx_descriptor *)),GFP_KERNEL);

    // prepare cleanup
    pulp_dma_cleanup[dma_cmd].descs = descs;
    pulp_dma_cleanup[dma_cmd].pages = pages;
    pulp_dma_cleanup[dma_cmd].n_pages = len;

    /*
     *  setup the transfers
     */ 
    size_b = 0;
    for (i=0;i<n_segments;i++) {
  
      addr_pulp += size_b;

      if (dma_cmd) { // PULP -> L3 // not yet tested
        addr_src = addr_pulp;
        addr_dst = addr_offset_vec[i];
      }
      else { // L3 -> PULP
        addr_src = addr_offset_vec[i]; 
        addr_dst = addr_pulp;
      }
      size_b = addr_end_vec[i] - addr_start_vec[i];

      // flush caches
      for (j=page_idxs_start[i]; j<(page_idxs_end[i]+1); j++) {
        // flush the whole page?
        if (!i) 
          offset_start = BF_GET(addr_start_vec[i],0,PAGE_SHIFT);
        else
          offset_start = 0;
  
        if ( i == (n_segments-1) )
          offset_end = BF_GET(addr_end_vec[i],0,PAGE_SHIFT);
        else 
          offset_end = PAGE_SIZE;
  
        pulp_mem_cache_flush(pages[j],offset_start,offset_end);
      }

      // prepare the transfers, fill the descriptors
      err = pulp_dma_xfer_prep(&descs[i], &pulp_dma_chan[dma_cmd],
                               addr_dst, addr_src, size_b, (i == n_segments-1));
      if (err) {
        printk(KERN_WARNING "PULP: Could not setup DMA transfer.\n");
        return err;
      }    
  
      // set callback parameters for last transaction
      if ( i == (n_segments-1) ) {
        descs[i]->callback = (dma_async_tx_callback)pulp_dma_xfer_cleanup;
        descs[i]->callback_param = &pulp_dma_cleanup[dma_cmd];
      }

      // submit the transaction
      descs[i]->cookie = dmaengine_submit(descs[i]);
    }
    if (DEBUG_LEVEL_DMA > 1)
      printk(KERN_INFO "PULP: DMA transactions for %i segments submitted.\n",n_segments);

#ifdef PROFILE_DMA
    time_dma_start = ktime_get();
#endif      
    // issue pending DMA requests and wait for callback notification
    dma_async_issue_pending(pulp_dma_chan[dma_cmd]);

    if (DEBUG_LEVEL_DMA > 1)
      printk(KERN_INFO "PULP: Pending DMA transactions issued.\n");

#ifdef PROFILE_DMA 
    // wait for finish
    for (j=0;j<100000;j++) {
      ret = dma_async_is_tx_complete(pulp_dma_chan[dma_cmd],descs[n_segments-1]->cookie,NULL,NULL);
      if (!ret)
        break;
      udelay(10);
    }
    // free transaction descriptors array
    kfree(descs); 

    // time measurement
    time_dma_end = ktime_get();
    time_dma_acc = ktime_us_delta(time_dma_end,time_dma_start);
    
    printk("PULP - DMA: size = %d [bytes]\n",request[2] & 0x7FFFFFFF);
    printk("PULP - DMA: time = %d [us]\n",time_dma_acc);
#endif
      
    break;

  case PULP_IOCTL_DMA_XFER_WAIT:
    printk(KERN_WARNING "PULP - DMA: IOCTL call not implemented.\n");
    break;
  // }}}

  case PULP_IOCTL_RAB_MH_ENA:
    retval = pulp_rab_mh_ena(my_dev.rab_config, arg);
    break;

  case PULP_IOCTL_RAB_MH_DIS:
    pulp_rab_mh_dis();
    break;

  case PULP_IOCTL_RAB_SOC_MH_ENA:
    retval = pulp_rab_soc_mh_ena(my_dev.rab_config, arg & 1);
    break;

  case PULP_IOCTL_RAB_SOC_MH_DIS:
    retval = pulp_rab_soc_mh_dis(my_dev.rab_config);
    break;

  case PULP_IOCTL_SMMU_ENA:
    #if PLATFORM == TE0808
      retval = pulp_smmu_ena(&my_dev, BIT_GET(arg, (SMMU_FLAGS_CC | SMMU_FLAGS_SHPT_EMU)));
    #endif
    break;

  case PULP_IOCTL_SMMU_DIS:
    #if PLATFORM == TE0808
      retval = pulp_smmu_dis(&my_dev);
    #endif
    break;

  case PULP_IOCTL_INFO_PASS: // pass info from user to kernel space

    // get gpio value from user space - arg already checked above
    byte = 0;
    n_bytes_left = 1*sizeof(unsigned); 
    n_bytes_read = n_bytes_left;    
    while (n_bytes_read > 0) {
      n_bytes_left = __copy_from_user((void *)((char *)request+byte),
                             (void __user *)((char *)arg+byte), n_bytes_read);
      byte += (n_bytes_read - n_bytes_left);
      n_bytes_read = n_bytes_left;
    }

    gpio_value = request[0];

    BIT_SET(gpio_value,BF_MASK_GEN(GPIO_RST_N,1));
    BIT_SET(gpio_value,BF_MASK_GEN(GPIO_CLK_EN,1));

    pulp_rab_ax_log_en = (gpio_value >> GPIO_RAB_AX_LOG_EN) & 0x1;

    break;

  case PULP_IOCTL_RAB_AX_LOG_READ: // userspace wants to read the AX log buffers
    #if RAB_AX_LOG_EN == 1
      pulp_rab_ax_log_to_user(arg);
    #endif
    break;

  default:
    return -ENOTTY;
  }
  // }}}

  return retval;
}
// }}}

// vim: ts=2 sw=2 sts=2 et foldmethod=marker tw=100
