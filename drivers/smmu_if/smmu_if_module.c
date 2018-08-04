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
#include <linux/module.h>     /* Needed by all modules */
#include <linux/kernel.h>     /* KERN_ALERT, container_of */
#include <linux/kdev_t.h>     /* MAJOR, MINOR */
//#include <asm/io.h>           /* ioremap, iounmap, iowrite32 */
#include <linux/cdev.h>       /* cdev struct */
#include <linux/fs.h>         /* file_operations struct */
#include <asm/uaccess.h>      /* copy_to_user, copy_from_user,access_ok */
#include <linux/delay.h>      /* msleep */
#include <linux/device.h>     /* class_create, device_create */
#include <linux/proc_fs.h>    /* for /proc */
#include <linux/mm.h>         /* vm_area_struct struct, page struct, PAGE_SHIFT, pageo_phys */
#include <asm/uaccess.h>      /* write */
#include <linux/spinlock.h>

#include <linux/platform_device.h>

#include <linux/iommu.h>      /* iommu stuff */

#include "smmu_if_module.h"

/***************************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pirmin Vogel <vogelpi@iis.ee.ethz.ch>");
MODULE_DESCRIPTION("ARM SMMU User-Space Interface Module");

/***************************************************************************************
 *
 * ██████╗ ███████╗██╗   ██╗    ██████╗ ████████╗██████╗ 
 * ██╔══██╗██╔════╝██║   ██║    ██╔══██╗╚══██╔══╝██╔══██╗
 * ██║  ██║█████╗  ██║   ██║    ██████╔╝   ██║   ██████╔╝
 * ██║  ██║██╔══╝  ╚██╗ ██╔╝    ██╔═══╝    ██║   ██╔══██╗
 * ██████╔╝███████╗ ╚████╔╝     ██║        ██║   ██║  ██║
 * ╚═════╝ ╚══════╝  ╚═══╝      ╚═╝        ╚═╝   ╚═╝  ╚═╝
 * 
 * We need to get a pointer to the device struct of the SMMU, which is to be passed to the 
 * IOMMU API.
 *                                                      
 ***************************************************************************************/

static int compare_dev_name(struct device *dev, void *data)
{
  // this function implements the comaparison logic. Return not zero if found.
  const char *name = data;

  return sysfs_streq(name, dev->of_node->name);
}

static struct device *find_dev(const char * name)
{
  struct device *dev = bus_find_device(&platform_bus_type, NULL, (void *)name, compare_dev_name);

  return dev;
}

/***************************************************************************************/

// connect to the device tree
static struct of_device_id smmu_if_of_match[] = {
  {
    .compatible = "smmu_if,smmu_if_zynqmp",
  }, { /* sentinel */}
};

MODULE_DEVICE_TABLE(of, smmu_if_of_match);

// method definition
static int smmu_if_probe(struct platform_device *pdev)
{
  printk(KERN_ALERT "SMMU_IF: Probing device.\n");

  return 0;
}

static int smmu_if_remove(struct platform_device *pdev)
{
  printk(KERN_ALERT "SMMU_IF: Removing device.\n");

  return 0;
}

// (un-)register to/from the device tree
static struct platform_driver smmu_if_platform_driver = {
  .probe  = smmu_if_probe,
  .remove = smmu_if_remove,
  .driver = {
    .name = "smmu_if",
    .owner = THIS_MODULE,
    .of_match_table = smmu_if_of_match,
    },
};

/***************************************************************************************/

// methods declarations
int     smmu_if_open       (struct inode *inode, struct file *filp);
int     smmu_if_release    (struct inode *p_inode, struct file *filp);
ssize_t smmu_if_write      (struct file *filp, const char __user *buf, size_t count, loff_t * offp);
int     smmu_fault_handler (struct iommu_domain *smmu_domain_ptr, struct device *smmu_dev_ptr, 
                            unsigned long iova, int hmkey, void * smmu_token_ptr);

void    smmu_fault_handler_worker(void);

// important structs
struct file_operations smmu_if_fops = {
  .owner   = THIS_MODULE,
  .open    = smmu_if_open,
  .release = smmu_if_release,
  .write   = smmu_if_write,
};

// global variables
SmmuIfDev my_dev;

// static variables
static struct class *my_class;

// for iommu attach/detach
static struct iommu_domain * smmu_domain_ptr;
static struct device *       smmu_dev_ptr;

// for fault handler
static int   smmu_token[16];
static void *smmu_token_ptr = (void *)&smmu_token;

static struct task_struct *user_task;
static struct mm_struct *user_mm;

// for fault handler worker
static char   smmu_fault_handler_wq_name[11] = "SMMU_IF_WQ";
static struct workqueue_struct *smmu_fault_handler_wq = 0;
static struct work_struct       smmu_fault_handler_w;

// lock for synchronization of fault handler (interrupt context) and worker thread
DEFINE_SPINLOCK(smmu_fault_lock);
static WorkerStatus  smmu_fault_status;
static int           smmu_fault_ret;
static unsigned long iova_faulty;

static unsigned long iova_array[N_PAGES];
static struct page  *pages_ptrs[N_PAGES]; 
static unsigned int  page_idx = 0;

static unsigned long user_offset;

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
static int __init smmu_if_init(void) {
  
  int err, i;
  printk(KERN_ALERT "SMMU_IF: Loading device driver.\n");

  my_dev.minor = 0;
  
  /*
   *  init module char device
   */
  // get dynamic device numbers
  err = alloc_chrdev_region(&my_dev.dev, my_dev.minor, SMMU_IF_N_DEV_NUMBERS, "SMMU_IF");
  if (err) {
    printk(KERN_WARNING "SMMU_IF: Can't get major device number.\n");
    goto fail_alloc_chrdev_region;
  }
  // create class struct
  if ((my_class = class_create(THIS_MODULE, "chardrv")) == NULL) {
    printk(KERN_WARNING "SMMU_IF: Error creating class.\n");
    err = -1;
    goto fail_create_class;
  }
  // create device and register it with sysfs
  if (device_create(my_class, NULL, my_dev.dev, NULL, "SMMU_IF") == NULL) {
    printk(KERN_WARNING "SMMU_IF: Error creating device.\n");
    err = -1;
    goto fail_create_device;
  }
  printk(KERN_INFO "SMMU_IF: Device created.\n");

  my_dev.major = MAJOR(my_dev.dev);
  my_dev.fops = &smmu_if_fops;

  /*
   *  struct cdev
   */
  cdev_init(&my_dev.cdev, my_dev.fops);
  my_dev.cdev.owner = THIS_MODULE;

  /*
   *  tell the kernel about the device
   */
  err = platform_driver_register(&smmu_if_platform_driver);
  if (err) {
    printk(KERN_WARNING "SMMU_IF: Error registering platform driver: %d\n",err);
    goto fail_register_device;
  }

  err = cdev_add(&my_dev.cdev, my_dev.dev, SMMU_IF_N_DEV_NUMBERS);
  if (err) {
    printk(KERN_WARNING "SMMU_IF: Error registering the device.\n");
    goto fail_register_device;
  }
  printk(KERN_INFO "SMMU_IF: Device registered.\n");

  /*
   *  create the /proc entry
   */
  if (proc_create("smmu_if", 0, NULL, &smmu_if_fops) == NULL) {
    printk(KERN_WARNING "SMMU_IF: /proc entry could not be created");
    goto fail_create_proc;
  }

  // get the device pointer to the ARM SMMU
  smmu_dev_ptr = find_dev("smmu_if");

  // init iova_array
  for (i = 0; i<N_PAGES; i++)
    iova_array[i] = 0;

  return 0;

  /*
   * error handling
   */
 fail_create_proc:
  cdev_del(&my_dev.cdev);
  platform_driver_unregister(&smmu_if_platform_driver);
 fail_register_device:
 fail_create_device: 
  class_destroy(my_class);
 fail_create_class: 
  unregister_chrdev_region(my_dev.dev, 1);
 fail_alloc_chrdev_region:
  return err;
}
module_init(smmu_if_init);
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
static void __exit smmu_if_exit(void) {

  printk(KERN_ALERT "SMMU_IF: Unloading device driver.\n");

  /*
   * remove the /proc entry
   */
  remove_proc_entry("smmu_if", NULL);

  // undo __init smmu_if_init
  cdev_del(&my_dev.cdev);
  platform_driver_unregister(&smmu_if_platform_driver);
  device_destroy(my_class, my_dev.dev);
  class_destroy(my_class);
  unregister_chrdev_region(my_dev.dev, 1);
}
module_exit(smmu_if_exit);
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

int smmu_if_open(struct inode *inode, struct file *filp)
{

  int ret;
  int data = 0; // arm_smmu_domain_set_attr requires 0 to set ARM_SMMU_DOMAIN_S1

  if (DEBUG_SMMU_IF > 0) {
    printk(KERN_INFO "smmu_dev_ptr          = %p\n", smmu_dev_ptr);
    printk(KERN_INFO "smmu_dev_ptr->of_node = %p\n", smmu_dev_ptr->of_node);
  }

  /*
   * set up smmu_domain
   */
  // allocate new domain
  smmu_domain_ptr = iommu_domain_alloc(&platform_bus_type);

  // set attributes
  ret = iommu_domain_set_attr(smmu_domain_ptr, DOMAIN_ATTR_NESTING, (void *)&data);
  if (ret) {
    printk(KERN_INFO "SMMU_IF: iommu_domain_set_attr() failed: %d\n", ret);
    return ret;
  }

  // register fault worker thread with CWM API - single-threaded workqueue
  smmu_fault_handler_wq = alloc_workqueue("%s", WQ_UNBOUND | WQ_HIGHPRI, 1, smmu_fault_handler_wq_name);
  if (smmu_fault_handler_wq == NULL) {
    printk(KERN_WARNING "SMMU_IF: Allocation of workqueue for fault handling failed.\n");
    return -ENOMEM;
  }

  // initialize the workqueue
  smmu_fault_ret    = 0;
  smmu_fault_status = READY;
  INIT_WORK(&smmu_fault_handler_w, (void *)smmu_fault_handler_worker);

  // prepare fault handler
  user_task = current;
  user_mm = current->mm;

  // set up fault handler
  iommu_set_fault_handler(smmu_domain_ptr, (iommu_fault_handler_t)&smmu_fault_handler, smmu_token_ptr);

  // finally attach the domain to the device
  ret = iommu_attach_device(smmu_domain_ptr, smmu_dev_ptr);
  if (ret) {
    printk(KERN_WARNING "SMMU_IF: Failed to attach IOMMU device.\n");
    return ret;    
  }

  printk(KERN_INFO "SMMU_IF: Device opened.\n");
  
  return 0;
}
// }}}

// write {{{
/***********************************************************************************
 *
 * ██╗    ██╗██████╗ ██╗████████╗███████╗
 * ██║    ██║██╔══██╗██║╚══██╔══╝██╔════╝
 * ██║ █╗ ██║██████╔╝██║   ██║   █████╗  
 * ██║███╗██║██╔══██╗██║   ██║   ██╔══╝  
 * ╚███╔███╔╝██║  ██║██║   ██║   ███████╗
 *  ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝   ╚═╝   ╚══════╝
 * 
 ***********************************************************************************/
ssize_t smmu_if_write(struct file *filp, const char __user *buf, size_t count, loff_t * offp)
{
  unsigned long left;

  if ( count != sizeof(unsigned *) ) {
    printk(KERN_WARNING "SMMU_IF: ERROR: Wrong number of Bytes to write for user-space offset %#x.\n", (unsigned)count);
    return 0;
  }

  // do the write
  left = copy_from_user((void *)&user_offset, buf, (unsigned long)count);
  if (left) {
    printk(KERN_WARNING "SMMU_IF: Passing of user-space offset failed.\n");
    return (ssize_t)(count - (size_t)left);
  }

  printk(KERN_INFO "SMMU_IF: User-space offest = %#lx.\n", user_offset);

  return (ssize_t)count;
};

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

int smmu_if_release(struct inode *p_inode, struct file *filp)
{
  // unmap everything
  int i;
  size_t size = PAGE_SIZE;

  // distroy the worker thread
  if (smmu_fault_handler_wq) {
    // Flush and destroy the workqueue, and reset workqueue pointer to default value.
    destroy_workqueue(smmu_fault_handler_wq);

    printk(KERN_INFO "SMMU_IF: Fault handler worker thread disabled.\n");
  }

  for (i=N_PAGES-1; i>=0; i--) {

    if (iova_array[i] > 0) {
      printk(KERN_INFO "SMMU_IF: iova_array[%i] = %#lx\n", i, iova_array[i]);

      // unmap the page
      iommu_unmap(smmu_domain_ptr, iova_array[i], size);

      iova_array[i] = 0;

      // unpin user-space memory
      if ( !PageReserved(pages_ptrs[i]) )
        SetPageDirty(pages_ptrs[i]);

      // clean up and free the pages struct
      put_page(pages_ptrs[i]);
    }
  }

  page_idx = 0;

  // detach the domain from the device
  iommu_detach_device(smmu_domain_ptr, smmu_dev_ptr);

  // free the domain
  iommu_domain_free(smmu_domain_ptr);

  printk(KERN_INFO "SMMU_IF: Device released.\n");
 
  return 0;
}
// }}}

// fault {{{
/***********************************************************************************
 *
 * ███████╗ █████╗ ██╗   ██╗██╗  ████████╗
 * ██╔════╝██╔══██╗██║   ██║██║  ╚══██╔══╝
 * █████╗  ███████║██║   ██║██║     ██║   
 * ██╔══╝  ██╔══██║██║   ██║██║     ██║   
 * ██║     ██║  ██║╚██████╔╝███████╗██║   
 * ╚═╝     ╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚═╝   
 * 
 ***********************************************************************************/

// top half - registered to the IOMMU API
int smmu_fault_handler(struct iommu_domain *smmu_domain_ptr, struct device *smmu_dev_ptr, 
                       unsigned long iova, int flags, void * smmu_token_ptr)
{

  int ret = 0;

  if (DEBUG_FAULT_HANDLER > 0)
    printk(KERN_INFO "SMMU_IF: Handling fault. iova = %#lx, hmkey = %i.\n", iova, flags);

  // prepare the job - only schedule one job at a time
  spin_lock(&smmu_fault_lock);
  while ( smmu_fault_status != READY ) {
    spin_unlock(&smmu_fault_lock);

    // busy waiting
    udelay(10);

    spin_lock(&smmu_fault_lock);
  }
  // pass iova
  iova_faulty       = iova;
  smmu_fault_status = START;
  spin_unlock(&smmu_fault_lock);

  // schedule the job
  queue_work(smmu_fault_handler_wq, &smmu_fault_handler_w);

  if (DEBUG_FAULT_HANDLER > 0)
    udelay(1000);

  // busy wait & sync with process context worker
  spin_lock(&smmu_fault_lock);
  while ( smmu_fault_status != DONE ) {
    spin_unlock(&smmu_fault_lock);

    // busy waiting
    udelay(10);

    spin_lock(&smmu_fault_lock);
  }
  // get return value
  ret               = smmu_fault_ret;
  smmu_fault_status = READY;
  spin_unlock(&smmu_fault_lock);

  return ret;
}

// bottom half - called by top half
void smmu_fault_handler_worker(void)
{
  int ret = 0; 
  unsigned long vaddr, offset, flags, iova;
  phys_addr_t paddr;
  size_t size;
  int prot;

  int write = 1;
  int force = 0;

  // sync with fault handler (interrupt context)
  spin_lock_irqsave(&smmu_fault_lock, flags);
  iova              = iova_faulty;
  smmu_fault_status = BUSY;
  spin_unlock_irqrestore(&smmu_fault_lock, flags); // release the spinlock

  offset = (unsigned long)(iova) & BF_MASK_GEN(0, PAGE_SHIFT);
  if (DEBUG_FAULT_HANDLER > 0)
    printk(KERN_INFO "SMMU_IF: faulty addr = %#lx, offset = %#lx\n", iova, offset);

  // align address to page border / 4kB
  iova  = (unsigned long)(iova)  & BF_MASK_GEN(PAGE_SHIFT,sizeof(unsigned long)*8-PAGE_SHIFT);
  vaddr  = iova - SVM_BASE_ADDR + user_offset;

  if (DEBUG_FAULT_HANDLER > 0)
    printk(KERN_INFO "SMMU_IF: mmap phys address = %#lx, user-space address = %#lx\n",
      iova, vaddr);

  // get pointer to user-space buffer and lock it into memory, get a single page
  down_read(&user_task->mm->mmap_sem);
  ret = get_user_pages_remote(user_task, user_task->mm, vaddr, 1, write ? FOLL_WRITE : 0, &pages_ptrs[page_idx], NULL);
  up_read(&user_task->mm->mmap_sem);
  if ( ret != 1 ) {
    printk(KERN_WARNING "SMMU_IF: Could not get requested user-space virtual address %#lx.\n", iova);
    ret = -EFAULT;
    goto smmu_fault_handler_error;
  }

  // virtual-to-physical address translation
  paddr = (phys_addr_t)page_to_phys(pages_ptrs[page_idx]);
  
  if (DEBUG_FAULT_HANDLER > 0)
    printk(KERN_INFO "SMMU_IF: Physical address = %#lx\n",(long unsigned)paddr);

  iova_array[page_idx] = iova;
  page_idx++;

  // prepare mapping
  prot = IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE; // for now, always read and write access + coherent
  size = PAGE_SIZE;

  // map it
  ret = iommu_map(smmu_domain_ptr, iova, paddr, size, prot);
  if (ret) {
    printk(KERN_WARNING "SMMU_IF: Could not map %#lx to SMMU, ERROR = %i.\n", iova, ret);
    goto smmu_fault_handler_error;
  }

  // sync with fault handler (interrupt context)
  smmu_fault_handler_error:
    spin_lock_irqsave(&smmu_fault_lock, flags);
    smmu_fault_ret    = ret;
    smmu_fault_status = DONE;
    spin_unlock_irqrestore(&smmu_fault_lock, flags); // release the spinlock

  return;
}
//}}}