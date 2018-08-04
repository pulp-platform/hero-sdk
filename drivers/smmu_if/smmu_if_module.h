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

// type definitions
typedef struct {
  dev_t dev; // device number
  struct file_operations *fops;
  struct cdev cdev;
  int minor;
  int major;
} SmmuIfDev;

typedef enum {
  READY = 0,
  START = 1,
  BUSY  = 2,
  DONE  = 3,
  ERROR = 4,
} WorkerStatus;

// constants
#define SMMU_IF_N_DEV_NUMBERS 1

#define DEBUG_SMMU_IF       0
#define DEBUG_FAULT_HANDLER 0

#define N_PAGES 256

#define SVM_BASE_ADDR  0x1000000000UL

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
