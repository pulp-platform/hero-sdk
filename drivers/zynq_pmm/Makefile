obj-m += zynq_pmm.o
zynq_pmm-objs := zynq_pm_module.o v7_pmu.o

ARCH=arm
CROSS_COMPILE=arm-xilinx-linux-gnueabi-

KERNEL_DIR := $(shell echo $(KERNEL_DIR))
ARM_INC_DIR1 := $(shell echo $(ARM_INC_DIR1))
PULP_INC_DIR1 := $(shell echo $(PULP_INC_DIR1))

ccflags-y := -I${ARM_INC_DIR1}
ccflags-y += -I${PULP_INC_DIR1} -I${PULP_INC_DIR2}
#ccflags-y += -I${PULP_INC_DIR1}

all: modules
modules:
	$(MAKE) -S ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_DIR) M=$(shell pwd) modules

clean:
	rm -f *.mod.*
	rm -f *.o
	rm -f *.ko
	rm -f Module.symvers
	rm -f modules.order
