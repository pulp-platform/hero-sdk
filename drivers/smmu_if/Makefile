obj-m         := smmu_if.o

PLATFORM      := ${shell echo ${PLATFORM}}

ARCH          := ${shell echo ${KERNEL_ARCH}}
CROSS_COMPILE := ${shell echo ${KERNEL_CROSS_COMPILE}}

KERNEL_DIR    := ${shell echo ${KERNEL_DIR}}
#ARM_INC_DIR1  := ${shell echo ${ARM_INC_DIR1}}
#PULP_INC_DIR1 := ${shell echo ${PULP_INC_DIR1}}
#PULP_INC_DIR2 := ${shell echo ${PULP_INC_DIR2}}

#ccflags-y := -I${ARM_INC_DIR1}
#ccflags-y += -I${PULP_INC_DIR1} -I${PULP_INC_DIR2}
ccflags-y += -DPLATFORM=${PLATFORM}

smmu_if-objs := smmu_if_module.o

all: modules

modules:

	$(MAKE) -S ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_DIR) M=$(shell pwd) modules

clean:
	rm -f modules.order
	rm -f *.mod.*
	rm -f *.o
	rm -f *.ko
	rm -f Module.symvers
	rm -f .*.o.cmd
	rm -f .*.ko.cmd
	rm -f *.read
	rm -f *.o_shipped
