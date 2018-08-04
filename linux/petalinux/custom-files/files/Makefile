APP = custom-files

# Add any other object files to this list below
APP_OBJS = custom-files.o

all: build install

build: $(APP)

clean:

.PHONY: install image

install: $(APP)
	$(TARGETINST) -d custom-files/etc              /etc
	$(TARGETINST) -d custom-files/root/sourceme.sh /home/root/sourceme.sh
	$(TARGETINST) -d custom-files/root/.ssh        /home/root/.ssh
	$(TARGETINST) -d custom-files/mount_nfs.sh     /mount_nfs.sh

help:
	@echo ""
	@echo "Quick reference for various supported build targets for $(INSTANCE)."
	@echo "----------------------------------------------------"
	@echo "  clean                  clean out build objects"
	@echo "  all                    build $(INSTANCE) and install to rootfs host copy"
	@echo "  build                  build subsystem"
	@echo "  install                install built objects to rootfs host copy"