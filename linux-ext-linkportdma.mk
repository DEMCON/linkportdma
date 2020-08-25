ifeq ($(BR2_LINUX_KERNEL_EXT_LINKPORTDMA),y)

LINUX_EXTENSIONS += linkportdma

LINKPORTDMA_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

define LINKPORTDMA_PATCH
	cp -u $(LINKPORTDMA_DIR)/bfin_lp.c $(LINUX_DIR)/source/drivers/char/bfin_lp.c
endef

LINUX_PRE_BUILD_HOOKS += LINKPORTDMA_PATCH

linkportdma-patch:
	$(LINKPORTDMA_PATCH)

$(LINUX_DIR)/.stamp_initramfs_rebuilt: linkportdma-patch

endif
