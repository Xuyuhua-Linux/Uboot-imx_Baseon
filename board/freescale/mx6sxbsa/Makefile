# (C) Copyright 2016 NXP Semiconductors, Inc.
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y  := mx6sxbsa.o

extra-$(CONFIG_USE_PLUGIN) :=  plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@
