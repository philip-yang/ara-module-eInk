# Copyright (c) 2015 Google, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Fabien Parent <fparent@baylibre.com>

CWD := $(shell pwd)
NUTTX_ROOT ?= ./nuttx
TOPDIR := $(NUTTX_ROOT)/nuttx
BUILDDIR := $(NUTTX_ROOT)/oot/nuttx

obj += board-eInk-module.o
obj += hid_button.o

-include $(NUTTX_ROOT)/nuttx/.config
-include $(NUTTX_ROOT)/nuttx/arch/arm/src/armv7-m/Toolchain.defs
-include $(NUTTX_ROOT)/nuttx/configs/ara/bridge/tsb-makefile.common

depend = \
	sed 's,\($*\)\.o[ :]*,\1.o $(@:.o=.d): ,g' < $(@:.o=.d) > $(@:.o=.d).$$$$; \
	rm $(@:.o=.d); \
	mv $(@:.o=.d).$$$$ $(@:.o=.d)

prepend-dir-to = $(addprefix $2/,$1)
prepend-dir = $(foreach d,$($1),$(call prepend-dir-to,$(d),$2))

all: nuttx_init build_fdk_bootstrap
	PATH=$(CWD)/manifesto:$(PATH) \
	OOT_OBJS="$(call prepend-dir,obj,$(CWD))" \
	./build.sh $(CWD) $(NUTTX_ROOT) && \
	cp $(BUILDDIR)/nuttx $(CWD)/nuttx.elf && \
	cp $(BUILDDIR)/nuttx.bin $(BUILDDIR)/System.map $(CWD)

build_fdk: $(obj)

build_fdk_bootstrap:
	$(MAKE) build_fdk

init:
	git submodule init
	git submodule update --remote

nuttx_init:
	cp scripts/Make.defs $(NUTTX_ROOT)/nuttx/
	cp .config $(NUTTX_ROOT)/nuttx/.config
	cd $(NUTTX_ROOT)/nuttx; $(MAKE) context

tftf: all
	./bootrom-tools/create-tftf --elf nuttx.elf --unipro-mfg 0x126 \
	                            --unipro-pid 0x1000 --ara-stage 2 \
	                            --start 0x`grep '\bReset_Handler$$' System.map | cut -d ' ' -f 1`

update:
	git submodule update --remote

%_defconfig: configs/%_defconfig
	echo "Loading $<..."
	cp $< .config

menuconfig:
	cp .config $(TOPDIR)/.config
	cd $(TOPDIR); APPSDIR=$(CONFIG_APPS_DIR) kconfig-mconf ./Kconfig
	cp $(TOPDIR)/.config .config

clean:
	rm -f $(obj) $(obj:.o=.d) nuttx.bin nuttx.elf System.map *.tftf

distclean: clean
	cd $(NUTTX_ROOT)/nuttx; $(MAKE) apps_distclean && $(MAKE) distclean
	rm -f .config

%.o: %.c
	echo "CC\t $@"
	$(CC) -MD -MP $(CFLAGS) -c $< -o $@
	$(call depend)

-include $(obj:.o=.d)

.PHONY: all clean distclean init
ifndef VERBOSE
.SILENT:
endif
