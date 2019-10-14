
CFLAGS := --c99 -c --cpu Cortex-M3 -g -O0 --apcs=interwork --split_sections

SRC_DIR = ../src
SVN_DIR := /c/'Program Files'/TortoiseSVN/bin
KEIL_PATH = /c/Keil_v5/ARM/ARMCC/bin
ARMCC := $(KEIL_PATH)/armcc
ARMAR := $(KEIL_PATH)/armar


SVN_VERSION:= $(shell $(SVN_DIR)/SubWCRev.exe ../ |grep revision |awk 'NR==1{print $$5}')
ifneq ($(SVN_VERSION),)
CMACRO := -DVERSION_TPMS=$(SVN_VERSION)
endif


INC := -I ../include/export -I ../include/import -I ../include/internal
INC += -I /c/Keil_v5/ARM/PACK/Keil/STM32F1xx_DFP/2.2.0/Device/Include
INC += -I /c/Keil_v5/ARM/CMSIS/Include  


OBJC = $(wildcard $(SRC_DIR)/*.c)
OBJS = $(patsubst %.c,%.o,$(OBJC))

LIB = LibTpms.lib

%.o:%.c
	@echo compiling $<
	@$(ARMCC) $(CFLAGS) $(INC) $(CMACRO) -o $@ $<
lib:$(OBJS)
	@echo ar $(LIB) $(OBJS)
	@$(ARMAR) --create $(LIB) $(OBJS)
install:
	cp $(LIB) ../bin
clean:
	@echo clean objs
	-rm -rf $(LIB)
	-rm -rf $(OBJS)

	