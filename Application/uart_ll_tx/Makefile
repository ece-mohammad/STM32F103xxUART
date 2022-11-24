# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# 
# targets:
# 	UART_LL_TX			: build example code for UART asynchronouse transmission using ST LL libraries
# 
# build:
#	Debug				: build for debug (optmization flags -Og)
#	Release				: build release (optimization flags -O2)
#	RelMinSize			: build for relase with minimum size (optimization flags -Os)
# 	
# ------------------------------------------------

include ../defines.mk

TARGET = UART_LL_TX

######################################
# building variables
######################################

# build type
ifeq ($(strip $(build)),)
build = Debug
endif

$(info Build type: $(build))

# enable debugging information
ifeq ($(build), Debug)
CFLAGS += -g -gdwarf-2 -DDEBUG -DUSE_FULL_ASSERT=1U
endif

# optimization flags
ifeq ($(build), Debug)
OPT = -Og

else ifeq ($(build), Release)
OPT = -O2

else ifeq ($(build), RelMinSize)
OPT = -Os

endif


#######################################
# paths
#######################################
# Build path
BUILD_DIR = $(ROOT_BUILD_DIR)/$(TARGET)/$(build)

# $(info Build dir: $(BUILD_DIR))

######################################
# Files
######################################


# application files: uart_ll_echo ----------------------------------------------------------------------
APP_C_SOURCES = \
$(addprefix Src/,$(filter %.c,$(shell ls Src)))

# $(info APP_C_SOURCES: $(APP_C_SOURCES))
# $(info APP_C_INCUDES: $(APP_C_INCUDES))

# modules files other than UART (ring buffer, serial_debug, etc) ----------------------------------------------------------------------
MODULES_C_SOURCES = \
../../Modules/buffer/ring_buffer.c \


# UART_LL module files ----------------------------------------------------------------------
UART_LL_C_SOURCES = \
../../Modules/serial/uart/ll/uart_ll.c \


# LL drivers C sources files ----------------------------------------------------------------------
LL_DRIVERS_C_SOURCES = \
../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_pwr.c \
../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_rcc.c \
../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_gpio.c \
../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_exti.c \
../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usart.c \
../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_utils.c \


ASM_SOURCES =  \
../Common/Startup/startup_stm32f103cbtx.s

########################################
# includes
########################################

# common includes

APP_C_INCUDES = \
Inc \
$(CONFIG_DIR) \


MODULES_C_INCLUDES = \
../../Modules \


LL_DRIVERS_C_INCLUDES = \
../../Drivers/STM32F1xx_HAL_Driver/Inc \
../../Drivers/CMSIS/Device/ST/STM32F1xx/Include \
../../Drivers/CMSIS/Include \


C_DEFS += -DUSE_FULL_LL_DRIVER

# AS includes
AS_INCLUDES = 


C_SOURCES = \
$(APP_COMMON_C_SOURCES) \
$(APP_C_SOURCES) \
$(MODULES_C_SOURCES) \
$(UART_LL_C_SOURCES) \
$(LL_DRIVERS_C_SOURCES) \

C_INCLUDES = $(addprefix -I, $(COMMON_C_INCLUDES) $(APP_COMMON_C_INCLUDES) $(APP_C_INCUDES) $(MODULES_C_INCLUDES) $(LL_DRIVERS_C_INCLUDES))

# $(info C_SOURCES: $(C_SOURCES))
# $(info ASM_SOURCES: $(ASM_SOURCES))
# $(info C_INCLUDES: $(C_INCLUDES))
# $(info C_DEFS: $(C_DEFS))
# $(info CFLAGS: $(CFLAGS))
# $(info LDFLAGS: $(LDFLAGS))

# default action: build all
all: $(TARGET)

$(TARGET): $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex 

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# $(info VPATH: $(VPATH))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@ECHO Compiling file: $@
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@
	@ECHO $(CRLF)

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@ECHO Compiling file: $@
	$(AS) -c $(CFLAGS) $< -o $@
	@ECHO $(CRLF)

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@ECHO Linking file: $@
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@
	@ECHO $(CRLF)

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir -p $@


#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
