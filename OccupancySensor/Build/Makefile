###############################################################################
#
# MODULE:   Makefile
#
# DESCRIPTION: Makefile for the ZLO Occupancy Sensor devices
# 
 ############################################################################
#
# This software is owned by NXP B.V. and/or its supplier and is protected
# under applicable copyright laws. All rights are reserved. We grant You,
# and any third parties, a license to use this software solely and
# exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5179].
# You, and any third parties must reproduce the copyright and warranty notice
# and any other legend of ownership on each copy or partial copy of the
# software.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright NXP B.V. 2017. All rights reserved
#
###############################################################################

SENSOR ?= OccupancySensor

# Application target name
TARGET = $(SENSOR)
PDM_BUILD_TYPE ?=_EEPROM

STACK_SIZE = 5000
MINIMUM_HEAP_SIZE = 2000

###############################################################################
#User definable make parameters that may be overwritten from the command line

JENNIC_CHIP ?= JN5169
JENNIC_CHIP_FAMILY ?= JN517x
JENNIC_PCB         ?= DEVKIT5


ifeq ($(JENNIC_CHIP_FAMILY), JN517x)
JENNIC_SDK              = JN-SW-4270
ifeq ($(OTA),1)
	ZNCLKCMD            = AppBuildZBProOta.ld
else
	ZNCLKCMD            = AppBuildZBPro.ld
endif
JENNIC_PCB              = DEVKIT5
ENDIAN                  = LITTLE_ENDIAN_PROCESSOR
DISABLE_LTO             = 1
endif 

ifeq ($(JENNIC_CHIP_FAMILY), JN516x)
JENNIC_SDK              = JN-SW-4170
ZNCLKCMD                = AppBuildZBPro.ld
JENNIC_PCB              = DEVKIT4
ENDIAN 	                = BIG_ENDIAN
DISABLE_LTO             = 1
endif

ifeq ($(JENNIC_CHIP_FAMILY),JN517x)
INTERRUPT_HANDLER_BBC = zps_isrMAC
LDFLAGS += -Wl,-u$(INTERRUPT_HANDLER_BBC) -Wl,-defsym,vAHI_IntHandlerBbc_select=$(INTERRUPT_HANDLER_BBC)
endif

###############################################################################
# Network settings

# Optional network.mk for channel
sinclude ../../../network.mk

# Channel (0 for default channels)
SINGLE_CHANNEL ?= 0
CFLAGS         += -DSINGLE_CHANNEL=$(SINGLE_CHANNEL)

###############################################################################
# Default DK4 development kit target hardware

DR ?= DR1199
ifeq ($(DR),DR1199)
OTA_HW = DR1199
endif

#sub srings to identify hardware varients for use in the OTA string
OTA_HW_VARIANT = r1v02-

ifeq ($(OTA),1)
CFLAGS  += -DBUILD_OTA
MANUFACTURER_CODE = 0x1037
CFLAGS += -DCLD_OTA_MANF_ID_VALUE=$(MANUFACTURER_CODE)
APP_CLUSTERS_OTA_SRC ?= 1

OTA_ENCRYPTED ?= 0
ifeq ($(OTA_ENCRYPTED),1)
CFLAGS  += -DOTA_ENCRYPTED
endif

OTA_INTERNAL_STORAGE ?= 1
ifeq ($(OTA_INTERNAL_STORAGE),1)
CFLAGS  += -DOTA_INTERNAL_STORAGE
ifeq ($(OTA_ENCRYPTED),1)
CFLAGS += -DINTERNAL_ENCRYPTED
endif
endif

ifeq ($(OTA_ENCRYPTED),0)
OTA_DEVICE_ID = 0x0406
OTA_STRING = $(OTA_HW)$(OTA_HW_VARIANT)-$(JENNIC_CHIP)0000000000000
else
OTA_DEVICE_ID = 0x1406
OTA_STRING = $(OTA_HW)$(OTA_HW_VARIANT)-$(JENNIC_CHIP)--ENCRYPTED00
endif
endif
KEEPALIVETIME ?= 6

ifeq ($(KEEPALIVETIME),0) 
#no need to do anything as the sleep will not be defined
else 
ifeq ($(KEEPALIVETIME),255)
CFLAGS += -DSLEEP_ENABLE
else
CFLAGS += -DSLEEP_ENABLE
endif
endif
CFLAGS += -D$(DR)
CFLAGS += -D$(SENSOR)

CFLAGS  += -DBUTTON_MAP_$(DR)

CFLAGS += -DSLEEPY_ZED

# Switch is an Initiator,
CFLAGS  += -DEZ_MODE_INITIATOR
#CFLAGS  += -DEZ_ENABLE_GROUP

# NFC NTAG ICODE (new) functionality
APP_NTAG_ICODE ?= 1
ifeq ($(APP_NTAG_ICODE), 1)
CFLAGS  += -DAPP_NTAG_ICODE
endif

# NFC NTAG AES (old) functionality
APP_NTAG_AES ?= 0
ifeq ($(APP_NTAG_AES), 1)
CFLAGS  += -DAPP_NTAG_AES
endif

###############################################################################
# Select the network stack (e.g. MAC, ZBPro, SE, HA)
JENNIC_STACK ?= ZCL
JENNIC_MAC ?= MiniMacShim

APP_CLUSTER_LIGHTING_SRC ?=1
APP_CLUSTERS_MEASUREMENT_AND_SENSING_SRC ?=1
#APP_CLUSTERS_OTA_SRC ?=1
#APP_CLUSTERS_ENERGY_AT_HOME_SRC ?=1
#APP_CLUSTERS_GREENPOWER_SRC ?=1
#APP_CLUSTERS_HVAC_SRC ?=1
#APP_CLUSTERS_SMART_ENERGY_SRC ?=1
#APP_CLUSTERS_IAS_SRC ?=1

###############################################################################
# End device or router / coord (e.g. ZCR, ZED)
ZBPRO_DEVICE_TYPE ?= ZED

###############################################################################
# BDB specific options
#BDB_SUPPORT_TOUCHLINK_INITIATOR ?= 1
#BDB_SUPPORT_TOUCHLINK_TARGET ?= 1
BDB_SUPPORT_NWK_STEERING ?= 1
BDB_SUPPORT_FIND_AND_BIND_INITIATOR ?= 1
#BDB_SUPPORT_FIND_AND_BIND_TARGET ?= 1
#BDB_SUPPORT_NWK_FORMATION ?= 1


###############################################################################
# Debug options define DEBUG either HW or SW
#DEBUG ?=SW
#DEBUG ?=HW
#
# Define which UART to use for HW debug
# DEBUG_PORT   ?= UART1
#  DEBUG_MODE  ?=  SWD    # For ARM core

###############################################################################
# Define TRACE to use with DBG module
TRACE =1
ifeq ($(TRACE), 1)
CFLAGS  += -DDBG_ENABLE
#CFLAGS += -DDEBUG_921600
CFLAGS += -DDEBUG_EZMODE
CFLAGS += -DDEBUG_ZCL
CFLAGS += -DDEBUG_CLD_ONOFF
CFLAGS += -DDEBUG_APP_BUTTON
CFLAGS += -DDEBUG_START_UP
CFLAGS += -DDEBUG_SENSOR_NODE
CFLAGS += -DDEBUG_SENSOR_TASK
CFLAGS += -DDEBUG_SCAN
CFLAGS += -DDEBUG_JOIN
CFLAGS += -DDEBUG_COMMISSION
CFLAGS += -DDEBUG_SENSOR_STATE
CFLAGS += -DDEBUG_SLEEP_HANDLER
CFLAGS += -DDEBUG_OCCUPANCY_SENSOR
CFLAGS += -DDEBUG_POLL_HANDLER
CFLAGS += -DDEBUG_PROCESS_BUTTONS
CFLAGS += -DDEBUG_ZCL_PROCESSING
CFLAGS += -DDEBUG_REPORT
CFLAGS += -DDEBUG_NWK_EVENT_HANDLER
CFLAGS += -DDEBUG_PIR_EVENTS
CFLAGS += -DDEBUG_ZBP_UTILS
CFLAGS += -DDEBUG_EVENT_HANDLER
CFLAGS += -DDEBUG_BLINK_LED
CFLAGS += -DDEBUG_APP_OTA
CFLAGS += -DDEBUG_APP_NTAG
CFLAGS += -DDEBUG_APP_ICODE
CFLAGS += -DDEBUG_BDB
endif

APP_CLUSTER_LIGHTING_SRC ?=1
APP_CLUSTERS_MEASUREMENT_AND_SENSING_SRC ?=1
#APP_CLUSTERS_OTA_SRC ?=1
#APP_CLUSTERS_ENERGY_AT_HOME_SRC ?=1
#APP_CLUSTERS_GREENPOWER_SRC ?=1
#APP_CLUSTERS_HVAC_SRC ?=1
#APP_CLUSTERS_SMART_ENERGY_SRC ?=1
#APP_CLUSTERS_IAS_SRC ?=1
CFLAGS += -D$(TARGET)

##############################################################################
# For 4x use string based PDM id's for newer families use 16 bit id numbers
ifneq ($(JENNIC_CHIP_FAMILY), JN514x)
CFLAGS   += -DPDM_USER_SUPPLIED_ID
endif

###############################################################################

CFLAGS += -DNXP_HARDWARE


###############################################################################
# PIR Sensor Flags
# Specify additional Component libraries
PIR_SENSOR_TYPE	    = PIR_TYPE_OPEN_COLLECTOR
#PIR_SENSOR_TYPE	= PIR_TYPE_PWM

CFLAGS += -D$(PIR_SENSOR_TYPE)

###############################################################################

###############################################################################
# Path definitions

# Use if application directory contains multiple targets
SDK_BASE_DIR        = ../../../../sdk/$(JENNIC_SDK)
APP_BASE            = ../..
APP_BLD_DIR         = $(APP_BASE)/$(TARGET)/Build
APP_SRC_DIR         = $(APP_BASE)/$(TARGET)/Source
APP_PIR_DIR         = $(APP_SRC_DIR)/PIR
HW_SRC_DIR			= $(COMPONENTS_BASE_DIR)/HardwareAPI/Source

APP_COMMON_SRC_DIR  = $(APP_BASE)/Common/Source
UTIL_SRC_DIR        = $(COMPONENTS_BASE_DIR)/ZigbeeCommon/Source

###############################################################################
# Application Source files

# Note: Path to source file is found using vpath below, so only .c filename is required
ifeq ($(JENNIC_CHIP_FAMILY),JN516x)
APPSRC = irq_$(JENNIC_CHIP_FAMILY).S
APPSRC += portasm_JN516x.S
APPSRC += port_JN516x.c
endif
ifeq ($(JENNIC_CHIP_FAMILY),JN517x)
APPSRC = port_JN517x.c
endif

APPSRC += pdum_gen.c
APPSRC += pdum_apdu.S
APPSRC += zps_gen.c
APPSRC += app_pdm.c
APPSRC += app_main.c
APPSRC += app_reporting.c
APPSRC += app_start_sensor.c

APPSRC += App_$(TARGET).c
APPSRC += app_zlo_sensor_node.c

APPSRC += app_zcl_sensor_task.c
APPSRC += app_occupancy_buttons.c
APPSRC += app_occupancy_sensor_state_machine.c
APPSRC += app_sleep_handler.c
APPSRC += app_nwk_event_handler.c
APPSRC += app_event_handler.c
#APPSRC += app_PIR_events.c
ifeq ($(JENNIC_CHIP_FAMILY), JN517x)
APPSRC += AHI_ModuleConfiguration.c 
endif

ifeq ($(PIR_SENSOR_TYPE), PIR_TYPE_OPEN_COLLECTOR)
APPSRC += app_PIR_OpenCollector_events.c
else
ifeq ($(PIR_SENSOR_TYPE), PIR_TYPE_PWM)
APPSRC += app_PIR_pwm_events.c
endif
endif

APPSRC += app_zcl_tick_handler.c
APPSRC += app_blink_led.c

ifeq ($(APP_NTAG_ICODE),1)
APPSRC += app_ntag_icode.c
#APPSRC += app_icode.c
endif
ifeq ($(APP_NTAG_AES),1)
APPSRC += app_ntag_aes.c
endif

ifeq ($(OTA),1)
APPSRC += app_ota_client.c
endif

APP_ZPSCFG = app.zpscfg

# Construct binary features list 

ifeq ($(APP_NTAG_ICODE),1)
TARGET_FEATURES:=$(TARGET_FEATURES)_NtagIcode
endif

ifeq ($(APP_NTAG_AES),1)
TARGET_FEATURES:=$(TARGET_FEATURES)_NtagAes
endif

ifeq ($(OTA),1)
TARGET_FEATURES:=$(TARGET_FEATURES)_Ota
ifeq ($(OTA_ENCRYPTED),1)
TARGET_FEATURES:=$(TARGET_FEATURES)Enc
endif
endif

# Construct binary hardware list

TARGET_HARDWARE:=_$(JENNIC_CHIP)_$(DR)

ifneq ($(SINGLE_CHANNEL), 0)
TARGET_HARDWARE:=$(TARGET_HARDWARE)_CH$(SINGLE_CHANNEL)
endif

###############################################################################
# Standard Application header search paths

INCFLAGS += -I$(APP_SRC_DIR)
INCFLAGS += -I$(APP_SRC_DIR)/..
INCFLAGS += -I$(APP_COMMON_SRC_DIR)

INCFLAGS += -I$(APP_PIR_DIR)

# Application specific include files
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/ZigbeeCommon/Include
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/ZCL/Include
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/ZCIF/Include
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/HardwareAPI/Include
#Add include for OTA private file
ifeq ($(OTA),1)
INCFLAGS += -I$(COMPONENTS_BASE_DIR)/ZCL/Clusters/OTA/Source
endif
###############################################################################
# NTAG ICODE library
ifeq ($(APP_NTAG_ICODE),1)
# NTAG library folder
APP_NTAG_LIB_DIR  = $(APP_BASE)/NFC
# NTAG library
LDLIBS           += NTAG_$(JENNIC_CHIP_FAMILY)
# NTAG library path
LDFLAGS          += -L$(APP_NTAG_LIB_DIR)/Library
# NTAG include path
INCFLAGS         += -I$(APP_NTAG_LIB_DIR)/Include
endif

###############################################################################
# NTAG AES library
ifeq ($(APP_NTAG_AES),1)
# NTAG library folder
APP_NTAG_LIB_DIR  = $(APP_BASE)/NTAG
# NTAG library
LDLIBS           += NTAG_$(JENNIC_CHIP_FAMILY)
# NTAG library path
LDFLAGS          += -L$(APP_NTAG_LIB_DIR)/Library
# NTAG include path
INCFLAGS         += -I$(APP_NTAG_LIB_DIR)/Include
endif

###############################################################################
# Application libraries
# Specify additional Component libraries

#APPLIBS +=

###############################################################################

# You should not need to edit below this line

###############################################################################
###############################################################################
# Configure for the selected chip or chip family

include $(SDK_BASE_DIR)/Chip/Common/Build/config.mk
include $(SDK_BASE_DIR)/Platform/Common/Build/config.mk
include $(SDK_BASE_DIR)/Stack/Common/Build/config.mk
include $(SDK_BASE_DIR)/Components/BDB/Build/config.mk

###############################################################################

TEMP = $(APPSRC:.c=.o)
APPOBJS_TMP = $(TEMP:.S=.o)
APPOBJS := $(addprefix $(APP_BLD_DIR)/,$(APPOBJS_TMP))

###############################################################################
# Application dynamic dependencies

APPDEPS_TMP = $(APPOBJS_TMP:.o=.d)
APPDEPS := $(addprefix $(APP_BLD_DIR)/,$(APPDEPS_TMP))

###############################################################################
# Linker

# Add application libraries before chip specific libraries to linker so
# symbols are resolved correctly (i.e. ordering is significant for GCC)

APPLDLIBS := $(foreach lib,$(APPLIBS),$(if $(wildcard $(addprefix $(COMPONENTS_BASE_DIR)/Library/lib,$(addsuffix _$(JENNIC_CHIP).a,$(lib)))),$(addsuffix _$(JENNIC_CHIP),$(lib)),$(addsuffix _$(JENNIC_CHIP_FAMILY),$(lib))))
LDLIBS := $(APPLDLIBS) $(LDLIBS)
###############################################################################
# Dependency rules

.PHONY: all clean
# Path to directories containing application source
vpath % $(APP_SRC_DIR):$(APP_COMMON_SRC_DIR):$(APP_PIR_DIR):$(ZCL_SRC_DIRS):$(ZCL_SRC):$(BDB_SRC_DIR):$(UTIL_SRC_DIR):$(HW_SRC_DIR)


ifeq ($(OTA),1)
all: Client.bin	
else
all: $(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).bin
endif

-include $(APPDEPS)
$(APP_BLD_DIR)/%.d:
	rm -f $*.o

ifeq ($(JENNIC_CHIP_FAMILY), JN517x)
$(APP_SRC_DIR)/pdum_gen.c $(APP_SRC_DIR)/pdum_gen.h: $(APP_COMMON_SRC_DIR)/$(APP_ZPSCFG) $(PDUMCONFIG)
	$(info Configuring the PDUM ...)
	$(PDUMCONFIG) -z $(TARGET) -e $(ENDIAN) -f $< -o $(APP_SRC_DIR)
	@echo

$(APP_SRC_DIR)/zps_gen.c $(APP_SRC_DIR)/zps_gen.h: $(APP_COMMON_SRC_DIR)/$(APP_ZPSCFG) $(ZPSCONFIG)
	$(info Configuring the Zigbee Protocol Stack ...)
	$(ZPSCONFIG) -n $(TARGET) -t $(JENNIC_CHIP) -l $(ZPS_NWK_LIB) -a $(ZPS_APL_LIB) -c $(TOOL_COMMON_BASE_DIR)/$(TOOLCHAIN_PATH) -e $(ENDIAN) -f $< -o $(APP_SRC_DIR)
	@echo

    $(APP_BLD_DIR)/pdum_gen.o $(APP_BLD_DIR)/zps_gen.o: CFLAGS += -fno-lto
endif

ifeq ($(JENNIC_CHIP_FAMILY), JN516x)
$(APP_SRC_DIR)/pdum_gen.c $(APP_SRC_DIR)/pdum_gen.h: $(APP_COMMON_SRC_DIR)/$(APP_ZPSCFG) $(PDUMCONFIG)
	$(info Configuring the PDUM ...)
	$(PDUMCONFIG) -z $(TARGET) -f $< -o $(APP_SRC_DIR)
	@echo

$(APP_SRC_DIR)/zps_gen.c $(APP_SRC_DIR)/zps_gen.h: $(APP_COMMON_SRC_DIR)/$(APP_ZPSCFG) $(ZPSCONFIG)
	$(info Configuring the Zigbee Protocol Stack ...)
	$(ZPSCONFIG) -n $(TARGET) -t $(JENNIC_CHIP) -l $(ZPS_NWK_LIB) -a $(ZPS_APL_LIB) -c $(TOOL_COMMON_BASE_DIR)/$(TOOLCHAIN_PATH) -f $< -o $(APP_SRC_DIR)
	@echo

    $(APP_BLD_DIR)/pdum_gen.o $(APP_BLD_DIR)/zps_gen.o: CFLAGS += -fno-lto
endif

$(APP_BLD_DIR)/%.o: %.S
	$(info Assembling $< ...)
	$(CC) -c -o $(subst Source,Build,$@) $(CFLAGS) $(INCFLAGS) $< -MD -MF $(APP_BLD_DIR)/$*.d -MP
	@echo

$(APP_BLD_DIR)/%.o: %.c
	$(info Compiling $< ...)
	$(CC) -c -o $(subst Source,Build,$@) $(CFLAGS) $(INCFLAGS) $< -MD -MF $(APP_BLD_DIR)/$*.d -MP
	@echo

$(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).elf: $(APPOBJS) $(addsuffix.a,$(addprefix $(COMPONENTS_BASE_DIR)/Library/lib,$(APPLDLIBS))) 
	$(info Linking $@ ...)
	$(CC) -Wl,--gc-sections -Wl,-u_AppColdStart -Wl,-u_AppWarmStart $(LDFLAGS) -T$(ZNCLKCMD) -L $(SDK_BASE_DIR)/Stack/ZCL/Build/ -o $@ -Wl,--start-group $(APPOBJS) $(addprefix -l,$(LDLIBS)) -lm -Wl,--end-group -Wl,-Map,$(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).map 
	$(SIZE) $@ 
	date +%c >> $(APP_BASE)/Doc/size.txt
	$(SIZE) $@ >> $(APP_BASE)/Doc/size.txt
	
ifeq ($(JENNIC_CHIP_FAMILY),JN517x)
$(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).bin: $(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).elf
	$(info Generating binary ...)
	$(OBJCOPY) -S -O binary $< $@
else	
$(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).bin: $(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).elf 
	$(info Generating binary ...)
	$(OBJCOPY) -j .version -j .bir -j .flashheader -j .vsr_table -j .vsr_handlers  -j .rodata -j .text -j .data -j .bss -j .heap -j .stack -S -O binary $< $@
endif

ifeq ($(OTA),1)	
$(info Generating OTA binary ...)
ifeq ($(JENNIC_CHIP_FAMILY), JN516x)
     JET_VERSION=4
else ifeq ($(JENNIC_CHIP_FAMILY), JN517x)
     JET_VERSION=5
endif

Client.bin:$(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).bin
	cp -f $(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).bin $(APP_BLD_DIR)/OTABuild/Sensor.bin
ifeq ($(OTA_ENCRYPTED),0)
	$(APP_BLD_DIR)/OTABuild/SensorCreateOtaClient.bat $(APP_BLD_DIR)/OTABuild $(JENNIC_SDK) $(MANUFACTURER_CODE) $(OTA_STRING) $(JET_VERSION) $(JENNIC_CHIP_FAMILY) $(OTA_DEVICE_ID)
	mv $(APP_BLD_DIR)/OTABuild/Client.bin $(APP_BLD_DIR)/OTABuild/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).bin
	mv $(APP_BLD_DIR)/OTABuild/ClientUpGradeImagewithOTAHeaderV2.bin $(APP_BLD_DIR)/OTABuild/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE)_V2.bin
	mv $(APP_BLD_DIR)/OTABuild/ClientUpGradeImagewithOTAHeaderV3.bin $(APP_BLD_DIR)/OTABuild/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE)_V3.bin
	mv $(APP_BLD_DIR)/OTABuild/ClientUpGradeImagewithOTAHeaderV2.ota $(APP_BLD_DIR)/OTABuild/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE)_V2.ota
	mv $(APP_BLD_DIR)/OTABuild/ClientUpGradeImagewithOTAHeaderV3.ota $(APP_BLD_DIR)/OTABuild/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE)_V3.ota
else
	$(APP_BLD_DIR)/OTABuild/SensorCreateOtaEncClient.bat $(APP_BLD_DIR)/OTABuild $(JENNIC_SDK) $(MANUFACTURER_CODE) $(OTA_STRING) $(JET_VERSION) $(JENNIC_CHIP_FAMILY) $(OTA_DEVICE_ID)
	mv $(APP_BLD_DIR)/OTABuild/Client.bin $(APP_BLD_DIR)/OTABuild/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE)_ENC.bin
	mv $(APP_BLD_DIR)/OTABuild/Client_UpgradeImagewithOTAHeaderV2_Enc.bin $(APP_BLD_DIR)/OTABuild/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE)_V2_ENC.bin
	mv $(APP_BLD_DIR)/OTABuild/Client_UpgradeImagewithOTAHeaderV3_Enc.bin $(APP_BLD_DIR)/OTABuild/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE)_V3_ENC.bin	
	mv $(APP_BLD_DIR)/OTABuild/Client_UpgradeImagewithOTAHeaderV2_Enc.ota $(APP_BLD_DIR)/OTABuild/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE)_V2_ENC.ota
	mv $(APP_BLD_DIR)/OTABuild/Client_UpgradeImagewithOTAHeaderV3_Enc.ota $(APP_BLD_DIR)/OTABuild/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE)_V3_ENC.ota	
endif
	rm -f $(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).bin
	rm -f $(APP_BLD_DIR)/OTABuild/Sensor.bin
endif	

###############################################################################

clean:
	rm -f $(APPOBJS) $(APPDEPS) $(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).bin $(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).elf $(APP_BLD_DIR)/$(TARGET)$(TARGET_FEATURES)$(TARGET_HARDWARE).map
	rm -f $(APP_SRC_DIR)/pdum_gen.* $(APP_SRC_DIR)/zps_gen*.* $(APP_SRC_DIR)/pdum_apdu.*

###############################################################################
