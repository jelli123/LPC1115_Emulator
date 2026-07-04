################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++M_DEPS)),)
-include $(C++M_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(CCM_DEPS)),)
-include $(CCM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXXM_DEPS)),)
-include $(CXXM_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Download-Test
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: Download-Test.axf

# Tool invocations
Download-Test.axf: $(OBJS) $(USER_OBJS) makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU C++ Linker'
	arm-none-eabi-c++ -nostdlib -L"/home/mirko/Workspaces/MCUXpresso_25.6.136/CMSIS_CORE_LPC11xx/Release" -L"/home/mirko/Projekte/Hardware/KNX/_GIT_Selfbus/software-arm-lib/sblib/Release" -Xlinker -Map="Download-Test.map" -Xlinker --gc-sections -flto -Os -mcpu=cortex-m0 -mthumb -T Download-Test_Release.ld -o "Download-Test.axf" $(OBJS) $(USER_OBJS) $(LIBS) -lCMSIS_CORE_LPC11xx -lsblib
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Download-Test.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-"/home/mirko/Projekte/Hardware/KNX/_GIT_Selfbus/software-arm-lib/examples/download-test/../../script/build-script/post-build-steps.sh" "/home/mirko/Projekte/Hardware/KNX/_GIT_Selfbus/software-arm-lib/examples/download-test/Release" "LPC1115_303" "Download-Test.axf" "Download-Test" "Release" "" "/home/mirko/Workspaces/hex"
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets
