################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/app_main.cpp \
../src/cr_startup_lpc11xx.cpp 

CPP_DEPS += \
./src/app_main.d \
./src/cr_startup_lpc11xx.d 

OBJS += \
./src/app_main.o \
./src/cr_startup_lpc11xx.o 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C++ Compiler'
	arm-none-eabi-c++ -D__NEWLIB__ -DNDEBUG -DCORE_M0 -D__USE_CMSIS=CMSIS_CORE_LPC11xx -DCPP_USE_HEAP -D__LPC11XX__ -I"/home/mirko/Workspaces/MCUXpresso_25.6.136/CMSIS_CORE_LPC11xx/inc" -I"/home/mirko/Projekte/Hardware/KNX/_GIT_Selfbus/software-arm-lib/sblib/inc" -Os -gdwarf-4 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -flto -ffat-lto-objects -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0 -mthumb -D__NEWLIB__ -fstack-usage -specs=nano.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-src

clean-src:
	-$(RM) ./src/app_main.d ./src/app_main.o ./src/cr_startup_lpc11xx.d ./src/cr_startup_lpc11xx.o

.PHONY: clean-src

