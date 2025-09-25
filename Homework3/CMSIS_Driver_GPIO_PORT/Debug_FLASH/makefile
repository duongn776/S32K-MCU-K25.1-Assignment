################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include Project_Settings/Startup_Code/subdir.mk
-include Drivers/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_SIZE += \
CMSIS_Driver_GPIO_PORT.siz \


# All Target
all: CMSIS_Driver_GPIO_PORT.elf secondary-outputs

# Tool invocations
CMSIS_Driver_GPIO_PORT.elf: $(OBJS) D:/Course_Online/MCU_S32K/assignment/Homework3/CMSIS_Driver_GPIO_PORT/Project_Settings/Linker_Files/S32K144_64_flash.ld $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Standard S32DS C Linker'
	arm-none-eabi-gcc -o "CMSIS_Driver_GPIO_PORT.elf" "@CMSIS_Driver_GPIO_PORT.args"  $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

CMSIS_Driver_GPIO_PORT.siz: CMSIS_Driver_GPIO_PORT.elf
	@echo 'Invoking: Standard S32DS Print Size'
	arm-none-eabi-size --format=berkeley CMSIS_Driver_GPIO_PORT.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) ./*
	-@echo ' '

secondary-outputs: $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
