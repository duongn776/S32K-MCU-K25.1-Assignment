################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/Driver_GPIO.c \
../Drivers/Src/Driver_PORT.c 

OBJS += \
./Drivers/Src/Driver_GPIO.o \
./Drivers/Src/Driver_PORT.o 

C_DEPS += \
./Drivers/Src/Driver_GPIO.d \
./Drivers/Src/Driver_PORT.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o: ../Drivers/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS C Compiler'
	arm-none-eabi-gcc "@Drivers/Src/Driver_GPIO.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


