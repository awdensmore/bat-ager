################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main.c \
../src/stm32l0xx_hal_msp.c \
../src/stm32l0xx_it.c 

OBJS += \
./src/main.o \
./src/stm32l0xx_hal_msp.o \
./src/stm32l0xx_it.o 

C_DEPS += \
./src/main.d \
./src/stm32l0xx_hal_msp.d \
./src/stm32l0xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DTRACE -DSTM32L053xx -I"../include" -I"../system/include" -I"../system/include/cmsis" -I../system/include/stm32l0 -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


