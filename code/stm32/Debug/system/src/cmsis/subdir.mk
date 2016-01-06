################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/cmsis/system_stm32f0xx.c 

S_UPPER_SRCS += \
../system/src/cmsis/startup_stm32f030x8.S 

OBJS += \
./system/src/cmsis/startup_stm32f030x8.o \
./system/src/cmsis/system_stm32f0xx.o 

C_DEPS += \
./system/src/cmsis/system_stm32f0xx.d 

S_UPPER_DEPS += \
./system/src/cmsis/startup_stm32f030x8.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/cmsis/%.o: ../system/src/cmsis/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -x assembler-with-cpp -DDEBUG -DSTM32F030x8 -DTRACE -I/home/projects/bat-ager/code/stm32/include -I/home/projects/bat-ager/code/stm32/include -I/home/projects/bat-ager/code/stm32/include/cmsis -I/home/projects/bat-ager/codestm32/system/include/stm32f0xx -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

system/src/cmsis/%.o: ../system/src/cmsis/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DSTM32F030x8 -DTRACE -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f0xx" -std=gnu11 -O -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


