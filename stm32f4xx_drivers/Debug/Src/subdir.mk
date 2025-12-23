################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/011_i2c_master_rx_testing_IT.c \
../Src/sysmem.c 

OBJS += \
./Src/011_i2c_master_rx_testing_IT.o \
./Src/sysmem.o 

C_DEPS += \
./Src/011_i2c_master_rx_testing_IT.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"C:/Embedded_programming/Codes/stm32f4xx_drivers/drivers/Inc" -I"C:/Embedded_programming/Codes/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/011_i2c_master_rx_testing_IT.cyclo ./Src/011_i2c_master_rx_testing_IT.d ./Src/011_i2c_master_rx_testing_IT.o ./Src/011_i2c_master_rx_testing_IT.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

