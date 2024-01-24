################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32F411E-DISCO/stm32f411e_disco_bus.c 

OBJS += \
./Drivers/BSP/STM32F411E-DISCO/stm32f411e_disco_bus.o 

C_DEPS += \
./Drivers/BSP/STM32F411E-DISCO/stm32f411e_disco_bus.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32F411E-DISCO/%.o Drivers/BSP/STM32F411E-DISCO/%.su Drivers/BSP/STM32F411E-DISCO/%.cyclo: ../Drivers/BSP/STM32F411E-DISCO/%.c Drivers/BSP/STM32F411E-DISCO/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/STM32F411E-DISCO -I../Drivers/BSP/Components/lsm303agr -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32F411E-2d-DISCO

clean-Drivers-2f-BSP-2f-STM32F411E-2d-DISCO:
	-$(RM) ./Drivers/BSP/STM32F411E-DISCO/stm32f411e_disco_bus.cyclo ./Drivers/BSP/STM32F411E-DISCO/stm32f411e_disco_bus.d ./Drivers/BSP/STM32F411E-DISCO/stm32f411e_disco_bus.o ./Drivers/BSP/STM32F411E-DISCO/stm32f411e_disco_bus.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32F411E-2d-DISCO

