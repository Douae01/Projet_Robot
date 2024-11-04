################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/VL53L0X.c \
../Core/Src/captDistIR.c \
../Core/Src/custom_memory_manager.c \
../Core/Src/dma_transport.c \
../Core/Src/drv_gpio.c \
../Core/Src/drv_i2c.c \
../Core/Src/drv_uart.c \
../Core/Src/freertos.c \
../Core/Src/groveLCD.c \
../Core/Src/main.c \
../Core/Src/microros_allocators.c \
../Core/Src/microros_time.c \
../Core/Src/motorCommand.c \
../Core/Src/quadEncoder.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/systemclock.c \
../Core/Src/util.c 

OBJS += \
./Core/Src/VL53L0X.o \
./Core/Src/captDistIR.o \
./Core/Src/custom_memory_manager.o \
./Core/Src/dma_transport.o \
./Core/Src/drv_gpio.o \
./Core/Src/drv_i2c.o \
./Core/Src/drv_uart.o \
./Core/Src/freertos.o \
./Core/Src/groveLCD.o \
./Core/Src/main.o \
./Core/Src/microros_allocators.o \
./Core/Src/microros_time.o \
./Core/Src/motorCommand.o \
./Core/Src/quadEncoder.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/systemclock.o \
./Core/Src/util.o 

C_DEPS += \
./Core/Src/VL53L0X.d \
./Core/Src/captDistIR.d \
./Core/Src/custom_memory_manager.d \
./Core/Src/dma_transport.d \
./Core/Src/drv_gpio.d \
./Core/Src/drv_i2c.d \
./Core/Src/drv_uart.d \
./Core/Src/freertos.d \
./Core/Src/groveLCD.d \
./Core/Src/main.d \
./Core/Src/microros_allocators.d \
./Core/Src/microros_time.d \
./Core/Src/motorCommand.d \
./Core/Src/quadEncoder.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/systemclock.d \
./Core/Src/util.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I"/home/douae/ENIB/SEN/Projet_Robot/WORKSPACE_F411_uROS/base_robot/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/VL53L0X.cyclo ./Core/Src/VL53L0X.d ./Core/Src/VL53L0X.o ./Core/Src/VL53L0X.su ./Core/Src/captDistIR.cyclo ./Core/Src/captDistIR.d ./Core/Src/captDistIR.o ./Core/Src/captDistIR.su ./Core/Src/custom_memory_manager.cyclo ./Core/Src/custom_memory_manager.d ./Core/Src/custom_memory_manager.o ./Core/Src/custom_memory_manager.su ./Core/Src/dma_transport.cyclo ./Core/Src/dma_transport.d ./Core/Src/dma_transport.o ./Core/Src/dma_transport.su ./Core/Src/drv_gpio.cyclo ./Core/Src/drv_gpio.d ./Core/Src/drv_gpio.o ./Core/Src/drv_gpio.su ./Core/Src/drv_i2c.cyclo ./Core/Src/drv_i2c.d ./Core/Src/drv_i2c.o ./Core/Src/drv_i2c.su ./Core/Src/drv_uart.cyclo ./Core/Src/drv_uart.d ./Core/Src/drv_uart.o ./Core/Src/drv_uart.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/groveLCD.cyclo ./Core/Src/groveLCD.d ./Core/Src/groveLCD.o ./Core/Src/groveLCD.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/microros_allocators.cyclo ./Core/Src/microros_allocators.d ./Core/Src/microros_allocators.o ./Core/Src/microros_allocators.su ./Core/Src/microros_time.cyclo ./Core/Src/microros_time.d ./Core/Src/microros_time.o ./Core/Src/microros_time.su ./Core/Src/motorCommand.cyclo ./Core/Src/motorCommand.d ./Core/Src/motorCommand.o ./Core/Src/motorCommand.su ./Core/Src/quadEncoder.cyclo ./Core/Src/quadEncoder.d ./Core/Src/quadEncoder.o ./Core/Src/quadEncoder.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/systemclock.cyclo ./Core/Src/systemclock.d ./Core/Src/systemclock.o ./Core/Src/systemclock.su ./Core/Src/util.cyclo ./Core/Src/util.d ./Core/Src/util.o ./Core/Src/util.su

.PHONY: clean-Core-2f-Src

