################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/encoder.c \
../Core/Src/flash_storage.c \
../Core/Src/floodfill.c \
../Core/Src/ir_sensor.c \
../Core/Src/main.c \
../Core/Src/maze.c \
../Core/Src/motion.c \
../Core/Src/motor.c \
../Core/Src/mpu6500.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/system_ui.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/encoder.o \
./Core/Src/flash_storage.o \
./Core/Src/floodfill.o \
./Core/Src/ir_sensor.o \
./Core/Src/main.o \
./Core/Src/maze.o \
./Core/Src/motion.o \
./Core/Src/motor.o \
./Core/Src/mpu6500.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/system_ui.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/encoder.d \
./Core/Src/flash_storage.d \
./Core/Src/floodfill.d \
./Core/Src/ir_sensor.d \
./Core/Src/main.d \
./Core/Src/maze.d \
./Core/Src/motion.d \
./Core/Src/motor.d \
./Core/Src/mpu6500.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/system_ui.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/encoder.cyclo ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/encoder.su ./Core/Src/flash_storage.cyclo ./Core/Src/flash_storage.d ./Core/Src/flash_storage.o ./Core/Src/flash_storage.su ./Core/Src/floodfill.cyclo ./Core/Src/floodfill.d ./Core/Src/floodfill.o ./Core/Src/floodfill.su ./Core/Src/ir_sensor.cyclo ./Core/Src/ir_sensor.d ./Core/Src/ir_sensor.o ./Core/Src/ir_sensor.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/maze.cyclo ./Core/Src/maze.d ./Core/Src/maze.o ./Core/Src/maze.su ./Core/Src/motion.cyclo ./Core/Src/motion.d ./Core/Src/motion.o ./Core/Src/motion.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/mpu6500.cyclo ./Core/Src/mpu6500.d ./Core/Src/mpu6500.o ./Core/Src/mpu6500.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/system_ui.cyclo ./Core/Src/system_ui.d ./Core/Src/system_ui.o ./Core/Src/system_ui.su

.PHONY: clean-Core-2f-Src

