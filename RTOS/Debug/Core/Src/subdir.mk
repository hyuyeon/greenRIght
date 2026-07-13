################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/buzzer.c \
../Core/Src/buzzerTask.c \
../Core/Src/can_rx_task.c \
../Core/Src/debug_uart.c \
../Core/Src/display_tasks.c \
../Core/Src/freertos.c \
../Core/Src/lcd.c \
../Core/Src/main.c \
../Core/Src/personDetect.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/ttc.c \
../Core/Src/turnJudgeTask.c 

OBJS += \
./Core/Src/buzzer.o \
./Core/Src/buzzerTask.o \
./Core/Src/can_rx_task.o \
./Core/Src/debug_uart.o \
./Core/Src/display_tasks.o \
./Core/Src/freertos.o \
./Core/Src/lcd.o \
./Core/Src/main.o \
./Core/Src/personDetect.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/ttc.o \
./Core/Src/turnJudgeTask.o 

C_DEPS += \
./Core/Src/buzzer.d \
./Core/Src/buzzerTask.d \
./Core/Src/can_rx_task.d \
./Core/Src/debug_uart.d \
./Core/Src/display_tasks.d \
./Core/Src/freertos.d \
./Core/Src/lcd.d \
./Core/Src/main.d \
./Core/Src/personDetect.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/ttc.d \
./Core/Src/turnJudgeTask.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/buzzer.cyclo ./Core/Src/buzzer.d ./Core/Src/buzzer.o ./Core/Src/buzzer.su ./Core/Src/buzzerTask.cyclo ./Core/Src/buzzerTask.d ./Core/Src/buzzerTask.o ./Core/Src/buzzerTask.su ./Core/Src/can_rx_task.cyclo ./Core/Src/can_rx_task.d ./Core/Src/can_rx_task.o ./Core/Src/can_rx_task.su ./Core/Src/debug_uart.cyclo ./Core/Src/debug_uart.d ./Core/Src/debug_uart.o ./Core/Src/debug_uart.su ./Core/Src/display_tasks.cyclo ./Core/Src/display_tasks.d ./Core/Src/display_tasks.o ./Core/Src/display_tasks.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/lcd.cyclo ./Core/Src/lcd.d ./Core/Src/lcd.o ./Core/Src/lcd.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/personDetect.cyclo ./Core/Src/personDetect.d ./Core/Src/personDetect.o ./Core/Src/personDetect.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/ttc.cyclo ./Core/Src/ttc.d ./Core/Src/ttc.o ./Core/Src/ttc.su ./Core/Src/turnJudgeTask.cyclo ./Core/Src/turnJudgeTask.d ./Core/Src/turnJudgeTask.o ./Core/Src/turnJudgeTask.su

.PHONY: clean-Core-2f-Src

