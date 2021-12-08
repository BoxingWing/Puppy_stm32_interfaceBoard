################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../COM_interpreter/Src/JY901_Interpreter.c \
../COM_interpreter/Src/Servo_Interpreter.c \
../COM_interpreter/Src/raspi_Interpreter.c 

OBJS += \
./COM_interpreter/Src/JY901_Interpreter.o \
./COM_interpreter/Src/Servo_Interpreter.o \
./COM_interpreter/Src/raspi_Interpreter.o 

C_DEPS += \
./COM_interpreter/Src/JY901_Interpreter.d \
./COM_interpreter/Src/Servo_Interpreter.d \
./COM_interpreter/Src/raspi_Interpreter.d 


# Each subdirectory must supply rules for building sources it contributes
COM_interpreter/Src/%.o: ../COM_interpreter/Src/%.c COM_interpreter/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32F405xx -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"E:/STM32_Projects/FourLineSerial/COM_interpreter/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-COM_interpreter-2f-Src

clean-COM_interpreter-2f-Src:
	-$(RM) ./COM_interpreter/Src/JY901_Interpreter.d ./COM_interpreter/Src/JY901_Interpreter.o ./COM_interpreter/Src/Servo_Interpreter.d ./COM_interpreter/Src/Servo_Interpreter.o ./COM_interpreter/Src/raspi_Interpreter.d ./COM_interpreter/Src/raspi_Interpreter.o

.PHONY: clean-COM_interpreter-2f-Src

