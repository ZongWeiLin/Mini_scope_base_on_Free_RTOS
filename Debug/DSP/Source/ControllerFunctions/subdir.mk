################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DSP/Source/ControllerFunctions/arm_pid_init_f32.c \
../DSP/Source/ControllerFunctions/arm_pid_init_q15.c \
../DSP/Source/ControllerFunctions/arm_pid_init_q31.c \
../DSP/Source/ControllerFunctions/arm_pid_reset_f32.c \
../DSP/Source/ControllerFunctions/arm_pid_reset_q15.c \
../DSP/Source/ControllerFunctions/arm_pid_reset_q31.c \
../DSP/Source/ControllerFunctions/arm_sin_cos_f32.c \
../DSP/Source/ControllerFunctions/arm_sin_cos_q31.c 

OBJS += \
./DSP/Source/ControllerFunctions/arm_pid_init_f32.o \
./DSP/Source/ControllerFunctions/arm_pid_init_q15.o \
./DSP/Source/ControllerFunctions/arm_pid_init_q31.o \
./DSP/Source/ControllerFunctions/arm_pid_reset_f32.o \
./DSP/Source/ControllerFunctions/arm_pid_reset_q15.o \
./DSP/Source/ControllerFunctions/arm_pid_reset_q31.o \
./DSP/Source/ControllerFunctions/arm_sin_cos_f32.o \
./DSP/Source/ControllerFunctions/arm_sin_cos_q31.o 

C_DEPS += \
./DSP/Source/ControllerFunctions/arm_pid_init_f32.d \
./DSP/Source/ControllerFunctions/arm_pid_init_q15.d \
./DSP/Source/ControllerFunctions/arm_pid_init_q31.d \
./DSP/Source/ControllerFunctions/arm_pid_reset_f32.d \
./DSP/Source/ControllerFunctions/arm_pid_reset_q15.d \
./DSP/Source/ControllerFunctions/arm_pid_reset_q31.d \
./DSP/Source/ControllerFunctions/arm_sin_cos_f32.d \
./DSP/Source/ControllerFunctions/arm_sin_cos_q31.d 


# Each subdirectory must supply rules for building sources it contributes
DSP/Source/ControllerFunctions/%.o DSP/Source/ControllerFunctions/%.su: ../DSP/Source/ControllerFunctions/%.c DSP/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -D__TARGET_FPU_VFP -DARM_MATH_CM4 -D__FPU_PRESENT=1U -D__FPU_USED=1U -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"D:/stm32/workspace/Side_project_4/fft/inc" -I"D:/stm32/workspace/Side_project_4/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DSP-2f-Source-2f-ControllerFunctions

clean-DSP-2f-Source-2f-ControllerFunctions:
	-$(RM) ./DSP/Source/ControllerFunctions/arm_pid_init_f32.d ./DSP/Source/ControllerFunctions/arm_pid_init_f32.o ./DSP/Source/ControllerFunctions/arm_pid_init_f32.su ./DSP/Source/ControllerFunctions/arm_pid_init_q15.d ./DSP/Source/ControllerFunctions/arm_pid_init_q15.o ./DSP/Source/ControllerFunctions/arm_pid_init_q15.su ./DSP/Source/ControllerFunctions/arm_pid_init_q31.d ./DSP/Source/ControllerFunctions/arm_pid_init_q31.o ./DSP/Source/ControllerFunctions/arm_pid_init_q31.su ./DSP/Source/ControllerFunctions/arm_pid_reset_f32.d ./DSP/Source/ControllerFunctions/arm_pid_reset_f32.o ./DSP/Source/ControllerFunctions/arm_pid_reset_f32.su ./DSP/Source/ControllerFunctions/arm_pid_reset_q15.d ./DSP/Source/ControllerFunctions/arm_pid_reset_q15.o ./DSP/Source/ControllerFunctions/arm_pid_reset_q15.su ./DSP/Source/ControllerFunctions/arm_pid_reset_q31.d ./DSP/Source/ControllerFunctions/arm_pid_reset_q31.o ./DSP/Source/ControllerFunctions/arm_pid_reset_q31.su ./DSP/Source/ControllerFunctions/arm_sin_cos_f32.d ./DSP/Source/ControllerFunctions/arm_sin_cos_f32.o ./DSP/Source/ControllerFunctions/arm_sin_cos_f32.su ./DSP/Source/ControllerFunctions/arm_sin_cos_q31.d ./DSP/Source/ControllerFunctions/arm_sin_cos_q31.o ./DSP/Source/ControllerFunctions/arm_sin_cos_q31.su

.PHONY: clean-DSP-2f-Source-2f-ControllerFunctions

