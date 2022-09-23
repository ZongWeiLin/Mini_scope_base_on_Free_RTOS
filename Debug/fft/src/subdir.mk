################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../fft/src/cr4_fft_1024_stm32.s \
../fft/src/cr4_fft_256_stm32.s \
../fft/src/cr4_fft_64_stm32.s 

OBJS += \
./fft/src/cr4_fft_1024_stm32.o \
./fft/src/cr4_fft_256_stm32.o \
./fft/src/cr4_fft_64_stm32.o 

S_DEPS += \
./fft/src/cr4_fft_1024_stm32.d \
./fft/src/cr4_fft_256_stm32.d \
./fft/src/cr4_fft_64_stm32.d 


# Each subdirectory must supply rules for building sources it contributes
fft/src/%.o: ../fft/src/%.s fft/src/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-fft-2f-src

clean-fft-2f-src:
	-$(RM) ./fft/src/cr4_fft_1024_stm32.d ./fft/src/cr4_fft_1024_stm32.o ./fft/src/cr4_fft_256_stm32.d ./fft/src/cr4_fft_256_stm32.o ./fft/src/cr4_fft_64_stm32.d ./fft/src/cr4_fft_64_stm32.o

.PHONY: clean-fft-2f-src

