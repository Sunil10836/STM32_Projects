################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bsp/Src/lcd.c \
../bsp/Src/rtc_ds1307.c 

OBJS += \
./bsp/Src/lcd.o \
./bsp/Src/rtc_ds1307.o 

C_DEPS += \
./bsp/Src/lcd.d \
./bsp/Src/rtc_ds1307.d 


# Each subdirectory must supply rules for building sources it contributes
bsp/Src/%.o bsp/Src/%.su bsp/Src/%.cyclo: ../bsp/Src/%.c bsp/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"D:/STM32_MCU1/STM32_MCU1_Workspace/STM32_RTC_DS1307_LCD/driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bsp-2f-Src

clean-bsp-2f-Src:
	-$(RM) ./bsp/Src/lcd.cyclo ./bsp/Src/lcd.d ./bsp/Src/lcd.o ./bsp/Src/lcd.su ./bsp/Src/rtc_ds1307.cyclo ./bsp/Src/rtc_ds1307.d ./bsp/Src/rtc_ds1307.o ./bsp/Src/rtc_ds1307.su

.PHONY: clean-bsp-2f-Src

