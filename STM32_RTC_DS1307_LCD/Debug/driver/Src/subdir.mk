################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../driver/Src/stm32f407xx_rcc_driver.c \
../driver/Src/stm32f446xx_gpio_driver.c \
../driver/Src/stm32f446xx_i2c_driver.c 

OBJS += \
./driver/Src/stm32f407xx_rcc_driver.o \
./driver/Src/stm32f446xx_gpio_driver.o \
./driver/Src/stm32f446xx_i2c_driver.o 

C_DEPS += \
./driver/Src/stm32f407xx_rcc_driver.d \
./driver/Src/stm32f446xx_gpio_driver.d \
./driver/Src/stm32f446xx_i2c_driver.d 


# Each subdirectory must supply rules for building sources it contributes
driver/Src/%.o driver/Src/%.su driver/Src/%.cyclo: ../driver/Src/%.c driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"D:/STM32_MCU1/STM32_MCU1_Workspace/STM32_RTC_DS1307_LCD/driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-driver-2f-Src

clean-driver-2f-Src:
	-$(RM) ./driver/Src/stm32f407xx_rcc_driver.cyclo ./driver/Src/stm32f407xx_rcc_driver.d ./driver/Src/stm32f407xx_rcc_driver.o ./driver/Src/stm32f407xx_rcc_driver.su ./driver/Src/stm32f446xx_gpio_driver.cyclo ./driver/Src/stm32f446xx_gpio_driver.d ./driver/Src/stm32f446xx_gpio_driver.o ./driver/Src/stm32f446xx_gpio_driver.su ./driver/Src/stm32f446xx_i2c_driver.cyclo ./driver/Src/stm32f446xx_i2c_driver.d ./driver/Src/stm32f446xx_i2c_driver.o ./driver/Src/stm32f446xx_i2c_driver.su

.PHONY: clean-driver-2f-Src

