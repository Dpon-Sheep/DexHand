################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MyForceControl.c \
../Core/Src/MyMotor.c \
../Core/Src/MyWeight.c \
../Core/Src/io_i2c.c \
../Core/Src/main.c \
../Core/Src/stm32f0xx_hal_msp.c \
../Core/Src/stm32f0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f0xx.c \
../Core/Src/usb_device.c \
../Core/Src/usbd_cdc.c \
../Core/Src/usbd_cdc_if.c \
../Core/Src/usbd_conf.c \
../Core/Src/usbd_core.c \
../Core/Src/usbd_ctlreq.c \
../Core/Src/usbd_desc.c \
../Core/Src/usbd_ioreq.c 

OBJS += \
./Core/Src/MyForceControl.o \
./Core/Src/MyMotor.o \
./Core/Src/MyWeight.o \
./Core/Src/io_i2c.o \
./Core/Src/main.o \
./Core/Src/stm32f0xx_hal_msp.o \
./Core/Src/stm32f0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f0xx.o \
./Core/Src/usb_device.o \
./Core/Src/usbd_cdc.o \
./Core/Src/usbd_cdc_if.o \
./Core/Src/usbd_conf.o \
./Core/Src/usbd_core.o \
./Core/Src/usbd_ctlreq.o \
./Core/Src/usbd_desc.o \
./Core/Src/usbd_ioreq.o 

C_DEPS += \
./Core/Src/MyForceControl.d \
./Core/Src/MyMotor.d \
./Core/Src/MyWeight.d \
./Core/Src/io_i2c.d \
./Core/Src/main.d \
./Core/Src/stm32f0xx_hal_msp.d \
./Core/Src/stm32f0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f0xx.d \
./Core/Src/usb_device.d \
./Core/Src/usbd_cdc.d \
./Core/Src/usbd_cdc_if.d \
./Core/Src/usbd_conf.d \
./Core/Src/usbd_core.d \
./Core/Src/usbd_ctlreq.d \
./Core/Src/usbd_desc.d \
./Core/Src/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Core/Inc -I../User -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/MyForceControl.cyclo ./Core/Src/MyForceControl.d ./Core/Src/MyForceControl.o ./Core/Src/MyForceControl.su ./Core/Src/MyMotor.cyclo ./Core/Src/MyMotor.d ./Core/Src/MyMotor.o ./Core/Src/MyMotor.su ./Core/Src/MyWeight.cyclo ./Core/Src/MyWeight.d ./Core/Src/MyWeight.o ./Core/Src/MyWeight.su ./Core/Src/io_i2c.cyclo ./Core/Src/io_i2c.d ./Core/Src/io_i2c.o ./Core/Src/io_i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f0xx_hal_msp.cyclo ./Core/Src/stm32f0xx_hal_msp.d ./Core/Src/stm32f0xx_hal_msp.o ./Core/Src/stm32f0xx_hal_msp.su ./Core/Src/stm32f0xx_it.cyclo ./Core/Src/stm32f0xx_it.d ./Core/Src/stm32f0xx_it.o ./Core/Src/stm32f0xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f0xx.cyclo ./Core/Src/system_stm32f0xx.d ./Core/Src/system_stm32f0xx.o ./Core/Src/system_stm32f0xx.su ./Core/Src/usb_device.cyclo ./Core/Src/usb_device.d ./Core/Src/usb_device.o ./Core/Src/usb_device.su ./Core/Src/usbd_cdc.cyclo ./Core/Src/usbd_cdc.d ./Core/Src/usbd_cdc.o ./Core/Src/usbd_cdc.su ./Core/Src/usbd_cdc_if.cyclo ./Core/Src/usbd_cdc_if.d ./Core/Src/usbd_cdc_if.o ./Core/Src/usbd_cdc_if.su ./Core/Src/usbd_conf.cyclo ./Core/Src/usbd_conf.d ./Core/Src/usbd_conf.o ./Core/Src/usbd_conf.su ./Core/Src/usbd_core.cyclo ./Core/Src/usbd_core.d ./Core/Src/usbd_core.o ./Core/Src/usbd_core.su ./Core/Src/usbd_ctlreq.cyclo ./Core/Src/usbd_ctlreq.d ./Core/Src/usbd_ctlreq.o ./Core/Src/usbd_ctlreq.su ./Core/Src/usbd_desc.cyclo ./Core/Src/usbd_desc.d ./Core/Src/usbd_desc.o ./Core/Src/usbd_desc.su ./Core/Src/usbd_ioreq.cyclo ./Core/Src/usbd_ioreq.d ./Core/Src/usbd_ioreq.o ./Core/Src/usbd_ioreq.su

.PHONY: clean-Core-2f-Src

