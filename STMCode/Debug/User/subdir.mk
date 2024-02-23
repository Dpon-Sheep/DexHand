################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/usbd_cdc.c \
../User/usbd_cdc_if.c \
../User/usbd_conf.c \
../User/usbd_core.c \
../User/usbd_ctlreq.c \
../User/usbd_desc.c \
../User/usbd_ioreq.c 

OBJS += \
./User/usbd_cdc.o \
./User/usbd_cdc_if.o \
./User/usbd_conf.o \
./User/usbd_core.o \
./User/usbd_ctlreq.o \
./User/usbd_desc.o \
./User/usbd_ioreq.o 

C_DEPS += \
./User/usbd_cdc.d \
./User/usbd_cdc_if.d \
./User/usbd_conf.d \
./User/usbd_core.d \
./User/usbd_ctlreq.d \
./User/usbd_desc.d \
./User/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
User/%.o User/%.su User/%.cyclo: ../User/%.c User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Core/Inc -I../User -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-User

clean-User:
	-$(RM) ./User/usbd_cdc.cyclo ./User/usbd_cdc.d ./User/usbd_cdc.o ./User/usbd_cdc.su ./User/usbd_cdc_if.cyclo ./User/usbd_cdc_if.d ./User/usbd_cdc_if.o ./User/usbd_cdc_if.su ./User/usbd_conf.cyclo ./User/usbd_conf.d ./User/usbd_conf.o ./User/usbd_conf.su ./User/usbd_core.cyclo ./User/usbd_core.d ./User/usbd_core.o ./User/usbd_core.su ./User/usbd_ctlreq.cyclo ./User/usbd_ctlreq.d ./User/usbd_ctlreq.o ./User/usbd_ctlreq.su ./User/usbd_desc.cyclo ./User/usbd_desc.d ./User/usbd_desc.o ./User/usbd_desc.su ./User/usbd_ioreq.cyclo ./User/usbd_ioreq.d ./User/usbd_ioreq.o ./User/usbd_ioreq.su

.PHONY: clean-User

