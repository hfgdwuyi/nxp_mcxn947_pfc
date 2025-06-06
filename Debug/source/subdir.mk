################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/can.c \
../source/command.c \
../source/lcd.c \
../source/main.c \
../source/semihost_hardfault.c \
../source/sensor.c 

C_DEPS += \
./source/can.d \
./source/command.d \
./source/lcd.d \
./source/main.d \
./source/semihost_hardfault.d \
./source/sensor.d 

OBJS += \
./source/can.o \
./source/command.o \
./source/lcd.o \
./source/main.o \
./source/semihost_hardfault.o \
./source/sensor.o 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.c source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MCXN947VDF -DCPU_MCXN947VDF_cm33 -DCPU_MCXN947VDF_cm33_core0 -DMCUXPRESSO_SDK -DSERIAL_PORT_TYPE_UART=1 -DSDK_OS_FREE_RTOS -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"D:\Git\nxp_mcxn947_pfc\source" -I"D:\Git\nxp_mcxn947_pfc\freertos\freertos-kernel\portable\GCC\ARM_CM33_NTZ\non_secure" -I"D:\Git\nxp_mcxn947_pfc\drivers" -I"D:\Git\nxp_mcxn947_pfc\device" -I"D:\Git\nxp_mcxn947_pfc\startup" -I"D:\Git\nxp_mcxn947_pfc\utilities" -I"D:\Git\nxp_mcxn947_pfc\component\uart" -I"D:\Git\nxp_mcxn947_pfc\component\serial_manager" -I"D:\Git\nxp_mcxn947_pfc\component\lists" -I"D:\Git\nxp_mcxn947_pfc\freertos\freertos-kernel\include" -I"D:\Git\nxp_mcxn947_pfc\CMSIS" -I"D:\Git\nxp_mcxn947_pfc\board" -O0 -fno-common -g3 -gdwarf-4 -mcpu=cortex-m33 -c -ffunction-sections -fdata-sections -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source

clean-source:
	-$(RM) ./source/can.d ./source/can.o ./source/command.d ./source/command.o ./source/lcd.d ./source/lcd.o ./source/main.d ./source/main.o ./source/semihost_hardfault.d ./source/semihost_hardfault.o ./source/sensor.d ./source/sensor.o

.PHONY: clean-source

