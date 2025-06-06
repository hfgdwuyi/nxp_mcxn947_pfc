################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../startup/boot_multicore_slave.c \
../startup/startup_mcxn947_cm33_core0.c 

C_DEPS += \
./startup/boot_multicore_slave.d \
./startup/startup_mcxn947_cm33_core0.d 

OBJS += \
./startup/boot_multicore_slave.o \
./startup/startup_mcxn947_cm33_core0.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.c startup/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MCXN947VDF -DCPU_MCXN947VDF_cm33 -DCPU_MCXN947VDF_cm33_core0 -DMCUXPRESSO_SDK -DSERIAL_PORT_TYPE_UART=1 -DSDK_OS_FREE_RTOS -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"D:\Git\nxp_mcxn947_pfc\source" -I"D:\Git\nxp_mcxn947_pfc\freertos\freertos-kernel\portable\GCC\ARM_CM33_NTZ\non_secure" -I"D:\Git\nxp_mcxn947_pfc\drivers" -I"D:\Git\nxp_mcxn947_pfc\device" -I"D:\Git\nxp_mcxn947_pfc\startup" -I"D:\Git\nxp_mcxn947_pfc\utilities" -I"D:\Git\nxp_mcxn947_pfc\component\uart" -I"D:\Git\nxp_mcxn947_pfc\component\serial_manager" -I"D:\Git\nxp_mcxn947_pfc\component\lists" -I"D:\Git\nxp_mcxn947_pfc\freertos\freertos-kernel\include" -I"D:\Git\nxp_mcxn947_pfc\CMSIS" -I"D:\Git\nxp_mcxn947_pfc\board" -O0 -fno-common -g3 -gdwarf-4 -mcpu=cortex-m33 -c -ffunction-sections -fdata-sections -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-startup

clean-startup:
	-$(RM) ./startup/boot_multicore_slave.d ./startup/boot_multicore_slave.o ./startup/startup_mcxn947_cm33_core0.d ./startup/startup_mcxn947_cm33_core0.o

.PHONY: clean-startup

