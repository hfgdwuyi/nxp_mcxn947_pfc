################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../freertos/freertos-kernel/portable/MemMang/heap_4.c 

C_DEPS += \
./freertos/freertos-kernel/portable/MemMang/heap_4.d 

OBJS += \
./freertos/freertos-kernel/portable/MemMang/heap_4.o 


# Each subdirectory must supply rules for building sources it contributes
freertos/freertos-kernel/portable/MemMang/%.o: ../freertos/freertos-kernel/portable/MemMang/%.c freertos/freertos-kernel/portable/MemMang/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MCXN947VDF -DCPU_MCXN947VDF_cm33 -DCPU_MCXN947VDF_cm33_core0 -DMCUXPRESSO_SDK -DSERIAL_PORT_TYPE_UART=1 -DSDK_OS_FREE_RTOS -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"D:\Project\ME Box\PS_pfc\SourceCode\frdmmcxn947_freertos_v1.0.3\source" -I"D:\Project\ME Box\PS_pfc\SourceCode\frdmmcxn947_freertos_v1.0.3\freertos\freertos-kernel\portable\GCC\ARM_CM33_NTZ\non_secure" -I"D:\Project\ME Box\PS_pfc\SourceCode\frdmmcxn947_freertos_v1.0.3\drivers" -I"D:\Project\ME Box\PS_pfc\SourceCode\frdmmcxn947_freertos_v1.0.3\device" -I"D:\Project\ME Box\PS_pfc\SourceCode\frdmmcxn947_freertos_v1.0.3\startup" -I"D:\Project\ME Box\PS_pfc\SourceCode\frdmmcxn947_freertos_v1.0.3\utilities" -I"D:\Project\ME Box\PS_pfc\SourceCode\frdmmcxn947_freertos_v1.0.3\component\uart" -I"D:\Project\ME Box\PS_pfc\SourceCode\frdmmcxn947_freertos_v1.0.3\component\serial_manager" -I"D:\Project\ME Box\PS_pfc\SourceCode\frdmmcxn947_freertos_v1.0.3\component\lists" -I"D:\Project\ME Box\PS_pfc\SourceCode\frdmmcxn947_freertos_v1.0.3\freertos\freertos-kernel\include" -I"D:\Project\ME Box\PS_pfc\SourceCode\frdmmcxn947_freertos_v1.0.3\CMSIS" -I"D:\Project\ME Box\PS_pfc\SourceCode\frdmmcxn947_freertos_v1.0.3\board" -O0 -fno-common -g3 -gdwarf-4 -mcpu=cortex-m33 -c -ffunction-sections -fdata-sections -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-freertos-2f-freertos-2d-kernel-2f-portable-2f-MemMang

clean-freertos-2f-freertos-2d-kernel-2f-portable-2f-MemMang:
	-$(RM) ./freertos/freertos-kernel/portable/MemMang/heap_4.d ./freertos/freertos-kernel/portable/MemMang/heap_4.o

.PHONY: clean-freertos-2f-freertos-2d-kernel-2f-portable-2f-MemMang

