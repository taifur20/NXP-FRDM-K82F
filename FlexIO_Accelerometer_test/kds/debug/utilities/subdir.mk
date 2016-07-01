################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/taifur/Desktop/nxp/FRDM_K82F/devices/MK82F25615/utilities/fsl_debug_console.c \
C:/Users/taifur/Desktop/nxp/FRDM_K82F/devices/MK82F25615/utilities/fsl_notifier.c \
C:/Users/taifur/Desktop/nxp/FRDM_K82F/devices/MK82F25615/utilities/fsl_sbrk.c 

OBJS += \
./utilities/fsl_debug_console.o \
./utilities/fsl_notifier.o \
./utilities/fsl_sbrk.o 

C_DEPS += \
./utilities/fsl_debug_console.d \
./utilities/fsl_notifier.d \
./utilities/fsl_sbrk.d 


# Each subdirectory must supply rules for building sources it contributes
utilities/fsl_debug_console.o: C:/Users/taifur/Desktop/nxp/FRDM_K82F/devices/MK82F25615/utilities/fsl_debug_console.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MK82FN256VLL15 -I../../ -I../../../../../../devices/MK82F25615/gcc -I../../../../../../devices/MK82F25615 -I../../../../../../devices/MK82F25615/utilities -I../../../../../../devices/MK82F25615/drivers -I../../../../../../CMSIS/Include -std=gnu99 -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

utilities/fsl_notifier.o: C:/Users/taifur/Desktop/nxp/FRDM_K82F/devices/MK82F25615/utilities/fsl_notifier.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MK82FN256VLL15 -I../../ -I../../../../../../devices/MK82F25615/gcc -I../../../../../../devices/MK82F25615 -I../../../../../../devices/MK82F25615/utilities -I../../../../../../devices/MK82F25615/drivers -I../../../../../../CMSIS/Include -std=gnu99 -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

utilities/fsl_sbrk.o: C:/Users/taifur/Desktop/nxp/FRDM_K82F/devices/MK82F25615/utilities/fsl_sbrk.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MK82FN256VLL15 -I../../ -I../../../../../../devices/MK82F25615/gcc -I../../../../../../devices/MK82F25615 -I../../../../../../devices/MK82F25615/utilities -I../../../../../../devices/MK82F25615/drivers -I../../../../../../CMSIS/Include -std=gnu99 -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


