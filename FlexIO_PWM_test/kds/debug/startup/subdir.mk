################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/taifur/Desktop/nxp/FRDM_K82F/devices/MK82F25615/system_MK82F25615.c 

S_UPPER_SRCS += \
C:/Users/taifur/Desktop/nxp/FRDM_K82F/devices/MK82F25615/gcc/startup_MK82F25615.S 

OBJS += \
./startup/startup_MK82F25615.o \
./startup/system_MK82F25615.o 

C_DEPS += \
./startup/system_MK82F25615.d 

S_UPPER_DEPS += \
./startup/startup_MK82F25615.d 


# Each subdirectory must supply rules for building sources it contributes
startup/startup_MK82F25615.o: C:/Users/taifur/Desktop/nxp/FRDM_K82F/devices/MK82F25615/gcc/startup_MK82F25615.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -x assembler-with-cpp -DDEBUG -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

startup/system_MK82F25615.o: C:/Users/taifur/Desktop/nxp/FRDM_K82F/devices/MK82F25615/system_MK82F25615.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MK82FN256VLL15 -I../../ -I../../../../../../devices/MK82F25615/gcc -I../../../../../../devices/MK82F25615 -I../../../../../../devices/MK82F25615/utilities -I../../../../../../devices/MK82F25615/drivers -I../../../../../../CMSIS/Include -std=gnu99 -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


