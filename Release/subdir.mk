################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AutoSkew.c \
../MIG-S1-usbdscr.c \
../MIG-S1.c \
../cyfxtx.c \
../dps.c \
../epcs.c \
../i2c_fpga.c \
../ispPower.c 

S_UPPER_SRCS += \
../cyfx_gcc_startup.S 

OBJS += \
./AutoSkew.o \
./MIG-S1-usbdscr.o \
./MIG-S1.o \
./cyfx_gcc_startup.o \
./cyfxtx.o \
./dps.o \
./epcs.o \
./i2c_fpga.o \
./ispPower.o 

C_DEPS += \
./AutoSkew.d \
./MIG-S1-usbdscr.d \
./MIG-S1.d \
./cyfxtx.d \
./dps.d \
./epcs.d \
./i2c_fpga.d \
./ispPower.d 

S_UPPER_DEPS += \
./cyfx_gcc_startup.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC C Compiler'
	arm-none-eabi-gcc -D__CYU3P_TX__=1 -I"C:\Program Files (x86)\Cypress\EZ-USB FX3 SDK\1.3\\firmware\u3p_firmware\inc" -I"..\." -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=arm926ej-s -mthumb-interwork -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.S
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC Assembler'
	arm-none-eabi-gcc -x assembler-with-cpp -I"C:\Program Files (x86)\Cypress\EZ-USB FX3 SDK\1.3\\firmware\u3p_firmware\inc" -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=arm926ej-s -mthumb-interwork -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


