################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/arm/src/chip/stm32_adc.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_allocateheap.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_can.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_dac.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_dma.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_dumpgpio.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_eth.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_exti_alarm.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_exti_gpio.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_flash.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_gpio.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_i2c.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_idle.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_irq.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_iwdg.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_lowputc.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_lse.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_lsi.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_mpuinit.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_otgfsdev.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_otgfshost.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_pminitialize.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_pmsleep.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_pmstandby.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_pmstop.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_pwm.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_pwr.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_qencoder.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_rcc.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_rng.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_rtc.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_rtcc.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_rtcounter.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_sdio.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_serial.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_spi.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_start.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_tim.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_timerisr.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_usbdev.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_userspace.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_waste.c \
../NuttX/nuttx/arch/arm/src/chip/stm32_wwdg.c \
../NuttX/nuttx/arch/arm/src/chip/stm32f10xxx_dma.c \
../NuttX/nuttx/arch/arm/src/chip/stm32f10xxx_rcc.c \
../NuttX/nuttx/arch/arm/src/chip/stm32f20xxx_dma.c \
../NuttX/nuttx/arch/arm/src/chip/stm32f20xxx_rcc.c \
../NuttX/nuttx/arch/arm/src/chip/stm32f30xxx_rcc.c \
../NuttX/nuttx/arch/arm/src/chip/stm32f40xxx_dma.c \
../NuttX/nuttx/arch/arm/src/chip/stm32f40xxx_rcc.c \
../NuttX/nuttx/arch/arm/src/chip/stm32l15xxx_rcc.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/arm/src/chip/stm32_vectors.S 

OBJS += \
./NuttX/nuttx/arch/arm/src/chip/stm32_adc.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_allocateheap.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_can.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_dac.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_dma.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_dumpgpio.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_eth.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_exti_alarm.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_exti_gpio.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_flash.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_gpio.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_i2c.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_idle.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_irq.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_iwdg.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_lowputc.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_lse.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_lsi.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_mpuinit.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_otgfsdev.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_otgfshost.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_pminitialize.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_pmsleep.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_pmstandby.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_pmstop.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_pwm.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_pwr.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_qencoder.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_rcc.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_rng.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_rtc.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_rtcc.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_rtcounter.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_sdio.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_serial.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_spi.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_start.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_tim.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_timerisr.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_usbdev.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_userspace.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_vectors.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_waste.o \
./NuttX/nuttx/arch/arm/src/chip/stm32_wwdg.o \
./NuttX/nuttx/arch/arm/src/chip/stm32f10xxx_dma.o \
./NuttX/nuttx/arch/arm/src/chip/stm32f10xxx_rcc.o \
./NuttX/nuttx/arch/arm/src/chip/stm32f20xxx_dma.o \
./NuttX/nuttx/arch/arm/src/chip/stm32f20xxx_rcc.o \
./NuttX/nuttx/arch/arm/src/chip/stm32f30xxx_rcc.o \
./NuttX/nuttx/arch/arm/src/chip/stm32f40xxx_dma.o \
./NuttX/nuttx/arch/arm/src/chip/stm32f40xxx_rcc.o \
./NuttX/nuttx/arch/arm/src/chip/stm32l15xxx_rcc.o 

C_DEPS += \
./NuttX/nuttx/arch/arm/src/chip/stm32_adc.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_allocateheap.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_can.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_dac.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_dma.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_dumpgpio.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_eth.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_exti_alarm.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_exti_gpio.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_flash.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_gpio.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_i2c.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_idle.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_irq.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_iwdg.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_lowputc.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_lse.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_lsi.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_mpuinit.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_otgfsdev.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_otgfshost.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_pminitialize.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_pmsleep.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_pmstandby.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_pmstop.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_pwm.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_pwr.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_qencoder.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_rcc.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_rng.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_rtc.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_rtcc.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_rtcounter.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_sdio.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_serial.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_spi.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_start.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_tim.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_timerisr.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_usbdev.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_userspace.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_waste.d \
./NuttX/nuttx/arch/arm/src/chip/stm32_wwdg.d \
./NuttX/nuttx/arch/arm/src/chip/stm32f10xxx_dma.d \
./NuttX/nuttx/arch/arm/src/chip/stm32f10xxx_rcc.d \
./NuttX/nuttx/arch/arm/src/chip/stm32f20xxx_dma.d \
./NuttX/nuttx/arch/arm/src/chip/stm32f20xxx_rcc.d \
./NuttX/nuttx/arch/arm/src/chip/stm32f30xxx_rcc.d \
./NuttX/nuttx/arch/arm/src/chip/stm32f40xxx_dma.d \
./NuttX/nuttx/arch/arm/src/chip/stm32f40xxx_rcc.d \
./NuttX/nuttx/arch/arm/src/chip/stm32l15xxx_rcc.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/arm/src/chip/%.o: ../NuttX/nuttx/arch/arm/src/chip/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/arm/src/chip/%.o: ../NuttX/nuttx/arch/arm/src/chip/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


