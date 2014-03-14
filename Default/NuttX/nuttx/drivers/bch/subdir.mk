################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/bch/bchdev_driver.c \
../NuttX/nuttx/drivers/bch/bchdev_register.c \
../NuttX/nuttx/drivers/bch/bchdev_unregister.c \
../NuttX/nuttx/drivers/bch/bchlib_cache.c \
../NuttX/nuttx/drivers/bch/bchlib_read.c \
../NuttX/nuttx/drivers/bch/bchlib_sem.c \
../NuttX/nuttx/drivers/bch/bchlib_setup.c \
../NuttX/nuttx/drivers/bch/bchlib_teardown.c \
../NuttX/nuttx/drivers/bch/bchlib_write.c 

OBJS += \
./NuttX/nuttx/drivers/bch/bchdev_driver.o \
./NuttX/nuttx/drivers/bch/bchdev_register.o \
./NuttX/nuttx/drivers/bch/bchdev_unregister.o \
./NuttX/nuttx/drivers/bch/bchlib_cache.o \
./NuttX/nuttx/drivers/bch/bchlib_read.o \
./NuttX/nuttx/drivers/bch/bchlib_sem.o \
./NuttX/nuttx/drivers/bch/bchlib_setup.o \
./NuttX/nuttx/drivers/bch/bchlib_teardown.o \
./NuttX/nuttx/drivers/bch/bchlib_write.o 

C_DEPS += \
./NuttX/nuttx/drivers/bch/bchdev_driver.d \
./NuttX/nuttx/drivers/bch/bchdev_register.d \
./NuttX/nuttx/drivers/bch/bchdev_unregister.d \
./NuttX/nuttx/drivers/bch/bchlib_cache.d \
./NuttX/nuttx/drivers/bch/bchlib_read.d \
./NuttX/nuttx/drivers/bch/bchlib_sem.d \
./NuttX/nuttx/drivers/bch/bchlib_setup.d \
./NuttX/nuttx/drivers/bch/bchlib_teardown.d \
./NuttX/nuttx/drivers/bch/bchlib_write.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/bch/%.o: ../NuttX/nuttx/drivers/bch/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


