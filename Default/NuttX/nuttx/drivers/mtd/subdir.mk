################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/mtd/at24xx.c \
../NuttX/nuttx/drivers/mtd/at25.c \
../NuttX/nuttx/drivers/mtd/at45db.c \
../NuttX/nuttx/drivers/mtd/flash_eraseall.c \
../NuttX/nuttx/drivers/mtd/ftl.c \
../NuttX/nuttx/drivers/mtd/m25px.c \
../NuttX/nuttx/drivers/mtd/mtd_partition.c \
../NuttX/nuttx/drivers/mtd/rammtd.c \
../NuttX/nuttx/drivers/mtd/ramtron.c \
../NuttX/nuttx/drivers/mtd/skeleton.c \
../NuttX/nuttx/drivers/mtd/smart.c \
../NuttX/nuttx/drivers/mtd/sst25.c \
../NuttX/nuttx/drivers/mtd/sst39vf.c \
../NuttX/nuttx/drivers/mtd/w25.c 

OBJS += \
./NuttX/nuttx/drivers/mtd/at24xx.o \
./NuttX/nuttx/drivers/mtd/at25.o \
./NuttX/nuttx/drivers/mtd/at45db.o \
./NuttX/nuttx/drivers/mtd/flash_eraseall.o \
./NuttX/nuttx/drivers/mtd/ftl.o \
./NuttX/nuttx/drivers/mtd/m25px.o \
./NuttX/nuttx/drivers/mtd/mtd_partition.o \
./NuttX/nuttx/drivers/mtd/rammtd.o \
./NuttX/nuttx/drivers/mtd/ramtron.o \
./NuttX/nuttx/drivers/mtd/skeleton.o \
./NuttX/nuttx/drivers/mtd/smart.o \
./NuttX/nuttx/drivers/mtd/sst25.o \
./NuttX/nuttx/drivers/mtd/sst39vf.o \
./NuttX/nuttx/drivers/mtd/w25.o 

C_DEPS += \
./NuttX/nuttx/drivers/mtd/at24xx.d \
./NuttX/nuttx/drivers/mtd/at25.d \
./NuttX/nuttx/drivers/mtd/at45db.d \
./NuttX/nuttx/drivers/mtd/flash_eraseall.d \
./NuttX/nuttx/drivers/mtd/ftl.d \
./NuttX/nuttx/drivers/mtd/m25px.d \
./NuttX/nuttx/drivers/mtd/mtd_partition.d \
./NuttX/nuttx/drivers/mtd/rammtd.d \
./NuttX/nuttx/drivers/mtd/ramtron.d \
./NuttX/nuttx/drivers/mtd/skeleton.d \
./NuttX/nuttx/drivers/mtd/smart.d \
./NuttX/nuttx/drivers/mtd/sst25.d \
./NuttX/nuttx/drivers/mtd/sst39vf.d \
./NuttX/nuttx/drivers/mtd/w25.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/mtd/%.o: ../NuttX/nuttx/drivers/mtd/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


