################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/nuttx/libxx/libxx_cxa_atexit.cxx \
../NuttX/nuttx/libxx/libxx_cxapurevirtual.cxx \
../NuttX/nuttx/libxx/libxx_delete.cxx \
../NuttX/nuttx/libxx/libxx_deletea.cxx \
../NuttX/nuttx/libxx/libxx_eabi_atexit.cxx \
../NuttX/nuttx/libxx/libxx_new.cxx \
../NuttX/nuttx/libxx/libxx_newa.cxx \
../NuttX/nuttx/libxx/libxx_stdthrow.cxx 

OBJS += \
./NuttX/nuttx/libxx/libxx_cxa_atexit.o \
./NuttX/nuttx/libxx/libxx_cxapurevirtual.o \
./NuttX/nuttx/libxx/libxx_delete.o \
./NuttX/nuttx/libxx/libxx_deletea.o \
./NuttX/nuttx/libxx/libxx_eabi_atexit.o \
./NuttX/nuttx/libxx/libxx_new.o \
./NuttX/nuttx/libxx/libxx_newa.o \
./NuttX/nuttx/libxx/libxx_stdthrow.o 

CXX_DEPS += \
./NuttX/nuttx/libxx/libxx_cxa_atexit.d \
./NuttX/nuttx/libxx/libxx_cxapurevirtual.d \
./NuttX/nuttx/libxx/libxx_delete.d \
./NuttX/nuttx/libxx/libxx_deletea.d \
./NuttX/nuttx/libxx/libxx_eabi_atexit.d \
./NuttX/nuttx/libxx/libxx_new.d \
./NuttX/nuttx/libxx/libxx_newa.d \
./NuttX/nuttx/libxx/libxx_stdthrow.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libxx/%.o: ../NuttX/nuttx/libxx/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


