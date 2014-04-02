################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../NuttX/apps/examples/elf/tests/helloxx/hello++1.cpp \
../NuttX/apps/examples/elf/tests/helloxx/hello++2.cpp \
../NuttX/apps/examples/elf/tests/helloxx/hello++3.cpp \
../NuttX/apps/examples/elf/tests/helloxx/hello++4.cpp 

OBJS += \
./NuttX/apps/examples/elf/tests/helloxx/hello++1.o \
./NuttX/apps/examples/elf/tests/helloxx/hello++2.o \
./NuttX/apps/examples/elf/tests/helloxx/hello++3.o \
./NuttX/apps/examples/elf/tests/helloxx/hello++4.o 

CPP_DEPS += \
./NuttX/apps/examples/elf/tests/helloxx/hello++1.d \
./NuttX/apps/examples/elf/tests/helloxx/hello++2.d \
./NuttX/apps/examples/elf/tests/helloxx/hello++3.d \
./NuttX/apps/examples/elf/tests/helloxx/hello++4.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/elf/tests/helloxx/%.o: ../NuttX/apps/examples/elf/tests/helloxx/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


