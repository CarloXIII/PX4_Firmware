################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/netutils/thttpd/fdwatch.c \
../NuttX/apps/netutils/thttpd/libhttpd.c \
../NuttX/apps/netutils/thttpd/tdate_parse.c \
../NuttX/apps/netutils/thttpd/thttpd.c \
../NuttX/apps/netutils/thttpd/thttpd_alloc.c \
../NuttX/apps/netutils/thttpd/thttpd_cgi.c \
../NuttX/apps/netutils/thttpd/thttpd_strings.c \
../NuttX/apps/netutils/thttpd/timers.c 

OBJS += \
./NuttX/apps/netutils/thttpd/fdwatch.o \
./NuttX/apps/netutils/thttpd/libhttpd.o \
./NuttX/apps/netutils/thttpd/tdate_parse.o \
./NuttX/apps/netutils/thttpd/thttpd.o \
./NuttX/apps/netutils/thttpd/thttpd_alloc.o \
./NuttX/apps/netutils/thttpd/thttpd_cgi.o \
./NuttX/apps/netutils/thttpd/thttpd_strings.o \
./NuttX/apps/netutils/thttpd/timers.o 

C_DEPS += \
./NuttX/apps/netutils/thttpd/fdwatch.d \
./NuttX/apps/netutils/thttpd/libhttpd.d \
./NuttX/apps/netutils/thttpd/tdate_parse.d \
./NuttX/apps/netutils/thttpd/thttpd.d \
./NuttX/apps/netutils/thttpd/thttpd_alloc.d \
./NuttX/apps/netutils/thttpd/thttpd_cgi.d \
./NuttX/apps/netutils/thttpd/thttpd_strings.d \
./NuttX/apps/netutils/thttpd/timers.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/netutils/thttpd/%.o: ../NuttX/apps/netutils/thttpd/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


