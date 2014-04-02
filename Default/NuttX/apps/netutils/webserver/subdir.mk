################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/netutils/webserver/httpd.c \
../NuttX/apps/netutils/webserver/httpd_cgi.c \
../NuttX/apps/netutils/webserver/httpd_fs.c \
../NuttX/apps/netutils/webserver/httpd_mmap.c \
../NuttX/apps/netutils/webserver/httpd_sendfile.c 

OBJS += \
./NuttX/apps/netutils/webserver/httpd.o \
./NuttX/apps/netutils/webserver/httpd_cgi.o \
./NuttX/apps/netutils/webserver/httpd_fs.o \
./NuttX/apps/netutils/webserver/httpd_mmap.o \
./NuttX/apps/netutils/webserver/httpd_sendfile.o 

C_DEPS += \
./NuttX/apps/netutils/webserver/httpd.d \
./NuttX/apps/netutils/webserver/httpd_cgi.d \
./NuttX/apps/netutils/webserver/httpd_fs.d \
./NuttX/apps/netutils/webserver/httpd_mmap.d \
./NuttX/apps/netutils/webserver/httpd_sendfile.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/netutils/webserver/%.o: ../NuttX/apps/netutils/webserver/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


