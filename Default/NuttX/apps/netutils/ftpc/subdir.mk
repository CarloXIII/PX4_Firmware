################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/netutils/ftpc/ftpc_cdup.c \
../NuttX/apps/netutils/ftpc/ftpc_chdir.c \
../NuttX/apps/netutils/ftpc/ftpc_chmod.c \
../NuttX/apps/netutils/ftpc/ftpc_cmd.c \
../NuttX/apps/netutils/ftpc/ftpc_connect.c \
../NuttX/apps/netutils/ftpc/ftpc_disconnect.c \
../NuttX/apps/netutils/ftpc/ftpc_filesize.c \
../NuttX/apps/netutils/ftpc/ftpc_filetime.c \
../NuttX/apps/netutils/ftpc/ftpc_getfile.c \
../NuttX/apps/netutils/ftpc/ftpc_getreply.c \
../NuttX/apps/netutils/ftpc/ftpc_help.c \
../NuttX/apps/netutils/ftpc/ftpc_idle.c \
../NuttX/apps/netutils/ftpc/ftpc_listdir.c \
../NuttX/apps/netutils/ftpc/ftpc_login.c \
../NuttX/apps/netutils/ftpc/ftpc_mkdir.c \
../NuttX/apps/netutils/ftpc/ftpc_noop.c \
../NuttX/apps/netutils/ftpc/ftpc_putfile.c \
../NuttX/apps/netutils/ftpc/ftpc_quit.c \
../NuttX/apps/netutils/ftpc/ftpc_rename.c \
../NuttX/apps/netutils/ftpc/ftpc_response.c \
../NuttX/apps/netutils/ftpc/ftpc_rmdir.c \
../NuttX/apps/netutils/ftpc/ftpc_rpwd.c \
../NuttX/apps/netutils/ftpc/ftpc_socket.c \
../NuttX/apps/netutils/ftpc/ftpc_transfer.c \
../NuttX/apps/netutils/ftpc/ftpc_unlink.c \
../NuttX/apps/netutils/ftpc/ftpc_utils.c 

OBJS += \
./NuttX/apps/netutils/ftpc/ftpc_cdup.o \
./NuttX/apps/netutils/ftpc/ftpc_chdir.o \
./NuttX/apps/netutils/ftpc/ftpc_chmod.o \
./NuttX/apps/netutils/ftpc/ftpc_cmd.o \
./NuttX/apps/netutils/ftpc/ftpc_connect.o \
./NuttX/apps/netutils/ftpc/ftpc_disconnect.o \
./NuttX/apps/netutils/ftpc/ftpc_filesize.o \
./NuttX/apps/netutils/ftpc/ftpc_filetime.o \
./NuttX/apps/netutils/ftpc/ftpc_getfile.o \
./NuttX/apps/netutils/ftpc/ftpc_getreply.o \
./NuttX/apps/netutils/ftpc/ftpc_help.o \
./NuttX/apps/netutils/ftpc/ftpc_idle.o \
./NuttX/apps/netutils/ftpc/ftpc_listdir.o \
./NuttX/apps/netutils/ftpc/ftpc_login.o \
./NuttX/apps/netutils/ftpc/ftpc_mkdir.o \
./NuttX/apps/netutils/ftpc/ftpc_noop.o \
./NuttX/apps/netutils/ftpc/ftpc_putfile.o \
./NuttX/apps/netutils/ftpc/ftpc_quit.o \
./NuttX/apps/netutils/ftpc/ftpc_rename.o \
./NuttX/apps/netutils/ftpc/ftpc_response.o \
./NuttX/apps/netutils/ftpc/ftpc_rmdir.o \
./NuttX/apps/netutils/ftpc/ftpc_rpwd.o \
./NuttX/apps/netutils/ftpc/ftpc_socket.o \
./NuttX/apps/netutils/ftpc/ftpc_transfer.o \
./NuttX/apps/netutils/ftpc/ftpc_unlink.o \
./NuttX/apps/netutils/ftpc/ftpc_utils.o 

C_DEPS += \
./NuttX/apps/netutils/ftpc/ftpc_cdup.d \
./NuttX/apps/netutils/ftpc/ftpc_chdir.d \
./NuttX/apps/netutils/ftpc/ftpc_chmod.d \
./NuttX/apps/netutils/ftpc/ftpc_cmd.d \
./NuttX/apps/netutils/ftpc/ftpc_connect.d \
./NuttX/apps/netutils/ftpc/ftpc_disconnect.d \
./NuttX/apps/netutils/ftpc/ftpc_filesize.d \
./NuttX/apps/netutils/ftpc/ftpc_filetime.d \
./NuttX/apps/netutils/ftpc/ftpc_getfile.d \
./NuttX/apps/netutils/ftpc/ftpc_getreply.d \
./NuttX/apps/netutils/ftpc/ftpc_help.d \
./NuttX/apps/netutils/ftpc/ftpc_idle.d \
./NuttX/apps/netutils/ftpc/ftpc_listdir.d \
./NuttX/apps/netutils/ftpc/ftpc_login.d \
./NuttX/apps/netutils/ftpc/ftpc_mkdir.d \
./NuttX/apps/netutils/ftpc/ftpc_noop.d \
./NuttX/apps/netutils/ftpc/ftpc_putfile.d \
./NuttX/apps/netutils/ftpc/ftpc_quit.d \
./NuttX/apps/netutils/ftpc/ftpc_rename.d \
./NuttX/apps/netutils/ftpc/ftpc_response.d \
./NuttX/apps/netutils/ftpc/ftpc_rmdir.d \
./NuttX/apps/netutils/ftpc/ftpc_rpwd.d \
./NuttX/apps/netutils/ftpc/ftpc_socket.d \
./NuttX/apps/netutils/ftpc/ftpc_transfer.d \
./NuttX/apps/netutils/ftpc/ftpc_unlink.d \
./NuttX/apps/netutils/ftpc/ftpc_utils.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/netutils/ftpc/%.o: ../NuttX/apps/netutils/ftpc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


