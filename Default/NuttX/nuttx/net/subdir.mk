################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/net/accept.c \
../NuttX/nuttx/net/bind.c \
../NuttX/nuttx/net/connect.c \
../NuttX/nuttx/net/getsockname.c \
../NuttX/nuttx/net/getsockopt.c \
../NuttX/nuttx/net/listen.c \
../NuttX/nuttx/net/net_arptimer.c \
../NuttX/nuttx/net/net_checksd.c \
../NuttX/nuttx/net/net_clone.c \
../NuttX/nuttx/net/net_close.c \
../NuttX/nuttx/net/net_dsec2timeval.c \
../NuttX/nuttx/net/net_dup.c \
../NuttX/nuttx/net/net_dup2.c \
../NuttX/nuttx/net/net_monitor.c \
../NuttX/nuttx/net/net_poll.c \
../NuttX/nuttx/net/net_sockets.c \
../NuttX/nuttx/net/net_timeo.c \
../NuttX/nuttx/net/net_timeval2dsec.c \
../NuttX/nuttx/net/net_vfcntl.c \
../NuttX/nuttx/net/netdev_count.c \
../NuttX/nuttx/net/netdev_findbyaddr.c \
../NuttX/nuttx/net/netdev_findbyname.c \
../NuttX/nuttx/net/netdev_foreach.c \
../NuttX/nuttx/net/netdev_ioctl.c \
../NuttX/nuttx/net/netdev_register.c \
../NuttX/nuttx/net/netdev_sem.c \
../NuttX/nuttx/net/netdev_txnotify.c \
../NuttX/nuttx/net/netdev_unregister.c \
../NuttX/nuttx/net/recv.c \
../NuttX/nuttx/net/recvfrom.c \
../NuttX/nuttx/net/send.c \
../NuttX/nuttx/net/sendto.c \
../NuttX/nuttx/net/setsockopt.c \
../NuttX/nuttx/net/socket.c 

OBJS += \
./NuttX/nuttx/net/accept.o \
./NuttX/nuttx/net/bind.o \
./NuttX/nuttx/net/connect.o \
./NuttX/nuttx/net/getsockname.o \
./NuttX/nuttx/net/getsockopt.o \
./NuttX/nuttx/net/listen.o \
./NuttX/nuttx/net/net_arptimer.o \
./NuttX/nuttx/net/net_checksd.o \
./NuttX/nuttx/net/net_clone.o \
./NuttX/nuttx/net/net_close.o \
./NuttX/nuttx/net/net_dsec2timeval.o \
./NuttX/nuttx/net/net_dup.o \
./NuttX/nuttx/net/net_dup2.o \
./NuttX/nuttx/net/net_monitor.o \
./NuttX/nuttx/net/net_poll.o \
./NuttX/nuttx/net/net_sockets.o \
./NuttX/nuttx/net/net_timeo.o \
./NuttX/nuttx/net/net_timeval2dsec.o \
./NuttX/nuttx/net/net_vfcntl.o \
./NuttX/nuttx/net/netdev_count.o \
./NuttX/nuttx/net/netdev_findbyaddr.o \
./NuttX/nuttx/net/netdev_findbyname.o \
./NuttX/nuttx/net/netdev_foreach.o \
./NuttX/nuttx/net/netdev_ioctl.o \
./NuttX/nuttx/net/netdev_register.o \
./NuttX/nuttx/net/netdev_sem.o \
./NuttX/nuttx/net/netdev_txnotify.o \
./NuttX/nuttx/net/netdev_unregister.o \
./NuttX/nuttx/net/recv.o \
./NuttX/nuttx/net/recvfrom.o \
./NuttX/nuttx/net/send.o \
./NuttX/nuttx/net/sendto.o \
./NuttX/nuttx/net/setsockopt.o \
./NuttX/nuttx/net/socket.o 

C_DEPS += \
./NuttX/nuttx/net/accept.d \
./NuttX/nuttx/net/bind.d \
./NuttX/nuttx/net/connect.d \
./NuttX/nuttx/net/getsockname.d \
./NuttX/nuttx/net/getsockopt.d \
./NuttX/nuttx/net/listen.d \
./NuttX/nuttx/net/net_arptimer.d \
./NuttX/nuttx/net/net_checksd.d \
./NuttX/nuttx/net/net_clone.d \
./NuttX/nuttx/net/net_close.d \
./NuttX/nuttx/net/net_dsec2timeval.d \
./NuttX/nuttx/net/net_dup.d \
./NuttX/nuttx/net/net_dup2.d \
./NuttX/nuttx/net/net_monitor.d \
./NuttX/nuttx/net/net_poll.d \
./NuttX/nuttx/net/net_sockets.d \
./NuttX/nuttx/net/net_timeo.d \
./NuttX/nuttx/net/net_timeval2dsec.d \
./NuttX/nuttx/net/net_vfcntl.d \
./NuttX/nuttx/net/netdev_count.d \
./NuttX/nuttx/net/netdev_findbyaddr.d \
./NuttX/nuttx/net/netdev_findbyname.d \
./NuttX/nuttx/net/netdev_foreach.d \
./NuttX/nuttx/net/netdev_ioctl.d \
./NuttX/nuttx/net/netdev_register.d \
./NuttX/nuttx/net/netdev_sem.d \
./NuttX/nuttx/net/netdev_txnotify.d \
./NuttX/nuttx/net/netdev_unregister.d \
./NuttX/nuttx/net/recv.d \
./NuttX/nuttx/net/recvfrom.d \
./NuttX/nuttx/net/send.d \
./NuttX/nuttx/net/sendto.d \
./NuttX/nuttx/net/setsockopt.d \
./NuttX/nuttx/net/socket.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/net/%.o: ../NuttX/nuttx/net/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


