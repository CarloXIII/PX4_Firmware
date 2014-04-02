################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/nshlib/nsh_builtin.c \
../NuttX/apps/nshlib/nsh_codeccmd.c \
../NuttX/apps/nshlib/nsh_console.c \
../NuttX/apps/nshlib/nsh_consolemain.c \
../NuttX/apps/nshlib/nsh_dbgcmds.c \
../NuttX/apps/nshlib/nsh_ddcmd.c \
../NuttX/apps/nshlib/nsh_envcmds.c \
../NuttX/apps/nshlib/nsh_fileapps.c \
../NuttX/apps/nshlib/nsh_fscmds.c \
../NuttX/apps/nshlib/nsh_init.c \
../NuttX/apps/nshlib/nsh_mmcmds.c \
../NuttX/apps/nshlib/nsh_mntcmds.c \
../NuttX/apps/nshlib/nsh_netcmds.c \
../NuttX/apps/nshlib/nsh_netinit.c \
../NuttX/apps/nshlib/nsh_parse.c \
../NuttX/apps/nshlib/nsh_proccmds.c \
../NuttX/apps/nshlib/nsh_romfsetc.c \
../NuttX/apps/nshlib/nsh_script.c \
../NuttX/apps/nshlib/nsh_session.c \
../NuttX/apps/nshlib/nsh_telnetd.c \
../NuttX/apps/nshlib/nsh_test.c \
../NuttX/apps/nshlib/nsh_timcmds.c \
../NuttX/apps/nshlib/nsh_usbdev.c 

OBJS += \
./NuttX/apps/nshlib/nsh_builtin.o \
./NuttX/apps/nshlib/nsh_codeccmd.o \
./NuttX/apps/nshlib/nsh_console.o \
./NuttX/apps/nshlib/nsh_consolemain.o \
./NuttX/apps/nshlib/nsh_dbgcmds.o \
./NuttX/apps/nshlib/nsh_ddcmd.o \
./NuttX/apps/nshlib/nsh_envcmds.o \
./NuttX/apps/nshlib/nsh_fileapps.o \
./NuttX/apps/nshlib/nsh_fscmds.o \
./NuttX/apps/nshlib/nsh_init.o \
./NuttX/apps/nshlib/nsh_mmcmds.o \
./NuttX/apps/nshlib/nsh_mntcmds.o \
./NuttX/apps/nshlib/nsh_netcmds.o \
./NuttX/apps/nshlib/nsh_netinit.o \
./NuttX/apps/nshlib/nsh_parse.o \
./NuttX/apps/nshlib/nsh_proccmds.o \
./NuttX/apps/nshlib/nsh_romfsetc.o \
./NuttX/apps/nshlib/nsh_script.o \
./NuttX/apps/nshlib/nsh_session.o \
./NuttX/apps/nshlib/nsh_telnetd.o \
./NuttX/apps/nshlib/nsh_test.o \
./NuttX/apps/nshlib/nsh_timcmds.o \
./NuttX/apps/nshlib/nsh_usbdev.o 

C_DEPS += \
./NuttX/apps/nshlib/nsh_builtin.d \
./NuttX/apps/nshlib/nsh_codeccmd.d \
./NuttX/apps/nshlib/nsh_console.d \
./NuttX/apps/nshlib/nsh_consolemain.d \
./NuttX/apps/nshlib/nsh_dbgcmds.d \
./NuttX/apps/nshlib/nsh_ddcmd.d \
./NuttX/apps/nshlib/nsh_envcmds.d \
./NuttX/apps/nshlib/nsh_fileapps.d \
./NuttX/apps/nshlib/nsh_fscmds.d \
./NuttX/apps/nshlib/nsh_init.d \
./NuttX/apps/nshlib/nsh_mmcmds.d \
./NuttX/apps/nshlib/nsh_mntcmds.d \
./NuttX/apps/nshlib/nsh_netcmds.d \
./NuttX/apps/nshlib/nsh_netinit.d \
./NuttX/apps/nshlib/nsh_parse.d \
./NuttX/apps/nshlib/nsh_proccmds.d \
./NuttX/apps/nshlib/nsh_romfsetc.d \
./NuttX/apps/nshlib/nsh_script.d \
./NuttX/apps/nshlib/nsh_session.d \
./NuttX/apps/nshlib/nsh_telnetd.d \
./NuttX/apps/nshlib/nsh_test.d \
./NuttX/apps/nshlib/nsh_timcmds.d \
./NuttX/apps/nshlib/nsh_usbdev.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/nshlib/%.o: ../NuttX/apps/nshlib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


