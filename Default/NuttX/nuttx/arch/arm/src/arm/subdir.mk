################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/arm/src/arm/up_allocpage.c \
../NuttX/nuttx/arch/arm/src/arm/up_assert.c \
../NuttX/nuttx/arch/arm/src/arm/up_blocktask.c \
../NuttX/nuttx/arch/arm/src/arm/up_checkmapping.c \
../NuttX/nuttx/arch/arm/src/arm/up_copystate.c \
../NuttX/nuttx/arch/arm/src/arm/up_dataabort.c \
../NuttX/nuttx/arch/arm/src/arm/up_doirq.c \
../NuttX/nuttx/arch/arm/src/arm/up_elf.c \
../NuttX/nuttx/arch/arm/src/arm/up_initialstate.c \
../NuttX/nuttx/arch/arm/src/arm/up_pginitialize.c \
../NuttX/nuttx/arch/arm/src/arm/up_prefetchabort.c \
../NuttX/nuttx/arch/arm/src/arm/up_releasepending.c \
../NuttX/nuttx/arch/arm/src/arm/up_reprioritizertr.c \
../NuttX/nuttx/arch/arm/src/arm/up_schedulesigaction.c \
../NuttX/nuttx/arch/arm/src/arm/up_sigdeliver.c \
../NuttX/nuttx/arch/arm/src/arm/up_syscall.c \
../NuttX/nuttx/arch/arm/src/arm/up_unblocktask.c \
../NuttX/nuttx/arch/arm/src/arm/up_undefinedinsn.c \
../NuttX/nuttx/arch/arm/src/arm/up_va2pte.c 

S_UPPER_SRCS += \
../NuttX/nuttx/arch/arm/src/arm/up_cache.S \
../NuttX/nuttx/arch/arm/src/arm/up_fullcontextrestore.S \
../NuttX/nuttx/arch/arm/src/arm/up_head.S \
../NuttX/nuttx/arch/arm/src/arm/up_nommuhead.S \
../NuttX/nuttx/arch/arm/src/arm/up_saveusercontext.S \
../NuttX/nuttx/arch/arm/src/arm/up_vectoraddrexcptn.S \
../NuttX/nuttx/arch/arm/src/arm/up_vectors.S \
../NuttX/nuttx/arch/arm/src/arm/up_vectortab.S \
../NuttX/nuttx/arch/arm/src/arm/vfork.S 

OBJS += \
./NuttX/nuttx/arch/arm/src/arm/up_allocpage.o \
./NuttX/nuttx/arch/arm/src/arm/up_assert.o \
./NuttX/nuttx/arch/arm/src/arm/up_blocktask.o \
./NuttX/nuttx/arch/arm/src/arm/up_cache.o \
./NuttX/nuttx/arch/arm/src/arm/up_checkmapping.o \
./NuttX/nuttx/arch/arm/src/arm/up_copystate.o \
./NuttX/nuttx/arch/arm/src/arm/up_dataabort.o \
./NuttX/nuttx/arch/arm/src/arm/up_doirq.o \
./NuttX/nuttx/arch/arm/src/arm/up_elf.o \
./NuttX/nuttx/arch/arm/src/arm/up_fullcontextrestore.o \
./NuttX/nuttx/arch/arm/src/arm/up_head.o \
./NuttX/nuttx/arch/arm/src/arm/up_initialstate.o \
./NuttX/nuttx/arch/arm/src/arm/up_nommuhead.o \
./NuttX/nuttx/arch/arm/src/arm/up_pginitialize.o \
./NuttX/nuttx/arch/arm/src/arm/up_prefetchabort.o \
./NuttX/nuttx/arch/arm/src/arm/up_releasepending.o \
./NuttX/nuttx/arch/arm/src/arm/up_reprioritizertr.o \
./NuttX/nuttx/arch/arm/src/arm/up_saveusercontext.o \
./NuttX/nuttx/arch/arm/src/arm/up_schedulesigaction.o \
./NuttX/nuttx/arch/arm/src/arm/up_sigdeliver.o \
./NuttX/nuttx/arch/arm/src/arm/up_syscall.o \
./NuttX/nuttx/arch/arm/src/arm/up_unblocktask.o \
./NuttX/nuttx/arch/arm/src/arm/up_undefinedinsn.o \
./NuttX/nuttx/arch/arm/src/arm/up_va2pte.o \
./NuttX/nuttx/arch/arm/src/arm/up_vectoraddrexcptn.o \
./NuttX/nuttx/arch/arm/src/arm/up_vectors.o \
./NuttX/nuttx/arch/arm/src/arm/up_vectortab.o \
./NuttX/nuttx/arch/arm/src/arm/vfork.o 

C_DEPS += \
./NuttX/nuttx/arch/arm/src/arm/up_allocpage.d \
./NuttX/nuttx/arch/arm/src/arm/up_assert.d \
./NuttX/nuttx/arch/arm/src/arm/up_blocktask.d \
./NuttX/nuttx/arch/arm/src/arm/up_checkmapping.d \
./NuttX/nuttx/arch/arm/src/arm/up_copystate.d \
./NuttX/nuttx/arch/arm/src/arm/up_dataabort.d \
./NuttX/nuttx/arch/arm/src/arm/up_doirq.d \
./NuttX/nuttx/arch/arm/src/arm/up_elf.d \
./NuttX/nuttx/arch/arm/src/arm/up_initialstate.d \
./NuttX/nuttx/arch/arm/src/arm/up_pginitialize.d \
./NuttX/nuttx/arch/arm/src/arm/up_prefetchabort.d \
./NuttX/nuttx/arch/arm/src/arm/up_releasepending.d \
./NuttX/nuttx/arch/arm/src/arm/up_reprioritizertr.d \
./NuttX/nuttx/arch/arm/src/arm/up_schedulesigaction.d \
./NuttX/nuttx/arch/arm/src/arm/up_sigdeliver.d \
./NuttX/nuttx/arch/arm/src/arm/up_syscall.d \
./NuttX/nuttx/arch/arm/src/arm/up_unblocktask.d \
./NuttX/nuttx/arch/arm/src/arm/up_undefinedinsn.d \
./NuttX/nuttx/arch/arm/src/arm/up_va2pte.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/arm/src/arm/%.o: ../NuttX/nuttx/arch/arm/src/arm/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

NuttX/nuttx/arch/arm/src/arm/%.o: ../NuttX/nuttx/arch/arm/src/arm/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


