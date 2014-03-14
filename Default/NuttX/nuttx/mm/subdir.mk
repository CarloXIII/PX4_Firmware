################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/mm/mm_addfreechunk.c \
../NuttX/nuttx/mm/mm_calloc.c \
../NuttX/nuttx/mm/mm_free.c \
../NuttX/nuttx/mm/mm_granalloc.c \
../NuttX/nuttx/mm/mm_grancritical.c \
../NuttX/nuttx/mm/mm_granfree.c \
../NuttX/nuttx/mm/mm_graninit.c \
../NuttX/nuttx/mm/mm_initialize.c \
../NuttX/nuttx/mm/mm_kernel.c \
../NuttX/nuttx/mm/mm_mallinfo.c \
../NuttX/nuttx/mm/mm_malloc.c \
../NuttX/nuttx/mm/mm_memalign.c \
../NuttX/nuttx/mm/mm_realloc.c \
../NuttX/nuttx/mm/mm_sem.c \
../NuttX/nuttx/mm/mm_shrinkchunk.c \
../NuttX/nuttx/mm/mm_size2ndx.c \
../NuttX/nuttx/mm/mm_user.c \
../NuttX/nuttx/mm/mm_zalloc.c 

OBJS += \
./NuttX/nuttx/mm/mm_addfreechunk.o \
./NuttX/nuttx/mm/mm_calloc.o \
./NuttX/nuttx/mm/mm_free.o \
./NuttX/nuttx/mm/mm_granalloc.o \
./NuttX/nuttx/mm/mm_grancritical.o \
./NuttX/nuttx/mm/mm_granfree.o \
./NuttX/nuttx/mm/mm_graninit.o \
./NuttX/nuttx/mm/mm_initialize.o \
./NuttX/nuttx/mm/mm_kernel.o \
./NuttX/nuttx/mm/mm_mallinfo.o \
./NuttX/nuttx/mm/mm_malloc.o \
./NuttX/nuttx/mm/mm_memalign.o \
./NuttX/nuttx/mm/mm_realloc.o \
./NuttX/nuttx/mm/mm_sem.o \
./NuttX/nuttx/mm/mm_shrinkchunk.o \
./NuttX/nuttx/mm/mm_size2ndx.o \
./NuttX/nuttx/mm/mm_user.o \
./NuttX/nuttx/mm/mm_zalloc.o 

C_DEPS += \
./NuttX/nuttx/mm/mm_addfreechunk.d \
./NuttX/nuttx/mm/mm_calloc.d \
./NuttX/nuttx/mm/mm_free.d \
./NuttX/nuttx/mm/mm_granalloc.d \
./NuttX/nuttx/mm/mm_grancritical.d \
./NuttX/nuttx/mm/mm_granfree.d \
./NuttX/nuttx/mm/mm_graninit.d \
./NuttX/nuttx/mm/mm_initialize.d \
./NuttX/nuttx/mm/mm_kernel.d \
./NuttX/nuttx/mm/mm_mallinfo.d \
./NuttX/nuttx/mm/mm_malloc.d \
./NuttX/nuttx/mm/mm_memalign.d \
./NuttX/nuttx/mm/mm_realloc.d \
./NuttX/nuttx/mm/mm_sem.d \
./NuttX/nuttx/mm/mm_shrinkchunk.d \
./NuttX/nuttx/mm/mm_size2ndx.d \
./NuttX/nuttx/mm/mm_user.d \
./NuttX/nuttx/mm/mm_zalloc.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/mm/%.o: ../NuttX/nuttx/mm/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


