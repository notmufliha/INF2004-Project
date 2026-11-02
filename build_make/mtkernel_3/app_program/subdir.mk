################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# app_program/*.c plus one level of subfolders (one per subsystem)
TEMP_SRCS = $(wildcard ../app_program/*.c) $(wildcard ../app_program/*/*.c)
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

# B4 uses ISO C99 as required by BARR-C. Preserve the kernel/SDK dialect.
B4_OBJS := $(patsubst ../%.c,mtkernel_3/%.o,$(wildcard ../app_program/subsystem4/*.c))
$(B4_OBJS): CFLAGS := $(filter-out -std=%,$(CFLAGS)) -std=c99 -Wall -Wextra -Wconversion -Wsign-conversion -Werror -Wno-cast-function-type

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	@mkdir -p $(dir $@)
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
