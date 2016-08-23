################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: I-WATCH_EE2024.axf

# Tool invocations
I-WATCH_EE2024.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\Wong Kok Woo\Downloads\EE2024Workspace\Lib_CMSISv1p30_LPC17xx\Debug" -L"C:\Users\Wong Kok Woo\Downloads\EE2024Workspace\Lib_EaBaseBoard\Debug" -L"C:\Users\Wong Kok Woo\Downloads\EE2024Workspace\Lib_MCU\Debug" -Xlinker --gc-sections -Xlinker -Map=I-WATCH_EE2024.map -mcpu=cortex-m3 -mthumb -T "I-WATCH_EE2024_Debug.ld" -o "I-WATCH_EE2024.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) I-WATCH_EE2024.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size I-WATCH_EE2024.axf; # arm-none-eabi-objdump -h -S I-WATCH_EE2024.axf >I-WATCH_EE2024.lss
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
