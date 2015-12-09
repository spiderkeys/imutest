################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -f

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include arduino/core/subdir.mk
-include Libraries/Wire/utility/subdir.mk
-include Libraries/Wire/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
AR += \
arduino.ar \

ELF += \
ImuTester.elf \

EEP += \
ImuTester.eep \

FLASH_IMAGE += \
ImuTester.hex \


# All Target
all: ImuTester

# Tool invocations
ImuTester: $(FLASH_IMAGE)
	@echo 'Building target: $@'
	@echo 'Printing size:'
	"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-size" -A "/home/openrov/eclipseArduino/workspace/ImuTester/Release/ImuTester.elf"
	@echo 'Finished building target: $@'
	@echo ' '

arduino.ar: $(AR_OBJ)
	@echo 'Starting archiver'
	"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-ar" rcs  "/home/openrov/eclipseArduino/workspace/ImuTester/Release/arduino.ar"   $(AR_OBJ)
	@echo 'Finished building: $@'
	@echo ' '

ImuTester.elf: $(LINK_OBJ) $(AR)
	@echo 'Starting combiner'
	"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-gcc" -Os -Wl,--gc-sections,--relax -mmcu=atmega2560  -o "/home/openrov/eclipseArduino/workspace/ImuTester/Release/ImuTester.elf"   $(LINK_OBJ) $(AR)  "/home/openrov/eclipseArduino/workspace/ImuTester/Release/arduino.ar" "-L/home/openrov/eclipseArduino/workspace/ImuTester/Release" -lm
	@echo 'Finished building: $@'
	@echo ' '

ImuTester.eep: $(ELF)
	@echo 'Create eeprom image'
	"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-objcopy" -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0  "/home/openrov/eclipseArduino/workspace/ImuTester/Release/ImuTester.elf" "/home/openrov/eclipseArduino/workspace/ImuTester/Release/ImuTester.eep"
	@echo 'Finished building: $@'
	@echo ' '

ImuTester.hex: $(EEP)
	@echo 'Create Flash image (ihex format)'
	"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-objcopy" -O ihex -R .eeprom  "/home/openrov/eclipseArduino/workspace/ImuTester/Release/ImuTester.elf" "/home/openrov/eclipseArduino/workspace/ImuTester/Release/ImuTester.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_DEPS)$(AR)$(FLASH_IMAGE)$(EEP)$(CXX_DEPS)$(S_UPPER_DEPS)$(ELF)$(C++_DEPS)$(ASM_DEPS)$(CC_DEPS)$(INO_DEPS)$(AR_OBJ)$(CPP_DEPS)$(LINK_OBJ)$(C_UPPER_DEPS)$(SIZEDUMMY) ImuTester
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
