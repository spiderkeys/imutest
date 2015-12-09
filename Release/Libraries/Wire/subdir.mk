################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire/Wire.cpp 

CPP_DEPS += \
./Libraries/Wire/Wire.cpp.d 

LINK_OBJ += \
./Libraries/Wire/Wire.cpp.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/Wire/Wire.cpp.o: /home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire/Wire.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR     -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/cores/arduino" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/variants/mega" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '


