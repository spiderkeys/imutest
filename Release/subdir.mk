################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../.ino.cpp \
../CI2C.cpp 

INO_SRCS += \
../ImuTester.ino 

INO_DEPS += \
./ImuTester.ino.d 

CPP_DEPS += \
./.ino.cpp.d \
./CI2C.cpp.d 

LINK_OBJ += \
./.ino.cpp.o \
./CI2C.cpp.o 


# Each subdirectory must supply rules for building sources it contributes
.ino.cpp.o: ../.ino.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR     -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/cores/arduino" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/variants/mega" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

CI2C.cpp.o: ../CI2C.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR     -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/cores/arduino" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/variants/mega" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

ImuTester.o: ../ImuTester.ino
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR     -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/cores/arduino" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/variants/mega" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '


