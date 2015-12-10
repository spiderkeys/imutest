################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../.ino.cpp \
../CAdaBNO055.cpp \
../CBMX.cpp \
../CBNO055.cpp \
../CI2C.cpp \
../CLSD.cpp \
../CModule.cpp \
../CPin.cpp \
../CTimer.cpp 

INO_SRCS += \
../ImuTester.ino 

INO_DEPS += \
./ImuTester.ino.d 

CPP_DEPS += \
./.ino.cpp.d \
./CAdaBNO055.cpp.d \
./CBMX.cpp.d \
./CBNO055.cpp.d \
./CI2C.cpp.d \
./CLSD.cpp.d \
./CModule.cpp.d \
./CPin.cpp.d \
./CTimer.cpp.d 

LINK_OBJ += \
./.ino.cpp.o \
./CAdaBNO055.cpp.o \
./CBMX.cpp.o \
./CBNO055.cpp.o \
./CI2C.cpp.o \
./CLSD.cpp.o \
./CModule.cpp.o \
./CPin.cpp.o \
./CTimer.cpp.o 


# Each subdirectory must supply rules for building sources it contributes
.ino.cpp.o: ../.ino.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR     -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/cores/arduino" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/variants/mega" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

CAdaBNO055.cpp.o: ../CAdaBNO055.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR     -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/cores/arduino" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/variants/mega" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

CBMX.cpp.o: ../CBMX.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR     -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/cores/arduino" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/variants/mega" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

CBNO055.cpp.o: ../CBNO055.cpp
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

CLSD.cpp.o: ../CLSD.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR     -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/cores/arduino" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/variants/mega" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

CModule.cpp.o: ../CModule.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR     -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/cores/arduino" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/variants/mega" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

CPin.cpp.o: ../CPin.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/tools/avr/bin/avr-g++" -c -g -Os -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10605 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR     -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/cores/arduino" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/variants/mega" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire" -I"/home/openrov/Downloads/arduino-1.6.5-r5/hardware/arduino/avr/libraries/Wire/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"   -Wall
	@echo 'Finished building: $<'
	@echo ' '

CTimer.cpp.o: ../CTimer.cpp
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


