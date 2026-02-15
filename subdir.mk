################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
./src/flute/flute.cpp 

OBJS += \
./src/flute/flute.o 

CPP_DEPS += \
./src/flute/flute.d 


# Each subdirectory must supply rules for building sources it contributes
src/flute/%.o: ./src/flute/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++14 -DBOOST_DISABLE_ASSERTS -O3 -march=native -Wall -c -fmessage-length=0 -pthread -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


