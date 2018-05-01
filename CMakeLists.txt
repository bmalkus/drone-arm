cmake_minimum_required(VERSION 3.8)
project(HelloWorld)

set(CMAKE_CXX_STANDARD 14)

include_directories(inc/)
include_directories(CMSIS/core)
include_directories(CMSIS/device)

add_definitions(-DSTM32)
add_definitions(-DSTM32F4)
add_definitions(-DSTM32F446xx)
add_definitions(-DSTM32F446RETx)
add_definitions(-DNUCLEO_F446RE)

set(SOURCE_FILES
        CMSIS/core/arm_common_tables.h
        CMSIS/core/arm_const_structs.h
        CMSIS/core/arm_math.h
        CMSIS/core/cmsis_armcc.h
        CMSIS/core/cmsis_armcc_V6.h
        CMSIS/core/cmsis_gcc.h
        CMSIS/core/core_cm0.h
        CMSIS/core/core_cm0plus.h
        CMSIS/core/core_cm3.h
        CMSIS/core/core_cm4.h
        CMSIS/core/core_cm7.h
        CMSIS/core/core_cmFunc.h
        CMSIS/core/core_cmInstr.h
        CMSIS/core/core_cmSimd.h
        CMSIS/core/core_sc000.h
        CMSIS/core/core_sc300.h
        CMSIS/device/stm32f446xx.h
        CMSIS/device/stm32f4xx.h
        CMSIS/device/system_stm32f4xx.h
        src/GyroAngleFilter.cpp
        src/GyroSimpleFilter.cpp
        src/HandlerHelper.cpp
        src/I2C.cpp
        src/IMU.cpp
        src/main.cpp
        src/Motor.cpp
        src/PWM.cpp
        src/PWMInput.cpp
        src/SimplePID.cpp
        src/syscalls.c
        src/system_stm32f4xx.c
        src/Timer.cpp
        src/USART.cpp
        src/USARTHelper.cpp
        src/utils.cpp
        startup/startup_stm32f446xx.s)

add_executable(HelloWorld ${SOURCE_FILES})