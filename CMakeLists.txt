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
        src/filter/GyroAngleFilter.cpp
        src/filter/GyroRateFilter.cpp
        src/util/HandlerHelper.cpp
        src/protocol/I2C.cpp
        src/IOwrapper/IMU.cpp
        src/main.cpp
        src/IOwrapper/Motor.cpp
        src/protocol/PWM.cpp
        src/protocol/PWMInput.cpp
        src/PID/RatePID.cpp
        src/syscalls.c
        src/system_stm32f4xx.c
        src/util/Timer.cpp
        src/protocol/USART.cpp
        src/util/USARTHelper.cpp
        src/util/misc.cpp
        startup/startup_stm32f446xx.s src/IOwrapper/Sticks.cpp inc/IOwrapper/Sticks.h inc/PID/common.h)

add_executable(HelloWorld ${SOURCE_FILES})
