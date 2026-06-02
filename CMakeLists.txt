cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME DJ_A_wheel)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # USER sources
    ${CMAKE_SOURCE_DIR}/USER/Src/bsp_usart.c
    ${CMAKE_SOURCE_DIR}/USER/Src/car_ctrl.c
    ${CMAKE_SOURCE_DIR}/USER/Src/Initialize.c
    ${CMAKE_SOURCE_DIR}/USER/Src/motor_can.c
    ${CMAKE_SOURCE_DIR}/Math/pid.c
    ${CMAKE_SOURCE_DIR}/USER/Src/pid_tim.c
    ${CMAKE_SOURCE_DIR}/USER/Src/jy901p.c
    ${CMAKE_SOURCE_DIR}/USER/Src/JY901P_Calibrate.c
    ${CMAKE_SOURCE_DIR}/BSP/bsp_can.c
    ${CMAKE_SOURCE_DIR}/BSP/bsp_can_irq.c
    ${CMAKE_SOURCE_DIR}/BSP/bsp_rc_uart.c
    ${CMAKE_SOURCE_DIR}/DVE/dve_dji_motor.c
    ${CMAKE_SOURCE_DIR}/DVE/dve_dm_motor.c
    ${CMAKE_SOURCE_DIR}/DVE/dve_t15.c
    ${CMAKE_SOURCE_DIR}/Math/math_utils.c
    ${CMAKE_SOURCE_DIR}/Task/rc_task.c
    ${CMAKE_SOURCE_DIR}/Task/chassis_task.c

    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/list.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # USER include directory
    ${CMAKE_SOURCE_DIR}/USER/Inc
    ${CMAKE_SOURCE_DIR}/Task
    ${CMAKE_SOURCE_DIR}/Math
    ${CMAKE_SOURCE_DIR}/BSP
    ${CMAKE_SOURCE_DIR}/DVE
    ${CMAKE_SOURCE_DIR}/Core/Inc

    # 添加FreeRTOS头文件路径
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
