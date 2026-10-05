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
set(CMAKE_PROJECT_NAME Gimbal)

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
    # Add user sources here
    Core/Src/freertos.c
    User/Bsp/Src/bsp_can.c
    User/Bsp/Src/bsp_dr16.c
    User/Bsp/Src/bsp_dwt.c
    User/Bsp/Src/bsp_PWM.c
    User/Bsp/Src/bsp_bmi088.c
    User/Bsp/Src/bsp_spi.c
    User/Bsp/Src/bsp_cdc.c
    User/Device/Src/DJI_Motor.cpp
    User/Device/Src/DM_Motor.cpp
    User/Device/Src/IMU.cpp
    User/Algorithm/Src/PID.cpp
    User/Algorithm/Src/Controller.cpp
    User/Algorithm/Src/QuaternionEKF.cpp
    User/Task/Src/CanReceiveTask.cpp
    User/Task/Src/GimbalTask.cpp
    User/Task/Src/ShootTask.cpp
    User/Task/Src/BoardComTask.cpp
    User/Task/Src/INSTask.cpp
    User/App/Src/Gimbal.cpp
    User/App/Src/INS.cpp
    User/App/Src/Shoot.cpp
    User/App/Src/BoardCom.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    User/Bsp/Inc
    User/Device/Inc
    User/Algorithm/Inc
    User/Task/Inc
    User/App/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# 编译完成后自动生成烧录文件 Gimbal.hex / Gimbal.bin 并打印占用尺寸
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex/.bin and printing size"
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
