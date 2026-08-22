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
    User/Device/Src/DJI_Motor.cpp
    User/Device/Src/BMI088driver.c
    User/Device/Src/BMI088Middleware.c
    User/Device/Src/DM_Motor.cpp
    User/Algorithm/Src/PID.cpp
    User/Algorithm/Src/controller.c
    User/Algorithm/Src/kalman_filter.c
    User/Algorithm/Src/QuaternionEKF.c
    User/Algorithm/Src/transfer_function.c
    User/Algorithm/Src/user_lib.c
    User/Task/Src/INS_Task.c
    User/Task/Src/CanReceiveTask.cpp
    User/Task/Src/GimbalTask.cpp
    User/Task/Src/ShootTask.cpp
    User/Task/Src/BoardComTask.cpp
    User/App/Src/Gimbal.cpp
    User/App/Src/INS.cpp
    User/App/Src/Shoot.cpp
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

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
