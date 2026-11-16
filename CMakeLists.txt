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
set(CMAKE_PROJECT_NAME H735_IMU)

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

# Collect application and board support sources
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/APP/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/APP/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/BSP/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Components/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Components/*.cpp"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        ${USER_SOURCES}
)

# Recursively add include directories to a target
function(recursion_include target path)
    if(NOT IS_DIRECTORY "${path}")
        return()
    endif()
    target_include_directories(${target} PRIVATE "${path}")
    file(GLOB CHILDREN CONFIGURE_DEPENDS
            LIST_DIRECTORIES true
            "${path}/*"
    )
    foreach(CHILD IN LISTS CHILDREN)
        if(IS_DIRECTORY "${CHILD}")
            recursion_include(${target} "${CHILD}")
        endif()
    endforeach()
endfunction()

# Add APP/BSP and all their subdirectories
recursion_include(${CMAKE_PROJECT_NAME}
        "${CMAKE_CURRENT_SOURCE_DIR}/APP"
)
recursion_include(${CMAKE_PROJECT_NAME}
        "${CMAKE_CURRENT_SOURCE_DIR}/BSP"
)
recursion_include(${CMAKE_PROJECT_NAME}
        "${CMAKE_CURRENT_SOURCE_DIR}/Components"
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

# 开启 printf / snprintf 的浮点格式化支持
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        -u _printf_float
)