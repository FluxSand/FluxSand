# ---------------------------------------------------------------------------------------
# Verion and Path Check
cmake_minimum_required(VERSION 3.10)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(
    FATAL_ERROR
    "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there."
  )
endif()

# ---------------------------------------------------------------------------------------
# Project
project(
  FluxSand
  DESCRIPTION "FluxSand Build"
  LANGUAGES C CXX
)

# ---------------------------------------------------------------------------------------
# Set C/C++ Compiler
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build)

# ---------------------------------------------------------------------------------------
# Building Options
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_C_FLAGS_DEBUG "-Og -g")
  set(CMAKE_ASM_FLAGS_DEBUG "-Og -g")
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG "")
  add_compile_definitions(DEBUG_BUILD)
  add_compile_options(-Werror -gdwarf-4)
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O3 -g")
  set(CMAKE_ASM_FLAGS_RELWITHDEBINFO "-O3 -g")
  set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "")
  add_compile_definitions(DEBUG_BUILD)
  add_compile_options(-Werror -gdwarf-4)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_C_FLAGS_RELEASE "-O3")
  set(CMAKE_ASM_FLAGS_RELEASE "-O3")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "")
elseif(CMAKE_BUILD_TYPE STREQUAL "MinSizeRel")
  set(CMAKE_C_FLAGS_MINSIZEREL "-Os")
  set(CMAKE_ASM_FLAGS_MINSIZEREL "-Os")
  set(CMAKE_EXE_LINKER_FLAGS_MINSIZEREL "")
else()
  message(FATAL_ERROR "Unknown build type.")
endif()

# ---------------------------------------------------------------------------------------
# PkgConfig
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

# ---------------------------------------------------------------------------------------
# Sources
file(GLOB BSP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/bsp/*.cpp")
file(GLOB DEV_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/bsp/*.cpp")


set(SOURCES src/main.cpp ${BSP_SOURCES} ${DEV_SOURCES})

# ---------------------------------------------------------------------------------------
# Executable
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# ---------------------------------------------------------------------------------------
# Include
target_include_directories(${PROJECT_NAME}.elf
  PRIVATE ${GPIOD_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/bsp
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/device
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/component
)

# ---------------------------------------------------------------------------------------
# Link
add_compile_options(-fexperimental-library)
target_link_libraries(${PROJECT_NAME}.elf ${GPIOD_LIBRARIES} pthread)
set(CMAKE_CXX_FLAGS "-stdlib=libc++")

# ---------------------------------------------------------------------------------------
# Compile Options
target_compile_options(${PROJECT_NAME}.elf PRIVATE ${GPIOD_CFLAGS_OTHER})

# ---------------------------------------------------------------------------------------
# OneMessage
add_subdirectory(third_party/om)
target_include_directories(
    OneMessage
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/third_party/
)

target_link_libraries(${PROJECT_NAME}.elf OneMessage)

# ---------------------------------------------------------------------------------------
# Build information
message(
  STATUS
  "------------------------- Host Platform ------------------------------")

string(TIMESTAMP CONFIG_TIMESTAMP "" UTC)
message(STATUS "Timestamp: ${CONFIG_TIMESTAMP}")
message(
  STATUS
  "Host: ${CMAKE_HOST_SYSTEM_NAME} ${CMAKE_HOST_SYSTEM_VERSION} ${CMAKE_HOST_SYSTEM_PROCESSOR}"
)

if(CMAKE_CROSSCOMPILING) # set to true by CMake if the CMAKE_SYSTEM_NAME has
  # been set manually
  message(
    STATUS
    "Target: ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_VERSION} ${CMAKE_SYSTEM_PROCESSOR}"
  )
endif()

message(STATUS "CMake: ${CMAKE_VERSION}")
message(STATUS "CMake generator: ${CMAKE_GENERATOR}")
message(STATUS "CMake build tool: ${CMAKE_BUILD_TOOL}")
message(STATUS "Configuration: ${CMAKE_BUILD_TYPE}")

message(
  STATUS
  "---------------------- Compiler Linker option ------------------------")

message(STATUS "C standard: ${CMAKE_C_STANDARD}")
message(STATUS "CXX standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "")
message(STATUS "C Compiler:     ${CMAKE_C_COMPILER}")
message(STATUS "CXX Compiler:   ${CMAKE_CXX_COMPILER}")
message(STATUS "")
message(STATUS "C flags (General):  ${CMAKE_C_FLAGS}")
message(STATUS "C flags (Debug):    ${CMAKE_C_FLAGS_DEBUG}")
message(STATUS "C flags (Release):  ${CMAKE_C_FLAGS_RELEASE}")
message(STATUS "CXX flags (General):${CMAKE_CXX_FLAGS}")
message(STATUS "CXX flags (Debug):  ${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "CXX flags (Release):${CMAKE_CXX_FLAGS_RELEASE}")
