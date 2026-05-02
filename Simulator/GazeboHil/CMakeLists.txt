cmake_minimum_required(VERSION 3.16)
project(HilPlugin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/opt/qt@5")
    set(Qt5_DIR "/opt/homebrew/opt/qt@5/lib/cmake/Qt5" CACHE PATH "Qt5 CMake config directory")
endif()

# Try several Gazebo Sim versions.
# If this fails, run:
#   pkg-config --list-all | grep gz-sim
# and replace gz-sim10/gz-sim9/gz-sim8 with your installed version.
find_package(gz-sim10 QUIET)
if(gz-sim10_FOUND)
    set(GZ_SIM_TARGET gz-sim10::gz-sim10)
else()
    find_package(gz-sim9 QUIET)
    if(gz-sim9_FOUND)
        set(GZ_SIM_TARGET gz-sim9::gz-sim9)
    else()
        find_package(gz-sim8 REQUIRED)
        set(GZ_SIM_TARGET gz-sim8::gz-sim8)
    endif()
endif()

find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(SDL2 REQUIRED)

add_library(HilPlugin SHARED
        core/src/HilPlugin.cpp
        core/src/SerialPort.cpp
        core/src/MavlinkBridge.cpp
        core/src/CsvLogger.cpp
        core/src/JoystickInput.cpp
)

target_include_directories(HilPlugin PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core/inc
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party
        ${SDL2_INCLUDE_DIRS}
)

target_link_libraries(HilPlugin PRIVATE
        ${GZ_SIM_TARGET}
        gz-plugin2::register
        SDL2::SDL2
)

target_compile_options(HilPlugin PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

set_target_properties(HilPlugin PROPERTIES
        OUTPUT_NAME "HilPlugin"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins"
)
# On macOS/Linux, termios is part of libc, no extra lib needed.