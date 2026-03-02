cmake_minimum_required(VERSION 3.16)
project(fsw LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(fsw
    main.cpp
    imu/mti03_driver.cpp
)

target_include_directories(fsw PRIVATE imu)
target_compile_options(fsw PRIVATE -Wall -Wextra -O2)