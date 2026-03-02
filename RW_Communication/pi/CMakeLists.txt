cmake_minimum_required(VERSION 3.16)
project(rw_i2c LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(rw_i2c
    main.cpp
    i2c_rw.cpp
)

target_include_directories(rw_i2c PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
