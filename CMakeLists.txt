cmake_minimum_required(VERSION 3.16)
project(AeroTelem VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Active les warnings de compilation (bonne pratique embarqué)
add_compile_options(-Wall -Wextra -Wpedantic)

include_directories(include)

add_executable(aerotelem_node
    src/main.cpp
    src/telemetry.cpp
)
