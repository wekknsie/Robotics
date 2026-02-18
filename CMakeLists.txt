cmake_minimum_required(VERSION 3.15)

project(Lab2KK VERSION 1.0
        DESCRIPTION "Very nice project"
        LANGUAGES CXX)

# Prefer target-specific settings in modern CMake. If you want a global default:
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# GTests setup
add_subdirectory(tests)

add_executable(Lab2KK
  src/main.cpp
  src/lib.cpp
)

# Tell the target where to find headers under include/
# Use PUBLIC to propagate include paths to dependents (if this were a library).
target_include_directories(Lab2KK PUBLIC ${CMAKE_SOURCE_DIR}/include)
