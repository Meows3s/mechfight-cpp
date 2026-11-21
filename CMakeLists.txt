cmake_minimum_required(VERSION 3.16)
project(mechroyale LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "build type" FORCE)
endif()

add_library(mechfight STATIC src/mf.cpp)
target_include_directories(mechfight PUBLIC src third_party)
target_compile_options(mechfight PRIVATE -O2 -Wall -Wextra)

add_executable(bot src/main.cpp src/bot.cpp src/map.cpp)
target_link_libraries(bot PRIVATE mechfight)
target_compile_options(bot PRIVATE -O2 -Wall -Wextra)
