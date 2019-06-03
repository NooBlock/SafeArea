cmake_minimum_required(VERSION 3.1)
project(safearea)


if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 ")
endif()

add_definitions(${COMPILE_DEFINITIONS})

#--------------Add security options --------------------
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FORTIFY_SOURCE=2 ")  #TODO: Check what it is
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -fstack-protector-strong")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -fPIE -fPIC")

#add_subdirectory(lib)
add_subdirectory(src)

