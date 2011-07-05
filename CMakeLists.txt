cmake_minimum_required(VERSION 2.8)

if(WIN32)
        set(CMAKE_FIND_LIBRARY_PREFIXES "")
        set(CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
else()
        set(CMAKE_FIND_LIBRARY_PREFIXES "lib")
        set(CMAKE_FIND_LIBRARY_SUFFIXES ".so")
endif()
  
project(msc2d)


message("${CMAKE_BUILD_TYPE}")
message("${CMAKE_CXX_COMPILER}")

if(WIN32)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/win32)
else()
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/linux)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "-fpermissive")

add_subdirectory(src/util)
add_subdirectory(src/mesh)
add_subdirectory(src/msc2d) 
add_subdirectory(src/main)
                    
