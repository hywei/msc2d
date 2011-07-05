include_directories(
  ${PROJECT_SOURCE_DIR}/include)

file(GLOB HEADERS *.h)
file(GLOB SOURCES *.cpp)

link_directories(
  ${PROJECT_SOURCE_DIR}/lib)

add_executable(msc ${HEADERS} ${SOURCES} ${MOCFILES})

add_dependencies (msc2d mesh util)
                                      

target_link_libraries(msc msc2d mesh util)
                  
        
                  
