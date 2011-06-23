include_directories(
  ${PROJECT_SOURCE_DIR}/include)

file(GLOB HEADERS *.h)
file(GLOB SOURCES *.cpp)

link_directories(
  ${PROJECT_SOURCE_DIR}/lib)

add_executable(mesh_lib ${HEADERS} ${SOURCES} ${MOCFILES})

add_dependencies (mesh util)
                                      

target_link_libraries(mesh_lib mesh util)
                  
        
                  
