include_directories( ${Boost_INCLUDE_DIR})

file(GLOB HEADERS *.h)
file(GLOB SOURCES *.cpp)

add_library(mesh STATIC ${HEADERS} ${SOURCES})
