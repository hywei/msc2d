file(GLOB HEADERS *.h)
file(GLOB SOURCES *.cpp)

add_library(util STATIC ${HEADERS} ${SOURCES})
