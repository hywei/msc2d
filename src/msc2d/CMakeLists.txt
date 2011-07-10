include_directories( ${Boost_INCLUDE_DIR})

file(GLOB HEADERS *.h)
file(GLOB SOURCES *.cc)

add_library(msc2d STATIC ${HEADERS} ${SOURCES})
