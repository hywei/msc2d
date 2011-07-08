include_directories( ${Boost_INCLUDE_DIR})

file(GLOB HEADERS mscomplex.h critical_point_finder.h)
file(GLOB SOURCES mscomplex.cc critical_point_finder.cc)

add_library(msc2d STATIC ${HEADERS} ${SOURCES})
