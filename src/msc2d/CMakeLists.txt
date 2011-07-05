include_directories( ${Boost_INCLUDE_DIR})

file(GLOB HEADERS *.h)
file(GLOB SOURCES mscomplex.cc msc_wrapper.cc critical_point_finder.cc)

add_library(msc2d STATIC ${HEADERS} ${SOURCES})
