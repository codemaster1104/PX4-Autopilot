px4_add_module(
    MODULE modules__waypoint_test
    MAIN waypoint_test
    SRCS waypoint_test.cpp
    DEPENDS
)

include_directories(${CMAKE_SOURCE_DIR}/src/modules/commander)
