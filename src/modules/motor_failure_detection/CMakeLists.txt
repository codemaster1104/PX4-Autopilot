px4_add_module(
    MODULE modules__motor_failure_detection
    MAIN motor_failure_detection
    SRCS motor_failure_detection.cpp
    DEPENDS
    )

include_directories(${CMAKE_SOURCE_DIR}/src/modules/commander)

