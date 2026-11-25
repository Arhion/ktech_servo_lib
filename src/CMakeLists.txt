cmake_minimum_required(VERSION 3.5)

project(servo_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



set(HEADERS
    libservo_controller.h

    servo_def.h
    servo_can_struct.h
    servo_rs485_struct.h
    state_def.h
    servo_struct.h

    servo_controller.h
    servo_can_controller.h
    servo_rs485_controller.h


    can_controller.h
    rs485_controller.h

    utils.h
    )

set(SOURCES

    libservo_controller.cpp

    servo_controller.cpp
    servo_can_controller.cpp
    servo_rs485_controller.cpp


    can_controller.cpp
    rs485_controller.cpp

    utils.cpp
    )



add_library(servo_controller SHARED
    ${HEADERS}
    ${SOURCES})


target_include_directories(servo_controller PUBLIC
    "/usr/local/include")


target_link_libraries(servo_controller PUBLIC
    "/usr/local/lib/libsocketcan.so"
    -ldl
    -lpthread)

