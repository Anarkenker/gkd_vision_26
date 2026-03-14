cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

# 创建目标 io
add_library(io STATIC
    hikrobot/hikrobot.cpp
    camera.cpp
    gkdcontrol.cpp
    gkdcontrol/send_control.cpp
    gkdcontrol/socket_interface.cpp
)

# hikrobot
target_include_directories(io PUBLIC hikrobot/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(io MvCameraControl usb-1.0 yaml-cpp)
