cmake_minimum_required(VERSION 3.16)
project(Fines_Serial)

add_compile_options(-Wno-sign-compare -Wno-unused-but-set-variable -Wno-deprecated-enum-float-conversion -Wno-strict-aliasing)

fins_add_node(${PROJECT_NAME} SHARED 
  src/fins_serial_station.cpp
  src/serial/serial.cc
  src/serial/impl/unix.cc
  src/serial/impl/list_ports/list_ports_linux.cc
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  include/serial
  include/utils
)

fins_link_ros_dependencies(${PROJECT_NAME}
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  fmt::fmt
  rt
  pthread
)
