cmake_minimum_required(VERSION 2.8)
project(EMC)

add_executable(monitoring src/map_room.cpp)
target_link_libraries(monitoring emc-framework)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


