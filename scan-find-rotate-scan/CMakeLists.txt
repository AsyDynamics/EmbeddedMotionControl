cmake_minimum_required(VERSION 2.8)
project(EMC)

add_executable(scan src/map_room.cpp)
target_link_libraries(scan emc-framework)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


