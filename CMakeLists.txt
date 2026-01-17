cmake_minimum_required(VERSION 3.12)
project(unit_two_project_builder)
set(CMAKE_CXX_STANDARD 17)

add_executable(unit_two ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_include_directories(unit_two PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../SDL/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../rtaudio-6.0.1
)

target_link_libraries(unit_two PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../SDL/build/libSDL3.so
    ${CMAKE_CURRENT_SOURCE_DIR}/../rtaudio-6.0.1/build/librtaudio.so
)