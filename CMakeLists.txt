cmake_minimum_required(VERSION 3.19)
project(unit_two_project_builder)
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

FetchContent_Declare(
    sculpt
    GIT_REPOSITORY https://github.com/p-short/synthesis_library.git
    GIT_TAG master
)

set(BUILD_SCULPT_LIB ON CACHE BOOL "Build sculpt lib")
set(BUILD_SCULPT_WITH_RTAUDIO OFF CACHE BOOL "Dont include RtAudio its already linked")
set(BUILD_SCULPT_CLIENT_APPS OFF CACHE BOOL "Dont build sculpt example projects")

FetchContent_MakeAvailable(sculpt)

add_executable(unit_two ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_include_directories(unit_two PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../SDL/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../rtaudio-6.0.1
    ${sculpt_SOURCE_DIR}/lib
)

target_link_libraries(unit_two PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../SDL/build/libSDL3.so
    ${CMAKE_CURRENT_SOURCE_DIR}/../rtaudio-6.0.1/build/librtaudio.so
    music_language
)