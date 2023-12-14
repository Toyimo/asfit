cmake_minimum_required(VERSION 3.16)
project(asfitdemo)

set(CMAKE_CXX_STANDARD 14)

link_directories (${PROJECT_SOURCE_DIR}/lib)
add_executable(${PROJECT_NAME} demo.cpp)

include_directories( ${PROJECT_SOURCE_DIR}/include)
target_link_libraries (${PROJECT_NAME} libasfit.so)