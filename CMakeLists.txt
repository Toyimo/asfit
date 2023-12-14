cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(spline_fitting)

# include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/alglib)

# set policy
if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()

# link file
set(CMAKE_CXX_STANDARD 11)
file(GLOB SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/alglib/*.cpp")
file(GLOB HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/alglib/*.h")

# generate library
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)
add_library(asfit SHARED ${CMAKE_CURRENT_SOURCE_DIR}/alglib_spline_fitting.cpp ${HEADER_FILES} ${SOURCE_FILES})
install(TARGETS asfit LIBRARY DESTINATION lib)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/alglib_spline_fitting.h  DESTINATION include) 

# generate file
add_executable(spline_fitting_test ${HEADER_FILES} ${SOURCE_FILES} alglib_spline_fitting.cpp demo.cpp)
target_link_libraries(spline_fitting_test)