cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(spline_fitting)

# include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/alglib)

# set policy
if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()

# set some important infomation
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)
set(CMAKE_BUILD_TYPE DEBUG)
# set(CMAKE_BUILD_TYPE RELEASE)
# if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O0 -Wall -g -ggdb")
#     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -O0   -Wall -g -ggdb")
# else ()
#     if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
#         set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O3")
#         set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -O3")
#     else()
#         set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
#         set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
#     endif ()
# endif ()

file(GLOB SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/alglib/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/utils/*.cpp")
file(GLOB HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/alglib/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/utils/*.h")
file(GLOB LIB_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB LIB_HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
file(GLOB LIB_UTILS_HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/utils/*.h")

# generate library

add_library(asfit SHARED ${LIB_SOURCE_FILES} ${HEADER_FILES} ${SOURCE_FILES})
install(TARGETS asfit LIBRARY DESTINATION lib)
install(FILES ${LIB_HEADER_FILES} DESTINATION include) 
install(FILES ${LIB_UTILS_HEADER_FILES} DESTINATION include/utils) 

# generate file
add_executable(spline_fitting_test ${HEADER_FILES} ${SOURCE_FILES} alglib_spline_fitting.cpp chp_spline_fitting.cpp test.cpp)
target_link_libraries(spline_fitting_test)