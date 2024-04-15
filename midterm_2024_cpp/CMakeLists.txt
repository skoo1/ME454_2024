cmake_minimum_required (VERSION 3.16)
project(midterm)
set(CMAKE_CXX_STANDARD 17)

include_directories(include)
find_library(MYMAT_LIB NAMES mymat PATHS "${CMAKE_CURRENT_SOURCE_DIR}/lib")
message(STATUS "Found mymat library: ${MYMAT_LIB}")

macro(macro1 arg)
  add_executable(earth_precession_${arg} src/earth_precession_${arg}.cpp)
  target_link_directories(earth_precession_${arg} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib)
  target_link_libraries(earth_precession_${arg} myQuaternion_${arg} mymat pthread)
  set_target_properties(earth_precession_${arg} PROPERTIES INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
  target_include_directories(earth_precession_${arg} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endmacro()

macro1(20230001)