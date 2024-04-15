cmake_minimum_required (VERSION 3.16)
project(midterm)
set(CMAKE_CXX_STANDARD 17)

# Include mymat library
include_directories(include)
find_library(MYMAT_LIB NAMES mymat PATHS "${CMAKE_CURRENT_SOURCE_DIR}/lib")

macro(macro1 arg)
  add_library(myQuaternion_${arg} SHARED src/myQuaternion_${arg}.cpp)
  target_link_directories(myQuaternion_${arg} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib)
  target_link_libraries(myQuaternion_${arg} mymat)
  set_target_properties(myQuaternion_${arg} PROPERTIES INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
  target_include_directories(myQuaternion_${arg} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endmacro()

macro(macro2 arg)
  add_executable(test src/test.cpp)
  target_link_directories(test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib)
  target_link_libraries(test mymat myQuaternion_${arg})
  set_target_properties(test PROPERTIES INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
  target_include_directories(test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endmacro()

macro1(20230001)
macro2(20230001)